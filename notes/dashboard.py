"""
Go2 dashboard v2 -- one page showing everything the robot is reporting.

    python3 dashboard.py                          # against the robot
    python3 dashboard.py --domain 1 --iface lo    # against unitree_mujoco
    python3 dashboard.py --camera-host 192.168.123.18:8080

    python3 dashboard.py --record walk1.jsonl     # live, and save it to a file
    python3 dashboard.py --replay walk1.jsonl     # play that back, no robot needed

Then open http://<this machine>:8090 .

WHAT THIS PROGRAM IS
--------------------
The robot broadcasts its state continuously over DDS -- a publish/subscribe
system where nobody connects to anybody, publishers just put messages on the
wire under a topic name and any subscriber on the same network and "domain"
receives them. We subscribe to two topics, keep the latest message from each,
and serve a web page that polls a JSON snapshot of them.

    rt/lowstate         ~500 Hz  joints, IMU, foot forces, battery
    rt/sportmodestate   ~50 Hz   gait, body velocity, body height

DDS DOMAINS AND INTERFACES
--------------------------
A "domain" is a number that partitions the network: participants on domain 0
never see domain 1. The real robot uses domain 0; unitree_mujoco defaults to
domain 1 (its simulate_python/config.py). The interface is which network card
to listen on -- the robot-facing one for real hardware, "lo" (loopback) when
the simulator is running on this same machine.

TOPIC NAMES DIFFER BETWEEN THE ROBOT AND THE SIMULATOR
------------------------------------------------------
The real robot publishes both a full-rate copy and a low-frequency copy:

    rt/lowstate  and  rt/lf/lowstate

The simulator publishes only the un-prefixed names. v1 of this dashboard
subscribed to the "lf" names alone and so showed nothing against the simulator.
v2 subscribes to both and reports which one is actually delivering -- see
/health, and the "source" line on the page.

WHERE THIS RUNS
---------------
Designed to run on the robot's Jetson (192.168.123.18): Python 3.8, standard
library plus unitree_sdk2py, no pip installs, no internet. Nothing here imports
anything outside those. three.js is vendored under dashboard_static/vendor/ and
served by this process, so the page loads with the laptop offline. See
docs/decisions.md D8 for why the browser-facing services live on the Jetson.
"""
import argparse
import calendar
import io
import json
import math
import mimetypes
import os
import collections
import re
import shutil
import signal
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, unquote

# The robot SDK is only needed to listen to a live robot. Replay mode reads a
# recorded file instead, so the page can be shown on a machine that has no SDK
# installed and no robot anywhere near it -- a laptop on a train, for instance.
# The walk library reads recordings and reports what is wrong with them. It is
# standard library only, so it works on the Jetson; if it is missing the panel
# says so rather than the whole dashboard failing to start.
try:
    import walk_library
except ImportError:
    walk_library = None

# The Sensors panel: depth camera and LiDAR stream on and off. Also standard
# library only; missing, the panel says so.
try:
    import sensors
except ImportError:
    sensors = None

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
    HAVE_SDK = True
except ImportError:
    HAVE_SDK = False

# ---------------------------------------------------------------------------
# JOINT ORDER -- the same convention as notes/make_model.py, repeated here so a
# reader of either file sees it. The SDK indexes its twelve motors 0..11:
#
#     0  FR hip   1  FR thigh   2  FR calf
#     3  FL hip   4  FL thigh   5  FL calf
#     6  RR hip   7  RR thigh   8  RR calf
#     9  RL hip  10  RL thigh  11  RL calf
#
# F/R is front/rear, R/L is right/left, so motor_state[4] is the front-LEFT
# thigh. Angles are radians. foot_force is indexed FR, FL, RR, RL to match.
# ---------------------------------------------------------------------------
JOINTS = [
    "FR hip", "FR thigh", "FR calf",
    "FL hip", "FL thigh", "FL calf",
    "RR hip", "RR thigh", "RR calf",
    "RL hip", "RL thigh", "RL calf",
]
FEET = ["FR", "FL", "RR", "RL"]

# ---------------------------------------------------------------------------
# FOOT LOAD FROM THE LEG MOTORS
#
# foot_force is a raw count from a pressure sensor in each foot, and on this
# robot the four do not agree: measured 2026-09-26 standing still, the rear
# right reads about 1.35x what the others read for the same load, so it always
# looked like the heaviest foot. foot_force_est is all zeros on the Go2.
#
# The motors give an honest answer instead. A leg standing still holds the
# ground's push with its thigh and knee motors, so from their torques and the
# leg's shape the push can be worked out (tau = -J^T F, solved for F). In
# newtons, and the four add up to the robot's weight, about 200 N with the
# LiDAR, camera and mounts on. Sideways (hip) motion is ignored; standing and
# walking keep the hip within a few degrees, where that costs under 1%.
# ---------------------------------------------------------------------------
THIGH_M = CALF_M = 0.213        # Go2 link lengths, from Unitree's go2_description URDF


def leg_load(q_thigh, q_calf, tau_thigh, tau_calf):
    """Upward force (N) the ground puts on one foot, from its two motors."""
    a, b = q_thigh, q_thigh + q_calf
    # Jacobian of the foot (x forward, z up) against the two joint angles.
    jxa, jxb = -THIGH_M * math.cos(a) - CALF_M * math.cos(b), -CALF_M * math.cos(b)
    jza, jzb = THIGH_M * math.sin(a) + CALF_M * math.sin(b), CALF_M * math.sin(b)
    det = jxa * jzb - jza * jxb
    if abs(det) < 1e-4:          # leg straight: the torques say nothing about load
        return 0.0
    return max(0.0, -(-jxb * tau_thigh + jxa * tau_calf) / det)

# SportModeState_.gait_type and .mode are small integers. These are the
# meanings from Unitree's sport-mode documentation; anything unrecognised is
# shown as the raw number rather than guessed at.
GAITS = {0: "idle", 1: "trot", 2: "trot running", 3: "climb stair", 4: "trot obstacle"}
MODES = {0: "idle", 1: "balance stand", 2: "pose", 3: "locomotion", 4: "reserve",
         5: "lie down", 6: "joint lock", 7: "damping", 8: "recovery stand",
         9: "reserve", 10: "sit", 11: "front flip", 12: "front jump",
         13: "front pounce"}

# Topics to try, in order. The first entry of each pair is what the simulator
# uses and what the robot also publishes at full rate; the "lf" variant is the
# robot's low-frequency copy, which is cheaper if you only need a few Hz.
LOW_TOPICS = ["rt/lowstate", "rt/lf/lowstate"]
SPORT_TOPICS = ["rt/sportmodestate", "rt/lf/sportmodestate"]

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard_static")
STALE_AFTER = 2.0          # seconds without a message before the page shouts

# Where the browser should fetch the camera MJPEG streams from.
#
# "auto" means "the same address you reached this page on, port 8080", which is
# resolved in the browser rather than here. That default matters: the dashboard
# is served by the Jetson, and the Jetson has more than one address -- the wired
# 192.168.123.18 and whatever it answers to over Wi-Fi. Hardcoding the wired one
# meant the page loaded fine over Wi-Fi with two dead camera panels, because the
# browser was still being told to fetch video from an address it could not
# reach. An explicit "host:port" still overrides it, as does the settings drawer.
CAMERA_HOST = os.environ.get("GO2_CAMERA_HOST", "auto:8080")


def local_addresses():
    """Every IPv4 address this machine answers to, with its interface name.

    Used only to tell the page which of the robot's addresses you arrived on,
    so the header can say "wired" or "Wi-Fi" rather than leaving you to guess.

    `ip` is present on the Jetson (Ubuntu 20.04) and on the laptop's WSL. If it
    is missing or behaves unexpectedly the list simply comes back empty and the
    header falls back to showing the raw address, which is still useful.
    """
    out = []
    try:
        text = subprocess.check_output(
            ["ip", "-o", "-4", "addr", "show"],
            stderr=subprocess.STDOUT).decode("utf-8", "replace")
    except (OSError, subprocess.CalledProcessError):
        return out

    for line in text.splitlines():
        # e.g. "2: eth0    inet 192.168.123.18/24 brd ... scope global eth0"
        m = re.search(r"^\d+:\s+(\S+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)", line)
        if not m:
            continue
        iface, addr = m.group(1), m.group(2)
        out.append({"iface": iface, "addr": addr, "kind": iface_kind(iface)})
    return out


def iface_kind(iface):
    """wired / wireless / loopback / other, from the interface name.

    Linux names are conventional rather than guaranteed, so this is a hint for
    a label on screen and nothing depends on it being right.
    """
    if iface == "lo":
        return "loopback"
    if iface.startswith(("wl", "wlan", "wlp", "ra", "ap")):
        return "wireless"
    if iface.startswith(("eth", "en", "usb")):
        return "wired"
    return "other"


class Stream(object):
    """The newest message on a topic, plus enough history to report a rate.

    We may subscribe to several topic names for the same thing (see
    LOW_TOPICS). Whichever one delivers is recorded in `.topic`, so the page
    can say where its numbers are actually coming from.
    """

    def __init__(self, names):
        self.names = names
        self.lock = threading.Lock()
        self.msg = None
        self.topic = None
        self.count = 0
        self.stamps = []       # arrival times inside the rate window
        self.last = 0.0

    def handler(self, name):
        def on_message(msg):
            now = time.time()
            with self.lock:
                self.msg = msg
                self.topic = name
                self.count += 1
                self.last = now
                self.stamps.append(now)
                if len(self.stamps) > 600:
                    self.stamps = self.stamps[-600:]
        return on_message

    def read(self):
        """Latest message, measured rate in Hz, seconds since last message."""
        now = time.time()
        with self.lock:
            msg, topic, last = self.msg, self.topic, self.last
            window = [s for s in self.stamps if now - s <= 1.0]
            self.stamps = window
        age = (now - last) if last else None
        return msg, topic, float(len(window)), age


low = Stream(LOW_TOPICS)
sport = Stream(SPORT_TOPICS)
started = time.time()


def snapshot():
    """Everything the page needs, as plain JSON.

    Keys from v1 are kept exactly as they were so anything already reading
    /data keeps working. v2 adds "source", "stale" and "quat".
    """
    low_msg, low_topic, low_hz, low_age = low.read()
    sport_msg, sport_topic, sport_hz, sport_age = sport.read()

    stale = low_age is None or low_age > STALE_AFTER
    out = {
        "connected": low_msg is not None and not stale,
        "stale": stale,
        "age": round(low_age, 3) if low_age is not None else None,
        "rates": {"lowstate": low_hz, "sportmode": sport_hz},
        "source": {"lowstate": low_topic, "sportmode": sport_topic},
        "uptime": round(time.time() - started, 1),
    }

    if low_msg is not None:
        m = low_msg
        r, p, y = m.imu_state.rpy
        out["power"] = {
            "soc": m.bms_state.soc,
            "volts": round(m.power_v, 2),
            "amps": round(m.power_a, 2),
            "temp": max(m.temperature_ntc1, m.temperature_ntc2),
        }
        out["imu"] = {
            "roll": round(r, 4), "pitch": round(p, 4), "yaw": round(y, 4),
            "quat": [round(v, 5) for v in m.imu_state.quaternion],
            "accel": [round(a, 2) for a in m.imu_state.accelerometer],
            "gyro": [round(g, 3) for g in m.imu_state.gyroscope],
        }
        out["joints"] = [
            {"name": JOINTS[i],
             "q": round(m.motor_state[i].q, 4),
             "dq": round(m.motor_state[i].dq, 2),
             "tau": round(m.motor_state[i].tau_est, 2),
             "temp": m.motor_state[i].temperature}
            for i in range(12)
        ]
        # "force" stays the raw sensor count (the old page reads it); "load"
        # is newtons from the motors, which the new page shows.
        ms = m.motor_state
        out["feet"] = [{"name": FEET[i], "force": int(m.foot_force[i]),
                        "load": round(leg_load(ms[3 * i + 1].q, ms[3 * i + 2].q,
                                               ms[3 * i + 1].tau_est, ms[3 * i + 2].tau_est), 1)}
                       for i in range(4)]

    out["streams"] = stream_report()

    if sport_msg is not None:
        s = sport_msg
        out["motion"] = {
            "mode": int(s.mode),
            "mode_name": MODES.get(int(s.mode), "mode %d" % int(s.mode)),
            "gait": GAITS.get(int(s.gait_type), "type %d" % int(s.gait_type)),
            "body_height": round(s.body_height, 3),
            "velocity": [round(v, 3) for v in s.velocity],
            "yaw_speed": round(s.yaw_speed, 3),
            "position": [round(v, 3) for v in s.position],
        }
    return out


# ---------------------------------------------------------------------------
# RECORD AND REPLAY
#
# The robot is often not in the room. Recording writes one snapshot per line to
# a file while the dashboard runs normally; replaying serves those snapshots
# back in their original timing, on a loop, with no robot and no DDS at all.
#
# That makes the page demonstrable anywhere, and it gives us a fixed input to
# develop against -- yesterday's foot-load scale question can be settled from a
# recording rather than by keeping the robot switched on.
#
# The format is JSON Lines: one complete /data document per line, each with an
# extra "t" holding seconds since the recording started. Plain text on purpose,
# so it can be inspected with a text editor or trimmed with head/tail.
# ---------------------------------------------------------------------------

class Recorder(object):
    """Appends snapshots to a file at a steady rate, on its own thread."""

    def __init__(self, path, hz):
        self.path = path
        self.interval = 1.0 / max(hz, 0.1)
        self.frames = 0
        self.t0 = time.time()
        self.running = True
        self.fh = open(path, "w")
        # Daemon, so Ctrl-C on the server does not hang waiting for it. The file
        # is flushed every frame, so being killed loses nothing.
        t = threading.Thread(target=self._loop, name="recorder")
        t.daemon = True
        t.start()

    def stop(self):
        self.running = False
        try:
            self.fh.close()
        except Exception:
            pass

    def _loop(self):
        while self.running:
            time.sleep(self.interval)
            if not self.running:
                break
            frame = snapshot()
            # Only record frames that actually carry data, so a recording never
            # begins with a stretch of nothing while the robot wakes up.
            if not frame.get("joints"):
                continue
            frame["t"] = round(time.time() - self.t0, 3)
            self.fh.write(json.dumps(frame) + "\n")
            self.fh.flush()          # so a killed process still leaves a usable file
            self.frames += 1
            if not self.running:
                break


class Replayer(object):
    """Serves a recorded file back in its original timing, looping forever."""

    def __init__(self, path):
        self.frames = []
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    self.frames.append(json.loads(line))
                except ValueError:
                    continue          # skip a truncated last line
        if not self.frames:
            # Raised, not exited: a bad file chosen from the page must produce
            # an error message, not take the whole server down.
            raise ValueError("%s contains no frames" % os.path.basename(path))

        self.path = path
        self.duration = self.frames[-1].get("t", 0.0) or 0.001
        self.started = time.time()

    def current(self):
        """The frame whose timestamp matches how far into the loop we are."""
        elapsed = (time.time() - self.started) % self.duration
        # Frames are in time order, so walk back from the end.
        chosen = self.frames[0]
        for f in self.frames:
            if f.get("t", 0.0) <= elapsed:
                chosen = f
            else:
                break

        out = dict(chosen)
        # A replay must never be mistaken for a live robot. Say so in the data
        # itself, not only in the page, so /data is honest to any reader.
        out["replay"] = {
            "file": os.path.basename(self.path),
            "position": round(elapsed, 1),
            "duration": round(self.duration, 1),
            "frames": len(self.frames),
        }
        out["stale"] = False
        out["age"] = 0.0
        out["uptime"] = round(time.time() - started, 1)
        return out


recorder = None
replayer = None
dds_ready = False          # did we ever subscribe? False when started with --replay
RECORDINGS_DIR = None
SESSIONS_DIR = None        # where record_session.py writes walks
WALKS_DIR = None           # where finished walks are kept, on the laptop
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
job = None                 # the one conversion allowed at a time
last_verdict = None        # the last finished walk, checked
SETTINGS_PATH = os.path.join(TOOLS_DIR, "settings.json")
RECORDER = None            # path to record_session.py
SUDO_PASSWORD = os.environ.get("GO2_PASSWORD", "")  # the Jetson's sudo password
walk = None                # the running WalkRecorder, if any
DDS_IFACE = None           # what we subscribed on, so the recorder can match
DDS_DOMAIN = None


# ---------------------------------------------------------------------------
# WALK RECORDING
#
# A "walk" is the big one: raw LiDAR packets plus the IMU at full rate, for
# feeding a SLAM pipeline later. That is a different job from the dashboard
# recording above -- roughly 160 MB a minute rather than 1 MB, and it is the
# actual sensor data rather than the numbers this page displays.
#
# The work is done by jetson/record_session.py, which has to run as root
# because it captures packets off the network. This class starts and stops it.
# Two details make that possible without the dashboard itself being root:
#
#   * stopping goes through a "stop file" rather than a signal. We could not
#     signal a root process anyway, and the recorder polls for the file and
#     shuts down cleanly, finishing its pcap properly.
#   * progress comes from a status.json the recorder writes once a second,
#     which we only ever read.
# ---------------------------------------------------------------------------

def _dir_size(path):
    total = 0
    try:
        for entry in os.listdir(path):
            try:
                total += os.path.getsize(os.path.join(path, entry))
            except OSError:
                pass
    except OSError:
        pass
    return total


def _stream_health(name, count, rate, expected, unit, silent_note,
                   fast_note=None):
    """One stream, judged against what it should be doing."""
    if count == 0:
        return {"name": name, "count": count, "rate": rate,
                "expected": expected, "unit": unit, "ok": False,
                "note": silent_note}
    if fast_note and rate > expected * 1.5:
        return {"name": name, "count": count, "rate": rate,
                "expected": expected, "unit": unit, "ok": False,
                "note": fast_note}
    ok = rate >= expected * 0.6
    return {"name": name, "count": count, "rate": rate, "expected": expected,
            "unit": unit, "ok": ok,
            "note": ("%.0f %s (expected about %.0f)" % (rate, unit, expected)
                     if ok else
                     "%.0f %s, well under the %.0f expected"
                     % (rate, unit, expected))}


class WalkRecorder(object):
    """Runs record_session.py on this machine and watches it."""

    def __init__(self, name, sessions_dir, recorder, password, duration=0,
                 iface=None, domain=None):
        self.name = name
        self.dir = os.path.join(sessions_dir, name)
        self.stop_file = os.path.join(tempfile.gettempdir(),
                                      "go2_walk_stop_%s" % name)
        self.started = time.time()
        self.output = []

        # sudo resets the environment, which silently breaks the recorder: it
        # loses PYTHONPATH (so unitree_sdk2py disappears), LD_LIBRARY_PATH (so
        # CycloneDDS disappears) and HOME (so a pip --user install is no longer
        # on the path -- which is exactly how unitree_sdk2py is installed on
        # the Jetson). Forward them explicitly rather than relying on sudo -E,
        # which a sudoers policy may refuse.
        passthrough = []
        for var in ("PYTHONPATH", "LD_LIBRARY_PATH", "CYCLONEDDS_HOME",
                    "CYCLONEDDS_URI", "HOME"):
            value = os.environ.get(var)
            if value:
                passthrough.append("%s=%s" % (var, value))
        # Python's per-user package directory is derived from HOME, so put it
        # on PYTHONPATH directly and stop depending on HOME surviving at all.
        try:
            import site
            user_site = site.getusersitepackages()
            if user_site and os.path.isdir(user_site):
                existing = os.environ.get("PYTHONPATH", "")
                merged = user_site + ((os.pathsep + existing) if existing else "")
                passthrough = [v for v in passthrough
                               if not v.startswith("PYTHONPATH=")]
                passthrough.append("PYTHONPATH=" + merged)
        except Exception:
            pass

        cmd = ["sudo", "-S", "env"] + passthrough + [
               sys.executable, recorder,
               "--name", name, "--dir", sessions_dir,
               "--stop-file", self.stop_file]
        if iface:
            cmd += ["--iface", iface]
        if domain is not None:
            cmd += ["--domain", str(domain)]
        if duration:
            cmd += ["--duration", str(duration)]

        self.proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        # sudo -S reads the password from stdin. Same approach go2-start.sh
        # already uses to sync the robot's clock.
        try:
            self.proc.stdin.write((password + "\n").encode("utf-8"))
            self.proc.stdin.flush()
        except (OSError, ValueError):
            pass

        t = threading.Thread(target=self._drain, name="walk-output")
        t.daemon = True
        t.start()

    def _drain(self):
        """Keep the pipe empty and keep the last few lines for error reports."""
        try:
            for line in self.proc.stdout:
                text = line.decode("utf-8", "replace").strip()
                if text:
                    self.output.append(text)
                    if len(self.output) > 40:
                        self.output = self.output[-40:]
        except (OSError, ValueError):
            pass

    def alive(self):
        return self.proc.poll() is None

    def status(self):
        """Live progress, read from the file the recorder writes."""
        info = {"name": self.name, "running": self.alive(),
                "seconds": round(time.time() - self.started, 1),
                "lidar_packets": 0, "lidar_bytes": 0,
                "imu_rows": 0, "imu_hz": 0.0}
        try:
            with open(os.path.join(self.dir, "status.json")) as fh:
                info.update(json.load(fh))
        except (OSError, ValueError):
            pass                      # not written yet, in the first second
        info["running"] = self.alive()

        # Everything below is about being able to tell, from the page, whether
        # this recording is going to be worth having -- while it is still
        # running and can be restarted, rather than tomorrow on the laptop.
        seconds = max(info.get("seconds") or 0.0, 0.001)
        packets = info.get("lidar_packets") or 0
        info["lidar_pps"] = round(packets / seconds, 1)
        info["bytes_on_disk"] = _dir_size(self.dir)

        free, total, minutes = disk_headroom(os.path.dirname(self.dir))
        info["disk_free"] = free
        info["disk_total"] = total
        info["minutes_left"] = round(minutes, 1) if minutes is not None else None

        # Judged against what each stream should be doing, not against zero.
        # "2,505 packets a second" means nothing on its own; "2,505 of an
        # expected 2,535" means the sensor is healthy, and "5,070" means every
        # packet is being recorded twice.
        info["health"] = [
            _stream_health("lidar", packets, info["lidar_pps"], 2535.0,
                           "packets/s",
                           "nothing from the sensor. Recording will produce a "
                           "session with no cloud in it at all.",
                           "about twice the XT16's rate -- every packet is "
                           "being recorded twice."),
            _stream_health("imu", info.get("imu_rows") or 0,
                           info.get("imu_hz") or 0.0, 500.0, "Hz",
                           "no IMU. Check the robot is still sending state."),
            _stream_health("odometry", info.get("odom_rows") or 0,
                           info.get("odom_hz") or 0.0, 300.0, "Hz",
                           "no odometry arriving. Every scan will land at the "
                           "same spot and the walk will be a smear. Stop, check "
                           "Streams, and start again."),
        ]
        # Depth only when the camera was on: record_session.py starts
        # depth_recorder.py only then, and a walk without depth is normal.
        depth = info.get("depth")
        if depth:
            frames = (depth.get("frames") or {}).get("depth", 0)
            dropped = (depth.get("dropped") or {}).get("depth", 0)
            h = _stream_health("depth", frames, depth.get("depth_fps") or 0.0, 15.0,
                               "fps", "the depth recorder is running but no frames "
                               "are arriving. Check the Sensors panel.")
            if dropped:
                h["note"] += ", %d dropped" % dropped
            info["health"].append(h)
        # Nothing has had time to arrive in the first few seconds, so do not
        # cry wolf before there is anything to judge.
        if seconds < 5.0:
            for h in info["health"]:
                h["ok"] = True
                h["note"] = "starting"
        return info

    def stop(self, timeout=40.0):
        """Ask it to finish, and wait so the pcap is closed properly.

        40 s rather than 15: with the depth camera on, the recorder also waits
        for depth_recorder.py to compress its last queued frames and close.
        """
        try:
            open(self.stop_file, "w").close()
        except OSError:
            pass
        deadline = time.time() + timeout
        while self.alive() and time.time() < deadline:
            time.sleep(0.2)
        if self.alive():
            # It should never come to this; a killed recorder still leaves a
            # readable pcap because every packet is written as it arrives.
            self.proc.kill()
            return False
        return True

    def failure(self):
        """A one-line reason if the recorder died early, else None."""
        if self.alive():
            return None
        text = " / ".join(self.output[-4:]) or "no output"
        if "password" in text.lower() or "sudo" in text.lower():
            return "could not get root on the robot (check the password): " + text
        return text


def _remove_walk_dir(target):
    """Delete a walk folder. Its files are root-owned, so this goes via sudo."""
    if not os.path.isdir(target):
        return True
    try:
        proc = subprocess.Popen(["sudo", "-S", "rm", "-rf", target],
                                stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        proc.communicate((SUDO_PASSWORD + "\n").encode("utf-8"), timeout=20)
    except Exception:
        return False
    return not os.path.isdir(target)


def list_walks():
    """Past walks, newest first, with whatever their session.json recorded."""
    out = []
    try:
        names = os.listdir(SESSIONS_DIR)
    except OSError:
        return out
    for n in sorted(names):
        d = os.path.join(SESSIONS_DIR, n)
        if not os.path.isdir(d):
            continue
        entry = {"name": n, "bytes": 0, "seconds": 0.0, "complete": False}
        for f in ("lidar.pcap", "imu.csv"):
            try:
                entry["bytes"] += os.path.getsize(os.path.join(d, f))
            except OSError:
                pass
        try:
            with open(os.path.join(d, "session.json")) as fh:
                meta = json.load(fh)
            entry["seconds"] = meta.get("seconds", 0.0)
            entry["complete"] = True
            entry["clock_ok"] = meta.get("clock_was_synced", False)
            entry["lidar_packets"] = (meta.get("lidar") or {}).get("packets", 0)
            entry["imu_rows"] = (meta.get("imu") or {}).get("rows", 0)
        except (OSError, ValueError):
            pass                      # still running, or ended badly
        try:
            entry["modified"] = int(os.path.getmtime(d))
        except OSError:
            entry["modified"] = 0
        out.append(entry)
    out.sort(key=lambda w: w["modified"], reverse=True)
    return out


# ---------------------------------------------------------------------------
# A BROWSER IS ALL IT TAKES
#
# Two jobs used to need the WSL laptop: setting the robot's clock (go2-start.sh
# runs `sudo date -s` over ssh) and copying walks off the robot (fetch_walk.sh,
# scp). Both are here now, so any laptop with a browser and a cable can record
# a walk and take it home.
# ---------------------------------------------------------------------------

# Anything earlier is a browser whose own clock is wrong, and setting the robot
# to it would spoil the walk as surely as 1970 does. Anything later than 2040 is
# a typo or a bug, not a date.
CLOCK_EARLIEST = calendar.timegm((2026, 1, 1, 0, 0, 0))
CLOCK_LATEST = calendar.timegm((2040, 1, 1, 0, 0, 0))


def set_clock(epoch_ms):
    """Set this machine's clock to the browser's. Returns (status, dict).

    Through sudo, the same way the recorder is started. It is refused while
    anything is recording: a walk's LiDAR and IMU are lined up by timestamp,
    and a jump of decades half way through is worse than a walk at 1970.
    """
    try:
        seconds = float(epoch_ms) / 1000.0
    except (TypeError, ValueError):
        return 400, {"error": "epoch_ms must be a number of milliseconds"}
    if not CLOCK_EARLIEST <= seconds <= CLOCK_LATEST:
        return 400, {"error": "refusing %s -- that is not a plausible date. Is "
                              "this computer's own clock right?"
                              % time.strftime("%Y-%m-%d %H:%M:%S",
                                              time.gmtime(max(seconds, 0)))}
    if walk is not None and walk.alive():
        return 409, {"error": "a walk is recording -- stop it first. Changing "
                              "the clock mid-walk breaks its timestamps"}
    if recorder is not None:
        return 409, {"error": "a dashboard recording is running -- stop it first"}

    before = time.time()
    try:
        proc = subprocess.Popen(
            ["sudo", "-S", "-p", "", "date", "-s", "@%.3f" % seconds],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        out, _ = proc.communicate((SUDO_PASSWORD + "\n").encode("utf-8"),
                                  timeout=15)
    except (OSError, subprocess.TimeoutExpired) as e:
        return 500, {"error": "could not run sudo date: %s" % e}
    text = out.decode("utf-8", "replace").strip()
    # Judge by the clock, not the exit code: that is what the walk depends on.
    if proc.returncode != 0 or abs(time.time() - seconds) > 5.0:
        return 500, {"error": "the clock was not set (%s)" % (text or
                              "exit %d" % proc.returncode)}
    return 200, {"clock": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
                 "moved_by": round(time.time() - before, 1),
                 "preflight": preflight()}


# Members this big get zip64 headers from the first byte. The decision has to
# be made before any data is written and cannot be revisited on a stream that
# cannot seek, so it is taken with plenty of room below the 2 GB limit.
ZIP64_FROM = 1 << 30


def _zip_info(arcname, st):
    """A ZipInfo for a file, built by hand rather than with from_file.

    A walk recorded before the clock was set has 1970 mtimes, and ZIP cannot
    store a date before 1980 -- from_file raises on it, the same error that
    breaks pip on this robot. Clamp rather than refuse: the walk's own
    session.json says whether the clock was right, not the zip's dates.
    """
    t = time.localtime(st.st_mtime)[:6]
    if t[0] < 1980:
        t = (1980, 1, 1, 0, 0, 0)
    zi = zipfile.ZipInfo(arcname, date_time=t)
    zi.file_size = st.st_size
    zi.compress_type = zipfile.ZIP_STORED
    # Root wrote these, sometimes 0600. Whoever unzips them should be able to
    # read them, so do not carry the robot's permissions across.
    zi.external_attr = (0o100644) << 16
    return zi


def _zip_add(zf, full, arcname, notes):
    """Copy one file into the zip. Unreadable ones go through `sudo cat`.

    The recorder runs as root. Under root's usual umask its files come out
    0644 and this process can read them directly, which is the fast path.
    Anything it made 0600 -- cyclonedds.log is written by a library, not by
    us -- is read through sudo instead of being left out, since a walk with a
    file quietly missing is the kind of fault this project keeps finding late.
    """
    try:
        st = os.stat(full)
    except OSError as e:
        notes.append("%s: left out, could not be examined (%s)" % (arcname, e))
        return
    zi = _zip_info(arcname, st)
    big = st.st_size >= ZIP64_FROM

    try:
        src = open(full, "rb")
    except PermissionError:
        src = None
    except OSError as e:
        notes.append("%s: left out, could not be opened (%s)" % (arcname, e))
        return

    if src is not None:
        with src:
            with zf.open(zi, "w", force_zip64=big) as dst:
                shutil.copyfileobj(src, dst, 1 << 20)
        return

    proc = subprocess.Popen(["sudo", "-S", "-p", "", "cat", "--", full],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    try:
        try:
            proc.stdin.write((SUDO_PASSWORD + "\n").encode("utf-8"))
            proc.stdin.close()
        except (OSError, ValueError):
            pass
        # Read the first chunk before creating the member, so a sudo that
        # fails leaves nothing behind rather than an empty file that looks
        # like a real, empty recording.
        first = proc.stdout.read(1 << 20)
        if not first:
            err = " ".join(proc.stderr.read().decode("utf-8", "replace").split())
            if proc.wait() != 0:
                notes.append("%s: left out, not readable even through sudo (%s)"
                             % (arcname, err or "exit %d" % proc.returncode))
                return
        with zf.open(zi, "w", force_zip64=big) as dst:
            dst.write(first)
            shutil.copyfileobj(proc.stdout, dst, 1 << 20)
        proc.wait()
        notes.append("%s: not readable by the dashboard's user, read through "
                     "sudo" % arcname)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def write_walk_zip(folder, name, out):
    """Write the walk folder to `out` as a zip whose top folder is `name`.

    ZIP_STORED, no compression: a pcap and PNG depth frames barely shrink, and
    deflating 160 MB a minute would keep one of the Jetson's four weak cores
    busy for minutes to save almost nothing. `out` is the HTTP socket, which
    cannot seek; zipfile copes (3.5 onwards) by writing each member's sizes
    after its data rather than going back to fill them in.
    """
    notes = []
    with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_STORED,
                         allowZip64=True) as zf:
        names = set()
        for root, dirs, files in os.walk(folder):
            dirs.sort()
            for f in sorted(files):
                full = os.path.join(root, f)
                rel = os.path.relpath(full, folder).replace(os.sep, "/")
                if os.path.islink(full):
                    # The recorder never makes links; one here was put there
                    # by hand and may point anywhere on the robot.
                    notes.append("%s: left out, it is a symbolic link" % rel)
                    continue
                names.add(rel)
                _zip_add(zf, full, name + "/" + rel, notes)

        # The XT16's angle calibration is shared by every walk, not part of
        # one, but a walk replayed without it comes out flat. If the robot has
        # a copy, send it along; walk_to_bag.sh can be pointed at it with
        # GO2_LIDAR_CORRECTION.
        cal = calibration_path()
        if cal and os.path.isfile(cal) and "xt16_correction.csv" not in names:
            _zip_add(zf, cal, name + "/xt16_correction.csv", notes)
            notes.append("xt16_correction.csv: the robot's shared copy of the "
                         "LiDAR angle calibration (%s), added for convenience"
                         % cal)

        if notes:
            zf.writestr(
                _readme_info(name + "/README-download.txt"),
                "Downloaded from the robot's dashboard, %s.\n\n%s\n"
                % (time.strftime("%Y-%m-%d %H:%M:%S"),
                   "\n".join("- " + n for n in notes)))


def _readme_info(arcname):
    t = time.localtime()[:6]
    if t[0] < 1980:
        t = (1980, 1, 1, 0, 0, 0)
    zi = zipfile.ZipInfo(arcname, date_time=t)
    zi.compress_type = zipfile.ZIP_STORED
    zi.external_attr = (0o100644) << 16
    return zi


def walk_for_download(name):
    """(folder, None) for a walk that may be downloaded, else (None, (status, why))."""
    if not name or not SAFE_NAME.match(name):
        return None, (400, "bad name")
    # Only names the Record panel itself lists. That, and SAFE_NAME, is what
    # keeps "../.." and friends out: nothing is joined onto a path until the
    # name has been matched against a directory listing.
    if name not in [w["name"] for w in list_walks()]:
        return None, (404, "no such walk")
    if walk is not None and walk.alive() and walk.name == name:
        return None, (409, "that walk is being recorded right now -- stop it "
                           "first")
    return os.path.join(SESSIONS_DIR, name), None


# ---------------------------------------------------------------------------
# Recording control, driven from the page.
#
# The page can start and stop a recording and pick one to play back, so you
# never have to restart the server or touch a terminal to do either. The file
# name is the only thing an outside caller supplies, so it is validated
# strictly: a plain name, no directory separators, resolved inside the
# recordings folder and nowhere else.
# ---------------------------------------------------------------------------

SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def recording_path(name):
    """Full path for a recording name, or None if the name is not acceptable."""
    if not name:
        return None
    if not name.endswith(".jsonl"):
        name = name + ".jsonl"
    stem = name[:-len(".jsonl")]
    if not SAFE_NAME.match(stem):
        return None
    full = os.path.normpath(os.path.join(RECORDINGS_DIR, name))
    # normpath alone is not enough -- confirm it really landed inside.
    if os.path.dirname(full) != os.path.normpath(RECORDINGS_DIR):
        return None
    return full


_scan_cache = {}


def scan_recording(path):
    """Frame count and duration of a recording, cached by size and mtime."""
    try:
        st = os.stat(path)
    except OSError:
        return None
    key = (path, st.st_size, int(st.st_mtime))
    if key in _scan_cache:
        return _scan_cache[key]

    frames, last_t = 0, 0.0
    try:
        with open(path) as fh:
            for line in fh:
                if not line.strip():
                    continue
                frames += 1
                # Only the last line's timestamp matters; parsing just the tail
                # of each line would be fiddlier than parsing all of them, and
                # these files are about a megabyte a minute.
                try:
                    last_t = json.loads(line).get("t", last_t)
                except ValueError:
                    pass
    except OSError:
        return None

    info = {"name": os.path.basename(path), "bytes": st.st_size,
            "modified": int(st.st_mtime), "frames": frames,
            "duration": round(last_t, 1)}
    _scan_cache.clear()               # only ever a handful of files; keep it small
    _scan_cache[key] = info
    return info


def list_recordings():
    out = []
    try:
        names = sorted(os.listdir(RECORDINGS_DIR))
    except OSError:
        return out
    for n in names:
        if not n.endswith(".jsonl"):
            continue
        info = scan_recording(os.path.join(RECORDINGS_DIR, n))
        if info:
            out.append(info)
    out.sort(key=lambda r: r["modified"], reverse=True)
    return out


def do_library(action, params):
    """Process, cancel or delete a walk in the library."""
    global job

    state = library_state()
    if not state.get("available"):
        return 409, {"error": state.get("why", "the walk library is unavailable")}

    if action == "library-cancel":
        if job is None or not job.alive():
            return 409, {"error": "nothing is running"}
        if not job.cancel():
            return 500, {"error": "could not stop the conversion"}
        return 200, library_state()

    name = params.get("name")
    if not name or not SAFE_NAME.match(name):
        return 400, {"error": "bad name"}
    target = os.path.normpath(os.path.join(WALKS_DIR, name))
    if os.path.dirname(target) != os.path.normpath(WALKS_DIR) \
            or not os.path.isdir(target):
        return 404, {"error": "no such walk"}

    if action == "library-forget":
        if job is not None and job.alive() and job.name == name:
            return 409, {"error": "that walk is being processed right now"}
        try:
            shutil.rmtree(target)
        except OSError as e:
            return 500, {"error": "could not delete %s: %s" % (name, e)}
        return 200, library_state()

    if action == "library-map":
        # Rebuild only the scan-matched map from the existing bag.
        if job is not None and job.alive():
            return 409, {"error": "already running %s" % job.name}
        bag = os.path.join(target, "bag")
        if not walk_library.bag_state(bag)["mcap"]:
            return 409, {"error": "%s has no bag yet -- process it first" % name}
        tool = os.path.join(TOOLS_DIR, "process_walk.sh")
        if not os.path.isfile(tool):
            return 409, {"error": "process_walk.sh is not next to dashboard.py"}
        try:
            job = Job("map", name, ["bash", tool, target, "--map-only"])
        except OSError as e:
            return 500, {"error": "could not start process_walk.sh: %s" % e}
        return 200, library_state()

    # library-process
    if job is not None and job.alive():
        return 409, {"error": "already processing %s -- walk_to_bag.sh must not "
                              "run twice at once, it corrupts both bags"
                              % job.name}
    # process_walk.sh: the bag, then the scan-matched map, in one run.
    tool = state["tool"]
    if not os.path.isfile(tool):
        return 409, {"error": "%s is not next to dashboard.py" % os.path.basename(tool)}
    try:
        job = Job("process", name, ["bash", tool, target])
    except OSError as e:
        return 500, {"error": "could not start %s: %s" % (os.path.basename(tool), e)}
    return 200, library_state()


def control_state():
    """What the page's recording panel needs to draw itself."""
    return {
        "mode": "replay" if replayer is not None else "live",
        "can_go_live": dds_ready,
        "dir": RECORDINGS_DIR,
        "recording": ({"file": os.path.basename(recorder.path),
                       "frames": recorder.frames,
                       "seconds": round(time.time() - recorder.t0, 1)}
                      if recorder is not None else None),
        "replaying": ({"file": os.path.basename(replayer.path),
                       "frames": len(replayer.frames),
                       "duration": round(replayer.duration, 1)}
                      if replayer is not None else None),
        "files": list_recordings(),
        "walk": {
            "recording": walk.status() if walk is not None else None,
            "last_verdict": last_verdict,
            "available": RECORDER is not None and os.path.isfile(RECORDER),
            "recorder": RECORDER,
            "dir": SESSIONS_DIR,
            "walks": list_walks(),
        },
    }


def do_control(action, params):
    """Handle one control request. Returns (http_status, response dict)."""
    global recorder, replayer

    if action == "record-start":
        if recorder is not None:
            return 409, {"error": "already recording"}
        if replayer is not None:
            return 409, {"error": "cannot record while replaying -- go live first"}
        if not dds_ready:
            return 409, {"error": "this server was started in replay mode and is "
                                  "not listening to a robot"}
        name = params.get("name") or time.strftime("go2-%Y%m%d-%H%M%S")
        path = recording_path(name)
        if path is None:
            return 400, {"error": "bad recording name: letters, digits, dot, dash "
                                  "and underscore only"}
        if os.path.exists(path):
            return 409, {"error": "%s already exists" % os.path.basename(path)}
        try:
            recorder = Recorder(path, float(params.get("hz") or 10.0))
        except (OSError, ValueError) as e:
            return 500, {"error": str(e)}
        return 200, control_state()

    if action == "record-stop":
        if recorder is None:
            return 409, {"error": "not recording"}
        recorder.stop()
        recorder = None
        return 200, control_state()

    if action == "replay":
        path = recording_path(params.get("file"))
        if path is None or not os.path.isfile(path):
            return 404, {"error": "no such recording"}
        if recorder is not None:
            # Recording captures live data; replaying replaces live data. Doing
            # both would quietly record the replay.
            return 409, {"error": "stop the recording first"}
        try:
            replayer = Replayer(path)
        except (OSError, ValueError) as e:
            return 400, {"error": str(e)}
        return 200, control_state()

    if action == "delete":
        path = recording_path(params.get("file"))
        if path is None or not os.path.isfile(path):
            return 404, {"error": "no such recording"}
        # Deleting the file being written, or the one being played, would leave
        # the server holding a handle to something that no longer exists.
        if recorder is not None and os.path.abspath(recorder.path) == os.path.abspath(path):
            return 409, {"error": "that recording is being written right now -- "
                                  "stop it first"}
        if replayer is not None and os.path.abspath(replayer.path) == os.path.abspath(path):
            return 409, {"error": "that recording is playing -- go live first"}
        try:
            os.remove(path)
        except OSError as e:
            return 500, {"error": str(e)}
        _scan_cache.clear()
        return 200, control_state()

    if action == "walk-start":
        global walk
        if walk is not None and walk.alive():
            return 409, {"error": "already recording a walk"}
        if RECORDER is None or not os.path.isfile(RECORDER):
            return 409, {"error": "record_session.py not found at %s -- copy it "
                                  "to the robot next to dashboard.py" % RECORDER}
        # Pre-flight. Both walks recorded so far were spoiled by something
        # that was already true before the button was pressed, and neither was
        # noticed until the file was on another machine. Refuse, say which
        # check failed and why it matters, and let it be overridden -- "the
        # sensor is unplugged" and "you know something I do not" are both
        # possible, and only one of them is common.
        pf = preflight()
        if not pf["ready"] and not params.get("force"):
            return 409, {"error": "not ready: " + "; ".join(pf["blockers"]),
                         "preflight": pf}

        name = params.get("name") or time.strftime("walk-%Y%m%d-%H%M%S")
        if not SAFE_NAME.match(name):
            return 400, {"error": "bad name: letters, digits, dot, dash and "
                                  "underscore only"}
        if os.path.exists(os.path.join(SESSIONS_DIR, name)):
            return 409, {"error": "%s already exists" % name}
        try:
            duration = float(params.get("duration") or 0)
        except (TypeError, ValueError):
            duration = 0
        try:
            walk = WalkRecorder(name, SESSIONS_DIR, RECORDER, SUDO_PASSWORD,
                                duration, DDS_IFACE, DDS_DOMAIN)
        except OSError as e:
            return 500, {"error": str(e)}

        # Give it a moment to fail. Wrong password, no root, a missing sensor:
        # all of those end the process almost immediately, and it is far better
        # to say so now than to show a recording that was never running.
        time.sleep(2.0)
        problem = walk.failure()
        if problem:
            # The recorder creates its session folder before it can fail, so a
            # failed start would otherwise leave an empty folder behind and the
            # name would be taken next time.
            _remove_walk_dir(os.path.join(SESSIONS_DIR, name))
            walk = None
            return 500, {"error": problem}
        return 200, control_state()

    if action == "walk-stop":
        if walk is None or not walk.alive():
            return 409, {"error": "no walk is being recorded"}
        session_dir = walk.dir
        clean = walk.stop()
        walk = None
        out = control_state()

        # Say straight away whether the walk is any good. These are the same
        # checks inspect_bag.py makes, but they read the recording rather than
        # a bag, so they need neither ROS nor the laptop and can answer in the
        # second after the button is pressed -- which is while you are still
        # standing next to the robot and can do something about it.
        global last_verdict
        if walk_library is not None:
            try:
                last_verdict = walk_library.describe(session_dir)
            except OSError:
                last_verdict = None
            out["verdict"] = last_verdict
        if not clean:
            out["warning"] = ("the recorder did not exit when asked and was "
                              "killed; the recording is probably still usable")
        return 200, out

    if action == "walk-delete":
        name = params.get("name")
        if not name or not SAFE_NAME.match(name):
            return 400, {"error": "bad name"}
        target = os.path.normpath(os.path.join(SESSIONS_DIR, name))
        if os.path.dirname(target) != os.path.normpath(SESSIONS_DIR) \
                or not os.path.isdir(target):
            return 404, {"error": "no such walk"}
        if walk is not None and walk.alive() and walk.name == name:
            return 409, {"error": "that walk is being recorded right now"}
        # The recorder runs as root, so the files it wrote are root-owned and
        # this process cannot remove them itself. Delete through sudo, the same
        # way the recording was started.
        if not _remove_walk_dir(target):
            return 500, {"error": "could not delete %s" % name}
        return 200, control_state()

    if action == "sport":
        return do_sport(params.get("move"))

    if action == "v2-layout":
        # Dashboard v2's card layout, kept on the robot so every device that
        # opens the page sees the same one. Stored beside the other settings.
        layout = params.get("layout")
        if not isinstance(layout, dict) or not isinstance(layout.get("items"), list) \
                or len(json.dumps(layout)) > 20000:
            return 400, {"error": "layout must be {name, items: [...]}"}
        values = read_settings()
        values["v2_layout"] = layout
        try:
            write_settings(values)
        except (OSError, ValueError) as e:
            return 500, {"error": "could not save the layout: %s" % e}
        return 200, {"saved": True}

    if action in ("set-clock", "set_clock"):
        # The browser's clock, for a robot that boots at 1970. The pre-flight
        # strip offers it when the clock chip is red. Dashed like every other
        # action; the underscore spelling is accepted for curl users.
        return set_clock(params.get("epoch_ms"))

    if action == "settings-save":
        values = params.get("settings")
        try:
            write_settings(values)
        except (OSError, ValueError) as e:
            return 500, {"error": "could not save settings: %s" % e}
        return 200, {"settings": read_settings(), "path": SETTINGS_PATH,
                     "saved": True}

    # --- the walk library ------------------------------------------------
    if action in ("library-process", "library-cancel", "library-forget",
                  "library-map"):
        return do_library(action, params)

    # --- the sensors panel -----------------------------------------------
    if action and action.startswith("sensors-"):
        if sensors is None:
            return 409, {"error": "sensors.py is not next to dashboard.py"}
        return sensors.control(action, params)

    if action == "live":
        if not dds_ready:
            return 409, {"error": "this server was started with --replay, so it "
                                  "never subscribed to a robot. Restart it "
                                  "without --replay to go live."}
        replayer = None
        return 200, control_state()

    return 400, {"error": "unknown action"}


def current():
    """What /data should return right now -- live, or a frame from a recording."""
    if replayer is not None:
        return replayer.current()
    return snapshot()


# ---------------------------------------------------------------------------
# SPORT MOVES
#
# The moves the Unitree app offers, sent the way the app sends them: through
# the sport service's request topic, via unitree_sdk2py's SportClient. Only a
# fixed list, one at a time. Damp is deliberately absent -- it drops the robot
# to the floor, LiDAR first -- and so are the flips and jumps. StopMove is the
# "stop" button: the robot stops walking and holds still.
# ---------------------------------------------------------------------------
SPORT_MOVES = ("StandUp", "StandDown", "BalanceStand", "Sit", "RiseSit",
               "RecoveryStand", "Hello", "Stretch", "Content", "Heart",
               "Dance1", "Dance2", "Scrape", "StopMove")
_sport = None
_sport_lock = threading.Lock()
# unitree_sdk2 RPC status codes worth saying in words. Only ones seen: 3102
# is what a request with no sport service behind it returns (2026-09-25).
SPORT_CODES = {3102: "the robot's motion service did not answer (is it on, and in "
                     "normal sport mode rather than another app's control?)"}


def do_sport(move):
    global _sport
    if move not in SPORT_MOVES:
        return 400, {"error": "not an allowed move: %s" % move}
    if not dds_ready:
        return 409, {"error": "not connected to the robot (this server is replaying)"}
    if move != "StopMove" and walk is not None and walk.alive():
        return 409, {"error": "a walk is recording -- moves are paused until it stops"}
    # One at a time: a second request waits for the first rather than
    # interleaving two moves on the robot.
    if not _sport_lock.acquire(timeout=12):
        return 409, {"error": "another move is still being sent"}
    try:
        if _sport is None:
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            client = SportClient()
            client.SetTimeout(5.0)
            client.Init()
            _sport = client
        fn = getattr(_sport, move, None)
        if fn is None:
            return 501, {"error": "this robot's SDK has no %s" % move}
        t0 = time.time()
        code = fn()
    except Exception as e:                      # noqa: BLE001 -- report, do not crash
        print("sport %s failed: %r" % (move, e), flush=True)
        return 500, {"error": "could not send %s: %s" % (move, e)}
    finally:
        _sport_lock.release()
    print("sport %s -> code %s in %.1f s" % (move, code, time.time() - t0), flush=True)
    if code != 0:
        why = SPORT_CODES.get(code, "code %s" % code)
        return 502, {"error": "%s was not done: %s" % (move, why),
                     "move": move, "code": code}
    return 200, {"move": move, "code": code}


# ---------------------------------------------------------------------------
# SYSTEM
#
# What dashboard v2's System page shows about this computer. All of it is
# read from /proc and /sys, so it costs nothing and needs no packages.
# ---------------------------------------------------------------------------
PROGRAMS = (("dashboard", "dashboard.py"), ("lidar_relay", "lidar_rela[y].py"),
            ("camera_server", "camera_serve[r].py"),
            ("realsense_camera", "realsense2_camer[a]"),
            ("recorder", "record_sessio[n].py"))


def _read(path, default=None):
    try:
        with io.open(path, encoding="utf-8", errors="replace") as fh:
            return fh.read().strip()
    except OSError:
        return default


def system_info():
    out = {"time": time.time(), "uptime": None, "load": None, "cpus": os.cpu_count(),
           "mem_total": None, "mem_available": None, "temp_c": None,
           "disk_free": None, "disk_total": None, "model": None,
           "version": None, "autostart": None, "programs": {},
           "addresses": local_addresses()}
    up = _read("/proc/uptime")
    if up:
        out["uptime"] = float(up.split()[0])
    try:
        out["load"] = list(os.getloadavg())
    except OSError:
        pass
    mem = {}
    for line in (_read("/proc/meminfo", "") or "").splitlines():
        k, _, v = line.partition(":")
        if v.strip().endswith("kB"):
            mem[k] = int(v.split()[0]) * 1024
    out["mem_total"], out["mem_available"] = mem.get("MemTotal"), mem.get("MemAvailable")
    temps = []
    for i in range(12):
        t = _read("/sys/class/thermal/thermal_zone%d/temp" % i)
        if t and t.lstrip("-").isdigit():
            temps.append(int(t) / 1000.0)
    out["temp_c"] = round(max(temps), 1) if temps else None
    try:
        u = shutil.disk_usage(SESSIONS_DIR if os.path.isdir(SESSIONS_DIR) else "/")
        out["disk_free"], out["disk_total"] = u.free, u.total
    except OSError:
        pass
    out["model"] = (_read("/proc/device-tree/model") or "").replace("\x00", "") or None
    here = os.path.dirname(os.path.abspath(__file__))
    out["version"] = _read(os.path.join(here, "VERSION")) or time.strftime(
        "%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(os.path.abspath(__file__))))
    try:
        r = subprocess.run(["systemctl", "is-enabled", "go2dev"], stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, timeout=2)
        out["autostart"] = r.stdout.decode().strip() or None
    except (OSError, subprocess.SubprocessError):
        pass
    for name, pattern in PROGRAMS:
        try:
            r = subprocess.run(["pgrep", "-f", pattern], stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, timeout=2)
            out["programs"][name] = bool(r.stdout.strip())
        except (OSError, subprocess.SubprocessError):
            out["programs"][name] = None
    return out


def health():
    """A small JSON summary for scripts and for go2-start.sh to poll."""
    low_msg, low_topic, low_hz, low_age = low.read()
    sport_msg, sport_topic, sport_hz, sport_age = sport.read()
    ok = low_msg is not None and low_age is not None and low_age <= STALE_AFTER
    if replayer is not None:
        ok = True
    out = {
        "status": "ok" if ok else "no-data",
        "mode": "replay" if replayer is not None else "live",
        "uptime": round(time.time() - started, 1),
        "streams": {
            "lowstate": {"topic": low_topic, "hz": low_hz,
                         "count": low.count, "age": low_age},
            "sportmodestate": {"topic": sport_topic, "hz": sport_hz,
                               "count": sport.count, "age": sport_age},
        },
        "camera_host": CAMERA_HOST,
        "subscribed": {"lowstate": LOW_TOPICS, "sportmodestate": SPORT_TOPICS},
        # Which addresses this server answers to. The page compares the one you
        # typed against this list to say whether you came in over the cable or
        # over Wi-Fi.
        "host": {"hostname": socket.gethostname(),
                 "addresses": local_addresses()},
    }
    if replayer is not None:
        out["replay"] = {"file": os.path.basename(replayer.path),
                         "frames": len(replayer.frames),
                         "duration": round(replayer.duration, 1)}
    if recorder is not None:
        out["recording"] = {"file": os.path.basename(recorder.path),
                            "frames": recorder.frames}
    return out


# ---------------------------------------------------------------------------
# Settings that survive a new browser
#
# The drawer's settings live in localStorage, which is per-browser and per-
# machine: open the dashboard on a phone and you get the defaults again, and
# clearing site data loses the lot. So the server keeps a copy on disk, and a
# browser that has never saved anything of its own starts from it.
#
# The precedence is deliberately one-way and boring:
#
#     built-in defaults  <-  settings.json on the server  <-  this browser
#
# A browser never has its choices overwritten by the server. Pressing "Save as
# defaults" pushes this browser's settings up to the file, for everyone else.
# ---------------------------------------------------------------------------

# The LiDAR's position on the robot, in metres from base_link. Unitree publish
# this in the SDK docs' SLAM and Navigation Services Interface page; it is not
# something to measure with a ruler unless the sensor has been remounted.
#
# It was a guess (x=0, z=0.30) until 2026-09-05. It is kept here so that the
# .ply exporter, the live cloud view and go2-start.sh all read one number
# rather than each carrying their own copy of it.
DEFAULT_SETTINGS = {
    # Where the sensor is on the network, and where its per-laser angle
    # calibration was saved. Both are read by the pre-flight check rather than
    # hardcoded, so a remounted or re-addressed sensor is one edit.
    "lidar": {"host": "192.168.123.20", "port": 9347, "correction_file": None},
    "lidar_offset": {"x": 0.171, "y": 0.0, "z": 0.0908, "yaw": 0.0,
                     "source": "Unitree SDK docs, SLAM and Navigation Services "
                               "Interface. Change only if the sensor is moved."},
}


def read_settings():
    """Whatever is on disk, on top of the built-in defaults."""
    out = json.loads(json.dumps(DEFAULT_SETTINGS))    # a deep copy
    try:
        with io.open(SETTINGS_PATH, encoding="utf-8") as fh:
            saved = json.load(fh)
    except (OSError, ValueError):
        return out
    if isinstance(saved, dict):
        for k, v in saved.items():
            if isinstance(v, dict) and isinstance(out.get(k), dict):
                out[k].update(v)
            else:
                out[k] = v
    return out


def write_settings(values):
    """Replace the saved settings. Written whole, then moved into place.

    Written to a temporary file and renamed rather than opened for writing,
    because a half-written settings.json is indistinguishable from a corrupt
    one and the page would silently fall back to defaults.
    """
    if not isinstance(values, dict):
        raise ValueError("settings must be an object")
    tmp = SETTINGS_PATH + ".tmp"
    with io.open(tmp, "w", encoding="utf-8") as fh:
        fh.write(json.dumps(values, indent=2, sort_keys=True))
    os.replace(tmp, SETTINGS_PATH)


# ---------------------------------------------------------------------------
# Pre-flight, and what the streams should look like
#
# Every fault this project has lost a walk to was silent at the time and only
# visible afterwards, in a file, on another machine. The clock was at 1970. The
# robot was lying down so nothing published odometry. Each packet was recorded
# twice. None of it looked wrong while it was happening.
#
# So the checks that used to run after a conversion run before the recording
# instead, and the answers are in words rather than numbers.
# ---------------------------------------------------------------------------

# What each topic should be delivering. The "lf" variants are the same data
# sent more slowly -- low frequency, not a second sensor -- so they get their
# own expectation rather than being judged against the fast one and reported as
# broken.
EXPECTED_HZ = {
    "rt/lowstate": 500.0,
    "rt/lf/lowstate": 50.0,
    # Measured on the robot, 2026-09-17: about 320 Hz. Earlier notes said 50.
    "rt/sportmodestate": 300.0,
    "rt/lf/sportmodestate": 10.0,
}

# A walk is about 160 MB a minute. Below this there is not enough room for a
# useful one, and starting is refused rather than filling the disk mid-walk.
MIN_FREE_BYTES = 2 * 1024 ** 3
BYTES_PER_MINUTE = 160 * 1024 ** 2

# The robot standing up is the difference between a map and a smear: odometry
# comes from rt/sportmodestate, which the motion service only publishes while
# it is up. Judged from the trunk height as well as the topic, because a stale
# subscription can look alive for a moment after the robot sits down.
STANDING_HEIGHT = 0.15

# Cached, because the page asks for pre-flight every couple of seconds and a
# TCP connect to a sensor that is not there costs a full timeout each time.
_lidar_probe = {"at": 0.0, "ok": False, "detail": ""}
LIDAR_PROBE_TTL = 10.0


def lidar_reachable():
    """Is the XT16 answering on its control port?

    Opens a TCP connection and closes it. Nothing is sent -- not even a read
    command. The same protocol carries the command that sets the sensor's
    destination address, which stopped this XT16 transmitting until it was
    power-cycled (docs/hard-won-facts.md), so the rule here is the same as in
    get_lidar_calibration.py: this code does not write to the sensor.
    """
    now = time.time()
    # Both ends of the window: setting the clock from a browser can move it
    # backwards a little, and a probe "from the future" would never expire.
    if 0 <= now - _lidar_probe["at"] < LIDAR_PROBE_TTL:
        return _lidar_probe["ok"], _lidar_probe["detail"]

    settings = read_settings()
    lidar = settings.get("lidar") or {}
    host = lidar.get("host", "192.168.123.20")
    port = int(lidar.get("port", 9347))

    sock = None
    try:
        sock = socket.create_connection((host, port), timeout=1.5)
        ok, detail = True, "answering at %s:%d" % (host, port)
    except (socket.timeout, OSError) as e:
        ok = False
        detail = ("no answer from %s:%d (%s). Check it is powered and the "
                  "cable is in; nothing can be recorded without it."
                  % (host, port, e))
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    _lidar_probe.update({"at": now, "ok": ok, "detail": detail})
    return ok, detail


def disk_headroom(path):
    """Free bytes and how many minutes of walk that is."""
    try:
        st = os.statvfs(path)
    except (OSError, AttributeError):
        return None, None, None
    free = st.f_bavail * st.f_frsize
    total = st.f_blocks * st.f_frsize
    return free, total, free / float(BYTES_PER_MINUTE)


def _pf(ok, headline, detail, blocking=False):
    return {"ok": bool(ok), "headline": headline, "detail": detail,
            "blocking": bool(blocking)}


def preflight():
    """Is this machine ready to record a walk that will be worth having?

    `blocking` marks the checks that make a recording useless rather than
    merely worse. The page refuses to start on those, but offers to override,
    because "the sensor is unplugged" and "you know something I do not" are
    both possible and only one of them is common.
    """
    checks = []

    # 1. The clock. Fatal, and the only one that cannot be noticed later: the
    #    LiDAR and the IMU are lined up by timestamp and nothing else.
    now = time.time()
    synced = now > 1577836800                  # 2020-01-01
    checks.append(_pf(
        synced, "clock is set",
        time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)) if synced else
        "reads %s -- the robot has no battery-backed clock and boots at 1970. "
        "Press \"Set from this computer\" beside this chip, or run "
        "go2-start.sh from the laptop."
        % time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
        blocking=True))

    # 2. The robot is talking to us at all.
    low_msg, low_topic, low_hz, low_age = low.read()
    talking = low_msg is not None and low_age is not None and low_age <= STALE_AFTER
    checks.append(_pf(
        talking, "robot is sending state",
        "%s at %.0f Hz" % (low_topic, low_hz) if talking else
        "nothing on %s. The robot is off, the cable is out, or this dashboard "
        "is on the wrong network interface." % " or ".join(LOW_TOPICS),
        blocking=True))

    # 3. Standing. A walk has to be walked standing, and the check says which of
    #    two different things is wrong, because they need different fixes.
    #    Observed 2026-09-17: rt/sportmodestate keeps arriving, at about 320 Hz,
    #    while the robot is lying down, so its absence means the motion data is
    #    not reaching us at all, not that the robot is down. The height is what
    #    says the robot is down.
    sport_msg, sport_topic, sport_hz, sport_age = sport.read()
    sport_live = (sport_msg is not None and sport_age is not None
                  and sport_age <= STALE_AFTER)
    height = None
    if sport_live:
        try:
            height = float(sport_msg.body_height)
        except (AttributeError, TypeError, ValueError):
            height = None
    standing = sport_live and (height is None or height >= STANDING_HEIGHT)
    checks.append(_pf(
        standing, "robot is standing",
        "%s at %.0f Hz%s" % (sport_topic, sport_hz,
                             ", trunk %.2f m" % height if height else "")
        if standing else
        ("body height %.2f m, below %.2f m, so the robot is lying or sitting. "
         "Stand it up before recording a walk." % (height, STANDING_HEIGHT))
        if sport_live and height is not None else
        "no motion data arriving on rt/sportmodestate. Without it there is no "
        "odometry, every scan is drawn at the same spot, and the walk comes out "
        "as a smear. Check Streams, and that go2-start.sh ran.",
        blocking=True))

    # 4. The sensor.
    lidar_ok, lidar_detail = lidar_reachable()
    checks.append(_pf(lidar_ok, "lidar is reachable", lidar_detail,
                      blocking=True))

    # 5. The angle calibration. Not needed to record -- needed to replay, on
    #    whichever machine does the conversion. Without it the finished cloud
    #    is one flat layer, which looks entirely plausible from above.
    cal = calibration_path()
    have_cal = bool(cal and os.path.isfile(cal))
    checks.append(_pf(
        have_cal, "angle calibration saved",
        cal if have_cal else
        "not saved. Each of the XT16's sixteen lasers sits at a different "
        "vertical angle, measured per unit at the factory; live, the driver "
        "asks the sensor for them, and a replay has nothing to ask. Without "
        "it the replayed cloud is completely flat. Run "
        "get_lidar_calibration.py once, while the sensor is connected."))

    # 6. Room to write.
    free, total, minutes = disk_headroom(SESSIONS_DIR)
    if free is None:
        checks.append(_pf(True, "disk space", "could not be measured"))
    else:
        checks.append(_pf(
            free >= MIN_FREE_BYTES, "room to record",
            "%.1f GB free -- about %.0f minutes of walk"
            % (free / 1024.0 ** 3, minutes) if free >= MIN_FREE_BYTES else
            "only %.1f GB free, about %.0f minutes. A walk is roughly 160 MB "
            "a minute." % (free / 1024.0 ** 3, minutes),
            blocking=free < MIN_FREE_BYTES))

    # 7. The recorder itself.
    have_recorder = RECORDER is not None and os.path.isfile(RECORDER)
    checks.append(_pf(
        have_recorder, "recorder is installed",
        RECORDER if have_recorder else
        "record_session.py is not next to dashboard.py. Run "
        "deploy_dashboard.sh.", blocking=True))

    blocking = [c for c in checks if c["blocking"] and not c["ok"]]
    return {
        "checks": checks,
        "passed": sum(1 for c in checks if c["ok"]),
        "total": len(checks),
        "ready": not blocking,
        "blockers": [c["headline"] for c in blocking],
    }


def calibration_path():
    """Where the XT16's angle calibration is kept, if anywhere.

    Two places, because two machines care. The laptop needs it at conversion
    time and keeps it beside the walks; the robot has no use for it but is
    where you notice it is missing, so it is looked for there too.
    """
    settings = read_settings()
    explicit = (settings.get("lidar") or {}).get("correction_file")
    candidates = [explicit] if explicit else []
    if WALKS_DIR:
        candidates.append(os.path.join(WALKS_DIR, "xt16_correction.csv"))
    candidates.append(os.path.join(TOOLS_DIR, "xt16_correction.csv"))
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    return candidates[0] if candidates else None


def stream_report():
    """Each stream with what it should be doing, for the Streams panel.

    A rate on its own says nothing: 50 Hz is healthy on rt/lf/lowstate and a
    fault on rt/lowstate. Pair every number with what was expected of it.
    """
    out = []
    for label, stream, topics in (("robot state", low, LOW_TOPICS),
                                  ("motion state", sport, SPORT_TOPICS)):
        msg, topic, hz, age = stream.read()
        expected = EXPECTED_HZ.get(topic or "", 0.0)
        if msg is None:
            note = "silent -- nothing has arrived on %s" % " or ".join(topics)
            ok = False
        elif age is not None and age > STALE_AFTER:
            note = "stopped %.1f s ago" % age
            ok = False
        elif expected and hz < expected * 0.6:
            note = "%.0f Hz, well under the %.0f Hz expected on %s" % (
                hz, expected, topic)
            ok = False
        else:
            note = "%.0f Hz on %s" % (hz, topic) + (
                " (expected about %.0f)" % expected if expected else "")
            ok = True
        out.append({"label": label, "topic": topic, "hz": round(hz, 1),
                    "expected": expected, "age": age, "ok": ok, "note": note})
    return out


# ---------------------------------------------------------------------------
# The walk library: recordings that have been brought back to a machine with
# the ROS tools on it, and the job that turns one into a bag.
#
# This is deliberately separate from the "Record a walk" panel above. That one
# runs on the robot and starts record_session.py. This one runs wherever the
# Hesai driver lives -- the laptop, inside WSL -- and drives walk_to_bag.sh.
# Both panels can be on screen at once; on the Jetson only the first is, because
# --walks is not given there and processing is reported as unavailable.
# ---------------------------------------------------------------------------
ANSI = re.compile(r"\x1b\[[0-9;]*m")
STAGE_LINE = re.compile(r"^([1-9])\.\s+(.+?)\s*$")
REPLAY_PROGRESS = re.compile(r"(\d+)s \(replay running, expected about (\d+)s\)")
# walk_to_bag.sh since 2026-09-17: "  12s  frame 911 of about 3150"
DECODE_PROGRESS = re.compile(r"frame (\d+) of about (\d+)")
# lidar_map.py: "  3149 scans to align", then "  200 scans, 189 matched (94%), 15 s"
MAP_TOTAL = re.compile(r"^\s*(\d+) scans to align")
MAP_PROGRESS = re.compile(r"^\s*(\d+) scans, (\d+) matched")
# dense_map.py: "  500 scans placed, 12 s"
DENSE_PROGRESS = re.compile(r"^\s*(\d+) scans placed")
# export_ply.py prints "  400 frames, 1,234,567 points so far" as it goes.
EXPORT_PROGRESS = re.compile(r"^\s*(\d+) frames, [\d,]+ points so far")

# Where each stage of process_walk.sh sits on the progress bar, from timings of
# odom-test-1 (315 s walk): copy and decode about a minute, writing the bag
# about a minute, the scan-matched map about six. Equal widths would make the
# bar a lie. For a --map-only rebuild, stage 7 takes the whole bar.
STAGE_SPAN = {1: (0, 2), 2: (2, 6), 3: (6, 7), 4: (7, 20), 5: (20, 32),
              6: (32, 34), 7: (34, 88), 8: (88, 100)}


class Job(object):
    """One long-running command, with its progress readable while it runs.

    Only one exists at a time. walk_to_bag.sh refuses to run twice concurrently
    anyway -- two conversions corrupt each other's output -- so a second job is
    rejected here rather than left to fail confusingly later.
    """

    def __init__(self, kind, name, argv, total=None):
        self.kind = kind                  # "process" or "map"
        self.name = name                  # which walk
        self.argv = argv
        # For the exporter, how many frames it will get through. Taken from the
        # bag's own metadata rather than guessed, and simply absent when it
        # cannot be read -- a bar that lies is worse than no bar.
        self.total = total or 0
        self.t0 = time.time()
        self.finished_at = None
        self.returncode = None
        self.cancelled = False
        self.stage = 0
        self.stage_text = "starting"
        self.percent = 0.0
        self.lines = collections.deque(maxlen=300)
        self.map_total = 0
        self.lock = threading.Lock()

        # start_new_session puts the script in its own process group, so
        # cancelling can signal the whole tree. walk_to_bag.sh starts a driver,
        # two publishers and a recorder; killing only the shell would leave all
        # of them running and the next conversion would then be fighting them.
        self.proc = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, bufsize=0, start_new_session=True)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        """Read output as it appears and turn it into a stage and a percentage.

        Read in raw chunks rather than by line, because the replay progress is
        printed with a carriage return and no newline -- readline() would block
        on it for the whole two minutes, which is exactly the stretch the bar
        is meant to be moving.
        """
        fd = self.proc.stdout.fileno()
        buf = b""
        while True:
            try:
                chunk = os.read(fd, 4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            buf = buf.replace(b"\r", b"\n")
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._line(raw.decode("utf-8", "replace"))
        if buf:
            self._line(buf.decode("utf-8", "replace"))

        self.proc.wait()
        with self.lock:
            self.returncode = self.proc.returncode
            self.finished_at = time.time()
            if self.returncode == 0:
                self.percent = 100.0
                self.stage_text = "done"
            elif self.cancelled:
                self.stage_text = "cancelled"
            else:
                self.stage_text = "failed"

    def _line(self, text):
        clean = ANSI.sub("", text).rstrip()
        if not clean.strip():
            return
        with self.lock:
            self.lines.append(clean)

            if self.kind == "export":
                return self._export_line(clean)

            m = STAGE_LINE.match(clean.strip())
            if m:
                n = int(m.group(1))
                self.stage = n
                self.stage_text = m.group(2)
                if n in STAGE_SPAN:
                    self.percent = float(self._span(n)[0])
                return

            m = DECODE_PROGRESS.search(clean)
            if m:
                done, total = float(m.group(1)), max(float(m.group(2)), 1.0)
                lo, hi = STAGE_SPAN[4]
                self.percent = min(hi, lo + (hi - lo) * min(done / total, 1.0))
                self.stage_text = "decoding the LiDAR -- frame %d of about %d" % (
                    int(done), int(total))
                return

            m = DENSE_PROGRESS.match(clean)
            if m and self.map_total:
                lo, hi = self._span(8)
                self.percent = min(hi, lo + (hi - lo) * min(float(m.group(1)) / self.map_total, 1.0))
                self.stage_text = "dense point cloud -- %s of %d scans" % (m.group(1), self.map_total)
                return

            m = MAP_TOTAL.match(clean)
            if m:
                self.map_total = int(m.group(1))
                return
            m = MAP_PROGRESS.match(clean)
            if m:
                lo, hi = self._span(7)
                done = float(m.group(1))
                if self.map_total:
                    self.percent = min(hi, lo + (hi - lo) * min(done / self.map_total, 1.0))
                    self.stage_text = "building the map -- %d of %d scans" % (
                        int(done), self.map_total)
                else:
                    self.stage_text = "building the map -- %d scans" % int(done)
                return

            m = REPLAY_PROGRESS.search(clean)
            if m:
                done, total = float(m.group(1)), max(float(m.group(2)), 1.0)
                lo, hi = STAGE_SPAN[4]
                self.percent = min(hi, lo + (hi - lo) * min(done / total, 1.0))
                self.stage_text = ("replaying and recording -- %ds of about %ds"
                                   % (int(done), int(total)))

    def _span(self, n):
        if self.kind == "map":
            return {7: (0, 82), 8: (82, 100)}.get(n, STAGE_SPAN[n])
        return STAGE_SPAN[n]

    def _export_line(self, clean):
        """Progress for export_ply.py, which has no numbered stages.

        Its two long passes are both over the same bag: one for the transforms
        and one for the clouds. Only the second reports frames, so the first
        shows as a stalled bar unless it is given a slice of its own.
        """
        m = EXPORT_PROGRESS.match(clean)
        if m and self.total:
            done = min(float(m.group(1)) / self.total, 1.0)
            self.percent = 10.0 + 80.0 * done
            self.stage_text = "reading clouds -- %s of %d frames" % (
                m.group(1), self.total)
        elif m:
            self.stage_text = "reading clouds -- %s frames" % m.group(1)
        elif clean.startswith("reading transforms"):
            self.percent, self.stage_text = 4.0, "reading transforms"
        elif clean.startswith("reading clouds"):
            self.percent, self.stage_text = 10.0, "reading clouds"
        elif clean.startswith("voxel filter") or clean.startswith("range filter"):
            self.percent, self.stage_text = 92.0, clean.strip()
        elif clean.startswith("wrote "):
            self.percent, self.stage_text = 99.0, clean.strip()

    def alive(self):
        return self.proc.poll() is None

    def cancel(self):
        """Stop the whole process group, politely.

        SIGTERM rather than SIGKILL, and to the group rather than the shell,
        for the same reason walk_to_bag.sh uses SIGTERM on the recorder:
        rosbag2 writes the mcap's closing marker and its metadata.yaml during
        shutdown. Killed outright it leaves a large file that plays back as
        nothing. A cancelled conversion is meant to be abandoned, not to leave
        a booby trap behind, so give the script its trap and its cleanup.
        """
        if not self.alive():
            return False
        self.cancelled = True
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except OSError:
            try:
                self.proc.terminate()
            except OSError:
                return False
        return True

    def state(self):
        with self.lock:
            return {
                "kind": self.kind,
                "name": self.name,
                "running": self.alive(),
                "cancelled": self.cancelled,
                "returncode": self.returncode,
                "stage": self.stage,
                "stage_text": self.stage_text,
                "percent": round(self.percent, 1),
                "elapsed": round((self.finished_at or time.time()) - self.t0, 1),
                "log": list(self.lines)[-40:],
            }


def library_state():
    """What the walk-library panel needs to draw itself."""
    if walk_library is None:
        return {"available": False,
                "why": "walk_library.py is not next to dashboard.py"}
    if not WALKS_DIR:
        return {"available": False,
                "why": "this server was started without --walks, so it has no "
                       "library to show. Processing needs the Hesai driver, "
                       "which lives on the laptop rather than on the robot."}

    tool = os.path.join(TOOLS_DIR, "process_walk.sh")
    return {
        "available": True,
        "root": WALKS_DIR,
        "can_process": os.path.isfile(tool) and os.path.isfile(
            os.path.join(TOOLS_DIR, "walk_to_bag.sh")),
        "tool": tool,
        "walks": [_with_map(w) for w in walk_library.list_walks(WALKS_DIR)],
        "can_map": os.path.isfile(os.path.join(TOOLS_DIR, "lidar_map.py")),
        "job": job.state() if job is not None else None,
    }


def _with_map(walk):
    """Note the scan-matched map files, if built, so the row can show them."""
    folder = os.path.join(walk["path"], "map3d")
    ply = os.path.join(folder, walk["name"] + "_map.ply")
    mcap_dir = os.path.join(folder, walk["name"] + "_map")
    mcap = None
    if os.path.isdir(mcap_dir):
        for f in sorted(os.listdir(mcap_dir)):
            if f.endswith(".mcap"):
                mcap = os.path.join(mcap_dir, f)
                break
    dense = os.path.join(folder, walk["name"] + "_dense.ply")
    walk["map"] = ({"ply": ply, "ply_bytes": os.path.getsize(ply), "mcap": mcap,
                    "dense": dense if os.path.isfile(dense) else None,
                    "dense_bytes": os.path.getsize(dense) if os.path.isfile(dense) else 0}
                   if os.path.isfile(ply) else None)
    return walk


def windows_path(p):
    """A /mnt/e/... path as Windows sees it, for pasting into Foxglove.

    Foxglove Desktop is a Windows program and cannot open a WSL path. There is
    no general translation, but the one case that matters here -- a drive
    mounted under /mnt -- is exact.
    """
    m = re.match(r"^/mnt/([a-z])(/.*)?$", p or "")
    if not m:
        return p
    return m.group(1).upper() + ":" + (m.group(2) or "/").replace("/", "\\")


SERVICE_LOGS = {
    "dashboard": "/tmp/dashboard.log",
    "lidar_relay": "/tmp/lidar_relay.log",
    "camera_server": "/tmp/camera_server.log",
    "realsense_camera": "/tmp/realsense_camera.log",
}


def _tail(path, n):
    """The last n lines of a text file, read from the end so a large log
    costs no more than a small one."""
    try:
        with open(path, "rb") as fh:
            fh.seek(0, 2)
            size = fh.tell()
            fh.seek(max(0, size - 64 * 1024))
            data = fh.read()
    except OSError as e:
        return ["(could not read %s: %s)" % (path, e)]
    return data.decode("utf-8", "replace").splitlines()[-n:]


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_):
        pass                                   # keep the console readable

    def do_GET(self):
        path = self.path.split("?", 1)[0]

        if path == "/":
            return self._static_file("index.html", inject=True)
        if path in ("/dashboard-v2", "/dashboard-v2/"):
            # The redesigned page, beside the old one until it has proven
            # itself on the robot. Same API underneath; see static v2/.
            return self._static_file("v2/index.html", inject=True)
        if path == "/api/system":
            return self._json(system_info())
        if path == "/camera":
            return self._static_file("camera.html", inject=True)
        # /lidar is deliberately absent: the XT16 point-cloud viewer is not
        # built yet. See docs/dashboard.md, "What is not here".
        if path == "/data":
            return self._json(current())
        if path == "/health":
            return self._json(health())
        if path == "/api/recordings":
            return self._json(control_state())
        if path == "/api/library":
            return self._json(library_state())
        if path == "/api/sensors":
            if sensors is None:
                return self._json({"available": False,
                                   "why": "sensors.py is not next to dashboard.py"})
            return self._json(sensors.state())
        if path == "/api/preflight":
            return self._json(preflight())
        if path == "/api/walk-download":
            query = parse_qs(self.path.split("?", 1)[1] if "?" in self.path else "")
            return self._walk_download((query.get("name") or [""])[0])
        if path == "/api/logs":
            # The last lines of one service's log, for Go2 Studio's developer
            # console, so reading them does not need a terminal on the robot.
            # Only these names: each maps to the file start_services.sh or
            # realsense_start.sh writes, and nothing the caller sends is ever
            # joined onto a path.
            query = parse_qs(self.path.split("?", 1)[1] if "?" in self.path else "")
            name = (query.get("name") or ["dashboard"])[0]
            if name not in SERVICE_LOGS:
                return self._json({"error": "unknown log", "names": sorted(SERVICE_LOGS)})
            return self._json({"name": name, "file": SERVICE_LOGS[name],
                               "lines": _tail(SERVICE_LOGS[name], 300)})
        if path == "/api/settings":
            return self._json({"settings": read_settings(),
                               "path": SETTINGS_PATH,
                               "saved": os.path.isfile(SETTINGS_PATH)})
        if path.startswith("/static/"):
            return self._static_file(path[len("/static/"):])
        self.send_error(404)

    def do_POST(self):
        """The page's recording controls. Everything else is read-only."""
        parts = self.path.split("?", 1)
        if parts[0] != "/api/control":
            return self.send_error(404)

        params = {}
        # Parameters may arrive in the query string or as a JSON body; accept
        # either so the endpoint is easy to drive from curl as well.
        if len(parts) == 2:
            for pair in parts[1].split("&"):
                if "=" in pair:
                    k, v = pair.split("=", 1)
                    params[unquote(k)] = unquote(v)
        length = int(self.headers.get("Content-Length") or 0)
        if length:
            try:
                body = json.loads(self.rfile.read(length).decode("utf-8"))
                if isinstance(body, dict):
                    params.update(body)
            except ValueError:
                pass

        status, payload = do_control(params.get("action"), params)
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _walk_download(self, name):
        """Stream a walk folder to the browser as a zip, as it is read.

        No Content-Length: the zip is made on the way out, and adding up its
        headers in advance would be one more thing to get subtly wrong. So the
        response ends when the connection closes, which under HTTP/1.1 has to
        be said out loud or the browser waits for more.

        The server is threaded (ThreadingMixIn), so a download of a gigabyte
        occupies one thread and the page's polling carries on beside it.
        """
        folder, problem = walk_for_download(name)
        if problem:
            return self._json(dict(error=problem[1]), problem[0])

        # Unplugging the cable mid-download is a normal thing to do on this
        # robot. Without a timeout the send would wait out TCP's own, about a
        # quarter of an hour, holding a thread and possibly a sudo cat.
        self.connection.settimeout(60)
        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", "application/zip")
        self.send_header("Content-Disposition",
                         'attachment; filename="%s.zip"' % name)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            write_walk_zip(folder, name, self.wfile)
        except Exception as e:
            # The headers are gone, so there is no status left to send. Most
            # often this is the browser cancelling; the zip it has is short and
            # will say so when opened.
            sys.stderr.write("walk download %s stopped: %s\n" % (name, e))
            sys.stderr.flush()

    # -- helpers ----------------------------------------------------------
    def _json(self, obj, status=200):
        self._send(json.dumps(obj).encode("utf-8"), "application/json",
                   status=status)

    def _static_file(self, rel, inject=False):
        """Serve a file from dashboard_static/, refusing anything outside it."""
        full = os.path.normpath(os.path.join(STATIC_DIR, rel))
        if not full.startswith(STATIC_DIR) or not os.path.isfile(full):
            return self.send_error(404)

        with open(full, "rb") as fh:
            body = fh.read()
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        if full.endswith(".js"):
            ctype = "text/javascript"           # some 3.8 mimetypes say x-javascript
        if inject:
            # The camera host is a deploy-time setting, not something to
            # hardcode in the HTML. The page reads it from this placeholder
            # unless the user has overridden it in the settings drawer.
            body = body.replace(b"__CAMERA_HOST__", CAMERA_HOST.encode("utf-8"))
        # The mesh blob and three.js never change between restarts; everything
        # else should not be cached while we are developing.
        cache = "max-age=86400" if rel.startswith("vendor/") or rel.endswith(
            (".bin", ".json")) else "no-store"
        self._send(body, ctype, cache)

    def _send(self, body, ctype, cache="no-store", status=200):
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(body)


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    global CAMERA_HOST, recorder, replayer, dds_ready, RECORDINGS_DIR
    global SESSIONS_DIR, RECORDER, SUDO_PASSWORD, DDS_IFACE, DDS_DOMAIN
    global WALKS_DIR
    ap = argparse.ArgumentParser(description="Go2 live dashboard")
    ap.add_argument("--iface", default=None,
                    help="network interface carrying DDS (robot: eth0, sim: lo)")
    ap.add_argument("--domain", type=int, default=None,
                    help="DDS domain id (robot: 0, unitree_mujoco: 1)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--camera-host", default=None,
                    help="host:port of camera_server.py, e.g. 192.168.123.18:8080")
    ap.add_argument("--record", metavar="FILE", default=None,
                    help="also write every snapshot to FILE, for later --replay")
    ap.add_argument("--record-hz", type=float, default=10.0,
                    help="snapshots per second to record (default 10)")
    ap.add_argument("--replay", metavar="FILE", default=None,
                    help="serve a recorded FILE on a loop instead of a live "
                         "robot; needs no robot, no network and no SDK")
    ap.add_argument("--recordings", metavar="DIR", default=None,
                    help="where the page saves and looks for recordings "
                         "(default: recordings/ beside this script)")
    ap.add_argument("--sessions", metavar="DIR", default=None,
                    help="where walk recordings go (default: sessions/ beside "
                         "this script)")
    ap.add_argument("--recorder", metavar="PATH", default=None,
                    help="path to record_session.py (default: beside this "
                         "script). Walk recording is offered only if it exists")
    ap.add_argument("--walks", metavar="DIR", default=None,
                    help="folder of finished walks to list and process "
                         "(default: the repo's walks/ folder if it exists). Turning "
                         "a walk into a bag needs the Hesai driver, so this is "
                         "a laptop setting; on the robot leave it unset")
    ap.add_argument("--sudo-password", default=None,
                    help="password for sudo on this machine, needed because "
                         "packet capture requires root. Defaults to the "
                         "GO2_PASSWORD environment variable")
    args = ap.parse_args()

    if args.record and args.replay:
        sys.exit("--record and --replay cannot be used together")

    RECORDINGS_DIR = os.path.abspath(
        args.recordings or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "recordings"))
    try:
        os.makedirs(RECORDINGS_DIR)
    except OSError:
        pass                      # already there, or read-only; listing copes

    here = os.path.dirname(os.path.abspath(__file__))
    SESSIONS_DIR = os.path.abspath(args.sessions or os.path.join(here, "sessions"))
    try:
        os.makedirs(SESSIONS_DIR)
    except OSError:
        pass
    RECORDER = os.path.abspath(args.recorder or
                               os.path.join(here, "record_session.py"))
    if args.sudo_password:
        SUDO_PASSWORD = args.sudo_password

    if args.camera_host:
        CAMERA_HOST = args.camera_host

    # The library is offered only where it can actually be used. Guessing the
    # usual laptop path is a convenience; on the Jetson that path does not
    # exist, so the panel correctly reports itself unavailable rather than
    # listing an empty folder and offering a button that cannot work.
    # The usual laptop path is the repo's own walks/ folder, beside notes/,
    # so it follows the repo wherever it lives (E: once, D: since 2026-10-19).
    repo_walks = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "walks")
    walks = args.walks or os.environ.get("GO2_WALKS_DIR") or repo_walks
    WALKS_DIR = os.path.abspath(walks) if os.path.isdir(walks) else None
    if args.walks and WALKS_DIR is None:
        sys.exit("--walks %s is not a directory" % args.walks)

    iface = args.iface or os.environ.get("UNITREE_IFACE", "eth0")
    domain = args.domain if args.domain is not None else int(
        os.environ.get("GO2_DDS_DOMAIN", "0"))
    DDS_IFACE, DDS_DOMAIN = iface, domain

    if not os.path.isdir(STATIC_DIR):
        sys.exit("missing %s -- run make_model.py and copy dashboard_static/"
                 % STATIC_DIR)

    if args.replay:
        # No DDS at all in this mode. Nothing is subscribed, no interface is
        # opened, and the robot SDK need not even be installed. The page's
        # "Go live" button stays disabled, because there is nothing to go to.
        try:
            replayer = Replayer(args.replay)
        except (OSError, ValueError) as e:
            sys.exit(str(e))
        print("replay  %s -- %d frames, %.1f s, looping"
              % (args.replay, len(replayer.frames), replayer.duration))
        print("        THIS IS A RECORDING, not a live robot")
    else:
        if not HAVE_SDK:
            sys.exit("unitree_sdk2py is not installed, so only --replay works "
                     "on this machine")
        ChannelFactoryInitialize(domain, iface)

        # Subscribe to every candidate name. Only one will deliver; whichever
        # does is what the page reports as its source. Subscribing to a topic
        # nobody publishes costs nothing.
        for name in LOW_TOPICS:
            ChannelSubscriber(name, LowState_).Init(low.handler(name), 10)
        for name in SPORT_TOPICS:
            ChannelSubscriber(name, SportModeState_).Init(sport.handler(name), 10)

        dds_ready = True
        print("dds     domain %d on %s" % (domain, iface))
        print("topics  %s" % ", ".join(LOW_TOPICS + SPORT_TOPICS))

        if args.record:
            # A path given on the command line is used as-is; the page's own
            # controls always write inside RECORDINGS_DIR.
            recorder = Recorder(args.record, args.record_hz)
            print("record  %s at %.0f Hz" % (args.record, args.record_hz))

    print("saves   %s" % RECORDINGS_DIR)
    if os.path.isfile(RECORDER):
        print("walks   %s  (recorder: %s)" % (SESSIONS_DIR, RECORDER))
    else:
        print("walks   disabled -- no record_session.py at %s" % RECORDER)
    if WALKS_DIR:
        n = len(walk_library.list_walks(WALKS_DIR)) if walk_library else 0
        print("library %s  (%d walk%s)" % (WALKS_DIR, n, "" if n == 1 else "s"))
    else:
        print("library disabled -- no --walks directory on this machine")
    print("cameras %s" % CAMERA_HOST)
    print("serving http://0.0.0.0:%d  (also /camera, /data, /health)" % args.port)
    ThreadedHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
