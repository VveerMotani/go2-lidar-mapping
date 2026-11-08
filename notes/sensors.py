"""
The dashboard's Sensors panel: switch the depth camera and the LiDAR stream to
the laptop on and off, pick the depth resolution, and show the frame rates.

Runs on the robot next to dashboard.py. Standard library only (Python 3.8).

It does not talk to the sensors itself. It starts and stops the same scripts
you would run by hand, and reads the status files they write:

    realsense_start.sh          camera node + relay     /tmp/realsense_relay_status.json
    lidar_relay.py              LiDAR copy to laptop    /tmp/lidar_relay_status.json
    camera_server.py            the dashboard's video   (no status file)

So the terminal and this panel can never disagree about what is running: both
look at the processes.

WHAT SWITCHING OFF DOES AND DOES NOT DO
---------------------------------------
LiDAR off stops the copy being forwarded to the laptop. The XT16 keeps
spinning and sending to the robot's computer, and walk recording captures its
packets there without the relay, so a walk still records LiDAR with the stream
off. It frees about 2.7 MB/s on the link for depth.

Depth on stops camera_server.py first, because both want the same camera. The
Cameras panel goes dark until the depth camera is switched off again, which
brings camera_server.py back.
"""
import json
import os
import re
import subprocess
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REALSENSE_START = os.path.join(HERE, "realsense_start.sh")
LIDAR_RELAY = os.path.join(HERE, "lidar_relay.py")

DEPTH_STATUS = "/tmp/realsense_relay_status.json"
LIDAR_STATUS = "/tmp/lidar_relay_status.json"
DEFAULT_LAPTOP = "192.168.123.99"

# The depth modes offered. Each is one the D435i lists for its depth sensor.
# "proven" is what streamed on this robot on 2026-09-17 with the ROS 1 node.
DEPTH_MODES = [
    {"id": "640x480x15", "label": "640 x 480, 15 fps", "proven": True},
    {"id": "848x480x15", "label": "848 x 480, 15 fps", "proven": True},
    {"id": "1280x720x15", "label": "1280 x 720, 15 fps", "proven": True},
    {"id": "640x480x30", "label": "640 x 480, 30 fps", "proven": False},
    {"id": "848x480x30", "label": "848 x 480, 30 fps", "proven": False},
    {"id": "1280x720x30", "label": "1280 x 720, 30 fps", "proven": False},
]
MODE_IDS = set(m["id"] for m in DEPTH_MODES)

# The XT16 at 10 Hz sends about 2,500 UDP packets a second (measured on the
# walks). Anything well under that means packets are not arriving.
LIDAR_PPS = 2500.0

_lock = threading.Lock()
_job = None          # the one start/stop running now, or the last to finish
_wanted = {"depth_mode": "640x480x15", "color": False}


# --------------------------------------------------------------------------
# What is running
# --------------------------------------------------------------------------
def _pgrep(pattern):
    """Command lines matching pattern. The pattern should use the [x] trick."""
    try:
        out = subprocess.check_output(["pgrep", "-af", pattern],
                                      stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return []
    return [l for l in out.decode("utf-8", "replace").splitlines() if l.strip()]


def _read_status(path, max_age=3.0):
    """A status file's contents, or None if missing or older than max_age."""
    try:
        with open(path) as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        return None
    if time.time() - float(d.get("time") or 0) > max_age:
        return None
    return d


def _clean_env():
    """The dashboard's environment minus its CycloneDDS setup.

    The dashboard runs with a CycloneDDS library path for the robot SDK. The
    camera is ROS 1, which does not use DDS at all, and it has been run by
    hand from a plain login shell every time it worked. Match that.
    """
    env = dict(os.environ)
    env.pop("CYCLONEDDS_HOME", None)
    env.pop("CYCLONEDDS_URI", None)
    parts = [p for p in env.get("LD_LIBRARY_PATH", "").split(":")
             if p and "cyclonedds" not in p]
    if parts:
        env["LD_LIBRARY_PATH"] = ":".join(parts)
    else:
        env.pop("LD_LIBRARY_PATH", None)
    env["PYTHONUNBUFFERED"] = "1"
    return env


def depth_state():
    node = _pgrep("realsense2_camer[a]/RealSenseNodeFactory")
    relay = _pgrep("realsense_ros1_rela[y].py")
    launch = _pgrep("roslaunch realsense2_camer[a]")
    mode, color = None, False
    for line in launch:
        m = re.search(r"depth_width:=(\d+) depth_height:=(\d+) depth_fps:=(\d+)", line)
        if m:
            mode = "x".join(m.groups())
        color = color or "enable_color:=true" in line
    on = bool(launch or node)
    # A status file can outlive its relay by a few seconds; ignore it when off.
    status = _read_status(DEPTH_STATUS) if relay else None
    fps = (status or {}).get("fps_out", {}).get("depth")
    fps_in = (status or {}).get("fps_in", {}).get("depth")
    want = int(mode.split("x")[2]) if mode else None
    if not on:
        note = "off"
    elif not relay:
        note = "camera node running, relay not running"
    elif status is None:
        note = "starting, no frames yet"
    elif not fps_in:
        note = "running, but the camera is sending no depth frames"
    elif not status.get("connected"):
        note = "streaming, laptop not connected (start realsense_client.py)"
    else:
        note = "streaming to the laptop"
    return {
        "on": on, "relay": bool(relay), "mode": mode, "color": color,
        "fps": fps, "fps_in": fps_in, "fps_expected": want,
        "color_fps": (status or {}).get("fps_out", {}).get("color"),
        "connected": bool(status and status.get("connected")),
        "frame": (status or {}).get("mode", {}).get("depth"),
        "note": note,
    }


def lidar_state():
    procs = _pgrep("lidar_rela[y].py")
    dest = None
    for line in procs:
        m = re.search(r"--dest[ =](\S+)", line)
        dest = m.group(1) if m else DEFAULT_LAPTOP
    status = _read_status(LIDAR_STATUS) if procs else None
    pps = None
    if status:
        pps = status.get("pps", {}).get("2368")
    if not procs:
        note = "off: the laptop gets no LiDAR (walk recording still captures it)"
    elif status is None:
        note = "running (restart it once to get a live rate)"
    elif not pps:
        note = "running, but no packets arriving from the LiDAR"
    elif pps < LIDAR_PPS * 0.6:
        note = "running, packets low"
    else:
        note = "forwarding to %s" % dest
    return {"on": bool(procs), "dest": dest, "pps": pps,
            "pps_expected": LIDAR_PPS, "note": note}


def cameras_state():
    return {"on": bool(_pgrep("camera_serve[r].py"))}


def state():
    with _lock:
        job = _job.describe() if _job else None
    return {
        "available": os.path.isfile(REALSENSE_START),
        "why": None if os.path.isfile(REALSENSE_START)
               else "realsense_start.sh is not next to dashboard.py",
        "depth": depth_state(),
        "lidar": lidar_state(),
        "cameras": cameras_state(),
        "modes": DEPTH_MODES,
        "wanted": dict(_wanted),
        "job": job,
    }


# --------------------------------------------------------------------------
# Switching
# --------------------------------------------------------------------------
class Job(object):
    """One start or stop, run in the background, its output kept for the page."""

    def __init__(self, title, argv, expected_s):
        self.title = title
        self.started = time.time()
        self.expected = expected_s
        self.lines = []
        self.returncode = None
        # start_new_session: the camera must outlive a dashboard restart.
        self.proc = subprocess.Popen(
            argv, cwd=HERE, env=_clean_env(), stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for raw in self.proc.stdout:
            line = raw.decode("utf-8", "replace").rstrip()
            if line:
                self.lines.append(line)
                del self.lines[:-30]
        self.returncode = self.proc.wait()

    def running(self):
        return self.returncode is None

    def describe(self):
        elapsed = time.time() - self.started
        return {"title": self.title, "running": self.running(),
                "returncode": self.returncode, "elapsed": round(elapsed, 1),
                "percent": 100 if not self.running()
                           else min(95, int(100 * elapsed / self.expected)),
                "log": list(self.lines)}


def _detached(argv, log):
    """Start a long-lived service the way start_services.sh does."""
    with open(log, "ab") as fh:
        subprocess.Popen(argv, cwd=HERE, env=_clean_env(),
                         stdin=subprocess.DEVNULL, stdout=fh, stderr=fh,
                         start_new_session=True)


def _run_job(title, argv, expected_s):
    global _job
    with _lock:
        if _job is not None and _job.running():
            return 409, {"error": "still busy: %s" % _job.title}
        _job = Job(title, argv, expected_s)
    return 200, state()


def control(action, params):
    """Returns (http_status, body). Actions start with 'sensors-'."""
    if action == "sensors-depth-on":
        if not os.path.isfile(REALSENSE_START):
            return 409, {"error": "realsense_start.sh is not next to dashboard.py"}
        mode = params.get("mode") or _wanted["depth_mode"]
        if mode not in MODE_IDS:
            return 400, {"error": "unknown depth mode %s" % mode}
        color = bool(params.get("color"))
        _wanted.update(depth_mode=mode, color=color)
        argv = ["bash", REALSENSE_START, "start", "--depth=%s" % mode]
        if color:
            argv.append("--color")
        return _run_job("depth camera on at %s%s"
                        % (mode, " with colour" if color else ""), argv, 35.0)

    if action == "sensors-depth-off":
        # --cameras: once the camera is free, start camera_server.py again so
        # the Cameras panel comes back. Done inside the script rather than a
        # `bash -c` here, whose own command line would contain the pattern and
        # match itself.
        return _run_job("depth camera off",
                        ["bash", REALSENSE_START, "stop", "--cameras"], 8.0)

    if action == "sensors-lidar-on":
        if _pgrep("lidar_rela[y].py"):
            return 200, state()
        if not os.path.isfile(LIDAR_RELAY):
            return 409, {"error": "lidar_relay.py is not next to dashboard.py"}
        dest = params.get("dest") or DEFAULT_LAPTOP
        if not re.match(r"^\d{1,3}(\.\d{1,3}){3}$", dest):
            return 400, {"error": "bad address %s" % dest}
        _detached(["python3", LIDAR_RELAY, "--dest", dest], "/tmp/lidar_relay.log")
        time.sleep(1.0)
        return 200, state()

    if action == "sensors-lidar-off":
        subprocess.call(["pkill", "-f", "lidar_rela[y].py"])
        time.sleep(0.5)
        return 200, state()

    return 400, {"error": "unknown action"}
