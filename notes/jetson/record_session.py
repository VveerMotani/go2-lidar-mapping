"""
Record a walk: the LiDAR and the IMU together, on one clock.

Runs on the robot's Jetson. Nothing here needs the laptop, so the robot can be
untethered while it records. Afterwards you copy the folder to the laptop and
replay it as many times as you like -- which is the only sane way to develop a
SLAM pipeline, because otherwise every experiment needs the robot switched on
and walking.

    sudo python3 record_session.py                       # until ctrl-c
    sudo python3 record_session.py --duration 120        # two minutes
    sudo python3 record_session.py --name corridor1

Writes into ~/go2dev/sessions/<name>/ :

    lidar.pcap       raw LiDAR packets, replayable by the Hesai driver
    imu.csv          the robot's IMU, one row per sample (~500 Hz)
    odom.csv         the robot's own fused odometry (~50 Hz) -- WHERE it was
    session.json     what was recorded, for how long, and whether the clock was sane
    depth.stream     the depth camera, when it was on (depth_recorder.py)
    status.json      live progress, rewritten once a second; the dashboard reads it
    cyclonedds.log   the DDS trace for this run

WHY sudo
--------
The LiDAR packets are already being consumed by lidar_relay.py, which binds
UDP port 2368. Two programs cannot bind the same port -- the kernel would split
the packets between them and corrupt both streams. So instead of binding the
port we *sniff* the network passively, which needs root but takes a copy of
every packet without disturbing whoever is actually receiving them. The relay
keeps working normally while this records.

THE CLOCK MATTERS MORE THAN ANYTHING ELSE HERE
----------------------------------------------
SLAM lines the two streams up by their timestamps, so both must come from the
same clock. They do: everything below is stamped with this machine's time.

But the Jetson has no battery-backed clock, so it starts at 1970 every boot
until something sets it. A recording made with a 1970 clock is not wrong so
much as useless -- so this program checks, and refuses to start unless you
insist. Run go2-start.sh first, which syncs it, or:

    sudo date -s @$(date +%s)      # from a machine that knows the time
"""
import argparse
import errno
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

# --- pcap file format -------------------------------------------------------
# A .pcap is a 24-byte header, then for each packet a 16-byte record header
# followed by the raw bytes off the wire. We write link type 1 (Ethernet)
# because that is exactly what we captured, and it is what the Hesai driver
# expects when replaying with source_type: 2.
PCAP_MAGIC = 0xA1B2C3D4
PCAP_GLOBAL = struct.Struct("<IHHiIII")
PCAP_PACKET = struct.Struct("<IIII")

ETH_P_ALL = 0x0003
ETH_HEADER_LEN = 14
ETHERTYPE_IPV4 = 0x0800
IP_PROTO_UDP = 17

DEFAULT_DIR = os.path.expanduser("~/go2dev/sessions")


class PcapWriter(object):
    """Writes captured Ethernet frames to a .pcap file."""

    def __init__(self, path):
        self.fh = open(path, "wb")
        self.fh.write(PCAP_GLOBAL.pack(PCAP_MAGIC, 2, 4, 0, 0, 65535, 1))
        self.packets = 0
        self.bytes = 0

    def write(self, frame, when):
        sec = int(when)
        usec = int((when - sec) * 1e6)
        self.fh.write(PCAP_PACKET.pack(sec, usec, len(frame), len(frame)))
        self.fh.write(frame)
        self.packets += 1
        self.bytes += len(frame)

    def close(self):
        try:
            self.fh.close()
        except Exception:
            pass


def udp_destination(frame):
    """(destination ip, destination port) of an Ethernet frame, or None.

    Hand-parsed rather than pulled from a library, because the Jetson has no
    internet and we are not installing anything on it.
    """
    if len(frame) < ETH_HEADER_LEN + 20:
        return None
    if struct.unpack("!H", frame[12:14])[0] != ETHERTYPE_IPV4:
        return None                      # not IPv4 (could be a VLAN tag, ARP...)

    ip = frame[ETH_HEADER_LEN:]
    ihl = (ip[0] & 0x0F) * 4             # IP header length is variable
    if ip[9] != IP_PROTO_UDP or len(ip) < ihl + 8:
        return None
    dst_ip = "%d.%d.%d.%d" % tuple(ip[16:20])
    dst_port = struct.unpack("!H", ip[ihl + 2:ihl + 4])[0]
    return dst_ip, dst_port


def local_ipv4_addresses():
    """Every IPv4 address belonging to this machine.

    Used to keep only packets addressed TO us. Without that filter the capture
    picks up lidar_relay.py's forwarded copies as well as the sensor's own
    packets -- both are UDP to port 2368 -- and every packet lands in the file
    twice. Measured on the first real recording: 5070 packets a second where
    the XT16 sends 2535.
    """
    found = set()
    try:
        out = subprocess.check_output(["ip", "-o", "-4", "addr", "show"],
                                      stderr=subprocess.DEVNULL)
        for line in out.decode("utf-8", "replace").splitlines():
            parts = line.split()
            for i, tok in enumerate(parts):
                if tok == "inet" and i + 1 < len(parts):
                    found.add(parts[i + 1].split("/")[0])
    except (OSError, subprocess.SubprocessError):
        pass
    if not found:
        # Fall back to whatever the hostname resolves to. Worse, but better
        # than capturing everything and silently doubling the file again.
        try:
            found.add(socket.gethostbyname(socket.gethostname()))
        except OSError:
            pass
    return found


# Kernel-side packet filter: "udp dst port 2368 or udp dst port 10110", as
# compiled by `tcpdump -y EN10MB -dd`. With it attached, the kernel hands this
# program only the LiDAR's packets instead of every packet on the cable.
#
# WHY (odom-test-2, 2026-09-17): with the depth camera streaming to the laptop,
# the cable also carried roughly 100 Mbit/s of depth and colour. Without a
# filter this loop received every one of those packets in Python just to throw
# it away, fell behind, and 13.5% of the LiDAR's packets were lost, spread
# evenly through the walk. odom-test-1, with no camera traffic, lost none.
LIDAR_BPF = [
    (0x28, 0, 0, 0x0000000c), (0x15, 0, 4, 0x000086dd), (0x30, 0, 0, 0x00000014),
    (0x15, 0, 12, 0x00000011), (0x28, 0, 0, 0x00000038), (0x15, 9, 8, 0x00000940),
    (0x15, 0, 9, 0x00000800), (0x30, 0, 0, 0x00000017), (0x15, 0, 7, 0x00000011),
    (0x28, 0, 0, 0x00000014), (0x45, 5, 0, 0x00001fff), (0xb1, 0, 0, 0x0000000e),
    (0x48, 0, 0, 0x00000010), (0x15, 1, 0, 0x00000940), (0x15, 0, 1, 0x0000277e),
    (0x6, 0, 0, 0x00040000), (0x6, 0, 0, 0x00000000),
]
LIDAR_BPF_PORTS = {2368, 10110}
SO_ATTACH_FILTER = 26


def attach_lidar_filter(sock):
    """Attach LIDAR_BPF to a packet socket. Returns True if the kernel took it."""
    import ctypes
    code = b"".join(struct.pack("HBBI", *ins) for ins in LIDAR_BPF)
    buf = ctypes.create_string_buffer(code)
    prog = struct.pack("HL", len(LIDAR_BPF), ctypes.addressof(buf))
    try:
        sock.setsockopt(socket.SOL_SOCKET, SO_ATTACH_FILTER, prog)
    except OSError:
        return False
    # Anything queued before the filter went on is unfiltered; drain it.
    sock.setblocking(False)
    try:
        while True:
            sock.recv(65535)
    except (BlockingIOError, OSError):
        pass
    sock.setblocking(True)
    return True


class LidarSniffer(threading.Thread):
    """Copies every LiDAR packet off the wire into a pcap, without consuming it."""

    daemon = True

    def __init__(self, writer, ports, local_ips):
        threading.Thread.__init__(self, name="lidar")
        self.writer = writer
        self.ports = set(ports)
        self.local_ips = set(local_ips)
        self.skipped = 0
        self.running = True
        self.error = None
        self.seen = 0

        try:
            self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW,
                                      socket.htons(ETH_P_ALL))
        except AttributeError:
            raise SystemExit("packet capture needs Linux -- run this on the Jetson")
        except PermissionError:
            raise SystemExit("packet capture needs root -- run with sudo")
        # The XT16 pushes ~2500 packets a second. A small buffer drops them
        # during any hiccup, and a dropped packet is a hole in the scan.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
        self.filtered = set(ports) <= LIDAR_BPF_PORTS and attach_lidar_filter(self.sock)
        self.sock.settimeout(0.5)

    def run(self):
        while self.running:
            try:
                frame = self.sock.recv(65535)
            except socket.timeout:
                continue
            except OSError as e:
                if e.errno == errno.EINTR:
                    continue
                self.error = str(e)
                return
            # Stamp on arrival, from the same clock the IMU uses.
            now = time.time()
            dest = udp_destination(frame)
            if dest is None or dest[1] not in self.ports:
                continue
            # Only packets addressed to this machine. Anything else on this
            # port is lidar_relay.py's forwarded copy heading for the laptop,
            # and recording both puts every packet in the file twice.
            if self.local_ips and dest[0] not in self.local_ips:
                self.skipped += 1
                continue
            self.writer.write(frame, now)
            self.seen += 1

    def stop(self):
        self.running = False


class ImuRecorder(object):
    """Writes every LowState_ message's IMU to a CSV, at whatever rate it arrives.

    One row per sample, ~500 a second. Plain text on purpose: it can be opened
    in anything, and a recording cut short by a flat battery is still readable
    up to the last line.

    DUPLICATES, AND WHY THIS CLASS NOW REFUSES THEM
    -----------------------------------------------
    The robot publishes the same state on two topics: "rt/lowstate" at full
    rate, and "rt/lf/lowstate", where lf means low frequency -- the Go2 SDK
    docs say so in as many words ("where 'lf' represents low frequency"). Same
    data, sent less often. Not a different sensor and not a different route.

    This recorder used to subscribe to BOTH and write both into one file,
    copying dashboard.py. For a dashboard that only wants the newest value that
    is harmless. For a recorder it is a bug: a slow copy of a sample you
    already have lands in the middle of the fast stream. Measured on
    Lab-loop-1/imu.csv, 12,508 rows over 24.05 s:

        520.0 Hz where the sensor runs at 500
        285 ticks appearing twice
        271 consecutive rows with a tick delta of zero
        27  consecutive rows where the tick goes BACKWARDS
        170 samples arriving less than 0.5 ms apart

    An IMU pre-integrator is entitled to object to a repeated tick and a
    negative time step. main() now subscribes to the fast topic only, and falls
    back to the slow one solely if the fast one is silent. This filter stays as
    a second line of defence, and because counting what it rejects is how the
    next recording tells us the fix held.
    """

    COLUMNS = ("host_time", "tick", "qw", "qx", "qy", "qz",
               "gx", "gy", "gz", "ax", "ay", "az",
               "roll", "pitch", "yaw", "temp")

    # A tick that goes backwards by less than this is a stale sample arriving
    # late. A much larger jump backwards is the 32-bit counter wrapping, which
    # is legitimate and must not be dropped -- at 500 Hz that happens roughly
    # every 99 days of uptime, but a filter that silently eats every sample
    # after it would be a fine bug to inherit.
    STALE_TICK_WINDOW = 1000

    def __init__(self, path):
        self.fh = open(path, "w")
        self.fh.write(",".join(self.COLUMNS) + "\n")
        self.rows = 0
        self.first = None
        self.last = None
        self.last_tick = None
        self.duplicates = 0
        self.out_of_order = 0
        self.source_topic = None
        self.lock = threading.Lock()

    def on_lowstate(self, msg):
        now = time.time()
        imu = msg.imu_state

        tick = int(msg.tick)
        with self.lock:
            if self.last_tick is not None:
                delta = tick - self.last_tick
                if delta == 0:
                    self.duplicates += 1
                    return
                if -self.STALE_TICK_WINDOW < delta < 0:
                    self.out_of_order += 1
                    return
            self.last_tick = tick
        # Unitree's quaternion is stored w, x, y, z. Written out in that order
        # here; imu_publisher.py reorders it for ROS, which wants x, y, z, w.
        row = "%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n" % (
            now, int(msg.tick),
            imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3],
            imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2],
            imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2],
            imu.rpy[0], imu.rpy[1], imu.rpy[2],
            int(imu.temperature))
        with self.lock:
            self.fh.write(row)
            self.rows += 1
            if self.first is None:
                self.first = now
            self.last = now

    def close(self):
        with self.lock:
            try:
                self.fh.close()
            except Exception:
                pass

    def rate(self):
        if not self.rows or self.first is None or self.last <= self.first:
            return 0.0
        return self.rows / (self.last - self.first)


class OdomRecorder(object):
    """Writes the robot's own odometry to a CSV, one row per SportModeState_.

    WHY THIS IS THE MOST IMPORTANT FILE IN A RECORDING AFTER THE LIDAR
    ------------------------------------------------------------------
    Until 2026-09-05 a walk recorded the LiDAR and the IMU and nothing else.
    An IMU tells you which way the robot is pointing; it cannot tell you where
    it is. So a replayed walk had no way to place each scan, every frame was
    drawn at the same pose, and two minutes of walking came out as a smear.

    The robot solves this for free. It continuously publishes a fused
    legged + IMU odometry on "rt/sportmodestate" -- the Go2 SDK's Odometer
    Services Interface calls it the interface that "fuses legged (motor) data
    with IMU integration data", and the ROS2 Services Interface page defines
    the message. It costs nothing to record and it is what odom_publisher.py
    turns back into the odom -> base_link transform on replay.

    It is dead reckoning and it drifts, badly, on a legged robot -- a slipping
    foot looks exactly like a body that moved. It is a motion prior, not a map.

    NOT AVAILABLE WHEN THE ROBOT IS LYING DOWN
    ------------------------------------------
    sportmodestate is published by the motion control service. With the robot
    sat down, or with that service switched off for low-level work, this file
    will be empty and nothing is broken. main() says so rather than leaving a
    zero-byte file to be discovered later.
    """

    COLUMNS = ("host_time", "px", "py", "pz", "vx", "vy", "vz", "yaw_speed",
               "qw", "qx", "qy", "qz", "body_height", "gait_type", "mode",
               "foot_fr", "foot_fl", "foot_rr", "foot_rl")

    def __init__(self, path):
        self.fh = open(path, "w")
        self.fh.write(",".join(self.COLUMNS) + "\n")
        self.rows = 0
        self.first = None
        self.last = None
        self.source_topic = None
        self.lock = threading.Lock()

    def on_state(self, msg):
        now = time.time()
        imu = msg.imu_state
        # foot_force is indexed FR, FL, RR, RL -- the same order dashboard.py
        # uses, so the two never have to be reconciled.
        row = ("%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
               "%.6f,%.6f,%.6f,%.6f,%.4f,%d,%d,%d,%d,%d,%d\n") % (
            now,
            msg.position[0], msg.position[1], msg.position[2],
            msg.velocity[0], msg.velocity[1], msg.velocity[2],
            msg.yaw_speed,
            # w, x, y, z as the robot stores it. odom_publisher.py reorders it
            # for ROS, which wants x, y, z, w.
            imu.quaternion[0], imu.quaternion[1], imu.quaternion[2],
            imu.quaternion[3],
            msg.body_height, int(msg.gait_type), int(msg.mode),
            int(msg.foot_force[0]), int(msg.foot_force[1]),
            int(msg.foot_force[2]), int(msg.foot_force[3]))
        with self.lock:
            self.fh.write(row)
            self.rows += 1
            if self.first is None:
                self.first = now
            self.last = now

    def close(self):
        with self.lock:
            try:
                self.fh.close()
            except Exception:
                pass

    def rate(self):
        if not self.rows or self.first is None or self.last <= self.first:
            return 0.0
        return self.rows / (self.last - self.first)


class DepthRecording(object):
    """depth_recorder.py, started beside this recorder when the camera is on.

    The camera is Intel's ROS 1 node (realsense_start.sh, or the dashboard's
    Sensors panel). This does not start the camera: a walk records depth only
    if the camera was already streaming, so turning it on stays a deliberate
    choice with its own CPU and disk cost.
    """

    NODE = "realsense2_camer[a]/RealSenseNodeFactory"
    LAUNCH = "roslaunch realsense2_camer[a]"

    def __init__(self, out_dir, mode):
        self.out_dir = out_dir
        self.proc = None
        self.why_not = None
        self.color = False
        here = os.path.dirname(os.path.abspath(__file__))
        tool = os.path.join(here, "depth_recorder.py")
        if mode == "off":
            self.why_not = "--depth off"
            return
        if not os.path.isfile(tool):
            self.why_not = "depth_recorder.py is not next to record_session.py"
            return
        launch = self._pgrep(self.LAUNCH)
        if not launch and not self._pgrep(self.NODE):
            self.why_not = "depth camera is off (turn it on in the Sensors panel)"
            return
        self.color = any("enable_color:=true" in line for line in launch)
        cmd = "source /opt/ros/noetic/setup.bash && exec python3 %s --out %s%s" % (
            _quote(tool), _quote(out_dir), " --color" if self.color else "")
        log = open(os.path.join(out_dir, "depth_recorder.log"), "wb")
        self.proc = subprocess.Popen(["bash", "-c", cmd], stdout=log, stderr=log,
                                     stdin=subprocess.DEVNULL, start_new_session=True)

    @staticmethod
    def _pgrep(pattern):
        try:
            out = subprocess.check_output(["pgrep", "-af", pattern],
                                          stderr=subprocess.DEVNULL)
        except (OSError, subprocess.CalledProcessError):
            return []
        return out.decode("utf-8", "replace").splitlines()

    def status(self):
        try:
            with open(os.path.join(self.out_dir, "depth_status.json")) as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def stop(self):
        if self.proc is None:
            return None
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except OSError:
                pass
            try:
                self.proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        try:
            with open(os.path.join(self.out_dir, "depth_summary.json")) as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return {"error": "depth_recorder.py wrote no summary; see depth_recorder.log"}


def _quote(path):
    return "'" + path.replace("'", "'\\''") + "'"


def free_megabytes(path):
    st = os.statvfs(path)
    return st.f_bavail * st.f_frsize / 1e6


def main():
    ap = argparse.ArgumentParser(description="record LiDAR + IMU together")
    ap.add_argument("--name", default=None,
                    help="session folder name (default: a timestamp)")
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to record; 0 means until ctrl-c")
    ap.add_argument("--iface", default="eth0", help="interface carrying DDS")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--lidar-port", type=int, default=2368)
    ap.add_argument("--gps-port", type=int, default=10110,
                    help="0 to skip the LiDAR's GPS packets")
    ap.add_argument("--no-lidar", action="store_true",
                    help="record only the IMU (does not need root)")
    ap.add_argument("--no-imu", action="store_true",
                    help="record only the LiDAR (does not need the robot's DDS)")
    ap.add_argument("--no-odom", action="store_true",
                    help="skip odom.csv. Rarely what you want: without it a "
                         "replayed walk has no way to place each scan and "
                         "comes out as a smear.")
    ap.add_argument("--depth", choices=["auto", "off"], default="auto",
                    help="auto: record the depth camera too if it is running "
                         "(default); off: never")
    ap.add_argument("--ignore-clock", action="store_true",
                    help="record even though the clock is obviously wrong")
    ap.add_argument("--stop-file", default=None, metavar="PATH",
                    help="stop cleanly when this file appears. Lets another "
                         "program end the recording without sending a signal, "
                         "which matters because this runs as root and the "
                         "dashboard does not.")
    args = ap.parse_args()

    if args.no_lidar and args.no_imu and args.no_odom:
        raise SystemExit("--no-lidar, --no-imu and --no-odom together would "
                         "record nothing")

    # --- the clock check, before anything else --------------------------
    now = time.time()
    clock_ok = now > 1577836800          # 2020-01-01; anything below is the 1970 reset
    if not clock_ok:
        print("the clock reads %s -- it has not been synced since boot"
              % time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
        print("a recording made now cannot be lined up with anything.")
        print("run go2-start.sh from the laptop first, or:  sudo date -s @<epoch>")
        if not args.ignore_clock:
            raise SystemExit(1)
        print("--ignore-clock given, continuing anyway\n")

    name = args.name or time.strftime("session-%Y%m%d-%H%M%S")
    out_dir = os.path.join(os.path.expanduser(args.dir), name)
    if os.path.exists(out_dir):
        raise SystemExit("%s already exists -- pick another --name" % out_dir)
    os.makedirs(out_dir)

    free = free_megabytes(out_dir)
    print("recording to %s" % out_dir)
    print("  free space  %.1f GB  (the LiDAR writes about 160 MB a minute)"
          % (free / 1000.0))
    if free < 2000:
        print("  WARNING: under 2 GB free")

    # --- LiDAR ----------------------------------------------------------
    sniffer = writer = None
    if not args.no_lidar:
        ports = [args.lidar_port] + ([args.gps_port] if args.gps_port else [])
        local_ips = local_ipv4_addresses()
        writer = PcapWriter(os.path.join(out_dir, "lidar.pcap"))
        sniffer = LidarSniffer(writer, ports, local_ips)
        sniffer.start()
        print("  lidar       sniffing UDP %s addressed to %s%s"
              % (", ".join(str(p) for p in ports),
                 ", ".join(sorted(local_ips)) or "anything (could not read "
                 "this machine's addresses -- the file may contain duplicates)",
                 " (kernel filter on)" if sniffer.filtered else
                 " (NO kernel filter: other traffic on the cable can cost LiDAR packets)"))

    # --- IMU and odometry, both over DDS ---------------------------------
    imu = ImuRecorder(os.path.join(out_dir, "imu.csv"))
    odom = OdomRecorder(os.path.join(out_dir, "odom.csv"))
    imu_fallback = None            # set below; used by the progress loop
    if args.no_imu and args.no_odom:
        print("  imu         skipped (--no-imu)")
        print("  odom        skipped (--no-odom)")
    else:
        # Imported here, not at the top, so a LiDAR-only recording works on a
        # machine without the robot SDK and the error is about the right thing.
        from unitree_sdk2py.core import channel as _channel
        from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                                 ChannelSubscriber)
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import (LowState_,
                                                            SportModeState_)

        # unitree_sdk2py builds its own CycloneDDS configuration rather than
        # reading CYCLONEDDS_URI, and that configuration hardcodes a trace file
        # at /tmp/cdds.LOG (see the SDK's core/channel_config.py). Every process
        # using the SDK therefore writes to that one path -- so this recorder
        # and dashboard.py fight over it, and since the recorder runs as root
        # while the dashboard does not, the loser fails to start its DDS at all
        # with nothing but "channel factory init error" to go on.
        #
        # Point ours at its own file inside the session folder instead. It is
        # also then a record of what DDS did during that particular recording.
        _channel.ChannelConfigHasInterface = \
            _channel.ChannelConfigHasInterface.replace(
                "/tmp/cdds.LOG", os.path.join(out_dir, "cyclonedds.log"))

        ChannelFactoryInitialize(args.domain, args.iface)

        # ONE topic, not two. See the note on ImuRecorder: "rt/lf/lowstate"
        # carries the same data at a lower rate, and subscribing to both wrote
        # duplicate and out-of-order samples into one file. The slow topic is
        # kept only as a fallback, armed below if the fast one stays silent.
        if not args.no_imu:
            imu.source_topic = "rt/lowstate"
            ChannelSubscriber(imu.source_topic, LowState_).Init(imu.on_lowstate, 10)

            def arm_imu_fallback():
                """Subscribe to the low-frequency topic as a last resort.

                Some data at 20 Hz beats no data at all, and the recording
                says which topic it used so nobody has to guess afterwards
                why the rate looks wrong.
                """
                imu.source_topic = "rt/lf/lowstate (fallback)"
                ChannelSubscriber("rt/lf/lowstate", LowState_).Init(
                    imu.on_lowstate, 10)

            imu_fallback = arm_imu_fallback
            print("  imu         %s, domain %d on %s"
                  % (imu.source_topic, args.domain, args.iface))

        # The odometry. Same reasoning, same pair of topic names.
        if not args.no_odom:
            odom.source_topic = "rt/sportmodestate"
            ChannelSubscriber(odom.source_topic, SportModeState_).Init(
                odom.on_state, 10)
            print("  odom        %s, domain %d on %s"
                  % (odom.source_topic, args.domain, args.iface))

    # --- depth camera, if it is on -----------------------------------------
    depth = DepthRecording(out_dir, args.depth)
    if depth.proc is not None:
        print("  depth       depth_recorder.py%s" % (" with colour" if depth.color else ""))
    else:
        print("  depth       not recorded: %s" % depth.why_not)

    # Progress is written here once a second. The session folder is readable
    # by everyone even though root owns it, so the dashboard can watch a
    # recording it started without needing any privileges of its own.
    status_path = os.path.join(out_dir, "status.json")

    def write_status(running, elapsed, why=None):
        try:
            tmp = status_path + ".tmp"
            with open(tmp, "w") as fh:
                json.dump({
                    "name": name, "running": running,
                    "seconds": round(elapsed, 1),
                    "lidar_packets": writer.packets if writer else 0,
                    "lidar_bytes": writer.bytes if writer else 0,
                    "imu_rows": imu.rows, "imu_hz": round(imu.rate(), 1),
                    "odom_rows": odom.rows, "odom_hz": round(odom.rate(), 1),
                    "depth": depth.status() if depth.proc is not None else None,
                    "stopped_because": why,
                }, fh)
            os.rename(tmp, status_path)   # atomic, so a reader never sees half
            os.chmod(status_path, 0o644)
        except OSError:
            pass

    if args.stop_file and os.path.exists(args.stop_file):
        os.remove(args.stop_file)         # clear a stale one from last time

    started = time.time()
    write_status(True, 0.0)
    print("\nrecording. ctrl-c to stop.\n")
    stopped_because = "ctrl-c"
    warned_no_odom = False
    try:
        while True:
            time.sleep(1.0)
            elapsed = time.time() - started
            pkts = sniffer.seen if sniffer else 0
            mb = (writer.bytes / 1e6) if writer else 0.0
            sys.stdout.write(
                "\r  %5.0fs   lidar %8d pkts %7.1f MB   imu %7d %4.0f Hz"
                "   odom %6d %3.0f Hz "
                % (elapsed, pkts, mb, imu.rows, imu.rate(),
                   odom.rows, odom.rate()))
            sys.stdout.flush()

            write_status(True, elapsed)

            # The fast topic gets three seconds to prove itself. If nothing has
            # arrived by then it is not going to, and 20 Hz of IMU is worth
            # more than an empty file.
            if imu_fallback is not None and elapsed > 3.0 and imu.rows == 0:
                print("\n  imu         nothing on rt/lowstate after 3s -- "
                      "falling back to rt/lf/lowstate")
                imu_fallback()
                imu_fallback = None

            # Said once, early, while there is still time to start again.
            # sportmodestate arrives even with the robot lying down (seen at
            # 320 Hz on 2026-09-17), so nothing here means the motion data is
            # not reaching the recorder at all.
            if not args.no_odom and elapsed > 5.0 and odom.rows == 0 \
                    and not warned_no_odom:
                warned_no_odom = True
                print("\n  odom        NOTHING on rt/sportmodestate. Motion "
                      "data is not reaching the recorder.")
                print("              Without it this walk cannot be replayed "
                      "as a map -- every scan")
                print("              will be drawn at the same pose. Stand the "
                      "robot up and re-record.")

            if sniffer is not None and sniffer.error:
                stopped_because = "capture error: %s" % sniffer.error
                break
            if args.duration and elapsed >= args.duration:
                stopped_because = "reached --duration"
                break
            if args.stop_file and os.path.exists(args.stop_file):
                stopped_because = "stopped from the dashboard"
                try:
                    os.remove(args.stop_file)
                except OSError:
                    pass
                break
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - started
    if sniffer is not None:
        sniffer.stop()
        sniffer.join(timeout=2.0)
    if writer is not None:
        writer.close()
    imu.close()
    odom.close()
    depth_summary = depth.stop()

    meta = {
        "name": name,
        "started_unix": started,
        "started_human": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(started)),
        "seconds": round(elapsed, 2),
        "stopped_because": stopped_because,
        "clock_was_synced": clock_ok,
        "lidar": None if writer is None else {
            "file": "lidar.pcap", "packets": writer.packets,
            "bytes": writer.bytes,
            "not_addressed_to_us": sniffer.skipped if sniffer else 0,
            "kernel_filter": bool(sniffer.filtered) if sniffer else False,
            "packets_per_second": round(writer.packets / elapsed, 1) if elapsed else 0,
        },
        "imu": {"file": "imu.csv", "rows": imu.rows, "hz": round(imu.rate(), 1),
                "columns": list(ImuRecorder.COLUMNS),
                # Which topic this actually came from, and what the duplicate
                # filter rejected. Both should be "rt/lowstate" and zero. If
                # they are not, read the note on ImuRecorder before trusting
                # the file.
                "source_topic": imu.source_topic,
                "duplicates_dropped": imu.duplicates,
                "out_of_order_dropped": imu.out_of_order},
        "odom": None if odom.rows == 0 else {
            "file": "odom.csv", "rows": odom.rows,
            "hz": round(odom.rate(), 1),
            "columns": list(OdomRecorder.COLUMNS),
            "source_topic": odom.source_topic},
        "depth": depth_summary if depth_summary is not None else {
            "recorded": False, "why": depth.why_not},
        "note": "host_time in imu.csv, odom.csv and the pcap timestamps all come "
                "from the same clock on this machine, so the streams can be aligned.",
    }
    meta_path = os.path.join(out_dir, "session.json")
    with open(meta_path, "w") as fh:
        json.dump(meta, fh, indent=2)
    try:
        os.chmod(meta_path, 0o644)
    except OSError:
        pass
    write_status(False, elapsed, stopped_because)

    print("\n\nstopped: %s" % stopped_because)
    if writer is not None:
        print("  lidar.pcap   %d packets, %.1f MB, %.0f pkt/s"
              % (writer.packets, writer.bytes / 1e6,
                 writer.packets / elapsed if elapsed else 0))
        if writer.packets == 0:
            print("               NOTHING CAPTURED -- is the LiDAR powered and "
                  "sending to this machine?")
    print("  imu.csv      %d rows, %.0f Hz, from %s"
          % (imu.rows, imu.rate(), imu.source_topic or "nothing"))
    if imu.duplicates or imu.out_of_order:
        print("               dropped %d duplicate and %d out-of-order samples"
              % (imu.duplicates, imu.out_of_order))
    if imu.rows == 0 and not args.no_imu:
        print("               NOTHING CAPTURED -- is the robot on, and is "
              "--iface/--domain right?")
    print("  odom.csv     %d rows, %.0f Hz" % (odom.rows, odom.rate()))
    if odom.rows == 0 and not args.no_odom:
        print("               NOTHING CAPTURED. No motion data reached the")
        print("               recorder. This walk has no odometry, so it cannot")
        print("               be replayed as a map, only as a pile of scans.")
    if depth_summary and "frames" in depth_summary:
        print("  depth.stream %d depth frames at %.1f fps (%d dropped), %d colour, %.1f MB"
              % (depth_summary["frames"]["depth"], depth_summary.get("depth_fps", 0),
                 depth_summary["dropped"]["depth"], depth_summary["frames"]["color"],
                 depth_summary["bytes"] / 1e6))
    elif depth_summary:
        print("  depth        %s" % depth_summary.get("error"))
    print("\ncopy it to the laptop with:")
    print("  scp -r unitree@192.168.123.18:%s ." % out_dir)


if __name__ == "__main__":
    main()
