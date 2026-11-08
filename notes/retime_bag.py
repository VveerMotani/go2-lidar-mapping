#!/usr/bin/env python3
"""
Put every message in a walk's bag on the robot's clock, from the recording.

    python3 retime_bag.py <lidar.pcap> <raw-bag-dir> <output-bag-dir> --session <dir>

Called by walk_to_bag.sh after the LiDAR has been decoded. Writes the final
bag:

  /lidar_points   from the raw bag; header stamp = the LiDAR's own packet time,
                  moved onto the robot's clock with an offset and drift
                  measured from lidar.pcap (see below)
  /odom, /tf      from <session>/odom.csv, odom -> base_link, recorded times
  /imu/data       from <session>/imu.csv, recorded times
  /tf_static      map -> odom and base_link -> hesai_lidar (Unitree's
                  documented mounting); a frozen odom -> base_link as well when
                  there is no odom.csv, so the frame tree still joins up
  /camera/depth/image_rect_raw, /camera/depth/camera_info
  /camera/color/image_raw/compressed, /camera/color/camera_info
                  from <session>/depth.stream when the depth camera was on:
                  full resolution, millimetres, the factory calibration and
                  the camera's own frames. base_link -> camera_link comes from
                  camera_mount.json when there is one (see depth_stream.py);
                  without it the camera's frames are not joined to the robot
  every message   log time = its header stamp, so Foxglove's timeline and
                  every transform lookup use the same time

The messages are built by the same code the live publishers use
(odom_publisher.py, imu_publisher.py), so a bag and a live session carry
identical fields, frames and covariances.

Without --session, every topic in the raw bag is copied through with its
existing stamps (the pre-2026-09-17 layout, where odometry was replayed).

WHY
---
Until 2026-09-17 the replay stamped everything with the time it was replayed
("now") and assumed the Hesai driver plays the pcap in real time. It does not.
Measured on odom-test-1 (315 s walk): reading the pcap from E: it played at
about 0.4x, so the LiDAR spread over 1,290 s against 315 s of odometry and
nearly every scan was drawn at the wrong pose; from WSL's own disk it played
about 8x too fast. Its pcap_play_synchronization setting does not hold.
Stamping from the recording makes the replay speed irrelevant.

THE LIDAR CLOCK
---------------
The XT16 stamps each packet from its own clock, which on this robot is never
set (it reads May 2020) but runs steadily. The recorder stamped the same
packets on arrival with the robot's clock. For each packet,
    offset = arrival time - LiDAR time
is the clock difference plus network delay. Taking the smallest offset in each
5-second window removes the delay, and a straight line through those minima
gives the difference and its drift. Measured on odom-test-1: drift 34 ppm
(10 ms over the walk), residual under 1 ms.

Nothing is estimated from outside the recording.
"""
import calendar
import csv
import json
import os
import struct
import sys
import time

import rosbag2_py

WINDOW_S = 5.0


# --------------------------------------------------------------------------
# LiDAR clock from the pcap
# --------------------------------------------------------------------------
def xt16_time(payload):
    """The packet's own time. Date and time sit 15 bytes from the end:
    year-1900, month, day, hour, minute, second, then microseconds (uint32)."""
    if len(payload) < 20:
        return None
    k = len(payload) - 15
    y, mo, d, h, mi, s = payload[k:k + 6]
    if not (1 <= mo <= 12 and 1 <= d <= 31):
        return None
    us = struct.unpack("<I", payload[k + 6:k + 10])[0]
    try:
        return calendar.timegm((1900 + y, mo, d, h, mi, s)) + us * 1e-6
    except (ValueError, OverflowError):
        return None


def lidar_clock(pcap_path):
    """Fit arrival = lidar + a + b * (lidar - t0). Returns a dict."""
    with open(pcap_path, "rb") as fh:
        magic = fh.read(4)
        end = "<" if magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1") else ">"
        nano = magic in (b"\x4d\x3c\xb2\xa1", b"\xa1\xb2\x3c\x4d")
        fh.seek(24)
        windows = {}
        t0 = None
        n = 0
        while True:
            head = fh.read(16)
            if len(head) < 16:
                break
            sec, frac, incl, _ = struct.unpack(end + "IIII", head)
            frame = fh.read(incl)
            if len(frame) < 42 or frame[12:14] != b"\x08\x00":
                continue
            ihl = (frame[14] & 0x0F) * 4
            if frame[14 + 9] != 17:                      # UDP only
                continue
            dport = struct.unpack("!H", frame[14 + ihl + 2:14 + ihl + 4])[0]
            if dport != 2368:
                continue
            lt = xt16_time(frame[14 + ihl + 8:])
            if lt is None:
                continue
            arrival = sec + frac * (1e-9 if nano else 1e-6)
            if t0 is None:
                t0 = lt
            w = int((lt - t0) // WINDOW_S)
            off = arrival - lt
            best = windows.get(w)
            if best is None or off < best[1]:
                windows[w] = (lt, off)
            n += 1
    if len(windows) < 2:
        raise SystemExit("could not read LiDAR packet times from %s" % pcap_path)
    xs = [v[0] - t0 for v in windows.values()]
    ys = [v[1] for v in windows.values()]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 0.0
    a = my - b * mx
    resid = max(abs(y - (a + b * x)) for x, y in zip(xs, ys))
    return {"t0": t0, "offset_s": a, "drift_ppm": b * 1e6,
            "max_residual_ms": resid * 1000.0, "packets": n,
            "windows": len(windows), "window_s": WINDOW_S}


# --------------------------------------------------------------------------
# Rewriting the bag
# --------------------------------------------------------------------------
def get_stamp(raw, at):
    little = raw[1] == 1
    sec, nsec = struct.unpack("<iI" if little else ">iI", raw[at:at + 8])
    return sec + nsec * 1e-9


def set_stamp(raw, at, t):
    little = raw[1] == 1
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1000000000:
        sec, nsec = sec + 1, nsec - 1000000000
    return raw[:at] + struct.pack("<iI" if little else ">iI", sec, nsec) + raw[at + 8:]


LIDAR_IN_BASE = (0.171, 0.0, 0.0908)      # Unitree docs; same as go2-start.sh


def stamp_of(t):
    from builtin_interfaces.msg import Time
    m = Time()
    m.sec = int(t)
    m.nanosec = min(int(round((t - int(t)) * 1e9)), 999999999)
    return m


def static_tf(parent, child, xyz):
    from geometry_msgs.msg import TransformStamped
    tr = TransformStamped()
    tr.header.frame_id = parent
    tr.child_frame_id = child
    tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = \
        (float(v) for v in xyz)
    tr.transform.rotation.w = 1.0
    return tr


def image_info_msg(info, stamp):
    from sensor_msgs.msg import CameraInfo
    m = CameraInfo()
    m.header.stamp = stamp
    m.header.frame_id = info.get("frame_id") or ""
    m.width, m.height = int(info["width"]), int(info["height"])
    m.distortion_model = info.get("distortion_model") or "plumb_bob"
    m.d = [float(v) for v in info.get("D", [])]
    m.k = [float(v) for v in info["K"]]
    m.r = [float(v) for v in info["R"]]
    m.p = [float(v) for v in info["P"]]
    return m


def write_depth(writer, session, spans):
    """depth.stream -> image topics, and the camera's static frames.

    Returns the TransformStamped list for /tf_static (camera frames), or [].
    """
    path = os.path.join(session, "depth.stream")
    if not os.path.isfile(path):
        return []
    import array
    from rclpy.serialization import serialize_message
    from rosbag2_py import TopicMetadata
    from sensor_msgs.msg import CompressedImage, Image
    from geometry_msgs.msg import TransformStamped
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    from depth_stream import DepthStream, find_camera_mount
    from scipy.spatial.transform import Rotation

    ds = DepthStream(path)
    print("  depth.stream: %d depth, %d colour frames" % (ds.depth_count, ds.color_count),
          flush=True)
    topics = [("/camera/depth/image_rect_raw", "sensor_msgs/msg/Image"),
              ("/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo")]
    if ds.color_count:
        topics += [("/camera/color/image_raw/compressed", "sensor_msgs/msg/CompressedImage"),
                   ("/camera/color/camera_info", "sensor_msgs/msg/CameraInfo")]
    for topic, mtype in topics:
        writer.create_topic(TopicMetadata(topic, mtype, "cdr"))

    def note(topic, t):
        sp = spans.setdefault(topic, [t, t, 0])
        sp[0], sp[1], sp[2] = min(sp[0], t), max(sp[1], t), sp[2] + 1

    done = 0
    for f in ds.frames:
        st = stamp_of(f.t)
        ns = int(f.t * 1e9)
        if f.kind == "depth":
            img = f.image
            msg = Image()
            msg.header.stamp = st
            msg.header.frame_id = f.header.get("frame_id") or "camera_depth_optical_frame"
            msg.height, msg.width = img.shape
            msg.encoding = "16UC1"
            msg.is_bigendian = 0
            msg.step = img.shape[1] * 2
            msg.data = array.array("B", img.astype("<u2").tobytes())
            writer.write("/camera/depth/image_rect_raw", serialize_message(msg), ns)
            note("/camera/depth/image_rect_raw", f.t)
            if f.info:
                writer.write("/camera/depth/camera_info",
                             serialize_message(image_info_msg(f.info, st)), ns)
            done += 1
            if done % 1000 == 0:
                print("  %d depth frames written" % done, flush=True)
        else:
            msg = CompressedImage()
            msg.header.stamp = st
            msg.header.frame_id = f.header.get("frame_id") or "camera_color_optical_frame"
            msg.format = "jpeg"
            msg.data = array.array("B", f.jpeg)
            writer.write("/camera/color/image_raw/compressed", serialize_message(msg), ns)
            note("/camera/color/image_raw/compressed", f.t)
            if f.info:
                writer.write("/camera/color/camera_info",
                             serialize_message(image_info_msg(f.info, st)), ns)

    statics = []
    for tr in ds.transforms.values():
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = tr["parent"], tr["child"]
        (m.transform.translation.x, m.transform.translation.y,
         m.transform.translation.z) = (float(v) for v in tr["t"])
        (m.transform.rotation.x, m.transform.rotation.y,
         m.transform.rotation.z, m.transform.rotation.w) = (float(v) for v in tr["q"])
        statics.append(m)
    T, mount, mount_path = find_camera_mount(session)
    if T is not None:
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = "base_link", "camera_link"
        (m.transform.translation.x, m.transform.translation.y,
         m.transform.translation.z) = (float(v) for v in T[:3, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        (m.transform.rotation.x, m.transform.rotation.y,
         m.transform.rotation.z, m.transform.rotation.w) = (float(v) for v in q)
        statics.append(m)
        print("  camera mount from %s (%s)" % (mount_path, mount.get("source", "no source noted")))
    else:
        print("  no camera_mount.json: depth is in the bag, but not joined to the "
              "robot's frames. Measure the camera and run depth_mount.py.")
    return statics


def write_session(writer, session, spans):
    """odom.csv and imu.csv straight into the bag, with their recorded times."""
    from rclpy.serialization import serialize_message
    from rosbag2_py import TopicMetadata
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    import imu_publisher
    import odom_publisher

    def note(topic, t):
        sp = spans.setdefault(topic, [t, t, 0])
        sp[0], sp[1], sp[2] = min(sp[0], t), max(sp[1], t), sp[2] + 1

    for topic, mtype in (("/odom", "nav_msgs/msg/Odometry"),
                         ("/tf", "tf2_msgs/msg/TFMessage"),
                         ("/tf_static", "tf2_msgs/msg/TFMessage"),
                         ("/imu/data", "sensor_msgs/msg/Imu")):
        writer.create_topic(TopicMetadata(topic, mtype, "cdr"))

    odom_csv = os.path.join(session, "odom.csv")
    have_odom = os.path.isfile(odom_csv)
    if have_odom:
        with open(odom_csv) as fh:
            for r in csv.DictReader(fh):
                try:
                    t = float(r["host_time"])
                    pos = (float(r["px"]), float(r["py"]), float(r["pz"]))
                    quat = odom_publisher.normalised(
                        (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])))
                    vel = (float(r["vx"]), float(r["vy"]), float(r["vz"]))
                    yaw_speed = float(r["yaw_speed"])
                except (KeyError, ValueError):
                    continue              # header repeat, or a truncated last line
                st = stamp_of(t)
                odom = Odometry()
                odom.header.stamp = st
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                p = odom.pose.pose
                p.position.x, p.position.y, p.position.z = pos
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
                odom.pose.covariance = odom_publisher.POSE_COV
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = vel
                odom.twist.twist.angular.z = yaw_speed
                odom.twist.covariance = odom_publisher.TWIST_COV
                writer.write("/odom", serialize_message(odom), int(t * 1e9))
                note("/odom", t)

                tr = TransformStamped()
                tr.header.stamp = st
                tr.header.frame_id = "odom"
                tr.child_frame_id = "base_link"
                tr.transform.translation.x, tr.transform.translation.y, \
                    tr.transform.translation.z = pos
                (tr.transform.rotation.x, tr.transform.rotation.y,
                 tr.transform.rotation.z, tr.transform.rotation.w) = quat
                writer.write("/tf", serialize_message(TFMessage(transforms=[tr])), int(t * 1e9))
                note("/tf", t)

    imu_csv = os.path.join(session, "imu.csv")
    if os.path.isfile(imu_csv):
        with open(imu_csv) as fh:
            for r in csv.DictReader(fh):
                try:
                    t = float(r["host_time"])
                    quat = (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"]))
                    gyro = (float(r["gx"]), float(r["gy"]), float(r["gz"]))
                    accel = (float(r["ax"]), float(r["ay"]), float(r["az"]))
                except (KeyError, ValueError):
                    continue
                msg = Imu()
                msg.header.stamp = stamp_of(t)
                imu_publisher.fill(msg, "imu_link", quat, gyro, accel)
                writer.write("/imu/data", serialize_message(msg), int(t * 1e9))
                note("/imu/data", t)

    camera_statics = write_depth(writer, session, spans)

    start = min(sp[0] for sp in spans.values())
    lidar_tf = static_tf("base_link", "hesai_lidar", LIDAR_IN_BASE)
    # GO2_CONFIG_DIR last: the Docker image keeps this robot's calibrations there.
    for folder in (session, os.path.dirname(os.path.abspath(session)),
                   os.environ.get("GO2_CONFIG_DIR", "")):
        if not folder:
            continue
        mount_path = os.path.join(folder, "lidar_mount.json")
        if os.path.isfile(mount_path):
            # Calibrated by lidar_mount.py: the documented position, plus the
            # tilt measured from the floor.
            from scipy.spatial.transform import Rotation
            with open(mount_path) as fh:
                mount = json.load(fh)
            lidar_tf = static_tf("base_link", "hesai_lidar", mount.get("xyz", LIDAR_IN_BASE))
            q = Rotation.from_euler("xyz", mount["rpy_deg"], degrees=True).as_quat()
            (lidar_tf.transform.rotation.x, lidar_tf.transform.rotation.y,
             lidar_tf.transform.rotation.z, lidar_tf.transform.rotation.w) = (float(v) for v in q)
            print("  LiDAR mount from %s" % mount_path)
            break
    statics = [static_tf("map", "odom", (0, 0, 0)), lidar_tf] + camera_statics
    if not have_odom:
        statics.append(static_tf("odom", "base_link", (0, 0, 0)))
    for tr in statics:
        tr.header.stamp = stamp_of(start)
    writer.write("/tf_static", serialize_message(TFMessage(transforms=statics)),
                 int(start * 1e9))


def main():
    import argparse
    ap = argparse.ArgumentParser(description="put a walk's bag on the recording's clock")
    ap.add_argument("pcap")
    ap.add_argument("raw_bag")
    ap.add_argument("out_bag")
    ap.add_argument("--session", default=None,
                    help="write odometry, IMU and the frame tree from this session's files")
    args = ap.parse_args()
    pcap, src, dst = args.pcap, args.raw_bag, args.out_bag
    started = time.time()

    clock = lidar_clock(pcap)
    with open(os.path.join(os.path.dirname(os.path.abspath(dst)), "lidar_clock.json"), "w") as fh:
        json.dump(clock, fh, indent=2)
    print("  LiDAR clock: offset %.3f s, drift %.1f ppm, fit within %.2f ms (%d packets)"
          % (clock["offset_s"], clock["drift_ppm"], clock["max_residual_ms"],
             clock["packets"]), flush=True)
    a, b, t0 = clock["offset_s"], clock["drift_ppm"] * 1e-6, clock["t0"]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for meta in reader.get_all_topics_and_types():
        if args.session and meta.name != "/lidar_points":
            continue
        writer.create_topic(meta)

    spans = {}
    statics = []
    bad_lidar = 0
    while reader.has_next():
        topic, raw, t_rec = reader.read_next()
        if args.session and topic != "/lidar_points":
            continue
        raw = bytes(raw)
        if topic == "/tf_static":
            statics.append((topic, raw))              # written at the start, below
            continue
        if topic == "/lidar_points":
            lt = get_stamp(raw, 4)
            if abs(lt - t0) > 86400:                  # not LiDAR time: driver misconfigured
                bad_lidar += 1
                continue
            t = lt + a + b * (lt - t0)
            raw = set_stamp(raw, 4, t)
        elif topic == "/tf":
            if len(raw) < 16 or struct.unpack("<I" if raw[1] == 1 else ">I", raw[4:8])[0] == 0:
                continue
            t = get_stamp(raw, 8)
        else:
            t = get_stamp(raw, 4)
        sp = spans.setdefault(topic, [t, t, 0])
        sp[0], sp[1], sp[2] = min(sp[0], t), max(sp[1], t), sp[2] + 1
        writer.write(topic, raw, int(t * 1e9))

    if bad_lidar:
        raise SystemExit("%d LiDAR frames were not stamped with LiDAR time; the replay "
                         "config needs use_timestamp_type: 0" % bad_lidar)
    if "/lidar_points" not in spans:
        raise SystemExit("no LiDAR frames in %s" % src)

    if args.session:
        write_session(writer, args.session, spans)
    else:
        start = min(sp[0] for sp in spans.values())
        for topic, raw in statics:
            writer.write(topic, raw, int(start * 1e9))
    del writer                                        # closes the file

    for topic, (lo, hi, n) in sorted(spans.items()):
        print("  %-14s %7d messages, %6.1f s" % (topic, n, hi - lo))
    print("  written in %.0f s" % (time.time() - started))
    if "/lidar_points" in spans and "/odom" in spans:
        L, O = spans["/lidar_points"], spans["/odom"]
        print("  LiDAR starts %+.2f s and ends %+.2f s relative to odometry"
              % (L[0] - O[0], L[1] - O[1]))
        if abs(L[0] - O[0]) > 5 or abs(L[1] - O[1]) > 5:
            print("  WARNING: LiDAR and odometry do not cover the same time")
            sys.exit(2)


if __name__ == "__main__":
    main()
