"""
Check a recorded bag properly, instead of looking at it.

    python3 inspect_bag.py ~/go2-lidar-mapping/walks/Lab-loop-1/bag

WHY THIS EXISTS
---------------
Both of the bugs found on the first recording day looked entirely fine on
screen, and neither could have been caught by eye:

  * The replayed cloud was FLAT -- one layer, no height -- because the per-laser
    angle calibration was missing. From the top-down view it looked like a
    perfectly reasonable room.
  * Every frame was drawn at the SAME POSE, because the frame tree bolted the
    LiDAR rigidly to the world. Two minutes of walking piled into one spot. It
    looked like a point cloud, because it was one.

So this measures rather than displays, and ends with a verdict per check.

WHAT GOOD LOOKS LIKE
--------------------
    points per frame     ~25,000-32,000 at 10 Hz, single return
    z range              spans a metre or more      <- catches the flat cloud
    dynamic /tf          present, odom -> base_link <- catches the smear
    /lidar_points rate   ~10 Hz, no long gaps
    /imu/data            ~500 Hz, no repeated or backwards timestamps
    /odom                path length matches the walk; loop returns near start
"""
import csv
import json
import os
import sys
from collections import Counter, defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from tf2_msgs.msg import TFMessage

import walk_library

try:
    from nav_msgs.msg import Odometry
except ImportError:                     # pragma: no cover
    Odometry = None

CLOUD_TOPIC = "/lidar_points"
IMU_TOPIC = "/imu/data"
ODOM_TOPIC = "/odom"

# sensor_msgs/PointField datatype numbers, for printing field types by name.
FIELD_TYPES = {1: "int8", 2: "uint8", 3: "int16", 4: "uint16",
               5: "int32", 6: "uint32", 7: "float32", 8: "float64"}

verdicts = []


def verdict(ok, headline, detail=None):
    verdicts.append((ok, headline, detail))


def xyz_of(msg):
    """The x, y, z of every point, as an (n, 3) float32 array.

    Reads the offsets out of the message rather than assuming x, y, z sit at
    the front. They do for this driver, but a silent wrong answer here would
    discredit every number below it.
    """
    offsets = {f.name: f.offset for f in msg.fields}
    for name in ("x", "y", "z"):
        if name not in offsets:
            return None
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = []
    for name in ("x", "y", "z"):
        off = offsets[name]
        chunk = np.ascontiguousarray(raw[:, off:off + 4]).tobytes()
        cols.append(np.frombuffer(chunk, dtype=np.float32))
    return np.stack(cols, axis=1)


def check_session(session, has_odom_topic):
    """Check the recording the bag was built from, if it is next door.

    imu_publisher.py replays with fresh timestamps, so duplicate and
    out-of-order samples in the recording are invisible once they reach a bag.
    They are exactly the fault that was found on 2026-09-04 -- the recorder
    subscribed to both rt/lowstate and rt/lf/lowstate and interleaved a slow
    copy of each sample into the fast stream -- so they are worth catching at
    the source.
    """
    imu_csv = os.path.join(session, "imu.csv")
    odom_csv = os.path.join(session, "odom.csv")
    meta = os.path.join(session, "session.json")
    if not os.path.isfile(imu_csv) and not os.path.isfile(meta):
        return                       # not a walk folder; nothing to say

    print("\nthe recording it came from (%s):" % session)

    if os.path.isfile(meta):
        with open(meta) as fh:
            d = json.load(fh)
        print("   walk          : %s, %.1f s" % (d.get("name"), d.get("seconds", 0)))
        verdict(bool(d.get("clock_was_synced")), "clock was synced",
                "yes" if d.get("clock_was_synced") else
                "NO. The LiDAR and IMU cannot be lined up, so this recording "
                "cannot be used for mapping.")
        src = (d.get("imu") or {}).get("source_topic")
        if src:
            print("   imu topic     : %s" % src)

    if os.path.isfile(imu_csv):
        # The tick analysis itself lives in walk_library, because the dashboard
        # runs the same check on the Jetson where numpy does not exist. One
        # implementation, so the two cannot drift apart -- in particular the
        # rule that a large negative jump is the 32-bit counter wrapping and a
        # small one is a stale sample.
        cont = walk_library.imu_continuity(imu_csv)
        repeats, backwards = cont["repeats"], cont["backwards"]
        times = []
        with open(imu_csv) as fh:
            for row in csv.DictReader(fh):
                try:
                    times.append(float(row["host_time"]))
                except (KeyError, ValueError):
                    continue
        if cont["rows"] > 2:
            span = times[-1] - times[0] if len(times) > 1 else 0
            hz = len(times) / span if span > 0 else 0
            print("   imu.csv       : %d rows, %.1f Hz" % (cont["rows"], hz))
            print("   repeated ticks: %d   backwards: %d" % (repeats, backwards))
            verdict(repeats == 0 and backwards == 0,
                    "imu.csv is a single stream",
                    "no repeated or out-of-order ticks"
                    if not (repeats or backwards) else
                    "%d repeated and %d out-of-order ticks. The recorder was "
                    "subscribed to both rt/lowstate and rt/lf/lowstate. Fixed "
                    "for recordings made after 2026-09-05; older ones keep it."
                    % (repeats, backwards))

    if os.path.isfile(odom_csv):
        verdict(has_odom_topic, "odometry reached the bag",
                "yes" if has_odom_topic else
                "odom.csv exists but no /odom in the bag -- odom_publisher.py "
                "did not run during conversion, so the scans will not "
                "accumulate")
    else:
        print("   odom.csv      : absent")
        print("                   Recorded before 2026-09-05, when the recorder")
        print("                   did not save odometry. Nothing can recover it")
        print("                   from here; the walk is a pile of scans.")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if not path:
        raise SystemExit("usage: inspect_bag.py <bag-directory>")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print("topics:")
    for k in sorted(types):
        print("   %-18s %s" % (k, types[k]))

    first_t = last_t = None
    counts = Counter()
    cloud_log_times = []
    cloud_points = []
    cloud_shown = False
    z_span_max = 0.0
    imu_stamps = []
    odom_positions = []
    static_pairs = []
    dynamic_pairs = Counter()
    dynamic_count = 0
    frames_seen = defaultdict(int)

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        counts[topic] += 1
        if first_t is None:
            first_t = stamp
        last_t = stamp

        if topic == "/tf_static":
            for tr in deserialize_message(data, TFMessage).transforms:
                static_pairs.append((tr.header.frame_id, tr.child_frame_id,
                                     tr.transform.translation.x,
                                     tr.transform.translation.y,
                                     tr.transform.translation.z))
        elif topic == "/tf":
            dynamic_count += 1
            for tr in deserialize_message(data, TFMessage).transforms:
                dynamic_pairs[(tr.header.frame_id, tr.child_frame_id)] += 1
        elif topic == IMU_TOPIC:
            m = deserialize_message(data, Imu)
            imu_stamps.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
        elif topic == ODOM_TOPIC and Odometry is not None:
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            odom_positions.append((p.x, p.y, p.z))
        elif topic == CLOUD_TOPIC:
            m = deserialize_message(data, PointCloud2)
            frames_seen[m.header.frame_id] += 1
            cloud_log_times.append(stamp / 1e9)
            n = m.width * m.height
            cloud_points.append(n)
            xyz = xyz_of(m)
            if xyz is not None and len(xyz):
                good = xyz[np.isfinite(xyz).all(axis=1)
                           & (np.abs(xyz).sum(axis=1) > 1e-6)]
                if len(good):
                    z_span_max = max(z_span_max, float(good[:, 2].max()
                                                       - good[:, 2].min()))
            if not cloud_shown:
                cloud_shown = True
                print("\nfirst %s message:" % CLOUD_TOPIC)
                print("   frame_id      :", m.header.frame_id)
                print("   header stamp  : %d.%09d"
                      % (m.header.stamp.sec, m.header.stamp.nanosec))
                print("   bag log time  : %d.%09d"
                      % (stamp // 10**9, stamp % 10**9))
                print("   difference    : %.3f s"
                      % (stamp / 1e9 - (m.header.stamp.sec
                                        + m.header.stamp.nanosec / 1e9)))
                print("   points        :", n)
                print("   point_step    : %d bytes" % m.point_step)
                print("   fields        : %s"
                      % ", ".join("%s(%s@%d)"
                                  % (f.name,
                                     FIELD_TYPES.get(f.datatype, f.datatype),
                                     f.offset)
                                  for f in m.fields))
                if xyz is not None and len(good):
                    print("   non-zero pts  : %d" % len(good))
                    print("   x/y/z ranges  : %.2f..%.2f  %.2f..%.2f  %.2f..%.2f"
                          % (good[:, 0].min(), good[:, 0].max(),
                             good[:, 1].min(), good[:, 1].max(),
                             good[:, 2].min(), good[:, 2].max()))

                names = {f.name for f in m.fields}
                # Per-point time is what a LIO package needs to undo the
                # distortion from the robot moving during a sweep. Its absence
                # is not an error, but it decides how much work SLAM is.
                verdict("timestamp" in names or "time" in names or "t" in names,
                        "per-point timestamps",
                        "present: LIO packages can deskew each sweep"
                        if names & {"timestamp", "time", "t"} else
                        "absent: every point in a frame shares one time, so a "
                        "sweep taken while walking is smeared")
                verdict("ring" in names, "per-point ring index",
                        "present" if "ring" in names else
                        "absent: some SLAM packages want to know which laser "
                        "a point came from")

    span = (last_t - first_t) / 1e9 if first_t is not None else 0.0

    print("\nmessage counts:")
    for k in sorted(counts):
        print("   %-18s %6d   %6.1f Hz" % (k, counts[k],
                                           counts[k] / span if span else 0))
    print("bag span      : %.2f s" % span)

    # --- the frame tree -----------------------------------------------------
    print("\nframe tree:")
    for f in static_pairs:
        print("   static   %-10s -> %-14s  xyz %.3f %.3f %.3f" % f)
    for (parent, child), n in sorted(dynamic_pairs.items()):
        print("   MOVING   %-10s -> %-14s  %d updates" % (parent, child, n))
    if not static_pairs and not dynamic_pairs:
        print("   NONE")

    verdict(bool(static_pairs), "static transforms",
            "%d recorded" % len(static_pairs) if static_pairs else
            "none: Foxglove will receive the cloud and draw nothing at all")

    # CONNECTIVITY, not just presence. A tree can have every transform it needs
    # and still be useless if they do not join up.
    #
    # This is exactly what went wrong on 2026-09-05: a session with no odom.csv
    # produced map->odom and base_link->hesai_lidar and nothing between them.
    # Two valid fragments, two roots. Foxglove was asked to display in "odom",
    # could not transform the cloud into it, and drew an EMPTY PANEL -- which
    # looks like the recording failed rather than like a missing transform.
    # The earlier "are there transforms?" check passed happily.
    parent_of = {}
    for parent, child, _x, _y, _z in static_pairs:
        parent_of[child] = parent
    for (parent, child) in dynamic_pairs:
        parent_of[child] = parent

    def root_of(frame):
        seen = set()
        while frame in parent_of and frame not in seen:
            seen.add(frame)
            frame = parent_of[frame]
        return frame

    all_frames = set(parent_of) | set(parent_of.values())
    roots = {root_of(f) for f in all_frames}
    if all_frames:
        cloud_frames = set(frames_seen) or {CLOUD_TOPIC}
        chains = []
        for f in sorted(cloud_frames):
            chain, cur, seen = [f], f, set()
            while cur in parent_of and cur not in seen:
                seen.add(cur)
                cur = parent_of[cur]
                chain.append(cur)
            chains.append(" -> ".join(reversed(chain)))
        print("   the cloud reaches: %s" % "; ".join(chains))
        verdict(len(roots) == 1, "frame tree joins up",
                "one root (%s)" % list(roots)[0] if len(roots) == 1 else
                "DISCONNECTED -- %d separate trees rooted at %s. Foxglove will "
                "show an EMPTY 3D panel if its display frame is in a different "
                "piece from the cloud. Set the display frame to '%s' to see the "
                "data now, and rebuild with walk_to_bag.sh for a joined-up tree."
                % (len(roots), ", ".join(sorted(roots)),
                   sorted(cloud_frames)[0] if cloud_frames else "hesai_lidar"))

    # The check that would have caught the smear on day one.
    verdict(dynamic_count > 0, "moving transform (/tf)",
            "%d updates across %d link(s)" % (dynamic_count, len(dynamic_pairs))
            if dynamic_count else
            "NONE. Every scan is drawn at the same pose, so a walk comes out "
            "as a smear rather than a room. It will look like a plausible "
            "point cloud while it does it. Needs odom.csv in the session and "
            "odom_publisher.py during conversion.")

    # --- the cloud ----------------------------------------------------------
    if cloud_points:
        pts = np.array(cloud_points)
        print("\n%s:" % CLOUD_TOPIC)
        print("   frames        : %d in %.1f s  (%.1f Hz)"
              % (len(pts), span, len(pts) / span if span else 0))
        print("   points/frame  : min %d  median %d  max %d"
              % (pts.min(), int(np.median(pts)), pts.max()))
        print("   largest z span: %.2f m" % z_span_max)
        print("   frame ids     : %s"
              % ", ".join("%s (%d)" % (k, v) for k, v in frames_seen.items()))

        # The flat-cloud check. A missing angle calibration collapses every
        # laser onto one plane and the result looks fine from above.
        verdict(z_span_max > 0.5, "cloud has height",
                "z spans %.2f m" % z_span_max if z_span_max > 0.5 else
                "z spans only %.2f m -- the per-laser angle calibration was "
                "missing, so all sixteen lasers were treated as horizontal. "
                "Point the replay config at xt16_correction.csv." % z_span_max)

        verdict(pts.min() > 1000, "frames are full",
                "smallest frame %d points" % pts.min() if pts.min() > 1000 else
                "smallest frame only %d points -- dropped UDP packets, or a "
                "frame split at the wrong azimuth" % pts.min())

        if len(cloud_log_times) > 2:
            gaps = np.diff(np.array(cloud_log_times))
            worst = float(gaps.max())
            expected = float(np.median(gaps))
            print("   frame gap     : median %.3f s  worst %.3f s"
                  % (expected, worst))
            # These are BAG LOG TIMES, which for a converted walk are replay
            # times, not sensor times -- walk_to_bag.sh replays through the
            # driver and the replay does not track real time exactly. A median
            # of 0.056 s here does not mean the LiDAR ran at 18 Hz, it means
            # the replay ran at about 1.8x. So this can only catch a replay
            # that genuinely STALLED, not an unsteady sensor. The sensor's own
            # cadence lives in the pcap and in the per-point timestamps.
            #
            # The threshold is absolute for that reason. An earlier version
            # used worst < median * 4 and failed a perfectly good 1374-frame
            # bag purely because the replay was uneven.
            stalled = worst > max(expected * 10, 2.0)
            verdict(not stalled, "replay did not stall",
                    "worst gap %.3f s (these are replay times, not sensor "
                    "times)" % worst if not stalled else
                    "worst gap %.3f s against a median of %.3f s -- the "
                    "replay stalled, so frames may be missing"
                    % (worst, expected))
    else:
        verdict(False, "point cloud present",
                "no %s messages at all" % CLOUD_TOPIC)

    # --- the IMU ------------------------------------------------------------
    if imu_stamps:
        t = np.array(imu_stamps)
        d = np.diff(t)
        repeats = int((d == 0).sum())
        backwards = int((d < 0).sum())
        print("\n%s:" % IMU_TOPIC)
        print("   samples       : %d  (%.0f Hz)"
              % (len(t), len(t) / span if span else 0))
        print("   repeated stamps: %d   backwards: %d" % (repeats, backwards))
        # Worth saying what this does and does not prove. imu_publisher.py
        # replays with --stamp now by default, so the bag carries fresh stamps
        # and will look clean however dirty the recording was. The check that
        # actually catches the recorder bug is on imu.csv, below.
        verdict(repeats == 0 and backwards == 0, "IMU timeline in the bag",
                "no repeated or backwards stamps (these are replay stamps, so "
                "this says nothing about the recording -- see imu.csv below)"
                if not (repeats or backwards)
                else "%d repeated and %d backwards stamps"
                     % (repeats, backwards))

    # --- the odometry -------------------------------------------------------
    if odom_positions:
        p = np.array(odom_positions)
        steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
        travelled = float(steps.sum())
        drift = float(np.linalg.norm(p[-1] - p[0]))
        print("\n%s:" % ODOM_TOPIC)
        print("   samples       : %d  (%.0f Hz)"
              % (len(p), len(p) / span if span else 0))
        print("   path length   : %.2f m" % travelled)
        print("   start to end  : %.2f m" % drift)
        verdict(travelled > 0.5, "the robot actually moved",
                "%.2f m of path" % travelled if travelled > 0.5 else
                "only %.2f m -- odometry was recorded but the robot barely "
                "moved, so there is little for SLAM to work with" % travelled)
        if travelled > 2.0:
            # Not a pass/fail: a loop is desirable, a straight line is fine.
            print("   %s" % ("returns near its start -- good for measuring "
                             "drift" if drift < travelled * 0.2 else
                             "does not return to its start; fine, but you "
                             "cannot see accumulated drift from it"))
    elif ODOM_TOPIC in types:
        verdict(False, "odometry has samples", "topic exists but is empty")

    # --- the recording behind the bag ---------------------------------------
    # A bag made by walk_to_bag.sh sits inside the session folder it came from.
    # When it does, check the source files too: some faults are only visible
    # there, because the replay hides them.
    check_session(os.path.dirname(os.path.abspath(path)),
                  has_odom_topic=bool(odom_positions))

    # --- the summary --------------------------------------------------------
    print("\n" + "-" * 68)
    bad = 0
    for ok, headline, detail in verdicts:
        mark = "  ok  " if ok else " FAIL "
        bad += 0 if ok else 1
        print("%s %-26s %s" % (mark, headline, detail or ""))
    print("-" * 68)
    print("%d of %d checks passed" % (len(verdicts) - bad, len(verdicts)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
