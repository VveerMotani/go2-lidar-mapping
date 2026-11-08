"""
Turn a recorded walk into one .ply point cloud you can open anywhere.

    python3 export_ply.py ~/go2-lidar-mapping/walks/Lab-loop-1/bag
    python3 export_ply.py <bag> --voxel 0.03 --color height
    python3 export_ply.py --live --seconds 15 --out /tmp/now.ply

The result opens in CloudCompare, MeshLab, Blender, Open3D, and most CAD
packages. No ROS, no robot, no network -- which is the point: a .ply is how you
hand this data to somebody who has never heard of rosbag2.

WHY NOT OPEN3D
--------------
Open3D would do the voxel filter and the writing in two lines. It is also a
large binary dependency that is fussy about Python versions -- go2_ros2_sdk's
own README warns it does not work on 3.12 -- and everything needed here is a
handful of numpy operations. So this writes the file itself and stays a pure
numpy program.

WHAT "ACCUMULATE" MEANS, AND WHEN IT IS A LIE
---------------------------------------------
Each LiDAR frame is 32,000 points measured from wherever the robot was standing
at that instant, in the sensor's own frame. Stacking them only produces a room
if every frame is first moved into a fixed frame using the robot's pose at that
moment -- which is what the bag's /tf carries, and what odom.csv before it
carried.

If the bag has no moving transform, every frame lands in the same place and the
result is a smear: a plausible-looking, entirely useless cloud of one spot seen
many times. That happened to both walks recorded on 2026-09-04. This program
refuses to pretend otherwise -- it says so, loudly, and keeps going only because
a single-position scan is still worth looking at.
"""
import argparse
import os
import struct
import sys
import time

import numpy as np

CLOUD_TOPIC = "/lidar_points"

# Written as binary_little_endian by default. A two-minute walk is tens of
# millions of points; as ascii that is a multi-gigabyte text file that most
# viewers will not load.
PLY_HEADER = """ply
format {fmt} 1.0
comment written by export_ply.py from a Unitree Go2 / Hesai XT16 recording
comment {note}
element vertex {n}
property float x
property float y
property float z
{extra}end_header
"""


# ---------------------------------------------------------------------------
# Reading points out of a PointCloud2 without assuming anything
# ---------------------------------------------------------------------------
def fields_of(msg):
    """{name: (offset, numpy dtype)} for the fields we can use."""
    types = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
    out = {}
    for f in msg.fields:
        if f.datatype in types:
            out[f.name] = (f.offset, types[f.datatype])
    return out


def read_points(msg, want=("x", "y", "z", "intensity")):
    """Named columns from a PointCloud2, as float32.

    Parsed by hand rather than with sensor_msgs_py.point_cloud2, whose
    read_points_numpy on Humble asserts that every field shares one datatype.
    Ours does not: x/y/z/intensity are float32, ring is uint16 and timestamp is
    float64. That assertion is what stopped lidar_snapshot.py on 2026-09-04.
    """
    have = fields_of(msg)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = {}
    for name in want:
        if name not in have:
            continue
        off, dt = have[name]
        width = np.dtype(dt).itemsize
        chunk = np.ascontiguousarray(raw[:, off:off + width]).tobytes()
        cols[name] = np.frombuffer(chunk, dtype=dt).astype(np.float32)
    return cols


# ---------------------------------------------------------------------------
# Poses
# ---------------------------------------------------------------------------
def quat_to_matrix(x, y, z, w):
    """A rotation matrix from a ROS quaternion (x, y, z, w).

    Written out rather than imported so this file does not need tf_transformations
    -- and so the ordering is visible. Unitree publish [w, x, y, z] and ROS uses
    [x, y, z, w]; getting that backwards produced a robot that rolled when it
    should have pitched, and it looked entirely believable.
    """
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ], dtype=np.float64)


def transform_matrix(tf):
    """A 4x4 from a geometry_msgs/TransformStamped."""
    t, r = tf.transform.translation, tf.transform.rotation
    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = quat_to_matrix(r.x, r.y, r.z, r.w)
    m[:3, 3] = (t.x, t.y, t.z)
    return m


class Poses(object):
    """Every transform in the bag, and where a frame was at a given time.

    A deliberately small replacement for tf2's buffer. tf2 would work, but it
    wants a node, a clock and a cache window sized to the recording, and all
    that is needed here is: walk the chain of parents, and for the one link
    that moves, pick the sample nearest in time.
    """

    def __init__(self):
        self.static = {}      # child -> (parent, 4x4)
        self.dynamic = {}     # child -> (parent, [times], [4x4])

    def add(self, msg, stamp_ns, is_static):
        for tf in msg.transforms:
            child = tf.child_frame_id.lstrip("/")
            parent = tf.header.frame_id.lstrip("/")
            m = transform_matrix(tf)
            if is_static:
                self.static[child] = (parent, m)
            else:
                t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                if child not in self.dynamic:
                    self.dynamic[child] = (parent, [], [])
                _p, times, mats = self.dynamic[child]
                times.append(t)
                mats.append(m)

    def finish(self):
        for child, (parent, times, mats) in list(self.dynamic.items()):
            order = np.argsort(np.asarray(times))
            self.dynamic[child] = (parent,
                                   np.asarray(times)[order],
                                   np.asarray(mats)[order])

    def moving(self):
        """True if anything in this bag actually moves."""
        return any(len(v[1]) > 1 for v in self.dynamic.values())

    def link(self, child, t):
        """child -> parent for one hop, at time t. None if there is no such link."""
        if child in self.dynamic:
            parent, times, mats = self.dynamic[child]
            if len(times) == 0:
                return None
            i = int(np.searchsorted(times, t))
            i = min(max(i, 0), len(times) - 1)
            # Nearest sample rather than interpolated. At 50 Hz odometry the
            # worst error is 10 ms of walking, about 5 mm -- an order of
            # magnitude under the sensor's own noise, and interpolating
            # rotations properly needs slerp and a good deal more code.
            if i > 0 and abs(times[i - 1] - t) < abs(times[i] - t):
                i -= 1
            return parent, mats[i]
        if child in self.static:
            return self.static[child]
        return None

    def to(self, target, source, t, max_hops=12):
        """4x4 taking points in `source` into `target` at time t."""
        m = np.eye(4, dtype=np.float64)
        frame = source.lstrip("/")
        target = target.lstrip("/")
        for _ in range(max_hops):
            if frame == target:
                return m
            hop = self.link(frame, t)
            if hop is None:
                return None
            parent, mat = hop
            m = mat.dot(m)
            frame = parent
        return None


# ---------------------------------------------------------------------------
# Downsampling and writing
# ---------------------------------------------------------------------------
def voxel_filter(xyz, extra, size):
    """One point per `size`-metre cube: the first one seen.

    Averaging each cube would be marginally prettier and needs a sort plus a
    segmented mean over tens of millions of points. Taking the first is a
    single np.unique, and at 3-5 cm the difference is invisible.
    """
    if size <= 0 or len(xyz) == 0:
        return xyz, extra
    keys = np.floor(xyz / size).astype(np.int64)
    # Combine the three indices into one so np.unique runs on a flat array;
    # a structured view would work too but is slower on big arrays.
    _u, idx = np.unique(
        keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791,
        return_index=True)
    idx.sort()                       # keep the original order, for tidy files
    return xyz[idx], {k: v[idx] for k, v in extra.items()}


def colourise(values, mode):
    """uint8 rgb from a scalar, so the file looks like something on opening.

    Viewers show an uncoloured cloud as a uniform grey fog, which is a poor
    first impression of a good scan. This is a plain blue-green-red ramp over
    the 2nd to 98th percentile, so a few stray points cannot flatten it.
    """
    if values is None or len(values) == 0:
        return None
    lo, hi = np.percentile(values, 2), np.percentile(values, 98)
    if hi - lo < 1e-9:
        hi = lo + 1.0
    v = np.clip((values - lo) / (hi - lo), 0.0, 1.0)
    r = np.clip(1.5 - abs(4.0 * v - 3.0), 0, 1)
    g = np.clip(1.5 - abs(4.0 * v - 2.0), 0, 1)
    b = np.clip(1.5 - abs(4.0 * v - 1.0), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def write_ply(path, xyz, intensity=None, rgb=None, ascii_mode=False, note=""):
    """Write the cloud. Binary little-endian unless asked otherwise."""
    n = len(xyz)
    extra = ""
    if intensity is not None:
        extra += "property float intensity\n"
    if rgb is not None:
        extra += ("property uchar red\nproperty uchar green\n"
                  "property uchar blue\n")
    header = PLY_HEADER.format(
        fmt="ascii" if ascii_mode else "binary_little_endian",
        note=note.replace("\n", " ")[:180], n=n, extra=extra)

    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        if ascii_mode:
            for i in range(n):
                row = "%.4f %.4f %.4f" % tuple(xyz[i])
                if intensity is not None:
                    row += " %.3f" % intensity[i]
                if rgb is not None:
                    row += " %d %d %d" % tuple(rgb[i])
                fh.write((row + "\n").encode("ascii"))
            return

        # One structured array, written in one go. Building the whole buffer
        # costs memory but is many times faster than per-point struct.pack,
        # and these files run to tens of millions of points.
        dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
        if intensity is not None:
            dtype.append(("intensity", "<f4"))
        if rgb is not None:
            dtype += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
        arr = np.empty(n, dtype=dtype)
        arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        if intensity is not None:
            arr["intensity"] = intensity
        if rgb is not None:
            arr["red"], arr["green"], arr["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        fh.write(arr.tobytes())


# ---------------------------------------------------------------------------
# From a bag
# ---------------------------------------------------------------------------
def from_bag(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage

    path = args.source
    if os.path.isdir(path) and not any(f.endswith(".mcap")
                                       for f in os.listdir(path)):
        sys.exit("%s has no .mcap in it" % path)

    def reader():
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
               rosbag2_py.ConverterOptions("", ""))
        return r

    # Pass one: every transform. They have to be complete before any cloud can
    # be placed, because /tf_static is written once at the start but the moving
    # transform for a given instant may arrive after the cloud it applies to.
    print("reading transforms...")
    poses = Poses()
    r = reader()
    while r.has_next():
        topic, data, _stamp = r.read_next()
        if topic in ("/tf", "/tf_static"):
            poses.add(deserialize_message(data, TFMessage), _stamp,
                      topic == "/tf_static")
    poses.finish()

    chain = "%s <- %s" % (args.frame, "hesai_lidar")
    if not poses.moving():
        print("")
        print("  !! this bag has no MOVING transform.")
        print("     Every frame will land in the same place, so the result is")
        print("     one position seen many times rather than a room. The walk")
        print("     was recorded without odom.csv and it cannot be added now.")
        print("     Exporting anyway -- a single-position scan is still useful.")
        print("")

    # Pass two: the clouds.
    print("reading clouds (%s, every %d, target frame %s)..."
          % (CLOUD_TOPIC, args.every, args.frame))
    chunks_xyz, chunks_i = [], []
    kept = skipped = seen = 0
    total = 0
    r = reader()
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic != CLOUD_TOPIC:
            continue
        seen += 1
        if (seen - 1) % args.every:
            continue

        msg = deserialize_message(data, PointCloud2)
        cols = read_points(msg)
        if "x" not in cols:
            continue
        xyz = np.stack([cols["x"], cols["y"], cols["z"]], axis=1)

        # The driver pads every frame to a fixed size with exact zeros for
        # lasers that got no return. Kept, they draw a dense blob at the origin
        # that dominates the whole file.
        keep = ~np.all(np.abs(xyz) < 1e-6, axis=1)
        xyz = xyz[keep]
        inten = cols.get("intensity")
        if inten is not None:
            inten = inten[keep]
        if len(xyz) == 0:
            continue

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        m = poses.to(args.frame, msg.header.frame_id, t)
        if m is None:
            skipped += 1
            continue
        xyz = xyz.dot(m[:3, :3].T) + m[:3, 3]

        chunks_xyz.append(xyz.astype(np.float32))
        if inten is not None:
            chunks_i.append(inten)
        kept += 1
        total += len(xyz)

        # Downsample as we go once the buffer is large, so a long walk does not
        # need every raw point in memory at once. 137 s at 32,000 points a
        # frame is 44 million points -- over a gigabyte before filtering.
        if args.voxel > 0 and total > args.chunk:
            chunks_xyz, chunks_i = _squash(chunks_xyz, chunks_i, args.voxel)
            total = sum(len(c) for c in chunks_xyz)
        if kept % 100 == 0:
            print("  %d frames, %s points so far" % (kept, "{:,}".format(total)))

    if not chunks_xyz:
        sys.exit("no usable clouds -- %d skipped for want of a transform to %s"
                 % (skipped, args.frame))

    xyz = np.concatenate(chunks_xyz)
    inten = np.concatenate(chunks_i) if chunks_i else None
    note = ("%d of %d frames from %s, frame %s%s"
            % (kept, seen, os.path.basename(path.rstrip("/")), args.frame,
               "" if poses.moving() else ", NO MOVING TRANSFORM"))
    if skipped:
        print("  %d frames had no transform to %s and were skipped"
              % (skipped, args.frame))
    return xyz, inten, note


def _squash(chunks_xyz, chunks_i, voxel):
    xyz = np.concatenate(chunks_xyz)
    extra = {"i": np.concatenate(chunks_i)} if chunks_i else {}
    xyz, extra = voxel_filter(xyz, extra, voxel)
    return [xyz], ([extra["i"]] if extra else [])


# ---------------------------------------------------------------------------
# From the live sensor
# ---------------------------------------------------------------------------
def from_live(args):
    """Collect for a few seconds off /lidar_points and stop.

    Deliberately simple: no tf listener, so the points come out in the sensor's
    own frame. A live snapshot is a picture of what the robot can see from
    where it is standing, which is exactly what that frame means. Accumulating
    a live walk into a map is what the bag path is for.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    collected = {"xyz": [], "i": [], "frames": 0, "frame_id": None}

    class Grab(Node):
        def __init__(self):
            super().__init__("export_ply_live")
            self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on, 10)

        def on(self, msg):
            cols = read_points(msg)
            if "x" not in cols:
                return
            xyz = np.stack([cols["x"], cols["y"], cols["z"]], axis=1)
            keep = ~np.all(np.abs(xyz) < 1e-6, axis=1)
            collected["xyz"].append(xyz[keep].astype(np.float32))
            if "intensity" in cols:
                collected["i"].append(cols["intensity"][keep])
            collected["frames"] += 1
            collected["frame_id"] = msg.header.frame_id
            print("\r  %d frames, %s points"
                  % (collected["frames"],
                     "{:,}".format(sum(len(c) for c in collected["xyz"]))),
                  end="")
            sys.stdout.flush()

    rclpy.init()
    node = Grab()
    print("listening on %s for %.0f s..." % (CLOUD_TOPIC, args.seconds))
    deadline = time.time() + args.seconds
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("")

    if not collected["xyz"]:
        sys.exit("nothing arrived on %s. Is the driver running?" % CLOUD_TOPIC)
    return (np.concatenate(collected["xyz"]),
            np.concatenate(collected["i"]) if collected["i"] else None,
            "%.0f s live capture, %d frames, frame %s"
            % (args.seconds, collected["frames"], collected["frame_id"]))


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="export a recorded or live point cloud as .ply")
    ap.add_argument("source", nargs="?",
                    help="a bag directory, eg ~/go2-lidar-mapping/walks/X/bag")
    ap.add_argument("--live", action="store_true",
                    help="capture from the running driver instead of a bag")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="how long to capture, with --live (default 10)")
    ap.add_argument("--out", default=None, help="output .ply")
    ap.add_argument("--frame", default="odom",
                    help="frame to place the points in (default odom)")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="grid size in metres, 0 to keep every point "
                         "(default 0.03)")
    ap.add_argument("--max-range", type=float, default=0.0,
                    help="drop points further than this from the sensor, in "
                         "metres. 0 keeps everything (default). A spinning "
                         "LiDAR indoors returns a long thin tail through "
                         "windows and off reflections -- on the first walk, "
                         "0.02%% of points beyond 80 m stretched the cloud's "
                         "extent to +-100 m while the median range was 6.8 m")
    ap.add_argument("--every", type=int, default=1,
                    help="use every Nth frame (default 1)")
    ap.add_argument("--color", choices=("intensity", "height", "none"),
                    default="intensity", help="what the colours mean")
    ap.add_argument("--ascii", action="store_true",
                    help="write a text ply. Much larger; for tiny clouds only")
    ap.add_argument("--chunk", type=int, default=8000000,
                    help="points to buffer before downsampling (default 8M)")
    args = ap.parse_args()

    if args.live == bool(args.source):
        sys.exit("give a bag directory, or --live. Not both, not neither.")

    t0 = time.time()
    xyz, inten, note = from_live(args) if args.live else from_bag(args)

    if args.max_range > 0:
        before = len(xyz)
        r = np.linalg.norm(xyz, axis=1)
        keep = r <= args.max_range
        xyz = xyz[keep]
        if inten is not None:
            inten = inten[keep]
        print("range filter at %.1f m: dropped %s of %s points"
              % (args.max_range, "{:,}".format(before - len(xyz)),
                 "{:,}".format(before)))

    raw = len(xyz)
    if args.voxel > 0:
        extra = {"i": inten} if inten is not None else {}
        xyz, extra = voxel_filter(xyz, extra, args.voxel)
        inten = extra.get("i")
        print("voxel filter at %.3f m: %s -> %s points"
              % (args.voxel, "{:,}".format(raw), "{:,}".format(len(xyz))))

    rgb = None
    if args.color == "intensity" and inten is not None:
        rgb = colourise(inten, args.color)
    elif args.color == "height":
        rgb = colourise(xyz[:, 2], args.color)

    out = args.out
    if not out:
        if args.live:
            out = time.strftime("live-%Y%m%d-%H%M%S.ply")
        else:
            base = os.path.dirname(os.path.abspath(args.source.rstrip("/")))
            out = os.path.join(base, os.path.basename(base) + ".ply")

    write_ply(out, xyz, inten, rgb, ascii_mode=args.ascii, note=note)

    size = os.path.getsize(out)
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    print("")
    print("wrote %s  (%.1f MB, %s points, %.0f s)"
          % (out, size / 1e6, "{:,}".format(len(xyz)), time.time() - t0))
    print("  extent  x %.2f..%.2f   y %.2f..%.2f   z %.2f..%.2f"
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    # The extent on its own is misleading. A handful of returns through a
    # window put the corners a hundred metres out while everything you care
    # about is within ten, and the cloud then opens zoomed so far back that it
    # looks empty. Say where the bulk of it actually is.
    rng = np.linalg.norm(xyz, axis=1)
    print("  range   median %.1f m, 99%% within %.1f m, furthest %.1f m"
          % (np.median(rng), np.percentile(rng, 99), rng.max()))
    if np.percentile(rng, 99) * 3 < rng.max():
        print("          a thin tail of distant returns stretches the extent."
              " --max-range %.0f trims it." % (np.percentile(rng, 99) + 1))
    # A cloud that is flat in z is the missing-calibration fault, and it is
    # invisible from above. Say it here rather than leaving it to be noticed.
    if hi[2] - lo[2] < 0.2:
        print("  !! z spans %.2f m -- this cloud is FLAT. The per-laser angle"
              % (hi[2] - lo[2]))
        print("     calibration was missing when the bag was built. See")
        print("     get_lidar_calibration.py.")
    print("")
    print("  Open it: CloudCompare, MeshLab or Blender. On Windows the path is")
    print("    %s" % _win(out))


def _win(p):
    if p.startswith("/mnt/") and len(p) > 6 and p[6] == "/":
        return p[5].upper() + ":" + p[6:].replace("/", "\\")
    return p


if __name__ == "__main__":
    main()
