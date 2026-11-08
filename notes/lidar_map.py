#!/usr/bin/env python3
"""
Build a sharp 3D map and the robot's track from a walk, by scan matching.

    python3 lidar_map.py ~/go2-lidar-mapping/walks/odom-test-1
    python3 lidar_map.py <session> --max-range 30 --voxel 0.05

Needs the walk converted first (walk_to_bag.sh). Writes into <session>/map3d/:

    <name>_map.ply       the whole walk as one coloured cloud, with the track
                         drawn through it in white. CloudCompare, MeshLab.
    <name>_map.mcap      for Foxglove. Scrub to any moment and see everything
                         mapped up to then, the track so far, and the current
                         scan:
                           /map_so_far     the map up to this moment (1 per s)
                           /track          the scan-matched, loop-closed track
                           /track_frontend the same before loop closure
                           /track_odom     Unitree's own odometry, for comparison
                           /scan           the current scan, thinned
                           /tf             map -> base_link, scan-matched
    trajectory.csv       time and pose of every scan (loop closed)
    trajectory_frontend.csv   the same before loop closure
    build.json           settings used, and loop closure's report

WHY THE MAP WAS SMEARED
-----------------------
export_ply.py places every scan using Unitree's leg odometry alone. That drifts:
a degree of heading error puts a wall 20 m away 35 cm out of place, and over a
five-minute walk the heading wanders by several degrees. Every scan is then a
slightly shifted copy of the room, and the copies blur together.

WHAT THIS DOES INSTEAD
----------------------
For each scan, start from where the leg odometry says the robot moved, then
slide and turn the scan until it lines up with the map built so far (ICP,
point to plane, Open3D). The odometry supplies a good first guess; the walls
decide the final answer. This is LiDAR odometry, the core of what LIO-SAM and
similar systems do.

It matches against its last 25 keyframes only, so what it gets slightly wrong
is never corrected by what it saw earlier: on odom-test-1 the corridor's
ceiling came out twice, 30-40 cm apart, on the way out and the way back. Once
every scan is matched, pose_graph.py closes the loops (keyframes that come
back to a place are aligned to it, and the whole track is adjusted to agree),
and only then are the map and the Foxglove file written. --no-loop-closure
writes the front end's track as before, for comparison.

HEIGHT, AND THE DEPTH CAMERA (2026-09-25, odom-test-2)
--------------------------------------------------------
On the flat, height moves as the legs' body_height says (body_height_step);
on stairs and ramps it is scan matched, and then the depth camera corrects it
(DepthHeight): the LiDAR decides the pose, the camera's floor within 1.5 m
then shifts it up or down only. Measured against the step before (height
from the odometry's pz, no depth in matching), over rebuilds with LiDAR
ranges 29.5, 30 and 30.5 m (see below why several):

    track below the floor at the foot of the stairs, up   5.7 cm -> 0.6-2.1
                                                  down    3.3 cm -> 2.7-2.8
    height noise on the stairs, 90th percentile   2.6 / 4.2 -> 1.8-2.4 / 3.4-3.8
    flat floors                                   no change beyond the noise

Tried and rejected:
* Depth points added to the ICP beside the LiDAR's (--depth-matching full),
  to 2.5 or 1.5 m: floors came out thicker (LiDAR floor 2.4 -> 2.9-3.4 cm,
  ceiling far-minus-near 0 -> +6 cm). The camera sees a patch of floor about
  1 m deep: it pins height to a centimetre but tilt only to half a degree,
  and a tilt error grows with the LiDAR's range. Hence height only.
* Height from scan matching everywhere with depth (--vertical free): no dip
  at the stairs, but thicker floors (2.1 -> 3.2 cm) and the ceiling +8 cm.
* The camera's height correction on every scan, not just stairs: ceiling
  far-minus-near +64 cm, end error 3.9 cm.
* The IMU's fast tilt with the track's slow tilt (a complementary filter,
  0.5-2 s): no thinner floor, passes disagreeing more (worst 4 -> 10 cm).
* A clock offset between camera and LiDAR: the fit is best at 0 ms.

The builds are close to repeatable but sensitive: rebuilt unchanged they
differed by up to 0.2 cm in floor-only thickness and 0.2 degrees in the end
error's tilt (Open3D's threads), and a change that should not matter (LiDAR
range 29.5 instead of 30 m) moves the floor-only thickness by 0.3-0.6 cm,
the dip at the stairs by 1-1.5 cm and the end error's tilt between 0.3 and
1.1 degrees. Compare settings over a few such rebuilds, not one.

LiDAR mounting from Unitree's documentation, the same as walk_to_bag.sh.
"""
import argparse
import array
import csv
import os
import struct
import sys
import time

import numpy as np
import open3d as o3d
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.serialization import serialize_message
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

LIDAR_IN_BASE = (0.171, 0.0, 0.0908)          # Unitree docs, no rotation
POINT_DTYPE = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                        "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                        "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})
LIDAR_MOUNT_FILE = "lidar_mount.json"


def find_lidar_mount(session):
    """base_link -> hesai_lidar: lidar_mount.json (walk folder, then walks
    folder) if lidar_mount.py has calibrated one, else Unitree's documented
    position with no rotation. Returns (4x4, description)."""
    import json
    # GO2_CONFIG_DIR last: the Docker image keeps this robot's calibrations there.
    for folder in (session, os.path.dirname(os.path.abspath(session)),
                   os.environ.get("GO2_CONFIG_DIR", "")):
        if not folder:
            continue
        path = os.path.join(folder, LIDAR_MOUNT_FILE)
        if os.path.isfile(path):
            with open(path) as fh:
                m = json.load(fh)
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler("xyz", m.get("rpy_deg", [0, 0, 0]),
                                            degrees=True).as_matrix()
            T[:3, 3] = m.get("xyz", LIDAR_IN_BASE)
            return T, "%s (roll %.2f, pitch %.2f, yaw %.1f deg)" % (
                path, m["rpy_deg"][0], m["rpy_deg"][1], m["rpy_deg"][2])
    return pose_matrix(LIDAR_IN_BASE, (0, 0, 0, 1)), "Unitree's documented position, no tilt"


def deskew(xyz, dt, t, odom, T_bl, slices=10):
    """Move every point to where the LiDAR was at the start of its scan.

    One turn of the XT16 takes 0.1 s, and each point carries its own time
    (dt, seconds after the scan's timestamp, which marks the start of the
    turn). Walking at 1 m/s the robot moves 10 cm and sways during a turn, so
    points from the end of a turn land displaced from those at its start.
    The motion within the turn comes from Unitree's odometry at 300 Hz, in
    `slices` steps across the 0.1 s.
    """
    T0_inv = np.linalg.inv(odom.at(t) @ T_bl)
    out = np.empty_like(xyz)
    edges = np.linspace(0.0, max(float(dt.max()), 1e-3) + 1e-9, slices + 1)
    which = np.clip(np.searchsorted(edges, dt, side="right") - 1, 0, slices - 1)
    for k in range(slices):
        m = which == k
        if not m.any():
            continue
        T = T0_inv @ odom.at(t + 0.5 * (edges[k] + edges[k + 1])) @ T_bl
        out[m] = (T[:3, :3] @ xyz[m].T).T + T[:3, 3]
    return out


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------
def pose_matrix(p, q_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
    T[:3, 3] = p
    return T


def voxel_keep(xyz, size, extra=None):
    """One point per voxel. Returns (xyz, extra)."""
    if len(xyz) == 0:
        return xyz, extra
    k = np.floor(xyz / size).astype(np.int64)
    k -= k.min(axis=0)
    span = k.max(axis=0) + 1
    key = (k[:, 0] * span[1] + k[:, 1]) * span[2] + k[:, 2]
    _, idx = np.unique(key, return_index=True)
    return xyz[idx], (extra[idx] if extra is not None else None)


def stamp_msg(t):
    s = Time()
    s.sec = int(t)
    s.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
    return s


def cloud_msg(xyz, values, frame, t, value_name="intensity"):
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp_msg(t)
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                  for n, o in (("x", 0), ("y", 4), ("z", 8), (value_name, 12))]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(xyz)
    msg.is_dense = True
    buf = np.empty((len(xyz), 4), dtype="<f4")
    buf[:, :3] = xyz
    buf[:, 3] = values
    msg.data = array.array("B", buf.tobytes())
    return msg


def path_msg(poses, t, stride=1):
    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = stamp_msg(t)
    for (pt, T) in poses[::stride]:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = stamp_msg(pt)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, T[:3, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = map(float, q)
        msg.poses.append(ps)
    return msg


def tf_msg(parent, child, T, t):
    tr = TransformStamped()
    tr.header.frame_id = parent
    tr.header.stamp = stamp_msg(t)
    tr.child_frame_id = child
    tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = \
        map(float, T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    (tr.transform.rotation.x, tr.transform.rotation.y,
     tr.transform.rotation.z, tr.transform.rotation.w) = map(float, q)
    m = TFMessage()
    m.transforms.append(tr)
    return m


def height_colours(z):
    """Blue low -> green -> yellow -> red high, from the 2nd to 98th percentile."""
    lo, hi = np.percentile(z, 2), np.percentile(z, 98)
    v = np.clip((z - lo) / max(hi - lo, 1e-3), 0, 1)
    r = np.clip(1.5 - np.abs(4 * v - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * v - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * v - 1), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def write_ply(path, xyz, rgb):
    with open(path, "wb") as fh:
        fh.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
                  "property float x\nproperty float y\nproperty float z\n"
                  "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                  "end_header\n" % len(xyz)).encode("ascii"))
        rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                        ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        fh.write(rec.tobytes())


# --------------------------------------------------------------------------
# odometry
# --------------------------------------------------------------------------
class Odometry(object):
    def __init__(self, path):
        t, p, q = [], [], []
        with open(path) as fh:
            for row in csv.DictReader(fh):
                t.append(float(row["host_time"]))
                p.append((float(row["px"]), float(row["py"]), float(row["pz"])))
                q.append((float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])))
        self.t = np.array(t)
        order = np.argsort(self.t)
        self.t = self.t[order]
        keep = np.concatenate([[True], np.diff(self.t) > 0])
        self.t = self.t[keep]
        self.p = np.array(p)[order][keep]
        self.slerp = Slerp(self.t, Rotation.from_quat(np.array(q)[order][keep]))
        # Body pitch in degrees from the IMU, for telling stairs from flat floor.
        self.pitch = Rotation.from_quat(np.array(q)[order][keep]).as_euler("ZYX", degrees=True)[:, 1]
        # The body's height above its feet, as the legs measure it (pz if absent).
        with open(path) as fh:
            h = [float(r.get("body_height") or "nan") for r in csv.DictReader(fh)]
        h = np.array(h)[order][keep]
        self.height = np.where(np.isfinite(h), h, self.p[:, 2])

    def height_at(self, t):
        return float(np.interp(t, self.t, self.height))

    def pitch_near(self, t, half_window=0.5):
        """Median body pitch over +-half_window s: one step's lurch is ignored."""
        i, j = np.searchsorted(self.t, [t - half_window, t + half_window])
        if j <= i:
            i, j = max(0, i - 1), min(len(self.t), i + 1)
        return float(np.median(self.pitch[i:j]))

    def at(self, t):
        t = min(max(t, self.t[0]), self.t[-1])
        p = np.array([np.interp(t, self.t, self.p[:, i]) for i in range(3)])
        T = np.eye(4)
        T[:3, :3] = self.slerp([t]).as_matrix()[0]
        T[:3, 3] = p
        return T


# --------------------------------------------------------------------------
# timing
# --------------------------------------------------------------------------
class Timers(object):
    """Seconds spent per stage, printed at the end, to see where time goes."""

    def __init__(self):
        self.total = {}

    def __call__(self, name):
        timers = self

        class _T(object):
            def __enter__(self):
                self.t = time.perf_counter()

            def __exit__(self, *_):
                timers.total[name] = timers.total.get(name, 0.0) + time.perf_counter() - self.t
        return _T()

    def report(self):
        return ", ".join("%s %.0f s" % (k, v) for k, v in
                         sorted(self.total.items(), key=lambda kv: -kv[1]) if v >= 0.5)


# --------------------------------------------------------------------------
# pose helpers
# --------------------------------------------------------------------------
def angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def keep_height(T_match, T_odom):
    """Everything from scan matching except height, which comes from the legs."""
    T = np.array(T_match)
    T[2, 3] = T_odom[2, 3]
    return T


def flat_height(T_match, T_prev, T_prev_odom, T_odom):
    """Scan matching for everything but height; height moves only as the legs say.

    Relative to the previous pose, not absolute: after a flight of stairs the
    robot is a floor up while its leg odometry still reads the same height.
    (--flat-height odom; see body_height_step for the default.)
    """
    T = np.array(T_match)
    T[2, 3] = T_prev[2, 3] + (T_odom[2, 3] - T_prev_odom[2, 3])
    return T


def body_height_step(T_match, T_prev, h_prev, h_now, T_lb):
    """Like flat_height, but the body (base_link) rises or falls by exactly
    as much as its height above the feet changed, and the LiDAR, 17 cm in
    front of the body's middle, follows from the matched tilt.

    Why not the odometry's own height (pz): on odom-test-2, as the robot
    pitched nose-up for the first stair (74.0 s), pz fell 10 cm while
    body_height stayed at 0.30 m, and the climbing detector (a 1 s median of
    pitch) switched to scan-matched height only 0.3 s later, by when the
    track had copied a 7 cm dip that it kept up the whole flight.
    """
    T = np.array(T_match)
    want = (T_prev @ T_lb)[2, 3] + (h_now - h_prev)
    T[2, 3] = want - T[2, :3] @ T_lb[:3, 3]
    return T


def keep_gravity(T_match, T_odom):
    """Heading and x, y from scan matching; roll, pitch and height from odometry.

    Roll and pitch in Unitree's odometry come from the IMU, which measures
    gravity directly and does not drift. Its height is the legs' own. Scan
    matching in a corridor, by contrast, is poorly constrained vertically:
    left free it drifted 0.6 m up over odom-test-1, which thickens the floor.
    Level floors only: pass --free-z for stairs or ramps.
    """
    yaw = Rotation.from_matrix(T_match[:3, :3]).as_euler("ZYX")[0]
    _, pitch, roll = Rotation.from_matrix(T_odom[:3, :3]).as_euler("ZYX")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
    T[:2, 3] = T_match[:2, 3]
    T[2, 3] = T_odom[2, 3]
    return T


def interpolate(t, t0, T0, t1, T1):
    a = 0.0 if t1 <= t0 else min(max((t - t0) / (t1 - t0), 0.0), 1.0)
    T = np.eye(4)
    T[:3, 3] = T0[:3, 3] + a * (T1[:3, 3] - T0[:3, 3])
    rots = Rotation.from_matrix(np.stack([T0[:3, :3], T1[:3, :3]]))
    T[:3, :3] = Slerp([0.0, 1.0], rots)([a]).as_matrix()[0]
    return T


def image_msg(img, frame, t):
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header.frame_id = frame
    msg.header.stamp = stamp_msg(t)
    msg.height, msg.width = img.shape
    msg.encoding = "16UC1"
    msg.is_bigendian = 0
    msg.step = img.shape[1] * 2
    msg.data = array.array("B", img.astype("<u2").tobytes())
    return msg


# --------------------------------------------------------------------------
# depth camera
# --------------------------------------------------------------------------
class DepthOnMap(object):
    """The walk's depth.stream, placed on the map as the track is built.

    Each depth frame is placed at the robot's pose at its own timestamp,
    interpolated between the two LiDAR scans either side, then through
    camera_mount.json (base_link -> camera_link, measured and refined) and
    Intel's factory camera_link -> optical frame.
    """

    def __init__(self, session, args, put):
        self.args = args
        self.put = put
        self.frames = []
        self.colors = []
        self.placeable = False
        self.why = None
        self.i = self.ci = 0
        self.every = max(1, int(round(15.0 / args.depth_rate)))
        self.pending = []
        self.so_far = np.zeros((0, 3), np.float32)
        self.ply_chunks = []
        self.placed = 0
        self.rays = None
        self.statics = None
        self.optical = None
        self.points_shown = 0
        path = os.path.join(session, "depth.stream")
        if args.no_depth:
            self.why = "--no-depth"
            return
        if not os.path.isfile(path):
            self.why = "no depth.stream in this walk (the camera was off)"
            return
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from depth_stream import DepthStream, find_camera_mount
        ds = DepthStream(path)
        self.frames = ds.depth_frames()
        self.colors = ds.color_frames()
        if not self.frames:
            self.why = "depth.stream has no depth frames"
            return
        info = ds.depth_info
        self.optical = self.frames[0].header.get("frame_id") or "camera_depth_optical_frame"
        T_co = ds.optical_in_camera_link(self.optical)
        T_bc, mount, mount_path = find_camera_mount(session)
        # Depth points in the camera's own frame need only the calibration and
        # Intel's frames, not the measured mount: shown from the camera's point
        # of view until the mount is known, and on the map after that.
        if info is not None:
            K = info["K"]
            self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]
            self.statics = list(ds.transforms.values())
            if T_bc is not None:
                q = Rotation.from_matrix(T_bc[:3, :3]).as_quat()
                self.statics.append({"parent": "base_link", "child": "camera_link",
                                     "t": list(T_bc[:3, 3]), "q": list(q)})
        if info is None:
            self.why = "no camera calibration in depth.stream"
        elif T_co is None:
            self.why = "the camera's own frames are missing from depth.stream"
        elif T_bc is None:
            self.why = ("no camera_mount.json: depth frames go into the file as images, "
                        "but cannot be placed on the map until the camera is measured "
                        "(depth_mount.py)")
        else:
            self.placeable = True
            self.T_bo = T_bc @ T_co                   # optical frame in base_link
            self.mount = "%s (%s)" % (mount_path, mount.get("source", "no source noted"))

    def points(self, img):
        s = self.args.depth_step
        d = img[::s, ::s].astype(np.float32) / 1000.0
        if self.rays is None or self.rays[0].shape != d.shape:
            v, u = np.mgrid[0:img.shape[0]:s, 0:img.shape[1]:s].astype(np.float32)
            self.rays = ((u - self.cx) / self.fx, (v - self.cy) / self.fy)
        ok = (d > 0.3) & (d < self.args.depth_max)
        z = d[ok]
        return np.stack([self.rays[0][ok] * z, self.rays[1][ok] * z, z], axis=1)

    def publish_statics(self, t):
        """The camera's own frames, once, so /depth_points can be drawn."""
        from geometry_msgs.msg import TransformStamped
        msg = TFMessage()
        for tr in self.statics:
            m = TransformStamped()
            m.header.frame_id, m.child_frame_id = tr["parent"], tr["child"]
            m.header.stamp = stamp_msg(t)
            (m.transform.translation.x, m.transform.translation.y,
             m.transform.translation.z) = (float(v) for v in tr["t"])
            (m.transform.rotation.x, m.transform.rotation.y,
             m.transform.rotation.z, m.transform.rotation.w) = (float(v) for v in tr["q"])
            msg.transforms.append(m)
        self.put("/tf_static", msg, t)
        self.statics = []

    def advance(self, t0, T0, t1, T1):
        """Every depth and colour frame stamped between two LiDAR scans."""
        if self.statics:
            self.publish_statics(t0)
        while self.i < len(self.frames) and self.frames[self.i].t <= t1:
            f = self.frames[self.i]
            self.i += 1
            if f.t < t0 or (self.i - 1) % self.every:
                continue
            img = f.image
            s = self.args.depth_step
            preview = img[::s, ::s]
            self.put("/depth_preview", image_msg(preview, f.header.get("frame_id") or
                                                 "camera_depth_optical_frame", f.t), f.t)
            if self.statics is None:                # no calibration
                continue
            pts = self.points(img)
            if len(pts) == 0:
                continue
            near, _ = voxel_keep(pts, 0.03)
            # Coloured by distance from the camera.
            self.put("/depth_points", cloud_msg(near, near[:, 2], self.optical, f.t), f.t)
            self.points_shown += 1
            if not self.placeable:
                continue
            T_mo = interpolate(f.t, t0, T0, t1, T1) @ self.T_bo
            world = (T_mo[:3, :3] @ pts.T).T + T_mo[:3, 3]
            world, _ = voxel_keep(world, 0.05)
            self.put("/depth_scan", cloud_msg(world, world[:, 2], "map", f.t), f.t)
            self.pending.append(world.astype(np.float32))
            self.ply_chunks.append(world.astype(np.float32))
            self.placed += 1
        while self.ci < len(self.colors) and self.colors[self.ci].t <= t1:
            f = self.colors[self.ci]
            self.ci += 1
            if f.t < t0 or (self.ci - 1) % self.every:
                continue
            from sensor_msgs.msg import CompressedImage
            msg = CompressedImage()
            msg.header.frame_id = f.header.get("frame_id") or "camera_color_optical_frame"
            msg.header.stamp = stamp_msg(f.t)
            msg.format = "jpeg"
            msg.data = array.array("B", f.jpeg)
            self.put("/color/compressed", msg, f.t)

    def publish_so_far(self, t, voxel):
        if not self.placeable:
            return
        if self.pending:
            self.so_far, _ = voxel_keep(np.concatenate([self.so_far] + self.pending), voxel)
            self.pending = []
        self.put("/depth_so_far", cloud_msg(self.so_far, self.so_far[:, 2], "map", t), t)

    def ply_points(self, voxel):
        if not self.ply_chunks:
            return np.zeros((0, 3), np.float32)
        xyz, _ = voxel_keep(np.concatenate(self.ply_chunks), voxel)
        return xyz


class DepthHeight(object):
    """Height from the depth camera: the one number the LiDAR is worst at.

    The XT16 sees no floor within 1.5 m, and on stairs its scan-matched height
    jitters (90th percentile 3-4 cm between scans on odom-test-2). The D435i
    sees exactly there, and close up it is precise: within one frame the floor
    scatters by 0.4 cm at 0.5-1 m and 0.9 cm at 1-1.5 m (IQR), against 3.7 cm
    at 2-2.5 m. But a 1 m patch of floor pins tilt poorly (1 cm of noise
    across it is 0.6 degrees), and tilt errors grow with the LiDAR's range:
    with depth points simply added to the ICP (--depth-matching full) the
    floor came out thicker (LiDAR floor 2.3 -> 2.9-3.4 cm). So the LiDAR
    decides everything, and then the camera moves the scan up or down only.

    Kept: the upward-facing depth points of the last 25 keyframes, placed
    with their final poses. correction(T, depth): the vertical shift that best
    lays this frame's upward-facing points onto them (point to plane, robust,
    a few Gauss-Newton steps), or None when too little floor is in common.
    """

    def __init__(self, keep=25, voxel=0.04):
        self.keep, self.voxel = keep, voxel
        self.frames = []
        self.tree = None

    def add(self, world):
        from scipy.spatial import cKDTree
        self.frames.append(voxel_keep(world, self.voxel)[0])
        self.frames = self.frames[-self.keep:]
        pts, _ = voxel_keep(np.concatenate(self.frames), self.voxel)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=20))
        n = np.asarray(pc.normals)
        up = np.abs(n[:, 2]) > 0.8
        if up.sum() < 100:
            return
        self.pts, self.n = pts[up], n[up]
        self.tree = cKDTree(self.pts)

    def correction(self, T, dxyz, limit=0.15):
        if self.tree is None or len(dxyz) < 50:
            return None
        pts, _ = voxel_keep(dxyz, 0.05)
        w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        dist, idx = self.tree.query(w, distance_upper_bound=limit)
        ok = np.isfinite(dist)
        if ok.sum() < 100:
            return None
        n = self.n[idx[ok]]
        r = np.einsum("ij,ij->i", w[ok] - self.pts[idx[ok]], n)
        nz = n[:, 2]
        dz = 0.0
        for _ in range(6):
            e = r + nz * dz
            wt = np.where(np.abs(e) < 0.05, (1 - (e / 0.05) ** 2) ** 2, 0.0)
            if wt.sum() < 30:
                return None
            dz -= float(np.sum(wt * nz * e) / np.sum(wt * nz * nz))
        return dz if abs(dz) < limit else None


def keep_previous(out):
    """Move the last build's files into map3d/previous/<when it was built>/.

    A rebuild never deletes the map it replaces: settings change, and the
    earlier result may have been the better one.
    """
    import shutil
    items = [f for f in os.listdir(out) if f != "previous"]
    if not items:
        return None
    built = max(os.path.getmtime(os.path.join(out, f)) for f in items)
    dest = os.path.join(out, "previous", time.strftime("%Y-%m-%d_%H%M%S", time.localtime(built)))
    os.makedirs(dest, exist_ok=True)
    for f in items:
        shutil.move(os.path.join(out, f), os.path.join(dest, f))
    return dest


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="scan-matched 3D map and track from a walk")
    ap.add_argument("session")
    ap.add_argument("--max-range", type=float, default=30.0,
                    help="metres; returns further than this are left out of the map")
    ap.add_argument("--voxel", type=float, default=0.05, help=".ply point spacing, metres")
    ap.add_argument("--foxglove-voxel", type=float, default=0.10,
                    help="point spacing of /map_so_far in the Foxglove file")
    ap.add_argument("--vertical", choices=["auto", "height", "gravity", "free"], default="auto",
                    help="how height is found. auto (default): from scan matching while "
                         "the body is tilted (stairs, ramps), from the legs on the flat. "
                         "height: always the legs' absolute height (one floor only). "
                         "gravity: height, roll and pitch from odometry. free: scan "
                         "matching alone")
    ap.add_argument("--incline", type=float, default=7.0,
                    help="auto: degrees of body pitch away from its level-ground value "
                         "that count as stairs or a ramp")
    ap.add_argument("--no-depth", action="store_true", help="leave the depth camera out")
    ap.add_argument("--depth-rate", type=float, default=5.0,
                    help="depth frames per second placed on the map and shown")
    ap.add_argument("--depth-step", type=int, default=4,
                    help="use every Nth depth pixel in each direction")
    ap.add_argument("--depth-max", type=float, default=4.0,
                    help="metres; the D435i's depth error grows quickly beyond this")
    ap.add_argument("--max-scans", type=int, default=0, help="stop early, for a quick look")
    ap.add_argument("--no-deskew", action="store_true",
                    help="treat each scan as instantaneous (for comparison)")
    ap.add_argument("--no-loop-closure", action="store_true",
                    help="keep the scan matcher's own track (pose_graph.py not run)")
    ap.add_argument("--flat-height", choices=["body", "odom"], default="body",
                    help="--vertical auto on the flat: height changes from the legs' body height "
                         "(default) or from the odometry's own height, pz (before 2026-09-25)")
    ap.add_argument("--depth-matching", choices=["height", "full", "off"], default="height",
                    help="height: the LiDAR matches, then the depth camera's floor sets the height "
                         "(see DepthHeight). full: depth points join the LiDAR's in the matching "
                         "(thickened floors). off: depth on the map only")
    ap.add_argument("--depth-height", choices=["stairs", "everywhere"], default="stairs",
                    help="with --depth-matching height: correct the height on stairs and ramps "
                         "only, or on every matched scan")
    ap.add_argument("--match-depth-max", type=float, default=1.5,
                    help="metres; depth used for scan matching (stereo error grows with distance squared)")
    ap.add_argument("--match-depth-step", type=int, default=8, help="every Nth depth pixel for matching")
    ap.add_argument("--match-depth-voxel", type=float, default=0.10,
                    help="grid the depth points are thinned to for matching, metres")
    args = ap.parse_args()

    session = os.path.abspath(args.session)
    name = os.path.basename(session)
    bag = os.path.join(session, "bag")
    out = os.path.join(session, "map3d")
    os.makedirs(out, exist_ok=True)
    kept = keep_previous(out)
    if kept:
        print("  previous map kept in %s" % kept, flush=True)
    if not os.path.isfile(os.path.join(session, "odom.csv")):
        sys.exit("no odom.csv in %s" % session)
    odom = Odometry(os.path.join(session, "odom.csv"))
    T_bl, mount_text = find_lidar_mount(session)
    T_lb = np.linalg.inv(T_bl)
    print("  LiDAR mount: %s" % mount_text, flush=True)
    timers = Timers()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lidar_points"]))
    total = 0
    for info in reader.get_metadata().topics_with_message_count:
        if info.topic_metadata.name == "/lidar_points":
            total = info.message_count
    if args.max_scans:
        total = min(total, args.max_scans)
    print("  %d scans to align" % total, flush=True)
    # The body's pitch on level ground, as the IMU reports it (about 2 degrees
    # on this robot, not 0). Stairs read 12 to 26 degrees away from it.
    level_pitch = float(np.median(odom.pitch))
    climbing_scans = 0

    mcap_dir = os.path.join(out, name + "_map")
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=mcap_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    created = set()

    def put(topic, msg, t):
        if topic not in created:
            created.add(topic)
            writer.create_topic(rosbag2_py.TopicMetadata(
                topic, type(msg).__module__.split(".")[0] + "/msg/" + type(msg).__name__, "cdr"))
        with timers("writing the Foxglove file"):
            writer.write(topic, serialize_message(msg), int(t * 1e9))

    depth = DepthOnMap(session, args, put)
    if depth.placeable:
        print("  depth camera: %d frames, placed on the map at %.0f per second; mount %s"
              % (len(depth.frames), args.depth_rate, depth.mount), flush=True)
    elif depth.frames:
        print("  depth camera: %d frames, shown as images only: %s"
              % (len(depth.frames), depth.why), flush=True)
    else:
        print("  depth camera: %s" % depth.why, flush=True)

    # State
    local_frames = []                 # recent keyframes, world coordinates
    target = None
    T_prev_corr = T_prev_odom = None
    T_last_key = None
    last_match = None                 # (T corrected, T odom) at the last real match
    n = matched = held = 0
    started = time.time()
    icp = o3d.pipelines.registration
    first_t = None
    # What the outputs need, per scan. The map and the Foxglove file are
    # written after loop closure has corrected the track, not as scans arrive.
    scan_t, scan_T = [], []           # time, LiDAR pose from the scan matcher
    scan_thin = []                    # the scan thinned for /scan, LiDAR frame
    key_cloud = {}                    # keyframes: scan index -> (xyz, intensity), LiDAR frame

    # Reading, filtering and deskewing each scan does not depend on scan
    # matching, so it runs ahead in worker processes on the other cores while
    # this process matches scans in order.
    import multiprocessing as mp
    PREP.update(odom=odom, T_bl=T_bl, max_range=args.max_range, deskew=not args.no_deskew,
                depth=None)
    depth_matching = depth.placeable and args.depth_matching != "off"
    depth_height = DepthHeight() if depth_matching and args.depth_matching == "height" else None
    if depth_matching:
        PREP["depth"] = dict(times=np.array([f.t for f in depth.frames]), frames=depth.frames,
                             K=(depth.fx, depth.fy, depth.cx, depth.cy), T_bo=depth.T_bo,
                             step=args.match_depth_step, max=args.match_depth_max)
        if depth_height is not None:
            print("  depth camera sets the height%s: nearest frame within 40 ms, to %.1f m"
                  % (" on stairs and ramps" if args.depth_height == "stairs" else "",
                     args.match_depth_max), flush=True)
        else:
            print("  depth camera in the scan matching: nearest frame within 40 ms, to %.1f m, "
                  "every %dth pixel, %.2f m grid" % (args.match_depth_max, args.match_depth_step,
                                                    args.match_depth_voxel), flush=True)
    with_depth = 0
    height_moves = []
    pool = mp.get_context("fork").Pool(max(1, (os.cpu_count() or 2) - 1))
    stream = pool.imap(prepare_scan, _raw_scans(reader), chunksize=2)
    for prepared in stream:
        if args.max_scans and n >= args.max_scans:
            break
        if prepared is None:
            continue
        t, raw_count, xyz, inten, dxyz = prepared
        if first_t is None:
            first_t = t
            put("/tf_static", tf_msg("base_link", "hesai_lidar", T_bl, t), t)
        # The first message is usually a partial turn of the sensor. Starting
        # the map from it left a 114-point target that nothing could match.
        if raw_count < (10000 if target is None else 2000):
            continue

        T_odom = odom.at(t) @ T_bl                        # LiDAR pose by odometry
        if T_prev_corr is None:
            guess = T_odom
        else:
            guess = T_prev_corr @ np.linalg.inv(T_prev_odom) @ T_odom

        T = guess
        # Standing still: if the legs say the robot has moved under 2 cm and
        # half a degree since the last real match, keep that match. Matching
        # a stationary robot only adds jitter (a few cm over odom-test-1's
        # first 80 s) and costs time.
        still = False
        if last_match is not None:
            d = np.linalg.inv(last_match[1]) @ T_odom
            still = np.linalg.norm(d[:3, 3]) < 0.02 and angle_deg(d[:3, :3]) < 0.5
        if still:
            T = last_match[0] @ np.linalg.inv(last_match[1]) @ T_odom
            held += 1
        elif target is not None:
            with timers("scan matching"):
                src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
                src = src.voxel_down_sample(0.2)
                if depth_matching and depth_height is None and dxyz is not None and len(dxyz):
                    dsrc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(dxyz))
                    src += dsrc.voxel_down_sample(args.match_depth_voxel)
                    with_depth += 1
                if len(src.points) > 200:
                    r1 = icp.registration_icp(src, target, 1.0, guess,
                                              icp.TransformationEstimationPointToPlane(icp.TukeyLoss(k=0.5)),
                                              icp.ICPConvergenceCriteria(max_iteration=30))
                    r2 = icp.registration_icp(src, target, 0.3, r1.transformation,
                                              icp.TransformationEstimationPointToPlane(icp.TukeyLoss(k=0.15)),
                                              icp.ICPConvergenceCriteria(max_iteration=30))
                    step = np.linalg.inv(guess) @ r2.transformation
                    # Trust the match when it covers the scan well and does not
                    # leap away from what the legs reported.
                    if r2.fitness > 0.5 and np.linalg.norm(step[:3, 3]) < 0.5 \
                            and angle_deg(step[:3, :3]) < 8:
                        T = np.array(r2.transformation)     # Open3D's is read-only
                        if args.vertical == "gravity":
                            T = keep_gravity(T, T_odom)
                        elif args.vertical == "height":
                            T = keep_height(T, T_odom)
                        elif args.vertical == "auto":
                            # odom-test-2, 2026-09-17: up a flight of stairs and
                            # back, and the leg odometry's height never moved.
                            # The IMU's pitch did, by 12 to 26 degrees.
                            if abs(odom.pitch_near(t) - level_pitch) >= args.incline:
                                climbing_scans += 1
                            else:
                                if args.flat_height == "body":
                                    T = body_height_step(T, T_prev_corr, odom.height_at(t_prev),
                                                         odom.height_at(t), T_lb)
                                else:
                                    T = flat_height(T, T_prev_corr, T_prev_odom, T_odom)
                        if depth_height is not None and dxyz is not None and (
                                args.depth_height == "everywhere"
                                or abs(odom.pitch_near(t) - level_pitch) >= args.incline):
                            dz = depth_height.correction(T, dxyz)
                            if dz is not None:
                                T[2, 3] += dz
                                height_moves.append(dz)
                                with_depth += 1
                        matched += 1
                        last_match = (T, T_odom)

        T_prev_corr, T_prev_odom, t_prev = T, T_odom, t

        with timers("building the map"):
            # Keyframe: moved 0.4 m or turned 8 degrees since the last one.
            # Also every 10th scan until the map has a few, so a robot standing
            # still at the start still gets a solid map to match against.
            is_key = T_last_key is None or (len(local_frames) < 5 and n % 10 == 0)
            if not is_key:
                d = np.linalg.inv(T_last_key) @ T
                is_key = np.linalg.norm(d[:3, 3]) > 0.4 or angle_deg(d[:3, :3]) > 8
            if is_key:
                T_last_key = T
                world = (T[:3, :3] @ xyz.T).T + T[:3, 3]
                kf, _ = voxel_keep(world, 0.15)
                if dxyz is not None and len(dxyz):
                    dw = (T[:3, :3] @ dxyz.T).T + T[:3, 3]
                    if depth_height is not None:
                        # The camera's floor, kept apart, for setting the height.
                        depth_height.add(dw)
                    elif depth_matching:
                        # The camera's floor ahead joins the map scans match against.
                        kf = np.concatenate([kf, voxel_keep(dw, 0.15)[0]])
                local_frames.append(kf)
                local_frames = local_frames[-25:]
                merged, _ = voxel_keep(np.concatenate(local_frames), 0.15)
                target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(merged))
                target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.6, max_nn=20))
                kx, ki = voxel_keep(xyz, args.voxel, inten)
                key_cloud[len(scan_t)] = (kx.astype(np.float32), ki)
        with timers("thinning scans for Foxglove"):
            sx, si = voxel_keep(xyz, 0.08, inten)
            scan_thin.append((sx.astype(np.float32), si))
        scan_t.append(t)
        scan_T.append(T)

        n += 1
        if n % 200 == 0:
            print("  %d scans, %d matched, %d standing (%.0f%%), %.0f s" %
                  (n, matched, held, 100.0 * (matched + held) / n, time.time() - started),
                  flush=True)

    pool.terminate()
    if not scan_t:
        sys.exit("no LiDAR scans found in %s" % bag)

    # ---- loop closure --------------------------------------------------
    times = np.array(scan_t)
    front = np.array([T @ T_lb for T in scan_T])            # base_link, scan matcher
    loop_report = None
    if args.no_loop_closure:
        corrected = front
    else:
        import pose_graph

        def cloud_of(s):
            c = key_cloud[s][0].astype(np.float64)
            return (T_bl[:3, :3] @ c.T).T + T_bl[:3, 3]
        with timers("loop closure"):
            corrected, loop_report = pose_graph.correct(
                times, front, cloud_of,
                lambda tq: odom.slerp([min(max(tq, odom.t[0]), odom.t[-1])])[0],
                lambda tq: abs(odom.pitch_near(tq) - level_pitch) >= args.incline,
                keys=sorted(key_cloud))

    # ---- the map and the Foxglove file, from the final track -------------
    track, track_odom, track_front = [], [], []
    full_xyz, full_i = [], []         # accumulated map for the .ply, in chunks
    fox_xyz = np.zeros((0, 3))
    fox_i = np.zeros(0)
    pending_xyz, pending_i = [], []
    next_pub = scan_t[0]
    for s, t in enumerate(scan_t):
        T_base = corrected[s]
        T = T_base @ T_bl
        if track:
            with timers("depth camera"):
                depth.advance(track[-1][0], track[-1][1], t, T_base)
        track.append((t, T_base))
        track_odom.append((t, odom.at(t)))
        track_front.append((t, front[s]))
        if s in key_cloud:
            with timers("building the map"):
                kx, ki = key_cloud[s]
                world = (T[:3, :3] @ kx.T).T + T[:3, 3]
                full_xyz.append(world.astype(np.float32))
                full_i.append(ki)
                if sum(len(c) for c in full_xyz) > 4000000:
                    a, b = voxel_keep(np.concatenate(full_xyz), args.voxel, np.concatenate(full_i))
                    full_xyz, full_i = [a], [b]
                px, pi = voxel_keep(world, args.foxglove_voxel, ki)
                pending_xyz.append(px)
                pending_i.append(pi)

        put("/tf", tf_msg("map", "base_link", T_base, t), t)
        sx, si = scan_thin[s]
        put("/scan", cloud_msg(sx, si, "hesai_lidar", t), t)
        if t >= next_pub or s == len(scan_t) - 1:
            with timers("map so far"):
                if pending_xyz:
                    fox_xyz, fox_i = voxel_keep(np.concatenate([fox_xyz] + pending_xyz),
                                                args.foxglove_voxel,
                                                np.concatenate([fox_i] + pending_i))
                    pending_xyz, pending_i = [], []
            put("/map_so_far", cloud_msg(fox_xyz, fox_i, "map", t), t)
            with timers("depth camera"):
                depth.publish_so_far(t, args.foxglove_voxel)
            put("/track", path_msg(track, t, stride=5), t)
            put("/track_odom", path_msg(track_odom, t, stride=5), t)
            if loop_report is not None:
                put("/track_frontend", path_msg(track_front, t, stride=5), t)
            next_pub = t + 1.0
    del writer

    # trajectory.csv, and the scan matcher's own track beside it
    for fname, poses in (("trajectory.csv", track), ("trajectory_frontend.csv", track_front)):
        with open(os.path.join(out, fname), "w") as fh:
            fh.write("time,x,y,z,qx,qy,qz,qw,odom_x,odom_y,odom_z\n")
            for (t, T), (_, To) in zip(poses, track_odom):
                q = Rotation.from_matrix(T[:3, :3]).as_quat()
                fh.write("%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n"
                         % ((t,) + tuple(T[:3, 3]) + tuple(q) + tuple(To[:3, 3])))

    # What this build used, so map_quality.py can re-place scans the same way.
    import json
    with open(os.path.join(out, "build.json"), "w") as fh:
        json.dump({"lidar_xyz": [float(v) for v in T_bl[:3, 3]],
                   "lidar_rpy_deg": [float(v) for v in
                                     Rotation.from_matrix(T_bl[:3, :3]).as_euler("xyz", degrees=True)],
                   "lidar_mount": mount_text, "deskew": not args.no_deskew,
                   "vertical": args.vertical, "max_range": args.max_range, "voxel": args.voxel,
                   "loop_closure": loop_report,
                   "depth_matching": ({"mode": args.depth_matching, "where": args.depth_height,
                                       "max_m": args.match_depth_max, "step": args.match_depth_step,
                                       "voxel_m": args.match_depth_voxel, "scans_with_depth": with_depth}
                                      if depth_matching else None),
                   "flat_height": args.flat_height,
                   "built": time.strftime("%Y-%m-%d %H:%M:%S")}, fh, indent=2)

    # The LiDAR-only map, for depth_mount.py to fit the camera against.
    with timers("writing the .ply"):
        xyz, _ = voxel_keep(np.concatenate(full_xyz), args.voxel, np.concatenate(full_i))
        np.save(os.path.join(out, "lidar_map.npy"), xyz.astype(np.float32))
        depth_xyz = depth.ply_points(args.voxel)
        if len(depth_xyz):
            xyz = np.concatenate([xyz, depth_xyz])
            xyz, _ = voxel_keep(xyz, args.voxel)

        # .ply with the track drawn through it
        rgb = height_colours(xyz[:, 2])
        pos = np.array([T[:3, 3] for _, T in track])
        line = [pos[0]]
        for a, b in zip(pos[:-1], pos[1:]):
            k = max(1, int(np.linalg.norm(b - a) / 0.01))
            line.extend(a + (b - a) * (i / k) for i in range(1, k + 1))
        line = np.array(line)
        thick = np.concatenate([line + off for off in
                                ([0, 0, 0], [0.02, 0, 0], [-0.02, 0, 0], [0, 0.02, 0], [0, -0.02, 0])])
        xyz_all = np.concatenate([xyz, thick.astype(np.float32)])
        rgb_all = np.concatenate([rgb, np.full((len(thick), 3), 255, np.uint8)])
        ply = os.path.join(out, name + "_map.ply")
        write_ply(ply, xyz_all, rgb_all)

    length = float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)))
    odom_pos = np.array([T[:3, 3] for _, T in track_odom])
    print()
    print("  scans %d, matched to the map %d, standing still %d, %.0f s" %
          (n, matched, held, time.time() - started))
    print("  track %.1f m, ends %.2f m from its start (odometry alone: %.2f m)"
          % (length, np.linalg.norm(pos[-1] - pos[0]), np.linalg.norm(odom_pos[-1] - odom_pos[0])))
    how = {"auto": " (scan matching on %d tilted scans, legs elsewhere)" % climbing_scans,
           "height": " (from odometry)", "gravity": " (from odometry)", "free": ""}[args.vertical]
    print("  height along the track %.2f to %.2f m%s" % (pos[:, 2].min(), pos[:, 2].max(), how))
    if depth.placeable:
        print("  depth frames placed on the map: %d" % depth.placed)
    if depth_height is not None and height_moves:
        hm = 100 * np.abs(np.array(height_moves))
        print("  depth camera moved the height on %d scans: median %.1f cm, 90th percentile %.1f cm"
              % (len(hm), np.median(hm), np.percentile(hm, 90)))
    elif depth_matching:
        print("  scans matched with the depth camera's points too: %d" % with_depth)
    print("  time: %s" % timers.report())
    print("  wrote %s  (%d points%s)" % (ply, len(xyz_all),
                                         ", LiDAR and depth" if len(depth_xyz) else ""))
    print("  wrote %s" % mcap_dir)


PREP = {}      # for prepare_scan in worker processes (inherited by fork)


def _raw_scans(reader):
    while reader.has_next():
        _, raw, _ = reader.read_next()
        yield bytes(raw)


def prepare_scan(raw):
    """Read, filter and deskew one scan. Runs in a worker process."""
    sec, nsec = struct.unpack("<iI", raw[4:12])
    t = sec + nsec * 1e-9
    pts = _points(raw)
    if pts is None:
        return None
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
    inten = pts["intensity"].astype(np.float32)
    ok = np.isfinite(xyz).all(axis=1)
    rng = np.linalg.norm(xyz, axis=1)
    ok &= (rng > 0.8) & (rng < PREP["max_range"])
    xyz, inten = xyz[ok], inten[ok]
    stamps = pts["timestamp"][ok]
    if PREP["deskew"] and len(stamps):
        xyz = deskew(xyz, stamps - stamps.min(), t, PREP["odom"], PREP["T_bl"])
    return t, len(pts), xyz, inten, depth_for_scan(t)


def depth_for_scan(t):
    """The depth frame nearest the scan (within 40 ms; the camera runs at
    15 fps), in the LiDAR's frame at the scan's time: through the camera
    mount, and the legs' motion between the frame and the scan (at most
    33 ms). None without depth matching or a frame close enough."""
    D = PREP.get("depth")
    if D is None or not len(D["times"]):
        return None
    j = int(np.clip(np.searchsorted(D["times"], t), 1, len(D["times"]) - 1))
    j = j if abs(D["times"][j] - t) < abs(D["times"][j - 1] - t) else j - 1
    f = D["frames"][j]
    if abs(f.t - t) > 0.04:
        return None
    img = f.image
    s = D["step"]
    d = img[::s, ::s].astype(np.float32) / 1000.0
    v, u = np.mgrid[0:img.shape[0]:s, 0:img.shape[1]:s].astype(np.float32)
    ok = (d > 0.3) & (d < D["max"])
    if not ok.any():
        return None
    fx, fy, cx, cy = D["K"]
    z = d[ok]
    p = np.stack([(u[ok] - cx) / fx * z, (v[ok] - cy) / fy * z, z], axis=1).astype(np.float64)
    odom, T_bl = PREP["odom"], PREP["T_bl"]
    T = np.linalg.inv(odom.at(t) @ T_bl) @ odom.at(f.t) @ D["T_bo"]
    return (T[:3, :3] @ p.T).T + T[:3, 3]


def _points(raw):
    """PointCloud2 serialized bytes -> structured array of x, y, z, intensity.

    Walks the CDR layout of sensor_msgs/PointCloud2 far enough to reach the
    data blob, without building a Python message: header, height, width,
    fields, is_bigendian, point_step, row_step, data.
    """
    o = 4

    def align(o, n):                                    # CDR aligns from byte 4
        return o + ((-(o - 4)) % n)

    o += 8                                              # stamp
    ln = struct.unpack_from("<I", raw, o)[0]; o += 4 + ln          # frame_id
    o = align(o, 4)
    height, width = struct.unpack_from("<II", raw, o); o += 8
    nf = struct.unpack_from("<I", raw, o)[0]; o += 4
    for _ in range(nf):
        o = align(o, 4)
        ln = struct.unpack_from("<I", raw, o)[0]; o += 4 + ln       # name
        o = align(o, 4)
        o += 4                                          # offset
        o += 1                                          # datatype
        o = align(o, 4)
        o += 4                                          # count
    o += 1                                              # is_bigendian
    o = align(o, 4)
    point_step, row_step = struct.unpack_from("<II", raw, o); o += 8
    ln = struct.unpack_from("<I", raw, o)[0]; o += 4
    if point_step != 26:
        return None
    count = ln // point_step
    return np.frombuffer(raw, dtype=POINT_DTYPE, count=count, offset=o)


if __name__ == "__main__":
    main()
