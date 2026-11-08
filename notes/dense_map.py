#!/usr/bin/env python3
"""
A dense, cleaned point cloud of a walk, from every scan.

    python3 dense_map.py ~/go2-lidar-mapping/walks/odom-test-2
    python3 dense_map.py <walk> --voxel 0.03 --min-scans 2 --no-carve

Runs after lidar_map.py (process_walk.sh does both) and uses that build's
track and LiDAR mount (map3d/trajectory.csv, map3d/build.json), so no scan
matching is repeated. Writes into map3d/:

    <walk>_dense.ply      the cleaned cloud, coloured by height, track in white
    <walk>_dense/         the same for Foxglove: /dense_map and /track
    <walk>_colour.ply     the same cloud in the camera's colours where a colour
                          frame saw it (colour_map.py), with colour_source
    dense_colour.npy      r g b colour_source per point, dense_map.npy's order
    <walk>_removed.ply    what cleaning removed, by colour: grey seen by too few
                          scans, red seen through (moving or false), magenta
                          isolated specks, orange outside the building. Open with the dense .ply to check the
                          cleaning took noise and not real surfaces.
    dense_map.npy         the cleaned points, for map_quality.py
    dense_report.json     how many points each step kept or removed

Depth camera frames are added too when the walk has depth.stream and the
camera's place on the robot is known (camera_mount.json, from depth_mount.py).
The XT16 cannot see the floor within about 1.5 m of the robot, nor low
obstacles between its 16 laser rings; the camera looks exactly there.
--no-depth leaves it out.

Depth is used out to 1.5 m only (--depth-max; 4.0 until 2026-09-25). Within
one depth frame, with no pose involved, the floor scatters around its own
plane by 0.4 cm at 0.5-1 m, 0.9 cm at 1-1.5 m, 1.8 cm at 1.5-2 m and 3.7 cm
at 2-2.5 m (IQR, odom-test-2; stereo error grows with distance squared),
while the LiDAR's floor is 2.3 cm thick. Out to 4 m, depth made
odom-test-2's floor thicker, not thinner: 2.3 cm without depth, 6.7 cm with
(points facing up, map_quality.py "floor only"); out to 1.5 m, 4.3-5.0 cm,
and map_quality's floor thickness 17.1 -> 11.6-12.6 cm. The camera and the
LiDAR agree on average (within 1 cm out to 3 m, walls and floor), so this is
noise, not calibration. Farther low objects are now left to the LiDAR.

Scans are processed on every processor core but one (--workers).

Grid: 2 cm (--voxel; 3 cm until 2026-09-25). Measured on both walks: twice
the points (odom-test-2 2.4 -> 5.0 million), floors no thicker (floor only
5.0 -> 5.1 cm, 2.7 -> 2.5 cm), 80% camera-coloured instead of 74%; but the
dense step took 341 s instead of 228 s and peaked at 5.0 GB instead of 3.1 GB
on odom-test-2, of the 7 GB WSL has. A walk twice as long may not fit:
--voxel 0.03 then.

WHY IT WAS SPARSE
-----------------
lidar_map.py adds a scan to its map only every 0.4 m or 8 degrees of turning
(its keyframes), so most scans were used to find the robot's position and
then discarded; and it thins to 5 cm. Here every scan is placed, deskewed,
into a finer grid.

HOW
---
1. Accumulate. Every scan's points go into a 3 cm voxel grid. Each voxel keeps
   the average position of the points in it (so detail is not snapped to the
   grid), their count, mean intensity, and how many separate scans hit it.

2. Clean, in order, each step reported:
   a. seen by at least --min-scans scans, counted in 10 cm cells (the
      carving grid). A real surface is seen again as the robot moves; a stray
      return is not. Counted per 3 cm voxel this removed 1.5 million voxels on
      odom-test-1, 44% of them ceiling: the XT16 sees the ceiling only from
      far away, where its points are spread out and rarely land in the same
      3 cm voxel twice.
   b. free-space carving. For a sample of scans, rays are walked from the
      sensor to each point; every voxel a ray passes through was empty at that
      moment. Cells that were hit at some point but seen through far more
      often held something that moved (a person walking past) or a false
      return (glass, reflections). They are removed. Samples within 15 cm of
      the floor under the sensor are not counted: beams skim the floor at a
      shallow angle and pass through floor cells by a few cm of noise, which
      removed floor on odom-test-1 (46% of what was carved).
   c. statistical outlier removal (Open3D): points whose average distance to
      their neighbours is far above typical are isolated specks.

   d. outdoors. Points seen through open windows, doors and glass (trees
      outside, the street) are real and seen many times, so a-c keep them.
      A floor plan is drawn: 20 cm squares with flat points 0.2 to 1.2 m above
      the floor the robot walked on (walls). From every square with points,
      16 horizontal lines are drawn 15 m out; if at least 30% of them meet no
      wall, the square is outdoors. Inside a hall, room or corridor nearly
      every line meets a wall. Squares within 40 cm of an indoor square stay,
      so the building's own outer walls, which do have open sky beyond them,
      are kept. Only flat points count as walls, so trees cannot block lines.
      --keep-outside skips this.

      A walk with a saved crop outline (<walk>/crop.json, crop_outline.py)
      is cropped to it instead, and this guess is not made (--no-crop to
      make it anyway): it kept trees within about 3 m of the building and
      removed the far end of odom-test-2's corridor and stretches of
      courtyard walls on both walks. Cut points are orange in _removed.ply
      either way.

      Tried first and dropped, each checked on odom-test-1 and odom-test-2:
      flood-filling the plan from the route (one gap leaked outside and kept
      the trees); requiring a wall between the route and the point (removed a
      hall seen through glass, 42-45% of points; and an open window has no
      wall in the way); requiring foliage-like scattered points (trees seen
      from far away came out as thin flat stripes and were kept).

Scans taken while the robot is not standing (body height under 0.25 m, lying
down or getting up) are left out. Lying down, the odometry reports the body
26 cm lower but the LiDAR drops only about 18 cm, so those scans put the floor
8 cm too high: odom-test-1's floor had a second layer from its last 40 s.
"""
import argparse
import json
import os
import struct
import sys
import time

import numpy as np
import rosbag2_py
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_map as L  # noqa: E402
from progress import Progress, phase  # noqa: E402


# --------------------------------------------------------------------------
# voxel accumulation
# --------------------------------------------------------------------------
class VoxelGrid(object):
    """Sparse voxel grid: per voxel the point sum, count, intensity sum and the
    number of distinct scans that hit it. Merged in batches with numpy."""

    ORIGIN = np.array([-5000.0, -5000.0, -5000.0])
    SPAN = 1 << 20

    def __init__(self, size):
        self.size = size
        self.keys = np.zeros(0, np.int64)
        self.sums = np.zeros((0, 3))
        self.count = np.zeros(0, np.int64)
        self.isum = np.zeros(0)
        self.scans = np.zeros(0, np.int64)
        self.pending = []

    def key(self, xyz):
        c = np.floor((xyz - self.ORIGIN) / self.size).astype(np.int64)
        return (c[:, 0] * self.SPAN + c[:, 1]) * self.SPAN + c[:, 2]

    def add_scan(self, xyz, intensity):
        k = self.key(xyz)
        uk, inv = np.unique(k, return_inverse=True)
        sums = np.stack([np.bincount(inv, xyz[:, i], len(uk)) for i in range(3)], axis=1)
        cnt = np.bincount(inv, minlength=len(uk))
        isum = np.bincount(inv, intensity, len(uk))
        self.pending.append((uk, sums, cnt, isum, np.ones(len(uk), np.int64)))
        if len(self.pending) >= 100:
            self.merge()

    def merge(self):
        if not self.pending:
            return
        parts = [(self.keys, self.sums, self.count, self.isum, self.scans)] + self.pending
        self.pending = []
        k = np.concatenate([p[0] for p in parts])
        uk, inv = np.unique(k, return_inverse=True)
        n = len(uk)
        sums = np.concatenate([p[1] for p in parts])
        self.sums = np.stack([np.bincount(inv, sums[:, i], n) for i in range(3)], axis=1)
        self.count = np.bincount(inv, np.concatenate([p[2] for p in parts]), n).astype(np.int64)
        self.isum = np.bincount(inv, np.concatenate([p[3] for p in parts]), n)
        self.scans = np.bincount(inv, np.concatenate([p[4] for p in parts]), n).astype(np.int64)
        self.keys = uk

    def points(self, mask=None):
        m = slice(None) if mask is None else mask
        c = self.count[m][:, None]
        return self.sums[m] / c, self.isum[m] / self.count[m]


# --------------------------------------------------------------------------
# free-space carving
# --------------------------------------------------------------------------
class Carver(object):
    """Counts, per coarse cell, how many sampled scans hit it and how many saw
    through it."""

    SPAN = 1 << 20

    def __init__(self, cell, rays, stop_short):
        self.cell = cell
        self.rays = rays
        self.stop_short = stop_short
        self.hit_parts, self.miss_parts = [], []
        self.rng = np.random.default_rng(0)

    def key(self, xyz):
        c = np.floor((xyz - VoxelGrid.ORIGIN) / self.cell).astype(np.int64)
        return (c[:, 0] * self.SPAN + c[:, 1]) * self.SPAN + c[:, 2]

    FLOOR_BELOW_SENSOR = 0.47      # m, measured by lidar_mount.py on both walks
    FLOOR_MARGIN = 0.15

    def add_scan(self, origin, world):
        self.hit_parts.append(np.unique(self.key(world)))
        pick = world[self.rng.choice(len(world), min(self.rays, len(world)), replace=False)]
        vec = pick - origin
        rng = np.linalg.norm(vec, axis=1)
        ok = rng > 0.5
        vec, rng = vec[ok] / rng[ok, None], rng[ok]
        steps = np.arange(0.3, rng.max(), self.cell)
        # samples along every ray, up to stop_short before its end point
        s = steps[None, :].repeat(len(rng), 0)
        valid = s < (rng[:, None] - self.stop_short)
        samples = origin + vec[:, None, :] * s[:, :, None]
        valid &= samples[:, :, 2] > origin[2] - self.FLOOR_BELOW_SENSOR + self.FLOOR_MARGIN
        self.miss_parts.append(np.unique(self.key(samples[valid])))

    def counts(self):
        def tally(parts):
            k = np.concatenate(parts) if parts else np.zeros(0, np.int64)
            return np.unique(k, return_counts=True)
        return tally(self.hit_parts), tally(self.miss_parts)


FLOOR_BELOW_BODY = 0.38     # m, floor below Unitree's body height while standing (measured)


def outdoors_mask(xyz, track, flat, cell=0.20, directions=16, reach=15.0, share=0.30, protect=2):
    """True for points outdoors: in floor-plan squares where at least `share`
    of `directions` horizontal lines run `reach` metres without meeting a wall,
    and not within `protect` squares of an indoor square (so the building's
    own outer walls, which have open sky beyond them, stay)."""
    from scipy import ndimage
    from scipy.spatial import cKDTree
    lo = np.minimum(xyz[:, :2].min(0), track[:, :2].min(0)) - reach
    ij = np.floor((xyz[:, :2] - lo) / cell).astype(np.int64)
    shape = tuple(ij.max(0) + int(reach / cell) + 2)
    _, near = cKDTree(track[:, :2]).query(xyz[:, :2])
    above_floor = xyz[:, 2] - (track[near, 2] - FLOOR_BELOW_BODY)
    band = (above_floor > 0.2) & (above_floor < 1.2) & flat
    counts = np.zeros(shape, np.int32)
    np.add.at(counts, (ij[band, 0], ij[band, 1]), 1)
    wall = ndimage.binary_closing(counts >= 3, iterations=1) | (counts >= 3)

    cells, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.ravel()
    ang = np.arange(directions) * 2 * np.pi / directions
    steps = np.arange(1, int(reach / cell))
    escaped = np.zeros(len(cells), np.int64)
    for c0 in range(0, len(cells), 50000):
        c = cells[c0:c0 + 50000]
        for dx, dy in zip(np.cos(ang), np.sin(ang)):
            px = np.rint(c[:, 0:1] + dx * steps[None, :]).astype(np.int64)
            py = np.rint(c[:, 1:2] + dy * steps[None, :]).astype(np.int64)
            hit = wall[px.clip(0, shape[0] - 1), py.clip(0, shape[1] - 1)].any(axis=1)
            escaped[c0:c0 + 50000] += ~hit
    open_cell = escaped >= share * directions
    indoor = np.zeros(shape, bool)
    indoor[cells[~open_cell, 0], cells[~open_cell, 1]] = True
    near_indoor = ndimage.binary_dilation(indoor, iterations=protect)
    outdoors = open_cell & ~near_indoor[cells[:, 0], cells[:, 1]]
    return outdoors[inv]


def sphericity(xyz, k=20):
    """Per point: smallest over largest eigenvalue of its k-neighbour
    covariance. Near 0 on flat surfaces, large where points scatter (foliage)."""
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pc.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(k))
    ev = np.linalg.eigvalsh(np.asarray(pc.covariances))
    return ev[:, 0] / np.maximum(ev[:, 2], 1e-12)


def lookup(sorted_keys, values, query):
    idx = np.searchsorted(sorted_keys, query)
    idx = np.clip(idx, 0, max(len(sorted_keys) - 1, 0))
    found = (len(sorted_keys) > 0) & (sorted_keys[idx] == query)
    return np.where(found, values[idx], 0)


# --------------------------------------------------------------------------
# depth camera
# --------------------------------------------------------------------------
def add_depth(grid, walk, track, args, standing_at=None):
    """Add the depth camera's points to the same voxel grid.

    Placed through the robot's pose at each frame's own time (interpolated
    between the two scans either side) and camera_mount.json. Each frame is
    counted as one more "scan" for the cleaning steps, exactly like a LiDAR
    scan, so a surface only the camera sees still has to be seen repeatedly.

    Frames taken while the robot is not standing are left out, as LiDAR
    scans are (standing_at(t) False). Until 2026-09-25 they were not: on
    odom-test-2 the 33 s lying down at the end added about 165 frames.
    """
    path = os.path.join(walk, "depth.stream")
    if args.no_depth or not os.path.isfile(path):
        return 0, []
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from depth_stream import DepthStream, find_camera_mount
    T_bc, mount, mount_path = find_camera_mount(walk)
    ds = DepthStream(path)
    frames = ds.depth_frames()
    if not frames:
        return 0, []
    if T_bc is None:
        print("  depth camera: %d frames, not added: no camera_mount.json (depth_mount.py)"
              % len(frames), flush=True)
        return 0, []
    optical = frames[0].header.get("frame_id") or "camera_depth_optical_frame"
    T_co = ds.optical_in_camera_link(optical)
    info = ds.depth_info
    if T_co is None or info is None:
        print("  depth camera: frames present but no calibration or frames in depth.stream",
              flush=True)
        return 0, []
    T_bo = T_bc @ T_co
    K = info["K"]
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    tt = track["time"]
    poses = [L.pose_matrix((track["x"][i], track["y"][i], track["z"][i]),
                           (track["qx"][i], track["qy"][i], track["qz"][i], track["qw"][i]))
             for i in range(len(tt))]
    every = max(1, int(round(15.0 / args.depth_rate)))
    started = time.time()
    added = used = 0
    rays = None
    coarse_key = Carver(args.carve_cell, 0, 0).key
    coarse = []
    lying = 0
    prog = Progress("adding the depth camera", (len(frames) + every - 1) // every, "frames")
    for k, f in enumerate(frames):
        if k % every == 0:
            prog.step()
        if k % every or not (tt[0] <= f.t <= tt[-1]):
            continue
        if standing_at is not None and not standing_at(f.t):
            lying += 1
            continue
        j = int(np.searchsorted(tt, f.t))
        if j <= 0 or j >= len(tt):
            continue
        T_mb = L.interpolate(f.t, tt[j - 1], poses[j - 1], tt[j], poses[j])
        img = f.image
        st = args.depth_step
        dm = img[::st, ::st].astype(np.float32) / 1000.0
        if rays is None or rays[0].shape != dm.shape:
            v, u = np.mgrid[0:img.shape[0]:st, 0:img.shape[1]:st].astype(np.float32)
            rays = ((u - cx) / fx, (v - cy) / fy)
        ok = (dm > 0.3) & (dm < args.depth_max)
        if not ok.any():
            continue
        z = dm[ok]
        pts = np.stack([rays[0][ok] * z, rays[1][ok] * z, z], axis=1).astype(np.float64)
        T = T_mb @ T_bo
        world = (T[:3, :3] @ pts.T).T + T[:3, 3]
        # -1 marks these as the camera's, not the LiDAR's: the wall plan the
        # cleaning steps use is drawn from LiDAR points only.
        grid.add_scan(world, np.full(len(world), -1.0))
        coarse.append(np.unique(coarse_key(world)))
        added += len(world)
        used += 1
    prog.end()
    grid.merge()
    print("  depth camera: %d frames added (%d points), %d left out while not standing, mount %s, %.0f s"
          % (used, added, lying, mount_path, time.time() - started), flush=True)
    return added, coarse


# --------------------------------------------------------------------------
# Foxglove file
# --------------------------------------------------------------------------
def write_foxglove(build, name, xyz, track_csv, colour=None):
    """<walk>_dense/<walk>_dense_0.mcap: the whole cleaned cloud as one message
    on /dense_map (frame "map", coloured by height), the track on /track, and
    with colour (rgb, source) the same cloud on /colour_map (colour_map.py).

    Stamped at the start of the walk, so it shows as soon as the file opens
    and stays shown wherever the timeline is.
    """
    import shutil
    from rclpy.serialization import serialize_message
    out = os.path.join(build, name + "_dense")
    if os.path.exists(out):
        shutil.rmtree(out)
    d = np.genfromtxt(track_csv, delimiter=",", names=True)
    t0 = float(d["time"][0])
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata("/dense_map", "sensor_msgs/msg/PointCloud2", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata("/track", "nav_msgs/msg/Path", "cdr"))
    cloud = L.cloud_msg(xyz, xyz[:, 2], "map", t0)
    writer.write("/dense_map", serialize_message(cloud), int(t0 * 1e9))
    if colour is not None:
        import colour_map
        writer.create_topic(rosbag2_py.TopicMetadata("/colour_map", "sensor_msgs/msg/PointCloud2", "cdr"))
        msg = colour_map.colour_cloud_msg(xyz, colour[0], colour[1], "map", t0)
        writer.write("/colour_map", serialize_message(msg), int(t0 * 1e9))
    poses = [(float(d["time"][i]), L.pose_matrix((d["x"][i], d["y"][i], d["z"][i]),
                                                 (d["qx"][i], d["qy"][i], d["qz"][i], d["qw"][i])))
             for i in range(len(d["time"]))]
    writer.write("/track", serialize_message(L.path_msg(poses, t0, stride=5)), int(t0 * 1e9))
    del writer
    return out


# --------------------------------------------------------------------------
# per-scan work, run in worker processes
# --------------------------------------------------------------------------
W = {}          # set once per worker (fork): odometry, track, mount, settings


def init_worker(state):
    W.update(state)


def process_scan(item):
    """One scan: place it, and return its voxel sums and carving samples."""
    k, raw = item
    tt = W["tt"]
    sec, nsec = struct.unpack("<iI", raw[4:12])
    t = sec + nsec * 1e-9
    i = int(np.searchsorted(tt, t))
    cand = [j for j in (i - 1, i) if 0 <= j < len(tt)]
    if not cand:
        return None
    j = min(cand, key=lambda j: abs(tt[j] - t))
    if abs(tt[j] - t) > 0.02:             # a scan the map builder did not place
        return None
    if np.interp(t, W["bt"], W["bh"]) < 0.25:   # lying down or getting up
        return {"not_standing": True}
    pts = L._points(raw)
    if pts is None or len(pts) < 2000:
        return None
    args = W["args"]
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
    inten = pts["intensity"].astype(np.float64)
    ok = np.isfinite(xyz).all(1)
    rng = np.linalg.norm(xyz, axis=1)
    ok &= (rng > 0.8) & (rng < args.max_range)
    xyz, inten, stamps = xyz[ok], inten[ok], pts["timestamp"][ok]
    T_bl = W["T_bl"]
    if W["deskewed"]:
        xyz = L.deskew(xyz, stamps - stamps.min(), t, W["odom"], T_bl)
    T = W["poses"][j] @ T_bl
    world = (T[:3, :3] @ xyz.T).T + T[:3, 3]
    grid = W["grid"]
    kk = grid.key(world)
    uk, inv = np.unique(kk, return_inverse=True)
    coarse = np.unique(W["carver_key"](world))
    out = {"n": len(world), "coarse": coarse,
           "voxels": (uk, np.stack([np.bincount(inv, world[:, c], len(uk)) for c in range(3)], 1),
                      np.bincount(inv, minlength=len(uk)), np.bincount(inv, inten, len(uk)))}
    carver = W["carver"]
    if carver is not None and k % args.carve_every == 0:
        carver.rng = np.random.default_rng(k)
        carver.hit_parts, carver.miss_parts = [], []
        carver.add_scan(T[:3, 3], world)
        out["carve"] = (carver.hit_parts[0], carver.miss_parts[0])
    return out


def scan_bytes(reader):
    k = 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        yield k, bytes(raw)
        k += 1


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="dense, cleaned point cloud of a walk from every scan")
    ap.add_argument("walk")
    ap.add_argument("--voxel", type=float, default=0.02,
                    help="grid size, metres (0.03 until 2026-09-25: half the points, 60%% of the memory)")
    ap.add_argument("--max-range", type=float, default=25.0)
    ap.add_argument("--min-scans", type=int, default=3,
                    help="keep voxels hit by at least this many scans")
    ap.add_argument("--no-carve", action="store_true", help="skip free-space carving")
    ap.add_argument("--carve-every", type=int, default=5, help="carve with every Nth scan")
    ap.add_argument("--carve-cell", type=float, default=0.10)
    ap.add_argument("--no-sor", action="store_true", help="skip statistical outlier removal")
    ap.add_argument("--outdoor-share", type=float, default=0.30,
                    help="share of 16 directions that must run 15 m without a wall for a "
                         "spot to count as outdoors: lower removes trees closer to the "
                         "building, higher protects poorly scanned parts of it")
    ap.add_argument("--no-depth", action="store_true", help="leave the depth camera out")
    ap.add_argument("--depth-rate", type=float, default=5.0,
                    help="depth frames per second added to the cloud")
    ap.add_argument("--depth-step", type=int, default=3, help="every Nth depth pixel")
    ap.add_argument("--depth-max", type=float, default=1.5,
                    help="metres; beyond this the D435i's floor is noisier than the LiDAR's "
                         "(see the docstring); 4.0 until 2026-09-25")
    ap.add_argument("--no-colour", action="store_true", help="leave the colour frames out")
    ap.add_argument("--colour-rate", type=float, default=5.0, help="colour frames per second used")
    ap.add_argument("--colour-max-range", type=float, default=10.0,
                    help="metres from the colour camera (6 m left the ceiling uncoloured)")
    ap.add_argument("--no-crop", action="store_true",
                    help="ignore the walk's crop.json and guess what is outdoors instead")
    ap.add_argument("--keep-outside", action="store_true",
                    help="keep points outside the building (through windows and doors)")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1),
                    help="processes placing scans (default: all cores but one)")
    args = ap.parse_args()

    walk = os.path.abspath(args.walk)
    name = os.path.basename(walk)
    build = os.path.join(walk, "map3d")
    for need in ("trajectory.csv", "build.json"):
        if not os.path.isfile(os.path.join(build, need)):
            sys.exit("map3d/%s is missing: build the map first (process_walk.sh --map-only)" % need)
    with open(os.path.join(build, "build.json")) as fh:
        info = json.load(fh)
    T_bl = np.eye(4)
    T_bl[:3, :3] = Rotation.from_euler("xyz", info["lidar_rpy_deg"], degrees=True).as_matrix()
    T_bl[:3, 3] = info["lidar_xyz"]
    deskewed = info.get("deskew", False)
    odom = L.Odometry(os.path.join(walk, "odom.csv"))

    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    tt = d["time"]
    poses = [L.pose_matrix((d["x"][i], d["y"][i], d["z"][i]),
                           (d["qx"][i], d["qy"][i], d["qz"][i], d["qw"][i])) for i in range(len(tt))]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(walk, "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lidar_points"]))

    started = time.time()
    grid = VoxelGrid(args.voxel)
    carver = None if args.no_carve else Carver(args.carve_cell, 3000, 2 * args.carve_cell)
    used = raw_points = 0
    coarse_parts = []
    not_standing = 0
    report = {"walk": name, "voxel_m": args.voxel, "track_build": info, "workers": args.workers}
    import csv
    bt, bh = [], []
    with open(os.path.join(walk, "odom.csv")) as fh:
        for r in csv.DictReader(fh):
            try:
                bt.append(float(r["host_time"]))
                bh.append(float(r["body_height"]))
            except (KeyError, ValueError):
                continue
    order = np.argsort(bt)
    state = {"bt": np.array(bt)[order], "bh": np.array(bh)[order], "tt": tt, "poses": poses, "T_bl": T_bl, "deskewed": deskewed, "odom": odom,
             "args": args, "grid": VoxelGrid(args.voxel),
             "carver": None if args.no_carve else Carver(args.carve_cell, 3000, 2 * args.carve_cell),
             "carver_key": Carver(args.carve_cell, 0, 0).key}
    import multiprocessing as mp
    prog = Progress("placing scans", len(tt), "scans")
    with mp.get_context("fork").Pool(args.workers, init_worker, (state,)) as pool:
        for res in pool.imap(process_scan, scan_bytes(reader), chunksize=4):
            prog.step()
            if res is None:
                continue
            if res.get("not_standing"):
                not_standing += 1
                continue
            uk, sums, cnt, isum = res["voxels"]
            grid.pending.append((uk, sums, cnt, isum, np.ones(len(uk), np.int64)))
            if len(grid.pending) >= 100:
                grid.merge()
            coarse_parts.append(res["coarse"])
            if "carve" in res:
                carver.hit_parts.append(res["carve"][0])
                carver.miss_parts.append(res["carve"][1])
            used += 1
            raw_points += res["n"]
            if used % 500 == 0:
                print("  %d scans placed, %.0f s" % (used, time.time() - started), flush=True)
    prog.end()
    grid.merge()
    depth_points, depth_coarse = add_depth(
        grid, walk, d, args, lambda t: np.interp(t, state["bt"], state["bh"]) >= 0.25)
    coarse_parts += depth_coarse
    report["depth_points"] = int(depth_points)
    report["scans_left_out_not_standing"] = not_standing
    print("  left out %d scans taken while the robot was not standing" % not_standing)
    phase("cleaning the cloud")
    print("  %d scans, %d points placed into %d voxels of %.0f cm, %.0f s"
          % (used, raw_points, len(grid.keys), 100 * args.voxel, time.time() - started), flush=True)
    report.update(scans=used, points_placed=int(raw_points), voxels=int(len(grid.keys)))

    # --- a. seen by enough scans ------------------------------------------
    xyz, inten = grid.points()
    ck, cc = np.unique(np.concatenate(coarse_parts), return_counts=True)
    seen_by = lookup(ck, cc, Carver(args.carve_cell, 0, 0).key(xyz))
    keep = seen_by >= args.min_scans
    report["removed_seen_by_too_few_scans"] = int((~keep).sum())
    print("  seen by fewer than %d scans (10 cm cells): removed %d voxels (%.1f%%)"
          % (args.min_scans, (~keep).sum(), 100.0 * (~keep).mean()), flush=True)

    # --- b. free-space carving ---------------------------------------------
    removed = [(xyz[~keep], (150, 150, 150))]
    if carver is not None:
        (hk, hc), (mk, mc) = carver.counts()
        cell_keys = carver.key(xyz)
        hits = lookup(hk, hc, cell_keys)
        misses = lookup(mk, mc, cell_keys)
        moved = keep & (misses >= 5) & (misses >= 3 * np.maximum(hits, 1))
        report["removed_seen_through"] = int(moved.sum())
        print("  seen through far more often than hit (moving things, false returns): removed %d "
              "voxels (%.1f%%), from %d carving scans"
              % (moved.sum(), 100.0 * moved.sum() / max(keep.sum(), 1), len(carver.hit_parts)),
              flush=True)
        removed.append((xyz[moved], (255, 40, 40)))
        keep &= ~moved
    xyz, inten = xyz[keep], inten[keep]

    # --- c. statistical outlier removal -----------------------------------
    if not args.no_sor and len(xyz):
        import open3d as o3d
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        _, idx = pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        idx = np.asarray(idx)
        report["removed_statistical_outliers"] = int(len(xyz) - len(idx))
        gone = np.ones(len(xyz), bool)
        gone[idx] = False
        removed.append((xyz[gone], (255, 0, 255)))
        print("  statistical outliers (isolated specks): removed %d points (%.1f%%)"
              % (len(xyz) - len(idx), 100.0 * (len(xyz) - len(idx)) / len(xyz)), flush=True)
        xyz, inten = xyz[idx], inten[idx]

    # --- d. inside the building ------------------------------------------
    import crop_outline
    crop = None if (args.no_crop or args.keep_outside) else crop_outline.load(walk)
    if crop is not None and len(xyz):
        # The walk's saved outline (crop_outline.py) replaces the guess below.
        ins = crop_outline.inside(xyz[:, :2], crop["polygons"])
        report["removed_outside_crop_outline"] = int((~ins).sum())
        report["crop_outline"] = os.path.join(walk, "crop.json")
        removed.append((xyz[~ins], (255, 150, 0)))
        print("  outside the walk's crop outline (crop.json, %d polygons): removed %d points (%.1f%%)"
              % (len(crop["polygons"]), (~ins).sum(), 100.0 * (~ins).mean()), flush=True)
        xyz, inten = xyz[ins], inten[ins]
    elif not args.keep_outside and len(xyz):
        standing = np.interp(d["time"], state["bt"], state["bh"]) >= 0.25
        track_xyz = np.stack([d["x"], d["y"], d["z"]], 1)[standing]
        # Walls: flat LiDAR points. The shape is measured among LiDAR points
        # only; with the camera's dense floor points mixed in, neighbourhoods
        # change and walls stop counting as flat, which made this filter
        # remove the corridor the robot had walked along.
        flat = np.zeros(len(xyz), bool)
        lidar_only = inten >= 0
        flat[lidar_only] = sphericity(xyz[lidar_only]) < 0.08
        ins = ~outdoors_mask(xyz, track_xyz, flat, share=args.outdoor_share)
        report["removed_outside_building"] = int((~ins).sum())
        removed.append((xyz[~ins], (255, 150, 0)))
        print("  outside the building (seen through windows and doors): removed %d points (%.1f%%)"
              % ((~ins).sum(), 100.0 * (~ins).mean()), flush=True)
        xyz, inten = xyz[ins], inten[ins]

    report["points_kept"] = int(len(xyz))
    np.save(os.path.join(build, "dense_map.npy"), xyz.astype(np.float32))

    # --- .ply with the track ---------------------------------------------
    pos = np.stack([d["x"], d["y"], d["z"]], 1)
    line = [pos[0]]
    for a, b in zip(pos[:-1], pos[1:]):
        k = max(1, int(np.linalg.norm(b - a) / 0.01))
        line.extend(a + (b - a) * (q / k) for q in range(1, k + 1))
    line = np.array(line)
    thick = np.concatenate([line + off for off in
                            ([0, 0, 0], [0.02, 0, 0], [-0.02, 0, 0], [0, 0.02, 0], [0, -0.02, 0])])
    ply = os.path.join(build, name + "_dense.ply")
    L.write_ply(ply, np.concatenate([xyz, thick]).astype(np.float32),
                np.concatenate([L.height_colours(xyz[:, 2]),
                                np.full((len(thick), 3), 255, np.uint8)]))
    rem_xyz = np.concatenate([r[0] for r in removed]) if removed else np.zeros((0, 3))
    rem_rgb = np.concatenate([np.tile(np.array(r[1], np.uint8), (len(r[0]), 1)) for r in removed]) \
        if removed else np.zeros((0, 3), np.uint8)
    L.write_ply(os.path.join(build, name + "_removed.ply"), rem_xyz.astype(np.float32), rem_rgb)
    # --- colour from the colour frames (colour_map.py) ---------------------
    colour = None
    if not args.no_colour and os.path.isfile(os.path.join(walk, "depth.stream")):
        import colour_map
        rgb, source, cstats = colour_map.colour_points(
            walk, xyz, d, rate=args.colour_rate, max_range=args.colour_max_range,
            standing_at=lambda t: np.interp(t, state["bt"], state["bh"]) >= 0.25,
            workers=args.workers)
        if rgb is None:
            print("  colour: none (%s)" % cstats, flush=True)
        else:
            from scipy.spatial import cKDTree
            tr = np.stack([d["x"], d["y"], d["z"]], 1)
            tk = cKDTree(tr[:, :2])
            cstats["coloured_share"] = colour_map.surface_shares(
                xyz, lambda p: tr[tk.query(p[:, :2])[1], 2] - FLOOR_BELOW_BODY, source)
            print("  colour: coloured share, floor %(floor).0f%%, walls %(walls).0f%%, ceiling "
                  "%(ceiling).0f%%" % {k: 100 * v for k, v in cstats["coloured_share"].items()}, flush=True)
            final = colour_map.final_colours(xyz, rgb, source)
            cply = os.path.join(build, name + "_colour.ply")
            colour_map.write_colour_ply(cply, xyz, final, source)
            np.save(os.path.join(build, "dense_colour.npy"),
                    np.column_stack([final, source]).astype(np.uint8))
            colour = (final, source)
            report["colour"] = cstats
            print("  wrote %s (camera colour where seen, height colours elsewhere, "
                  "colour_source 1/0)" % cply, flush=True)
    phase("writing the clean cloud")
    if colour is None:
        # No colour this time: remove an earlier build's, which no longer
        # lines up with this cloud (mesh_map.py reads dense_colour.npy).
        for old in (os.path.join(build, name + "_colour.ply"), os.path.join(build, "dense_colour.npy")):
            if os.path.isfile(old):
                os.remove(old)
                print("  removed %s from an earlier build (no colour this time)" % os.path.basename(old))
    fox = write_foxglove(build, name, xyz, os.path.join(build, "trajectory.csv"), colour)
    print("  wrote %s (Foxglove)" % fox)
    report["seconds"] = round(time.time() - started, 1)
    with open(os.path.join(build, "dense_report.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    print("  kept %d points; wrote %s (%.0f MB), %.0f s"
          % (len(xyz), ply, os.path.getsize(ply) / 1e6, time.time() - started))


if __name__ == "__main__":
    main()
