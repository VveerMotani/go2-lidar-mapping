#!/usr/bin/env python3
"""
Measure a walk's map, so changes to the mapping can be compared in numbers.

    python3 map_quality.py ~/go2-lidar-mapping/walks/odom-test-1
    python3 map_quality.py <walk> --build map3d/previous/2026-09-17_140215

Reads a build (default: the walk's current map3d) and reports:

  floor thickness      in 1 m cells along the track, the spread (middle half)
                       of the lowest surface. A perfect map of a flat floor
                       gives a few cm: the sensor's own noise.
  floor wander         how much the floor's height differs between cells on
                       the ground level (5th to 95th percentile).

                       Both count everything within 60 cm of a cell's lowest
                       point, so a cell with a wall in it counts the wall's
                       bottom 60 cm: in odom-test-2's corridors those edge
                       cells read 25 to 40 cm while cells on the track read
                       5 to 10 (map3d/loop_closure_floor_thickness.png).
                       Compare builds with them, not rooms.
                       A cell's lowest surface is its 2nd-percentile point:
                       odom-test-1 has one cell with about 110 points 1.2 m
                       below the floor (a reflection or a gap), and when they
                       crossed 2% of the cell its wander read 88 cm, not 38.
  floor, floor only    the floor thickness again, from points whose surface
                       faces up only: no wall bottoms (added 2026-09-25).
  stairs               per flight of stairs (body pitched 7+ degrees): how
                       far the track goes below the floor at the foot of the
                       flight (6-7 cm on odom-test-2, from the leg odometry's
                       height as the body pitched up; see
                       lidar_map.body_height_step), how far above the landing
                       at its head, and the scan-to-scan height noise on it.
  ceiling vs distance  the ceiling's height depending on how far away the
                       LiDAR was when it saw it. A flat ceiling should read
                       the same at every distance; a height that grows with
                       distance means an angle error (the XT16 only sees the
                       ceiling from far away, so this is its most sensitive
                       symptom).

  passes agree         where the walk goes over the same place twice, do the two
                       passes put it in the same place? The track is cut into
                       2 m stretches; every pair of stretches within 3 m of
                       each other with at least 6 m of walking between them is
                       aligned by ICP, and the correction ICP finds is how far
                       apart the passes are: height (a doubled floor or
                       ceiling), sideways (only across walls, the directions
                       the geometry pins down), heading and tilt. A perfect
                       track gives about zero.
  end vs start         where the robot really ended relative to where it
                       started, measured by aligning its last standing scans
                       to its first, against where the track says it ended.
                       The difference is the track's error after the whole
                       walk. (The robot need not end where it started:
                       odom-test-1 ended 17 cm away. Nor lying down, which
                       drops the body 26 cm, so the ends used are standing.)

Ceiling and floor both assume level floors and ceilings along the walk.

The ceiling and passes tests re-read the scans from the bag and place them
with the build's own track and LiDAR mount (map3d/build.json; builds before it
existed used the documented mount without deskewing).

Tried and dropped (2026-09-25): "floor step between passes", the per 1 m cell
difference in the floor's median height between passes. It read 11 cm on
odom-test-1 after loop closure where a slice through the map shows a single
floor: a cell's median depends on which side it was seen from and what else
sits low in it, more than on how well the passes agree.
"""
import argparse
import json
import os
import struct
import sys

import numpy as np
import rosbag2_py
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_map as L  # noqa: E402


def floor_stats(m, track_xyz):
    trk = track_xyz[::5]
    dist, idx = cKDTree(trk[:, :2]).query(m[:, :2])
    p = m[dist < 1.5]
    near_z = trk[idx[dist < 1.5], 2]
    cell = np.floor(p[:, :2]).astype(np.int64)
    key = cell[:, 0] * 100000 + cell[:, 1]
    order = np.argsort(key)
    p, key, near_z = p[order], key[order], near_z[order]
    starts = np.flatnonzero(np.r_[True, np.diff(key) != 0])
    ends = np.r_[starts[1:], len(key)]
    thick, level, robot = [], [], []
    for s, e in zip(starts, ends):
        z = np.sort(p[s:e, 2])
        if len(z) < 200:
            continue
        low = z[z < z[int(0.02 * len(z))] + 0.6]
        if len(low) < 100:
            continue
        thick.append(np.percentile(low, 75) - np.percentile(low, 25))
        level.append(np.median(low))
        robot.append(np.median(near_z[s:e]))
    level, robot = np.array(level), np.array(robot)
    ground = robot < np.percentile(robot, 20) + 0.5        # cells on the lowest level walked
    return (float(np.median(thick)), len(thick),
            float(np.percentile(level[ground], 95) - np.percentile(level[ground], 5)))


def floor_only(m, track_xyz):
    """Floor thickness from floor points only: the same 1 m cells as
    floor_stats, but only points within 0.6 m of the cell's lowest whose
    surface faces up (normal within 18 degrees of vertical), so the bottom of a
    wall does not count. Returns (median thickness, cells)."""
    import open3d as o3d
    trk = track_xyz[::5]
    dist, _ = cKDTree(trk[:, :2]).query(m[:, :2])
    p = m[dist < 1.5].astype(np.float64)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=20))
    up = np.abs(np.asarray(pc.normals)[:, 2]) > 0.95
    cell = np.floor(p[:, :2]).astype(np.int64)
    key = cell[:, 0] * 100000 + cell[:, 1]
    order = np.argsort(key)
    p, key, up = p[order], key[order], up[order]
    starts = np.flatnonzero(np.r_[True, np.diff(key) != 0])
    ends = np.r_[starts[1:], len(key)]
    thick = []
    for s, e in zip(starts, ends):
        z = p[s:e, 2]
        if len(z) < 200:
            continue
        lowest = np.sort(z)[int(0.02 * len(z))]
        f = z[up[s:e] & (z < lowest + 0.6)]
        if len(f) < 100:
            continue
        thick.append(np.percentile(f, 75) - np.percentile(f, 25))
    return float(np.median(thick)), len(thick)


def stairs(walk, build, incline=7.0):
    """Height along the track on stairs. For every flight (body pitched
    `incline` degrees or more away from level, standing, over 3 s):
      bottom dip     how far the track goes below the floor it stands on at
                     the foot of the flight, within 1 s of leaving or
                     reaching it. The height should never be below that floor.
      top overshoot  how far above the landing at the head of the flight.
      noise          scan-to-scan height noise on the flight (distance from a
                     1 s running median).
    (The first version, 2026-09-25, also called the last step up and the
    first step down a "dip": those are just a step.)"""
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    t, z = d["time"], d["z"]
    odom = L.Odometry(os.path.join(walk, "odom.csv"))
    level = float(np.median(odom.pitch))
    bt, bh = standing_series(walk)
    standing = np.interp(t, bt, bh) >= 0.25
    climb = np.array([abs(odom.pitch_near(x) - level) >= incline for x in t]) & standing
    idx = np.flatnonzero(climb)
    runs = [r for r in np.split(idx, np.flatnonzero(np.diff(idx) > 5) + 1)
            if len(r) and t[r[-1]] - t[r[0]] > 3.0]
    out = []
    for r in runs:
        t0, t1 = t[r[0]], t[r[-1]]
        before = np.median(z[(t > t0 - 1.5) & (t < t0 - 0.3)])
        after = np.median(z[(t > t1 + 0.3) & (t < t1 + 1.5)])
        start = (t >= t0 - 0.3) & (t <= t0 + 1.0)
        end = (t >= t1 - 1.0) & (t <= t1 + 0.3)
        up = after > before
        if up:
            dip, over = before - z[start].min(), z[end].max() - after
        else:
            dip, over = after - z[end].min(), z[start].max() - before
        on = (t >= t0) & (t <= t1)
        smooth = np.array([np.median(z[(t > x - 0.5) & (t < x + 0.5)]) for x in t[on]])
        out.append(dict(start=t0 - t[0], end=t1 - t[0], rise=after - before, up=up,
                        dip=dip, over=over,
                        noise=float(np.median(np.abs(z[on] - smooth))),
                        noise90=float(np.percentile(np.abs(z[on] - smooth), 90))))
    return out


def ceiling_vs_distance(walk, build, every=3):
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    tt = d["time"]
    info = {}
    path = os.path.join(build, "build.json")
    if os.path.isfile(path):
        with open(path) as fh:
            info = json.load(fh)
    rpy = info.get("lidar_rpy_deg", [0.0, 0.0, 0.0])
    T_bl = np.eye(4)
    T_bl[:3, :3] = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    T_bl[:3, 3] = info.get("lidar_xyz", L.LIDAR_IN_BASE)
    deskewed = info.get("deskew", False)
    odom = L.Odometry(os.path.join(walk, "odom.csv")) if deskewed else None

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(walk, "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lidar_points"]))
    track_tree = cKDTree(np.stack([d["x"], d["y"]], 1)[::5])
    heights, dists = [], []
    k = 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        k += 1
        if k % every:
            continue
        raw = bytes(raw)
        sec, nsec = struct.unpack("<iI", raw[4:12])
        t = sec + nsec * 1e-9
        i = int(np.argmin(abs(tt - t)))
        if abs(tt[i] - t) > 0.15:
            continue
        pts = L._points(raw)
        if pts is None or len(pts) < 2000:
            continue
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
        ok = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) < 30)
        xyz, stamps = xyz[ok], pts["timestamp"][ok]
        if deskewed:
            xyz = L.deskew(xyz, stamps - stamps.min(), t, odom, T_bl)
        T_mb = L.pose_matrix((d["x"][i], d["y"][i], d["z"][i]),
                             (d["qx"][i], d["qy"][i], d["qz"][i], d["qw"][i]))
        T = T_mb @ T_bl
        w = (T[:3, :3] @ xyz.T).T + T[:3, 3]
        horiz = np.linalg.norm(xyz[:, :2], axis=1)
        up = w[:, 2] - d["z"][i]
        # Ceiling candidates: well above the robot, over ground the robot walked.
        sel = (up > 1.2) & (up < 4.0)
        if not sel.any():
            continue
        near, _ = track_tree.query(w[sel, :2])
        keep = near < 1.2
        heights.append(up[sel][keep])
        dists.append(horiz[sel][keep])
    h = np.concatenate(heights) if heights else np.zeros(0)
    r = np.concatenate(dists) if dists else np.zeros(0)
    rows = []
    for lo, hi in ((5, 10), (10, 15), (15, 25)):
        s = h[(r >= lo) & (r < hi)]
        if len(s) > 200:
            rows.append((lo, hi, len(s), float(np.median(s)),
                         float(np.percentile(s, 75) - np.percentile(s, 25))))
    return rows, rpy, deskewed


def standing_series(walk):
    """(time, body height) from odom.csv: under 0.25 m the robot is lying down."""
    import csv
    t, h = [], []
    with open(os.path.join(walk, "odom.csv")) as fh:
        for r in csv.DictReader(fh):
            try:
                t.append(float(r["host_time"]))
                h.append(float(r["body_height"]))
            except (KeyError, ValueError):
                continue
    order = np.argsort(t)
    return np.array(t)[order], np.array(h)[order]


def placed_scans(walk, build, every=2, max_range=15.0):
    """Every `every`th scan the build placed while the robot stood, deskewed and
    placed with the build's track: (time, track index, LiDAR pose, world xyz)."""
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    tt = d["time"]
    info = {}
    if os.path.isfile(os.path.join(build, "build.json")):
        with open(os.path.join(build, "build.json")) as fh:
            info = json.load(fh)
    T_bl = np.eye(4)
    T_bl[:3, :3] = Rotation.from_euler("xyz", info.get("lidar_rpy_deg", [0, 0, 0]), degrees=True).as_matrix()
    T_bl[:3, 3] = info.get("lidar_xyz", L.LIDAR_IN_BASE)
    odom = L.Odometry(os.path.join(walk, "odom.csv")) if info.get("deskew") else None
    bt, bh = standing_series(walk)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(walk, "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lidar_points"]))
    k = 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        k += 1
        if k % every:
            continue
        raw = bytes(raw)
        sec, nsec = struct.unpack("<iI", raw[4:12])
        t = sec + nsec * 1e-9
        i = int(np.argmin(abs(tt - t)))
        if abs(tt[i] - t) > 0.02 or np.interp(t, bt, bh) < 0.25:
            continue
        pts = L._points(raw)
        if pts is None or len(pts) < 2000:
            continue
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(xyz).all(1) & (r > 1.0) & (r < max_range)
        xyz, st = xyz[ok], pts["timestamp"][ok]
        if odom is not None:
            xyz = L.deskew(xyz, st - st.min(), t, odom, T_bl)
        T = L.pose_matrix((d["x"][i], d["y"][i], d["z"][i]),
                          (d["qx"][i], d["qy"][i], d["qz"][i], d["qw"][i])) @ T_bl
        yield t, i, T, (T[:3, :3] @ xyz.T).T + T[:3, 3]


def _cloud(xyz, voxel=0.05):
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pc = pc.voxel_down_sample(voxel)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=20))
    return pc


def passes_agree(walk, build, stretch=2.0, apart=6.0, near=3.0):
    """Pass-to-pass disagreement and the end-vs-start check (see the top)."""
    import open3d as o3d
    reg = o3d.pipelines.registration
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    pos = np.stack([d["x"], d["y"], d["z"]], 1)
    path = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1))]
    chunk_of = (path // stretch).astype(int)
    chunks, starts, ends = {}, [], []
    for t, i, T, w in placed_scans(walk, build):
        ch = chunks.setdefault(chunk_of[i], {"pts": [], "pos": []})
        ch["pts"].append(L.voxel_keep(w, 0.05)[0])
        ch["pos"].append(T[:3, 3])
        if len(starts) < 10:
            starts.append((T, w))
        ends = (ends + [(T, w)])[-10:]
    keys = sorted(chunks)
    C = {c: (_cloud(np.concatenate(chunks[c]["pts"])), np.mean(chunks[c]["pos"], 0)) for c in keys}
    rows = []
    for a in keys:
        for b in keys:
            if b <= a or (b - a) * stretch < apart:
                continue
            ca, cb = C[a][1], C[b][1]
            if np.linalg.norm(ca[:2] - cb[:2]) > near or abs(ca[2] - cb[2]) > 0.5:
                continue
            r = reg.registration_icp(C[b][0], C[a][0], 0.25, np.eye(4),
                                     reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=0.1)),
                                     reg.ICPConvergenceCriteria(max_iteration=50))
            if r.fitness < 0.3:
                continue
            T = np.array(r.transformation)
            n = np.asarray(C[a][0].normals)[np.asarray(r.correspondence_set)[:, 1]]
            ev, V = np.linalg.eigh((n.T @ n)[:2, :2])
            strong = ev >= 0.15 * ev.max()                # directions walls pin down
            rows.append((abs(T[2, 3]), np.linalg.norm(V[:, strong].T @ T[:2, 3]),
                         abs(np.degrees(Rotation.from_matrix(T[:3, :3]).as_euler("ZYX")[0])),
                         np.degrees(np.arccos(np.clip(T[2, 2], -1, 1)))))
    R = np.array(rows).reshape(-1, 4)

    def stack(lst):
        inv = np.linalg.inv(lst[0][0])
        return _cloud(np.concatenate([(inv[:3, :3] @ w.T).T + inv[:3, 3] for _, w in lst])), lst[0][0]
    A, TA = stack(starts)
    B, TB = stack(ends)
    said = np.linalg.inv(TA) @ TB
    r = reg.registration_icp(B, A, 1.0, said, reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=0.3)))
    r = reg.registration_icp(B, A, 0.2, r.transformation,
                             reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=0.1)))
    real = np.array(r.transformation)
    err = np.linalg.inv(real) @ said
    return R, dict(said=said, real=real, fitness=r.fitness, err_m=float(np.linalg.norm(err[:3, 3])),
                   err_dz=float(err[2, 3]), err_deg=float(L.angle_deg(err[:3, :3])))


def main():
    ap = argparse.ArgumentParser(description="measure a walk's map")
    ap.add_argument("walk")
    ap.add_argument("--build", default=None, help="a map3d folder (default: the current one)")
    ap.add_argument("--no-ceiling", action="store_true", help="skip the slower ceiling test")
    ap.add_argument("--no-passes", action="store_true",
                    help="skip the passes-agree and end-vs-start tests (about 30 s)")
    ap.add_argument("--points", choices=["lidar", "dense"], default="lidar",
                    help="floor measures on lidar_map.npy (keyframes, 5 cm) or dense_map.npy "
                         "(every scan, cleaned; dense_map.py)")
    args = ap.parse_args()
    walk = os.path.abspath(args.walk)
    build = os.path.abspath(args.build) if args.build else os.path.join(walk, "map3d")
    if not os.path.isabs(args.build or "") and args.build:
        build = os.path.join(walk, args.build)

    m = np.load(os.path.join(build, "lidar_map.npy" if args.points == "lidar" else "dense_map.npy"))
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    track = np.stack([d["x"], d["y"], d["z"]], 1)
    thick, cells, wander = floor_stats(m, track)
    print("  build: %s (%s points: %d)" % (build, args.points, len(m)))
    print("  floor thickness      %.1f cm (median of %d 1 m cells)" % (100 * thick, cells))
    fthick, fcells = floor_only(m, track)
    print("  floor, floor only    %.1f cm (median of %d 1 m cells; points facing up, no walls)"
          % (100 * fthick, fcells))
    print("  floor wander         %.0f cm across the ground level" % (100 * wander))
    print("  track ends           %.2f m from its start" % np.linalg.norm(track[-1] - track[0]))
    for f in stairs(walk, build):
        print("  stairs %3.0f-%3.0f s    %s %.2f m; below the floor at the foot %.1f cm, above the "
              "landing at the head %.1f cm; height noise %.1f cm (90th percentile %.1f)"
              % (f["start"], f["end"], "up" if f["up"] else "down", abs(f["rise"]), 100 * f["dip"],
                 100 * f["over"], 100 * f["noise"], 100 * f["noise90"]))
    if not args.no_ceiling:
        rows, rpy, deskewed = ceiling_vs_distance(walk, build)
        print("  ceiling vs distance  (LiDAR tilt roll %.2f pitch %.2f, %s)"
              % (rpy[0], rpy[1], "deskewed" if deskewed else "not deskewed"))
        for lo, hi, n, med, iqr in rows:
            print("      seen %2d-%2d m away: %.2f m above the robot, middle half spans %3.0f cm (%d points)"
                  % (lo, hi, med, 100 * iqr, n))
        if len(rows) >= 2:
            print("      far minus near:  %+.0f cm (0 for a perfect map)" % (100 * (rows[-1][3] - rows[0][3])))
    if not args.no_passes:
        R, end = passes_agree(walk, build)
        print("  passes agree         %d pairs of 2 m stretches over the same place (0 for a perfect track)"
              % len(R))
        for name, col, unit, f in (("height", 0, "cm", 100), ("sideways", 1, "cm", 100),
                                   ("heading", 2, "deg", 1), ("tilt", 3, "deg", 1)):
            if len(R):
                v = R[:, col] * f
                print("      %-9s median %4.1f %s, 90th percentile %4.1f, worst %4.1f"
                      % (name, np.median(v), unit, np.percentile(v, 90), v.max()))
        s, r = end["said"][:3, 3], end["real"][:3, 3]
        print("  end vs start         (standing) track says %.2f m, scans say %.2f m (fitness %.2f)"
              % (np.linalg.norm(s), np.linalg.norm(r), end["fitness"]))
        print("      track error at the end: %.1f cm (height %+.1f cm), %.2f deg"
              % (100 * end["err_m"], 100 * end["err_dz"], end["err_deg"]))


if __name__ == "__main__":
    main()
