#!/usr/bin/env python3
"""
Measure how the LiDAR is tilted on the robot, from walks on flat floor.

    python3 lidar_mount.py ~/go2-lidar-mapping/walks/odom-test-1 ~/go2-lidar-mapping/walks/odom-test-2
    python3 lidar_mount.py <walk> ... --dry-run

Unitree's documentation gives the XT16's position on the body (0.171 m
forward, 0.0908 m up) and no rotation. The maps say otherwise: on odom-test-1
the corridor ceiling came out 2.21 m high when seen from 5-10 m away and
2.70 m from 15-25 m, which is what a couple of degrees of tilt does to points
seen at a shallow angle far away.

HOW
---
On flat floor, the floor's direction in a LiDAR scan and "up" as the robot's
IMU measures it are the same direction, seen from two sensors. For every
usable scan:

  * the floor is fitted as a plane (RANSAC) to the scan's points below the
    sensor, 1 to 6 m away, after deskewing; its normal is "up" in the LiDAR's
    frame;
  * Unitree's odometry gives the body's roll and pitch from its IMU, which
    measures gravity directly; that gives "up" in the body's frame.

The LiDAR's roll and pitch on the body are the rotation that turns the first
into the second, fitted over all scans at once with robust weights. The
robot's own sway while walking is in both measurements and cancels.

Used scans: body level (not on stairs), standing height (not lying down), a
floor plane with enough points at a plausible height below the sensor.

HEADING (YAW)
-------------
The floor cannot show which way the LiDAR faces, so yaw is measured from a
built map instead: while walking, the robot moves along its own forward axis,
so the direction of travel along the scan-matched track, minus the heading
that track gives the robot, is the error in the mount's yaw. On odom-test-1
and odom-test-2 (built with yaw 0) it was -91 degrees on both: the XT16 is
mounted turned a quarter turn. Unitree's documented position is kept.

Yaw has to be fixed before roll and pitch are fitted, because with the sensor
turned 90 degrees the LiDAR's roll is the body's pitch and vice versa.

Each walk is fitted separately as well as together: two walks agreeing is the
check that this is the mount and not one walk's floor.

Writes lidar_mount.json into the walks folder; lidar_map.py and retime_bag.py
use it. Rebuild maps afterwards (process_walk.sh --map-only).
"""
import argparse
import csv
import json
import os
import struct
import sys
import time

import numpy as np
import open3d as o3d
import rosbag2_py
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_map as L  # noqa: E402


def body_height_series(path):
    t, h = [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            try:
                t.append(float(r["host_time"]))
                h.append(float(r["body_height"]))
            except (KeyError, ValueError):
                continue
    order = np.argsort(t)
    return np.array(t)[order], np.array(h)[order]


def collect(walk, every, T_bl, max_scans=0):
    """(n_lidar, up_body, time) for every usable scan in one walk."""
    odom = L.Odometry(os.path.join(walk, "odom.csv"))
    bt, bh = body_height_series(os.path.join(walk, "odom.csv"))
    level = float(np.median(odom.pitch))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(walk, "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lidar_points"]))
    pairs = []
    k = seen = 0
    why = {"tilted body": 0, "not standing": 0, "no clear floor": 0}
    while reader.has_next():
        _, raw, _ = reader.read_next()
        k += 1
        if k % every:
            continue
        if max_scans and seen >= max_scans:
            break
        seen += 1
        raw = bytes(raw)
        sec, nsec = struct.unpack("<iI", raw[4:12])
        t = sec + nsec * 1e-9
        if abs(odom.pitch_near(t) - level) > 4.0:
            why["tilted body"] += 1
            continue
        if np.interp(t, bt, bh) < 0.25:
            why["not standing"] += 1
            continue
        pts = L._points(raw)
        if pts is None or len(pts) < 5000:
            continue
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
        ok = np.isfinite(xyz).all(axis=1)
        xyz, stamps = xyz[ok], pts["timestamp"][ok]
        xyz = L.deskew(xyz, stamps - stamps.min(), t, odom, T_bl)
        horiz = np.linalg.norm(xyz[:, :2], axis=1)
        # Where the floor can be: 0.25 to 0.75 m below the sensor (it stands
        # about 0.48 m up), 1.3 to 6 m out (closer is below the lowest laser).
        cand = xyz[(xyz[:, 2] < -0.25) & (xyz[:, 2] > -0.75) & (horiz > 1.3) & (horiz < 6.0)]
        found = None
        for _ in range(3):
            # In a corridor the lower walls can outnumber the floor, and the
            # plane fit then finds a wall. Take it out and look again.
            if len(cand) < 800:
                break
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cand))
            plane, inliers = pc.segment_plane(0.03, 3, 300)
            n = np.array(plane[:3])
            dist = plane[3]
            if n[2] < 0:
                n, dist = -n, -dist
            # Plane: n.x + dist = 0, so the sensor is `dist` above it along n.
            if len(inliers) >= 800 and np.degrees(np.arccos(min(1.0, n[2]))) < 20 \
                    and 0.25 < dist < 0.8:
                found = (n, dist)
                break
            keep = np.ones(len(cand), bool)
            keep[inliers] = False
            cand = cand[keep]
        if found is None:
            why["no clear floor"] += 1
            continue
        n, dist = found
        up_body = odom.at(t)[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        pairs.append((n / np.linalg.norm(n), up_body, t, dist))
    return pairs, why, seen


def yaw_from_build(walk):
    """Mount yaw error from a built map: direction of travel minus heading.

    Returns (yaw the build should have used, degrees; walking samples) or None.
    """
    build = os.path.join(walk, "map3d")
    traj = os.path.join(build, "trajectory.csv")
    if not os.path.isfile(traj):
        return None
    used = 0.0
    info_path = os.path.join(build, "build.json")
    if os.path.isfile(info_path):
        with open(info_path) as fh:
            used = float(json.load(fh).get("lidar_rpy_deg", [0, 0, 0])[2])
    d = np.genfromtxt(traj, delimiter=",", names=True)
    t = d["time"]
    p = np.stack([d["x"], d["y"]], 1)
    yaw = Rotation.from_quat(np.stack([d["qx"], d["qy"], d["qz"], d["qw"]], 1)).as_euler("ZYX")[:, 0]
    step = 5
    mv = p[step:] - p[:-step]
    walking = np.linalg.norm(mv, axis=1) / (t[step:] - t[:-step]) > 0.3
    if walking.sum() < 50:
        return None
    diff = np.degrees((np.arctan2(mv[walking, 1], mv[walking, 0]) - yaw[:-step][walking]
                       + np.pi) % (2 * np.pi) - np.pi)
    # The body moves along its heading, so the build's heading was off by diff:
    # the mount's yaw should have been larger by -diff.
    return used - float(np.median(diff)), int(walking.sum())


def fit(pairs, yaw_deg=0.0):
    """Roll and pitch (degrees) of the LiDAR on the body, and residuals."""
    N = np.array([p[0] for p in pairs])
    U = np.array([p[1] for p in pairs])

    def rot(x):
        return Rotation.from_euler("xyz", [x[0], x[1], yaw_deg], degrees=True).as_matrix()

    def res(x):
        return ((rot(x) @ N.T).T - U).ravel()

    r = least_squares(res, [0.0, 0.0], loss="soft_l1", f_scale=np.sin(np.radians(1.0)))
    err = np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", (rot(r.x) @ N.T).T, U), -1, 1)))
    return r.x, err


def main():
    ap = argparse.ArgumentParser(description="calibrate the LiDAR's tilt on the robot from walks")
    ap.add_argument("walks", nargs="+")
    ap.add_argument("--every", type=int, default=3, help="use every Nth scan")
    ap.add_argument("--max-scans", type=int, default=0)
    ap.add_argument("--out", default=None, help="folder for lidar_mount.json (default: walks folder)")
    ap.add_argument("--yaw", type=float, default=None,
                    help="mount yaw in degrees; default: measured from the walks' built maps")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    started = time.time()
    # --- yaw first ---------------------------------------------------------
    yaws = {}
    for walk in args.walks:
        r = yaw_from_build(os.path.abspath(walk))
        if r is not None:
            yaws[os.path.basename(os.path.abspath(walk))] = r
            print("  %-14s heading check from its map: mount yaw %+.1f deg (%d walking samples)"
                  % (os.path.basename(os.path.abspath(walk)), r[0], r[1]))
    if args.yaw is not None:
        yaw = args.yaw
    elif yaws:
        yaw = float(np.median([v[0] for v in yaws.values()]))
    else:
        sys.exit("no built map to measure yaw from: process a walk first, or give --yaw")
    if len(yaws) > 1 and np.ptp([v[0] for v in yaws.values()]) > 5:
        sys.exit("the walks disagree on yaw by over 5 degrees; check the maps")
    print("  using mount yaw %+.1f deg" % yaw)

    T_docs = np.eye(4)
    T_docs[:3, :3] = Rotation.from_euler("z", yaw, degrees=True).as_matrix()
    T_docs[:3, 3] = L.LIDAR_IN_BASE
    per_walk, all_pairs = {}, []
    for walk in args.walks:
        walk = os.path.abspath(walk)
        pairs, why, seen = collect(walk, args.every, T_docs, args.max_scans)
        name = os.path.basename(walk)
        if len(pairs) < 50:
            print("  %s: only %d usable scans (%s), skipped" % (name, len(pairs), why))
            continue
        x, err = fit(pairs, yaw)
        heights = np.array([p[3] for p in pairs])
        per_walk[name] = {"roll_deg": round(float(x[0]), 3), "pitch_deg": round(float(x[1]), 3),
                          "scans": len(pairs), "median_error_deg": round(float(np.median(err)), 3),
                          "lidar_height_above_floor_m": round(float(np.median(heights)), 3)}
        # Halves of the walk, as a stability check.
        half = len(pairs) // 2
        a, _ = fit(pairs[:half], yaw)
        b, _ = fit(pairs[half:], yaw)
        print("  %-14s %4d of %4d scans used: roll %+.2f, pitch %+.2f deg; floor-to-IMU disagreement "
              "median %.2f deg; first half %+.2f/%+.2f, second half %+.2f/%+.2f; LiDAR %.3f m above floor"
              % (name, len(pairs), seen, x[0], x[1], np.median(err), a[0], a[1], b[0], b[1],
                 np.median(heights)), flush=True)
        print("                 skipped: %s" % ", ".join("%s %d" % kv for kv in why.items()))
        all_pairs += pairs

    if not per_walk:
        sys.exit("no walk had enough usable scans")
    x, err = fit(all_pairs, yaw)
    _, err0 = fit_identity(all_pairs, yaw)
    spread = (max(w["pitch_deg"] for w in per_walk.values()) - min(w["pitch_deg"] for w in per_walk.values()),
              max(w["roll_deg"] for w in per_walk.values()) - min(w["roll_deg"] for w in per_walk.values()))
    print()
    print("  all walks: roll %+.2f, pitch %+.2f deg from %d scans" % (x[0], x[1], len(all_pairs)))
    print("  floor vs IMU disagreement, median: %.2f deg with no tilt, "
          "%.2f deg with this tilt" % (np.median(err0), np.median(err)))
    if len(per_walk) > 1:
        print("  walks agree to within pitch %.2f, roll %.2f deg" % spread)
    print("  %.0f s" % (time.time() - started))

    if len(per_walk) > 1 and max(spread) > 1.0:
        print("  NOT WRITTEN: the walks disagree by over 1 degree, so this is not only the mount.")
        sys.exit(2)
    if args.dry_run:
        return
    out = args.out or os.path.dirname(os.path.abspath(args.walks[0]))
    path = os.path.join(out, L.LIDAR_MOUNT_FILE)
    with open(path, "w") as fh:
        json.dump({
            "parent": "base_link", "child": "hesai_lidar",
            "xyz": list(L.LIDAR_IN_BASE),
            "rpy_deg": [round(float(x[0]), 3), round(float(x[1]), 3), round(yaw, 2)],
            "source": "position from Unitree's documentation; yaw from the direction of travel "
                      "along built maps; roll and pitch fitted from the floor against the IMU "
                      "over %d scans (%s)" % (len(all_pairs), ", ".join(per_walk)),
            "yaw_per_walk": {k: round(v[0], 2) for k, v in yaws.items()},
            "median_disagreement_deg": {"documented_no_tilt": round(float(np.median(err0)), 3),
                                        "with_this_tilt": round(float(np.median(err)), 3)},
            "per_walk": per_walk,
            "date": time.strftime("%Y-%m-%d"),
        }, fh, indent=2)
    print("  wrote %s" % path)
    print("  Rebuild the maps to use it:  process_walk.sh <walk> --map-only")


def fit_identity(pairs, yaw_deg=0.0):
    N = np.array([p[0] for p in pairs])
    U = np.array([p[1] for p in pairs])
    R = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    return None, np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", (R @ N.T).T, U), -1, 1)))


if __name__ == "__main__":
    main()
