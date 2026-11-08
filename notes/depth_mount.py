#!/usr/bin/env python3
"""
Find exactly where the depth camera sits on the robot, from a walk.

    python3 depth_mount.py <walk> --from-lidar 12,0,-8,15
    python3 depth_mount.py <walk> --measured 0.29,0,0.01,15
    python3 depth_mount.py <walk>                      (start from camera_mount.json)

Unitree publishes where the LiDAR is mounted, not the RealSense. This starts
from a tape measurement and refines it against the walk's LiDAR map:

  --from-lidar DX,DY,DZ,TILT   centimetres from the middle of the LiDAR's
                               window to the centre of the camera's glass
                               (forward, left, up) and degrees pointing down.
                               The same four numbers as the Launchpad's
                               "Measure where the camera is" step.
  --measured X,Y,Z,PITCH       metres in base_link, degrees down.

Writes camera_mount.json into the walks folder (or --out), which
retime_bag.py and lidar_map.py then use for every walk.

HOW
---
Needs the walk processed first (process_walk.sh): the LiDAR-only map
(map3d/lidar_map.npy), the scan-matched track (map3d/trajectory.csv) and the
recording's depth.stream.

For ~60 depth frames spread over the walk, each depth point is placed on the
map through the robot's pose at that instant and the candidate mount. The
mount is then adjusted (Gauss-Newton, point to plane, robust weights) until
the depth points lie on the LiDAR's surfaces. Every frame shares one mount, so
frames seen from different headings pin down different directions of it.

It reports how far it moved from the measurement and how well each direction
was determined. A direction the walk never constrained (for example a camera
that only ever saw the floor cannot fix its forward position) is flagged, and
the measurement is kept for it rather than trusting a guess.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from depth_stream import MOUNT_FILE, DepthStream, find_camera_mount  # noqa: E402

LIDAR_IN_BASE = np.array([0.171, 0.0, 0.0908])      # Unitree docs


def mount_matrix(xyz, rpy_deg):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T


def load_track(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["time"]
    pos = np.stack([d["x"], d["y"], d["z"]], axis=1)
    rot = Rotation.from_quat(np.stack([d["qx"], d["qy"], d["qz"], d["qw"]], axis=1))
    return t, pos, Slerp(t, rot)


def main():
    ap = argparse.ArgumentParser(description="refine the depth camera's mount from a walk")
    ap.add_argument("walk")
    ap.add_argument("--from-lidar", default=None, metavar="DX,DY,DZ,TILT")
    ap.add_argument("--measured", default=None, metavar="X,Y,Z,PITCH")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--step", type=int, default=6, help="every Nth depth pixel")
    ap.add_argument("--depth-max", type=float, default=3.5)
    ap.add_argument("--out", default=None,
                    help="where to write camera_mount.json (default: the walks folder)")
    ap.add_argument("--search", action="store_true",
                    help="no tape measurement: try a grid of plausible mounts and keep the "
                         "one whose depth lands closest to the LiDAR's surfaces")
    ap.add_argument("--dry-run", action="store_true", help="report only, write nothing")
    args = ap.parse_args()

    walk = os.path.abspath(args.walk)
    for need in ("depth.stream", "map3d/lidar_map.npy", "map3d/trajectory.csv"):
        if not os.path.isfile(os.path.join(walk, need)):
            sys.exit("%s is missing. Record the walk with the depth camera on, and "
                     "process it (process_walk.sh) first." % need)

    # --- starting point --------------------------------------------------
    if args.from_lidar:
        dx, dy, dz, tilt = (float(v) for v in args.from_lidar.split(","))
        xyz0 = LIDAR_IN_BASE + np.array([dx, dy, dz]) / 100.0
        rpy0 = np.array([0.0, tilt, 0.0])
        source = "tape measurement from the LiDAR: %s cm, %s deg down" % (
            ",".join(args.from_lidar.split(",")[:3]), args.from_lidar.split(",")[3])
    elif args.measured:
        x, y, z, pitch = (float(v) for v in args.measured.split(","))
        xyz0, rpy0 = np.array([x, y, z]), np.array([0.0, pitch, 0.0])
        source = "tape measurement in base_link: %s" % args.measured
    elif args.search:
        xyz0, rpy0 = np.array([0.28, 0.0, 0.0]), np.array([0.0, 15.0, 0.0])
        source = "searched from the walk, no tape measurement"
    else:
        T, m, path = find_camera_mount(walk)
        if T is None:
            sys.exit("no starting point: give --from-lidar (as measured), --measured or --search")
        xyz0, rpy0 = np.array(m["xyz"]), np.array(m.get("rpy_deg", [0, 0, 0]))
        source = m.get("measured", {}).get("source") or m.get("source", path)
    X = mount_matrix(xyz0, rpy0)

    # --- data --------------------------------------------------------------
    started = time.time()
    lidar = np.load(os.path.join(walk, "map3d", "lidar_map.npy")).astype(np.float64)
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(lidar))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=20))
    normals = np.asarray(pc.normals)
    tree = cKDTree(lidar)

    t_track, pos, slerp = load_track(os.path.join(walk, "map3d", "trajectory.csv"))
    ds = DepthStream(os.path.join(walk, "depth.stream"))
    frames = [f for f in ds.depth_frames() if t_track[0] <= f.t <= t_track[-1]]
    if len(frames) < 10:
        sys.exit("only %d depth frames overlap the track" % len(frames))
    info = ds.depth_info
    optical = frames[0].header.get("frame_id") or "camera_depth_optical_frame"
    T_co = ds.optical_in_camera_link(optical)
    if info is None or T_co is None:
        sys.exit("depth.stream lacks the calibration or the camera's own frames")
    fx, fy, cx, cy = info["K"][0], info["K"][4], info["K"][2], info["K"][5]

    pick = np.linspace(0, len(frames) - 1, min(args.frames, len(frames))).astype(int)
    C, R_i, t_i = [], [], []
    for k in pick:
        f = frames[k]
        img = f.image
        s = args.step
        d = img[::s, ::s].astype(np.float64) / 1000.0
        v, u = np.mgrid[0:img.shape[0]:s, 0:img.shape[1]:s]
        ok = (d > 0.3) & (d < args.depth_max)
        z = d[ok]
        q = np.stack([(u[ok] - cx) / fx * z, (v[ok] - cy) / fy * z, z], axis=1)
        c = (T_co[:3, :3] @ q.T).T + T_co[:3, 3]            # in camera_link
        R = slerp([f.t]).as_matrix()[0]
        p = np.array([np.interp(f.t, t_track, pos[:, i]) for i in range(3)])
        C.append(c)
        R_i.append(np.repeat(R[None], len(c), axis=0))
        t_i.append(np.repeat(p[None], len(c), axis=0))
    C = np.concatenate(C)
    R_i = np.concatenate(R_i)
    t_i = np.concatenate(t_i)
    print("  %d depth frames, %d points, against %d LiDAR map points"
          % (len(pick), len(C), len(lidar)), flush=True)

    def residuals(X, max_d):
        b = (X[:3, :3] @ C.T).T + X[:3, 3]                  # in base_link
        p = np.einsum("nij,nj->ni", R_i, b) + t_i           # in the map
        dist, idx = tree.query(p, distance_upper_bound=max_d)
        ok = np.isfinite(dist)
        n = normals[idx[ok]]
        r = np.einsum("ni,ni->n", n, p[ok] - lidar[idx[ok]])
        return b[ok], n, R_i[ok], r, ok.mean()

    def median_error(X):
        _, _, _, r, frac = residuals(X, 0.3)
        return float(np.median(np.abs(r))) if len(r) else float("nan"), frac

    def refine(X, stages=((0.30, 15), (0.15, 20), (0.08, 30), (0.04, 30))):
        """Gauss-Newton, point to plane, coarse to fine. Returns (X, JTJ).

        Height and tilt trade off against each other when the camera mostly
        sees floor, so the fine stages get many iterations: on a synthetic
        test a shorter schedule stopped 4 cm and 4 degrees short.
        """
        JTJ = None
        for max_d, iters in stages:
            for _ in range(iters):
                b, n, R, r, _ = residuals(X, max_d)
                if len(r) < 500:
                    break
                u = np.einsum("nji,nj->ni", R, n)                # R_i^T n
                J = np.concatenate([np.cross(b, u), u], axis=1)  # d r / d [w, v]
                k = max_d / 3.0
                w = np.where(np.abs(r) < k, 1.0, k / np.abs(r))  # Huber
                JTJ = (J * w[:, None]).T @ J
                g = (J * w[:, None]).T @ r
                xi = -np.linalg.solve(JTJ + 1e-6 * np.eye(6), g)
                dR = Rotation.from_rotvec(xi[:3]).as_matrix()
                X_new = np.eye(4)
                X_new[:3, :3] = dR @ X[:3, :3]
                X_new[:3, 3] = dR @ X[:3, 3] + xi[3:]
                X = X_new
                if np.linalg.norm(xi) < 1e-6:
                    break
        return X, JTJ

    if args.search:
        # Nothing measured to start from: try plausible places for the camera
        # on the robot's head and keep the fit that ends closest to the
        # LiDAR's surfaces. The fit pulls in from about 13 cm and 12 degrees
        # away, so this grid covers the possibilities.
        best = None
        tried = 0
        for x in (0.20, 0.28, 0.36):
            for y in (-0.06, 0.0, 0.06):
                for z in (-0.10, 0.0, 0.10):
                    for pitch in (0.0, 10.0, 20.0, 30.0):
                        Xs, _ = refine(mount_matrix([x, y, z], [0.0, pitch, 0.0]),
                                       stages=((0.30, 8), (0.15, 8)))
                        err, frac = median_error(Xs)
                        score = err + (0.0 if frac > 0.5 else 1.0)
                        tried += 1
                        if best is None or score < best[0]:
                            best = (score, Xs, (x, y, z, pitch), err, frac)
        xyz0 = np.array(best[2][:3])
        rpy0 = np.array([0.0, best[2][3], 0.0])
        X = best[1]
        print("  searched %d starting points; best came from xyz %.2f %.2f %.2f, pitch %.0f deg "
              "(median %.1f cm at that stage)" % ((tried,) + best[2] + (100 * best[3],)), flush=True)

    before, frac_before = median_error(mount_matrix(xyz0, rpy0))
    X, JTJ = refine(X)
    after, frac_after = median_error(X)

    # How well each direction was pinned down. Eigenvalues of JTJ, scaled per
    # point; a tiny one means the walk did not constrain that combination.
    weak = []
    if JTJ is not None:
        evals, evecs = np.linalg.eigh(JTJ / max(len(C), 1))
        names = ["roll", "pitch", "yaw", "forward", "left", "up"]
        for val, vec in zip(evals, evecs.T):
            if val < 1e-4:
                weak.append(names[int(np.argmax(np.abs(vec)))])

    xyz = X[:3, 3]
    rpy = Rotation.from_matrix(X[:3, :3]).as_euler("xyz", degrees=True)
    print("  measured  xyz %.3f %.3f %.3f m, roll %.1f pitch %.1f yaw %.1f deg"
          % (tuple(xyz0) + tuple(rpy0)))
    print("  refined   xyz %.3f %.3f %.3f m, roll %.1f pitch %.1f yaw %.1f deg"
          % (tuple(xyz) + tuple(rpy)))
    print("  moved %.1f cm and %.1f deg from the measurement"
          % (100 * np.linalg.norm(xyz - xyz0),
             np.degrees(np.linalg.norm(Rotation.from_matrix(
                 X[:3, :3] @ mount_matrix(xyz0, rpy0)[:3, :3].T).as_rotvec()))))
    print("  depth to LiDAR surface, median: %.1f cm before, %.1f cm after "
          "(points near a surface: %.0f%% -> %.0f%%)"
          % (100 * before, 100 * after, 100 * frac_before, 100 * frac_after))
    if weak:
        print("  WEAK: this walk barely constrained %s. Keeping the measurement for "
              "it would be safer; record a walk that turns and faces walls." % ", ".join(weak))
    print("  %.0f s" % (time.time() - started))

    if after > 0.05 or frac_after < 0.5:
        print("  NOT WRITTEN: the fit is poor (over 5 cm, or under half the points near "
              "a surface). Check the measurement's signs and that the map is sharp.")
        sys.exit(2)
    if args.dry_run:
        return
    out_dir = args.out or os.path.dirname(walk)
    path = os.path.join(out_dir, MOUNT_FILE)
    with open(path, "w") as fh:
        json.dump({
            "parent": "base_link", "child": "camera_link",
            "xyz": [round(float(v), 4) for v in xyz],
            "rpy_deg": [round(float(v), 3) for v in rpy],
            "source": "refined from walk %s: median %.1f cm to the LiDAR map"
                      % (os.path.basename(walk), 100 * after),
            "measured": {"xyz": [round(float(v), 4) for v in xyz0],
                         "rpy_deg": [float(v) for v in rpy0], "source": source},
            "refined": {"walk": os.path.basename(walk), "frames": int(len(pick)),
                        "points": int(len(C)), "median_before_m": round(before, 4),
                        "median_after_m": round(after, 4), "weak_directions": weak,
                        "date": time.strftime("%Y-%m-%d")},
        }, fh, indent=2)
    print("  wrote %s" % path)
    print("  Rebuild the map to place the depth on it:  process_walk.sh %s --map-only" % walk)


if __name__ == "__main__":
    main()
