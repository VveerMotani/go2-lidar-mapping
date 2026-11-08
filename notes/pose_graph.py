#!/usr/bin/env python3
"""
Loop closure: correct a scan-matched track so passes over the same place agree.

    python3 pose_graph.py ~/go2-lidar-mapping/walks/odom-test-2 --out /tmp/test-build
    python3 pose_graph.py <walk> --out DIR --set neighbours=2   (other settings)

lidar_map.py calls correct() on every build (unless --no-loop-closure). Run
alone, as above, it corrects an existing build's track (map3d/trajectory.csv)
into DIR without touching the build, which is how its settings were tested.

WHAT WAS WRONG (measured 2026-09-25, before this existed)
---------------------------------------------------------
lidar_map.py matches each scan against its last 25 keyframes only. Anything
it gets slightly wrong is never corrected by what it saw earlier. Measured
by aligning 2 m stretches of the walk to other stretches over the same place
(map_quality.py, "passes agree"):

                        odom-test-1        odom-test-2
    heading             0.1 deg            0.2 deg          fine
    sideways            1.7 cm             5.3 cm           small
    height              3.0 cm (worst 12)  1.7 cm (worst 12)
    tilt                0.6 deg (worst 1.6) 0.4 deg (worst 2.0)

Tilt is the damage: 1 degree at 8 m moves the floor 14 cm, and more on a
ceiling seen from 10 m. In odom-test-1's corridor the ceiling came out
twice, 30-40 cm apart, on the way out and the way back.

The track's end error, from aligning the last standing scans to the first
(the robot really ended 17 cm and 3 cm from where it started): 4.2 cm and
1.28 deg on odom-test-1, 9.9 cm and 1.20 deg on odom-test-2.

HOW
---
A pose graph over the front end's keyframes (every 0.4 m or 8 degrees),
solved once the whole walk has been matched:

  odometry edges  the front end's motion between consecutive keyframes:
                  locally very good, so stiff, but it drifts. The rotation
                  is kept relative; the step is kept as the front end
                  measured it, on the map facing the keyframe's heading
                  (see PoseGraph), height from the legs on the flat.
  loop edges      where the walk comes back within 3 m of a keyframe after
                  at least 8 m of walking, the two places' submaps (a
                  keyframe and its four neighbours either side, about 3 m
                  of walk) are aligned by point-to-plane ICP. Each edge is
                  weighted by its own geometry (the point-to-plane
                  information matrix): a corridor constrains nothing along
                  its length, and the edge then says nothing in that
                  direction instead of something wrong.

Solved by robust least squares (scipy, soft-L1), so an edge that disagrees
with everything else loses its pull rather than bending the track. Every
scan between two keyframes is then moved by the two keyframes' corrections,
blended by time. About 10 s per walk.

Nodes are base_link poses; clouds are in base_link.

RESULT (map_quality.py, before -> after, both walks rebuilt 2026-09-25)
------
                        odom-test-1              odom-test-2
    end error           4.2 cm 1.28 -> 2.2 0.17  9.9 cm 1.20 -> 1.0 0.30
    height, passes      3.0 cm -> 1.2            1.7 cm -> 0.8
      worst             12.3 cm -> 5.4           12.1 cm -> 4.8
    sideways            1.7 cm -> 0.5            5.3 cm -> 0.6
    tilt                0.6 deg -> 0.4           0.4 deg -> 0.2
    floor, dense cloud  14.6 cm -> 8.5           17.9 cm -> 17.1 (its thick
                                                 cells are walls at corridor edges)

WHAT DID NOT WORK (2026-09-25, each measured on both walks)
-----------------
* Gravity priors: the IMU's roll and pitch (Unitree's odometry quaternion)
  at every keyframe, soft at 1 degree. The track's tilt differs from the
  IMU's by 1.5-2.5 degrees for 10-40 s at a time, which looked like the
  scan matcher drifting. It is at least partly the IMU's estimate: pulled
  towards it, keyframes turned by up to 2.6 degrees and odom-test-1's
  passes disagreed more (tilt 0.6 -> 0.8 deg, height 90th percentile
  7.5 -> 28 cm). At 3 degrees it was no better than none. --gravity keeps it
  for comparison. (The same lesson as --vertical gravity in lidar_map.py,
  now with the mount calibrated.)
* Odometry steps kept in the keyframe's own tilted frame: correcting a 2
  degree tilt turned every step, and 10 m of walking gained 35 cm of height
  (keyframes moved 0.41 m on odom-test-1). Hence level steps.
* Loop ICP accepted only below 8 cm RMS: that is the typical RMS at a 10 cm
  grid (median 8.1-8.6 cm), so half of good matches were thrown away (8 of
  48 kept on odom-test-1). Now 12 cm; the robust solver copes with the rest.
* Submaps of a keyframe and two neighbours: odom-test-1's loop around its
  start came out worse than before (height 90th percentile 7.5 -> 11.6 cm,
  worst 14.5): 1.5 m of walk does not pin tilt down with the XT16, which
  sees no floor within 1.5 m. Four neighbours fixed it; six was no better
  and four times slower. Weaker loop edges (a fifth of the weight) helped
  less than larger submaps.

  Found later the same day (lidar_map.py, HEIGHT AND THE DEPTH CAMERA):
  rebuilding with a change that should not matter (LiDAR range 29.5 m
  instead of 30) moves the end error's tilt between 0.3 and 1.1 degrees and
  passes' worst height by a centimetre or two. Four against six neighbours
  was within that; the rejections above were well outside it.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEFAULTS = dict(
    key_move=0.4, key_turn=8.0,           # keyframe spacing, m and degrees
    loop_near=3.0, loop_apart=8.0,        # revisit: within 3 m, 8 m of walking between
    loop_dz=0.8,                          # same level only
    loop_every=2,                         # try loops from every 2nd keyframe
    neighbours=4,                         # submap: keyframe +- this many (about 3 m of walk)
    icp_voxel=0.10, max_range=20.0,
    fit_min=0.50, rmse_max=0.12,          # accept a loop ICP (rmse is 8-9 cm typically)
    jump_max=0.6, turn_max=4.0,           # ... that stays this close to the track
    loop_sigma=0.02, loop_points=25,      # loop edge weight: 2 cm over 25 points' worth
    odo_rot_deg=(0.2, 0.2, 0.05),         # per odometry edge: roll, pitch, yaw
    odo_xyz=(0.01, 0.01, 0.005),          # per odometry edge, m (z: legs on the flat)
    odo_z_climbing=0.03,                  # z on stairs, where height is scan matched
    gravity_deg=1.0,                      # IMU roll/pitch, when gravity is on
    gravity=False,                        # off: it made the maps worse (see WHAT DID NOT WORK)
)


# --------------------------------------------------------------------------
# small SE(3) helpers
# --------------------------------------------------------------------------
def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def sqrt_info(info):
    """L with L.T @ L == info, for a symmetric positive semi-definite info."""
    w, V = np.linalg.eigh(0.5 * (info + info.T))
    return (V * np.sqrt(np.clip(w, 0, None))).T


def blend(T0, T1, a):
    T = np.eye(4)
    T[:3, 3] = (1 - a) * T0[:3, 3] + a * T1[:3, 3]
    r = Rotation.from_matrix(np.stack([T0[:3, :3], T1[:3, :3]]))
    rel = (r[0].inv() * r[1]).as_rotvec()
    T[:3, :3] = (r[0] * Rotation.from_rotvec(a * rel)).as_matrix()
    return T


# --------------------------------------------------------------------------
# the graph
# --------------------------------------------------------------------------
class PoseGraph(object):
    """Nodes: poses X_i (world from node). Edge (i, j, Z, info): X_i^-1 X_j
    should equal Z; the error, log(Z^-1 X_i^-1 X_j) as [rotation vector,
    translation] in j's frame, is weighted by info (6x6). Gravity (i, up):
    the node's "up" in its own frame should be `up`.

    A level edge keeps the relative rotation the same way, but its
    translation is the step in the world as seen facing node i's heading
    (turned about the vertical only). The front end's steps are measured
    that way: height from the legs, x and y on the map. Kept in node i's
    own tilted frame instead, correcting a tilt of 2 degrees turned every
    step with it, and 10 m of walking gained 35 cm of height that nobody
    measured (0.41 m on odom-test-1, the first version of this file)."""

    def __init__(self, poses):
        poses = np.asarray(poses)
        self.R0 = Rotation.from_matrix(poses[:, :3, :3])
        self.t0 = poses[:, :3, 3].copy()
        self.n = len(poses)
        self.edges = []
        self.grav = []

    @staticmethod
    def heading(R):
        m = R.as_matrix() if isinstance(R, Rotation) else R
        return Rotation.from_euler("z", np.arctan2(m[..., 1, 0], m[..., 0, 0]))

    def add_edge(self, i, j, Z, info, kind, level=False):
        Z = np.array(Z, float)
        if level:
            Z[:3, 3] = self.heading(self.R0[i]).inv().apply(self.t0[j] - self.t0[i])
        self.edges.append((i, j, Z, sqrt_info(info), kind, level))

    def add_gravity(self, i, up_body, sigma_rad):
        self.grav.append((i, np.asarray(up_body, float) / np.linalg.norm(up_body), 1.0 / sigma_rad))

    def _state(self, x):
        d = x.reshape(-1, 6)
        R = self.R0 * Rotation.from_rotvec(d[:, :3])
        t = self.t0 + self.R0.apply(d[:, 3:])
        return R, t

    def _prepare(self):
        E = self.edges
        self.I = np.array([e[0] for e in E], int)
        self.J = np.array([e[1] for e in E], int)
        Z = np.stack([e[2] for e in E])
        self.RZinv = Rotation.from_matrix(Z[:, :3, :3]).inv()
        self.tZ = Z[:, :3, 3]
        self.Ls = np.stack([e[3] for e in E])
        self.level = np.array([e[5] for e in E], bool)
        self.G = np.array([g[0] for g in self.grav], int)
        self.up = np.array([g[1] for g in self.grav]).reshape(-1, 3)
        self.gw = np.array([g[2] for g in self.grav])

    def edge_errors(self, x):
        R, t = self._state(x)
        Ri_inv = R[self.I].inv()
        Rrel = Ri_inv * R[self.J]
        trel = Ri_inv.apply(t[self.J] - t[self.I])
        Re = self.RZinv * Rrel
        te = self.RZinv.apply(trel - self.tZ)
        if self.level.any():
            m = self.level
            I = self.I[m]
            step = self.heading(R[I]).inv().apply(t[self.J[m]] - t[I])
            te[m] = step - self.tZ[m]
        return np.hstack([Re.as_rotvec(), te])

    def residuals(self, x):
        e = self.edge_errors(x)
        r = [np.einsum("eab,eb->ea", self.Ls, e).ravel()]
        if len(self.G):
            R, _ = self._state(x)
            est = R[self.G].inv().apply([0.0, 0.0, 1.0])
            r.append((np.cross(est, self.up) * self.gw[:, None]).ravel())
        r.append(1e3 * x[:6])                               # node 0 stays put
        return np.concatenate(r)

    def sparsity(self):
        ne, ng = len(self.I), len(self.G)
        S = lil_matrix((6 * ne + 3 * ng + 6, 6 * self.n), dtype=np.int8)
        for k, (i, j) in enumerate(zip(self.I, self.J)):
            S[6 * k:6 * k + 6, 6 * i:6 * i + 6] = 1
            S[6 * k:6 * k + 6, 6 * j:6 * j + 6] = 1
        for k, i in enumerate(self.G):
            S[6 * ne + 3 * k:6 * ne + 3 * k + 3, 6 * i:6 * i + 3] = 1
        S[6 * ne + 3 * ng:, 0:6] = 1
        return S

    def solve(self, f_scale=3.0, max_nfev=60):
        self._prepare()
        x0 = np.zeros(6 * self.n)
        res = least_squares(self.residuals, x0, jac_sparsity=self.sparsity(), method="trf",
                            loss="soft_l1", f_scale=f_scale, max_nfev=max_nfev, x_scale="jac")
        R, t = self._state(res.x)
        X = np.tile(np.eye(4), (self.n, 1, 1))
        X[:, :3, :3] = R.as_matrix()
        X[:, :3, 3] = t
        return X, res


# --------------------------------------------------------------------------
# loop edges
# --------------------------------------------------------------------------
def pick_keyframes(poses, move, turn):
    keys = [0]
    for s in range(1, len(poses)):
        d = inv(poses[keys[-1]]) @ poses[s]
        if np.linalg.norm(d[:3, 3]) > move or angle_deg(d[:3, :3]) > turn:
            keys.append(s)
    if keys[-1] != len(poses) - 1:
        keys.append(len(poses) - 1)
    return np.array(keys)


def loop_candidates(K, o):
    """(a, b) keyframe index pairs, a < b: b comes back within loop_near of a
    after loop_apart of walking. Per keyframe a, the nearest b of each
    separate return."""
    pos = K[:, :3, 3]
    path = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1))]
    pairs = []
    for a in range(0, len(K), o["loop_every"]):
        far = path - path[a] >= o["loop_apart"]
        near = (np.linalg.norm(pos[:, :2] - pos[a, :2], axis=1) <= o["loop_near"]) \
            & (np.abs(pos[:, 2] - pos[a, 2]) <= o["loop_dz"])
        ok = np.flatnonzero(far & near)
        if not len(ok):
            continue
        runs = np.split(ok, np.flatnonzero(np.diff(ok) > 1) + 1)
        for run in runs:
            d = np.linalg.norm(pos[run, :2] - pos[a, :2], axis=1)
            pairs.append((a, int(run[np.argmin(d)])))
    return pairs


class Submaps(object):
    """A keyframe and its neighbours either side, in the keyframe's frame,
    thinned, with normals. Built once each. (ICP runs in this process: Open3D
    uses every core itself, and its thread pool deadlocks in forked workers.)"""

    def __init__(self, K, clouds, neighbours, voxel):
        self.K, self.clouds, self.nb, self.voxel = K, clouds, neighbours, voxel
        self.cache = {}

    def __getitem__(self, k):
        if k not in self.cache:
            import open3d as o3d
            Tk_inv = inv(self.K[k])
            parts = []
            for m in range(max(0, k - self.nb), min(len(self.K), k + self.nb + 1)):
                T = Tk_inv @ self.K[m]
                parts.append((T[:3, :3] @ self.clouds[m].T).T + T[:3, 3])
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(parts)))
            pc = pc.voxel_down_sample(self.voxel)
            pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
            self.cache[k] = pc
        return self.cache[k]


def match(pair, K, submaps, o):
    """ICP between two keyframes' submaps. Returns (a, b, Z, info, stats) or
    (a, b, None, None, why)."""
    import open3d as o3d
    reg = o3d.pipelines.registration
    a, b = pair
    src, tgt = submaps[a], submaps[b]
    init = inv(K[b]) @ K[a]                        # maps a's points into b's frame
    r1 = reg.registration_icp(src, tgt, 0.5, init,
                              reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=0.3)),
                              reg.ICPConvergenceCriteria(max_iteration=40))
    r2 = reg.registration_icp(src, tgt, 0.2, r1.transformation,
                              reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=0.1)),
                              reg.ICPConvergenceCriteria(max_iteration=40))
    T = np.array(r2.transformation)
    step = inv(init) @ T
    jump, turn = float(np.linalg.norm(step[:3, 3])), angle_deg(step[:3, :3])
    stats = dict(fitness=float(r2.fitness), rmse=float(r2.inlier_rmse), jump=jump, turn=turn)
    if r2.fitness < o["fit_min"] or r2.inlier_rmse > o["rmse_max"]:
        return a, b, None, None, dict(stats, why="poor fit")
    if jump > o["jump_max"] or turn > o["turn_max"]:
        return a, b, None, None, dict(stats, why="too far from the track")
    # Point-to-plane information in b's frame, [rotation, translation].
    cs = np.asarray(r2.correspondence_set)
    q = np.asarray(src.points)[cs[:, 0]]
    q = (T[:3, :3] @ q.T).T + T[:3, 3]
    n = np.asarray(tgt.normals)[cs[:, 1]]
    J = np.hstack([np.cross(q, n), n])
    H = J.T @ J / len(J) * o["loop_points"] / o["loop_sigma"] ** 2
    # The edge says X_a^-1 X_b = inv(T).
    return a, b, inv(T), H, stats


# --------------------------------------------------------------------------
# the whole correction
# --------------------------------------------------------------------------
def correct(times, poses, cloud_of, imu_rot_at, climbing_at=None, options=None, log=print,
            keys=None):
    """Loop-close a track.

    times, poses   every scan's time and base_link pose (N, N x 4 x 4)
    cloud_of(s)    scan s's points in base_link (called for keyframes only)
    imu_rot_at(t)  the IMU's body orientation at time t (scipy Rotation);
                   used only with the gravity setting
    climbing_at(t) True on stairs and ramps (height scan matched, less sure)
    keys           scan indices to use as keyframes (default: every 0.4 m or
                   8 degrees, as lidar_map.py picks them)

    Returns (corrected poses N x 4 x 4, report dict).
    """
    o = dict(DEFAULTS)
    o.update(options or {})
    started = time.time()
    poses = np.asarray(poses)
    if keys is None:
        keys = pick_keyframes(poses, o["key_move"], o["key_turn"])
    keys = np.asarray(keys)
    K = poses[keys]
    log("  loop closure: %d keyframes of %d scans" % (len(keys), len(poses)))
    clouds = []
    for s in keys:
        c = cloud_of(s)
        c = c[np.linalg.norm(c, axis=1) < o["max_range"]]
        clouds.append(c.astype(np.float64))

    g = PoseGraph(K)
    for k in range(len(keys) - 1):
        info = np.zeros(6)
        info[:3] = 1.0 / np.radians(o["odo_rot_deg"]) ** 2
        sig = list(o["odo_xyz"])
        if climbing_at is not None and (climbing_at(times[keys[k]]) or climbing_at(times[keys[k + 1]])):
            sig[2] = o["odo_z_climbing"]
        info[3:] = 1.0 / np.array(sig) ** 2
        g.add_edge(k, k + 1, inv(K[k]) @ K[k + 1], np.diag(info), "odometry", level=True)
    if o["gravity"]:
        for k, s in enumerate(keys):
            up = imu_rot_at(times[s]).inv().apply([0.0, 0.0, 1.0])
            g.add_gravity(k, up, np.radians(o["gravity_deg"]))

    pairs = loop_candidates(K, o)
    submaps = Submaps(K, clouds, o["neighbours"], o["icp_voxel"])
    results = [match(p, K, submaps, o) for p in pairs]
    rejected = {}
    loops = []
    tried = []
    for a, b, Z, H, st in results:
        tried.append(dict(st, a=int(keys[a]), b=int(keys[b]), kept=Z is not None))
        if Z is None:
            rejected[st["why"]] = rejected.get(st["why"], 0) + 1
            continue
        g.add_edge(a, b, Z, H, "loop")
        loops.append((a, b, st))
    log("  loop closure: %d revisits tried, %d kept%s (%.0f s)"
        % (len(pairs), len(loops),
           "".join(", %d %s" % (v, k) for k, v in sorted(rejected.items())),
           time.time() - started))

    X, res = g.solve()
    # How far each edge disagrees with the track, before and after.
    kinds = np.array([e[4] for e in g.edges])
    before = g.edge_errors(np.zeros(6 * g.n))
    x_after = np.concatenate([
        np.hstack([(Rotation.from_matrix(K[i, :3, :3]).inv() * Rotation.from_matrix(X[i, :3, :3])).as_rotvec(),
                   K[i, :3, :3].T @ (X[i, :3, 3] - K[i, :3, 3])]) for i in range(g.n)])
    after = g.edge_errors(x_after)

    def summary(e, m):
        if not m.any():
            return None
        tr = np.linalg.norm(e[m, 3:], axis=1) * 100
        rot = np.degrees(np.linalg.norm(e[m, :3], axis=1))
        return dict(cm_median=round(float(np.median(tr)), 2), cm_90=round(float(np.percentile(tr, 90)), 2),
                    deg_median=round(float(np.median(rot)), 3), deg_90=round(float(np.percentile(rot, 90)), 3))

    moved = np.array([np.linalg.norm(X[i, :3, 3] - K[i, :3, 3]) for i in range(g.n)])
    turned = np.array([angle_deg(K[i, :3, :3].T @ X[i, :3, :3]) for i in range(g.n)])

    # Every scan: the two surrounding keyframes' corrections, blended by time.
    out = np.empty_like(poses)
    kt = times[keys]
    for s in range(len(poses)):
        j = int(np.searchsorted(keys, s))
        if j < len(keys) and keys[j] == s:
            out[s] = X[j]
            continue
        a_, b_ = max(j - 1, 0), min(j, len(keys) - 1)
        Pa = X[a_] @ inv(K[a_]) @ poses[s]
        Pb = X[b_] @ inv(K[b_]) @ poses[s]
        w = 0.0 if kt[b_] <= kt[a_] else float(np.clip((times[s] - kt[a_]) / (kt[b_] - kt[a_]), 0, 1))
        out[s] = blend(Pa, Pb, w)

    report = dict(
        keyframes=int(len(keys)), revisits_tried=len(pairs), loop_edges=len(loops),
        rejected=rejected, gravity=bool(o["gravity"]), tried=tried,
        loop_misfit_before=summary(before, kinds == "loop"),
        loop_misfit_after=summary(after, kinds == "loop"),
        odometry_change=summary(after, kinds == "odometry"),
        largest_move_m=round(float(moved.max()), 3), largest_turn_deg=round(float(turned.max()), 3),
        solver=dict(status=int(res.status), evaluations=int(res.nfev), cost=float(res.cost)),
        seconds=round(time.time() - started, 1),
        settings={k: v for k, v in o.items()})
    if loops:
        b_, a_ = report["loop_misfit_before"], report["loop_misfit_after"]
        log("  loop edges disagree with the track by %.1f cm, %.2f deg (median) before; "
            "%.1f cm, %.2f deg after" % (b_["cm_median"], b_["deg_median"], a_["cm_median"], a_["deg_median"]))
    log("  track moved by up to %.2f m and %.2f deg; %.0f s"
        % (report["largest_move_m"], report["largest_turn_deg"], report["seconds"]))
    return out, report


# --------------------------------------------------------------------------
# standalone: correct an existing build's track
# --------------------------------------------------------------------------
def _read_keyframe_scans(walk, times, want, T_bl, odom, max_range, deskew):
    """Deskewed scans at the wanted times, in base_link. {scan index: xyz}."""
    import struct
    import rosbag2_py
    import lidar_map as L
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(walk, "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lidar_points"]))
    want_t = times[want]
    out = {}
    while reader.has_next():
        _, raw, _ = reader.read_next()
        raw = bytes(raw)
        sec, nsec = struct.unpack("<iI", raw[4:12])
        t = sec + nsec * 1e-9
        j = int(np.searchsorted(want_t, t))
        hit = [m for m in (j - 1, j) if 0 <= m < len(want_t) and abs(want_t[m] - t) < 0.02]
        if not hit:
            continue
        pts = L._points(raw)
        if pts is None:
            continue
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(xyz).all(1) & (r > 0.8) & (r < max_range)
        xyz, st = xyz[ok], pts["timestamp"][ok]
        if deskew and len(st):
            xyz = L.deskew(xyz, st - st.min(), t, odom, T_bl)
        xyz, _ = L.voxel_keep(xyz, 0.05)
        out[int(want[hit[0]])] = (T_bl[:3, :3] @ xyz.T).T + T_bl[:3, 3]
    return out


def main():
    import shutil
    import lidar_map as L
    ap = argparse.ArgumentParser(description="loop-close an existing build's track")
    ap.add_argument("walk")
    ap.add_argument("--build", default=None, help="map3d folder to read (default: the walk's)")
    ap.add_argument("--out", required=True, help="folder for the corrected trajectory.csv")
    ap.add_argument("--gravity", action="store_true",
                    help="add the IMU's roll and pitch as soft priors (made maps worse; for comparison)")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a setting, e.g. --set gravity_deg=2")
    args = ap.parse_args()
    walk = os.path.abspath(args.walk)
    build = os.path.abspath(args.build) if args.build else os.path.join(walk, "map3d")
    with open(os.path.join(build, "build.json")) as fh:
        info = json.load(fh)
    T_bl = np.eye(4)
    T_bl[:3, :3] = Rotation.from_euler("xyz", info["lidar_rpy_deg"], degrees=True).as_matrix()
    T_bl[:3, 3] = info["lidar_xyz"]
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    times = d["time"]
    poses = np.stack([L.pose_matrix((d["x"][i], d["y"][i], d["z"][i]),
                                    (d["qx"][i], d["qy"][i], d["qz"][i], d["qw"][i]))
                      for i in range(len(times))])
    odom = L.Odometry(os.path.join(walk, "odom.csv"))
    opts = {"gravity": args.gravity}
    for s in args.set:
        k, v = s.split("=", 1)
        opts[k] = json.loads(v)
    o = dict(DEFAULTS)
    o.update(opts)
    keys = pick_keyframes(poses, o["key_move"], o["key_turn"])
    t0 = time.time()
    scans = _read_keyframe_scans(walk, times, keys, T_bl, odom, o["max_range"], info.get("deskew", True))
    print("  read %d keyframe scans, %.0f s" % (len(scans), time.time() - t0))
    level = float(np.median(odom.pitch))
    corrected, report = correct(
        times, poses, lambda s: scans.get(int(s), np.zeros((0, 3))),
        lambda t: odom.slerp([min(max(t, odom.t[0]), odom.t[-1])])[0],
        lambda t: abs(odom.pitch_near(t) - level) >= 7.0, opts)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "trajectory.csv"), "w") as fh:
        fh.write("time,x,y,z,qx,qy,qz,qw,odom_x,odom_y,odom_z\n")
        for i, T in enumerate(corrected):
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            fh.write("%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n"
                     % ((times[i],) + tuple(T[:3, 3]) + tuple(q) + (d["odom_x"][i], d["odom_y"][i], d["odom_z"][i])))
    shutil.copy(os.path.join(build, "trajectory.csv"), os.path.join(args.out, "trajectory_frontend.csv"))
    info["loop_closure"] = report
    with open(os.path.join(args.out, "build.json"), "w") as fh:
        json.dump(info, fh, indent=2)
    print("  wrote %s" % args.out)


if __name__ == "__main__":
    main()
