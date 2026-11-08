#!/usr/bin/env python3
"""
Colour a walk's dense cloud from the colour frames the camera recorded.

    python3 colour_map.py ~/go2-lidar-mapping/walks/odom-test-2        (after dense_map.py)

dense_map.py calls colour_points() on every build of a walk with colour in
its depth.stream, and writes:

    <walk>_colour.ply     the cleaned cloud: camera colour where a colour
                          frame saw the point, height colours elsewhere,
                          and a per-point property colour_source (1 camera,
                          0 height colours), so a grey point can always be
                          told from grey paint. CloudCompare shows it as a
                          scalar field.
    <walk>_dense/         /colour_map beside /dense_map: fields rgb (packed,
                          Foxglove "RGB" colour mode) and colour_source.
    dense_colour.npy      N x 4 uint8, r g b colour_source, in dense_map.npy's
                          order, for measuring.

Run alone, as above, it re-colours an existing build's dense cloud and
rewrites only <walk>_colour.ply and dense_colour.npy.

HOW
---
Each colour frame (5 a second by default) is placed through the track at
its own time (interpolated between scans), camera_mount.json and Intel's
factory camera_link -> camera_color_optical_frame (1.5 cm from the depth
lens). Points within 10 m are projected into it. A point takes the frame's
colour only if nothing nearer hides it: it must be no farther than the depth
frame recorded with that colour frame (within 40 ms) measured along its ray,
+4 cm + 3% of the distance; where depth has no reading, a depth buffer
rendered from the cloud decides. Each point's colour is the weighted mean
over the frames that saw it: a frame counts 1/distance^2 (closer is sharper
and better placed), less while the robot turns fast (motion blur).

CHECKED (odom-test-2, 2026-09-25)
-------
* Held-out photos: with the frames within 3 s of 12 test frames left out,
  the coloured cloud seen from each test frame's pose correlates with the
  real photo at 0.62 (brightness, median); turned 2 degrees, 0.56.
  (map3d/colour_views.png shows views of the result.)
* Timing: colours agree best between frames with the frame times as
  recorded (disagreement 16.5 of 255 at 0 ms, 19-21 at +-66 ms).
* Calibration: turning the colour camera 1 degree about any axis from
  Intel's factory figure made frames agree less (16.5 -> 16.6-17.2).
* Coverage: 73% of the dense cloud: floor 94%, walls 69%, ceiling 56%
  (projection alone, without hiding, had estimated 56%: floor 80, walls 61,
  ceiling 8.6).

Tried and dropped: hiding judged by a depth buffer rendered from the cloud
alone coloured 26% of the floor (seen at a grazing angle the floor's own
distance changes by 15-20 cm across a buffer cell, so nearer floor hid
farther floor); a 6 m reach left the ceiling at 3%.

The colour is what the camera saw: a glossy floor carries its reflections,
exposure differs between frames, and anything the cleaning kept (a person
walking by) is coloured too.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_map as L  # noqa: E402

C = {}          # per-process state for the frame workers (inherited by fork)


def _setup(walk, xyz, track, rate, max_range, standing_at, time_shift=0.0, skip=None, turn_deg=None):
    from scipy.spatial import cKDTree
    from depth_stream import DepthStream, find_camera_mount
    ds = DepthStream(os.path.join(walk, "depth.stream"))
    frames = ds.color_frames()
    if not frames:
        return "no colour frames in depth.stream"
    T_bc, _, _ = find_camera_mount(walk)
    if T_bc is None:
        return "no camera_mount.json"
    T_co = ds.optical_in_camera_link(frames[0].header.get("frame_id") or "camera_color_optical_frame")
    info = frames[0].info
    if T_co is None or not info:
        return "no colour calibration or frames in depth.stream"
    K = info["K"]
    if turn_deg is not None:
        # a small extra rotation of the colour camera, in its optical frame
        # (x right, y down, z forward): for checking its calibration
        R = np.eye(4)
        R[:3, :3] = Rotation.from_rotvec(np.radians(turn_deg)).as_matrix()
        T_co = T_co @ R
    tt = track["time"]
    poses = np.stack([L.pose_matrix((track["x"][i], track["y"][i], track["z"][i]),
                                    (track["qx"][i], track["qy"][i], track["qz"][i], track["qw"][i]))
                      for i in range(len(tt))])
    # turning speed along the track, deg/s, for the motion blur weight
    rot = Rotation.from_matrix(poses[:, :3, :3])
    rate_deg = np.r_[0, np.degrees((rot[:-1].inv() * rot[1:]).magnitude()) / np.maximum(np.diff(tt), 1e-3)]
    every = max(1, int(round(15.0 / rate)))
    use = []
    for k, f in enumerate(frames):
        t = f.t + time_shift
        if k % every or not (tt[0] < t < tt[-1]):
            continue
        if standing_at is not None and not standing_at(t):
            continue
        if skip is not None and skip(f.t):
            continue
        use.append(f)
    depth = ds.depth_frames()
    di = ds.depth_info
    T_do = ds.optical_in_camera_link(depth[0].header.get("frame_id") or "camera_depth_optical_frame")         if depth else None
    C.update(xyz=xyz, tree=cKDTree(xyz), tt=tt, poses=poses, rate_deg=rate_deg,
             T_bo=T_bc @ T_co, K=(K[0], K[4], K[2], K[5]), w=info["width"], h=info["height"],
             max_range=max_range, time_shift=time_shift,
             depth=depth if (depth and di and T_do is not None) else None,
             depth_t=np.array([f.t for f in depth]) if depth else None,
             depth_K=(di["K"][0], di["K"][4], di["K"][2], di["K"][5]) if di else None,
             T_bd=(T_bc @ T_do) if T_do is not None else None)
    return use


def camera_pose(t):
    tt, poses = C["tt"], C["poses"]
    j = int(np.clip(np.searchsorted(tt, t), 1, len(tt) - 1))
    return L.interpolate(t, tt[j - 1], poses[j - 1], tt[j], poses[j]) @ C["T_bo"], \
        float(np.interp(t, tt, C["rate_deg"]))


def depth_near(t):
    """(depth image in metres, depth optical frame pose) of the depth frame
    recorded within 40 ms of t, or None."""
    if C.get("depth") is None:
        return None
    dt = C["depth_t"]
    j = int(np.clip(np.searchsorted(dt, t), 1, len(dt) - 1))
    j = j if abs(dt[j] - t) < abs(dt[j - 1] - t) else j - 1
    if abs(dt[j] - t) > 0.04:
        return None
    tq = dt[j] + C["time_shift"]
    tt, poses = C["tt"], C["poses"]
    k = int(np.clip(np.searchsorted(tt, tq), 1, len(tt) - 1))
    T = L.interpolate(tq, tt[k - 1], poses[k - 1], tt[k], poses[k]) @ C["T_bd"]
    return C["depth"][j].image.astype(np.float32) / 1000.0, T


def visible(T, cell=4, grow=1, depth=None):
    """Indices of cloud points this camera pose sees, their pixel (u, v) and
    distance.

    Occlusion: with `depth` (the depth frame recorded with this colour frame,
    from depth_near), a point is seen if it is no farther than what the depth
    camera measured along its ray (+4 cm + 3% of the distance). Where the
    depth has no reading, and without depth, by a depth buffer rendered from
    the cloud itself. That alone coloured only 26% of odom-test-2's floor:
    seen at a grazing angle, the floor's own distance changes by 15-20 cm
    across the buffer's cells, so nearer floor "hid" farther floor."""
    from scipy.ndimage import minimum_filter
    fx, fy, cx, cy = C["K"]
    W, H = C["w"], C["h"]
    idx = np.array(C["tree"].query_ball_point(T[:3, 3], C["max_range"]), dtype=np.int64)
    if len(idx) == 0:
        return idx, None, None, None
    p = C["xyz"][idx].astype(np.float64)
    pc = (p - T[:3, 3]) @ T[:3, :3]                   # into the optical frame
    z = pc[:, 2]
    ok = z > 0.3
    u = np.full(len(z), -1.0)
    v = np.full(len(z), -1.0)
    u[ok] = fx * pc[ok, 0] / z[ok] + cx
    v[ok] = fy * pc[ok, 1] / z[ok] + cy
    ok &= (u >= 1) & (u < W - 1) & (v >= 1) & (v < H - 1)
    idx, u, v, z = idx[ok], u[ok], v[ok], z[ok]
    if not len(idx):
        return idx, None, None, None
    gw, gh = W // cell + 1, H // cell + 1
    gi = (v // cell).astype(np.int64) * gw + (u // cell).astype(np.int64)
    zbuf = np.full(gw * gh, np.inf)
    np.minimum.at(zbuf, gi, z)
    zbuf = minimum_filter(zbuf.reshape(gh, gw), size=2 * grow + 1).ravel()
    seen = z <= zbuf[gi] + 0.05 + 0.03 * z
    if depth is not None:
        dimg, Td = depth
        dfx, dfy, dcx, dcy = C["depth_K"]
        pd = (C["xyz"][idx].astype(np.float64) - Td[:3, 3]) @ Td[:3, :3]
        zd = pd[:, 2]
        good = zd > 0.2
        ud = np.full(len(zd), -1)
        vd = np.full(len(zd), -1)
        ud[good] = np.rint(dfx * pd[good, 0] / zd[good] + dcx).astype(np.int64)
        vd[good] = np.rint(dfy * pd[good, 1] / zd[good] + dcy).astype(np.int64)
        good &= (ud >= 0) & (ud < dimg.shape[1]) & (vd >= 0) & (vd < dimg.shape[0])
        meas = np.zeros(len(zd), np.float32)
        meas[good] = dimg[vd[good], ud[good]]
        has = good & (meas > 0.2)
        seen = np.where(has, zd <= meas + 0.04 + 0.03 * zd, seen)
    return idx[seen], u[seen], v[seen], z[seen]


def _frame(f):
    import cv2
    T, turn = camera_pose(f.t + C["time_shift"])
    idx, u, v, z = visible(T, depth=depth_near(f.t))
    if not len(idx):
        return None
    img = cv2.imdecode(np.frombuffer(f.jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    rgb = img[v.astype(np.int64), u.astype(np.int64)][:, ::-1]      # BGR -> RGB
    w = (1.0 / np.maximum(z, 0.5) ** 2) / (1.0 + (turn / 30.0) ** 2)
    return idx, rgb.astype(np.float32), w.astype(np.float32)


def colour_points(walk, xyz, track, rate=5.0, max_range=10.0, standing_at=None, workers=None,
                  log=print, time_shift=0.0, skip=None, turn_deg=None):
    """Camera colour for every point a colour frame saw.

    Returns (rgb N x 3 uint8, source N uint8 (1 camera, 0 not seen), stats)
    or (None, None, why) when the walk has no usable colour."""
    started = time.time()
    use = _setup(walk, xyz, track, rate, max_range, standing_at, time_shift, skip, turn_deg)
    if isinstance(use, str):
        return None, None, use
    n = len(xyz)
    sums = np.zeros((n, 3))
    sq = np.zeros(n)
    wsum = np.zeros(n)
    views = np.zeros(n, np.int32)
    import multiprocessing as mp
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    from progress import Progress
    prog = Progress("colouring the cloud", len(use), "frames")
    with mp.get_context("fork").Pool(workers) as pool:
        for res in pool.imap_unordered(_frame, use, chunksize=4):
            prog.step()
            if res is None:
                continue
            idx, rgb, w = res
            for c in range(3):
                sums[:, c] += np.bincount(idx, rgb[:, c] * w, n)
            sq += np.bincount(idx, (rgb ** 2).sum(1) * w, n)
            wsum += np.bincount(idx, w, n)
            views += np.bincount(idx, minlength=n).astype(np.int32)
    prog.end()
    seen = wsum > 0
    rgb = np.zeros((n, 3), np.uint8)
    mean = sums[seen] / wsum[seen, None]
    rgb[seen] = np.clip(np.rint(mean), 0, 255).astype(np.uint8)
    # how much the frames that saw a point disagree about its colour
    var = np.clip(sq[seen] / wsum[seen] - (mean ** 2).sum(1), 0, None)
    spread = np.sqrt(var / 3.0)
    multi = views[seen] >= 3
    stats = dict(frames=len(use), points=int(n), coloured=int(seen.sum()),
                 share=round(float(seen.mean()), 3),
                 views_median=float(np.median(views[seen])) if seen.any() else 0.0,
                 disagreement=round(float(np.median(spread[multi])), 2) if multi.any() else None,
                 seconds=round(time.time() - started, 1))
    log("  colour: %d frames, %d of %d points coloured (%.0f%%), median %.0f views each, "
        "frames disagree by %.1f (0-255, points seen 3+ times), %.0f s"
        % (stats["frames"], stats["coloured"], n, 100 * stats["share"], stats["views_median"],
           stats["disagreement"] or 0, stats["seconds"]))
    return rgb, seen.astype(np.uint8), stats


def surface_shares(xyz, track_z_at, source):
    """Share coloured on floor, walls and ceiling (normals and height above
    the track), like the coverage check of 2026-09-25."""
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=20))
    nz = np.abs(np.asarray(pc.normals)[:, 2])
    above = xyz[:, 2] - track_z_at(xyz)
    floor = (nz > 0.9) & (above < 0.0)
    ceiling = (nz > 0.9) & (above > 1.0)
    walls = nz < 0.3
    return {k: round(float(source[m].mean()), 3) for k, m in
            (("floor", floor), ("walls", walls), ("ceiling", ceiling), ("all", np.ones(len(xyz), bool)))}


def final_colours(xyz, rgb, source):
    """Camera colour where there is one, height colours elsewhere."""
    out = L.height_colours(xyz[:, 2])
    if rgb is not None:
        out[source == 1] = rgb[source == 1]
    return out


def write_colour_ply(path, xyz, rgb, source):
    with open(path, "wb") as fh:
        fh.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
                  "property float x\nproperty float y\nproperty float z\n"
                  "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                  "property uchar colour_source\n"
                  "comment colour_source: 1 camera colour, 0 height colours (not seen by the camera)\n"
                  "end_header\n" % len(xyz)).encode("ascii"))
        rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                        ("r", "u1"), ("g", "u1"), ("b", "u1"), ("s", "u1")])
        rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        rec["s"] = source
        fh.write(rec.tobytes())


def colour_cloud_msg(xyz, rgb, source, frame, t):
    """PointCloud2 with x y z, rgb (packed as Foxglove and PCL expect) and
    colour_source."""
    import array
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = L.stamp_msg(t)
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
                  PointField(name="colour_source", offset=16, datatype=PointField.FLOAT32, count=1)]
    msg.is_bigendian = False
    msg.point_step = 20
    msg.row_step = 20 * len(xyz)
    msg.is_dense = True
    rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"), ("s", "<f4")])
    rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rec["rgb"] = (rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2]
    rec["s"] = source
    msg.data = array.array("B", rec.tobytes())
    return msg


def main():
    ap = argparse.ArgumentParser(description="colour an existing build's dense cloud from the colour frames")
    ap.add_argument("walk")
    ap.add_argument("--rate", type=float, default=5.0, help="colour frames per second used")
    ap.add_argument("--max-range", type=float, default=10.0, help="metres from the camera")
    args = ap.parse_args()
    walk = os.path.abspath(args.walk)
    name = os.path.basename(walk)
    build = os.path.join(walk, "map3d")
    xyz = np.load(os.path.join(build, "dense_map.npy"))
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    rgb, source, stats = colour_points(walk, xyz, d, args.rate, args.max_range)
    if rgb is None:
        sys.exit("  no colour: %s" % stats)
    out = final_colours(xyz, rgb, source)
    write_colour_ply(os.path.join(build, name + "_colour.ply"), xyz, out, source)
    np.save(os.path.join(build, "dense_colour.npy"), np.column_stack([out, source]).astype(np.uint8))
    print("  wrote %s" % os.path.join(build, name + "_colour.ply"))
    print(json.dumps(stats))


if __name__ == "__main__":
    main()
