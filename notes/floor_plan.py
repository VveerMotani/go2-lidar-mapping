#!/usr/bin/env python3
"""
2D floor plans, one per floor, from a walk's dense cloud.

    python3 floor_plan.py ~/go2-lidar-mapping/walks/odom-test-2

Runs after dense_map.py. Writes into map3d/, per level found:

    <walk>_plan_L<n>.png    the plan: walls and obstacles black, floor seen
                            grey, nothing seen white, the track where it was
                            on this level in red; 1 m grid, north (map +y) up.
    <walk>_plan_L<n>.pgm    the same as a ROS map_server occupancy map
    <walk>_plan_L<n>.yaml   (0 free, 100 occupied, -1 unknown; 5 cm)
    floor_plans.json        the levels: height, floor area, wall area

HOW
---
1. Levels: where the robot walked. A level is a height at which the robot's
   feet (the track, 0.38 m below the body) covered at least 10 m (--min-walk)
   within 15 cm of it. Measured 2026-09-26: floors 47-82 m, a stair landing
   4-8 m, each 30 cm of a staircase about 1.4 m. The level's height is then
   taken from the floor itself: the horizontal surfaces (normals within 18
   degrees of vertical) seen from above, in 5 cm bins, the biggest within
   30 cm of the robot's height; with less than 5 m2 of such floor, the robot's
   own height. Levels closer than 1.2 m are one.
   Before 2026-09-26 a level was any horizontal surface of 5 m2 or more that
   the robot had been above within 6 m. On a two-storey walk that took a
   2.1 m high surface over the ground-floor corridor (seen from below; the
   robot walked the floor above it later) as a third floor, and, being
   bigger, it hid the real landing next to it.
2. Per level, 5 cm pixels: occupied where there are points 0.2 to 1.8 m
   above that level (walls, doors, furniture: what a robot would bump into),
   at least 3 points in the pixel; free where there is floor within 10 cm of
   the level and nothing occupied; unknown otherwise. Occupied only within
   1 m of the level's own floor (the ground floor's ceiling, 0.5 m above
   odom-test-2's top landing, filled that level's plan with "walls"). Only the part of the
   cloud within 2.5 m above the level and above the level below is used, so
   the floor above does not appear as walls.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

RES = 0.05


def seen_from_above(xyz, track, reach=6.0, cell=1.0):
    """True where the robot was higher than the point somewhere within
    `reach` metres: a horizontal surface seen from above is a floor, one
    only ever seen from below is a ceiling. (The first version took every
    horizontal surface and found a "floor" at +2.17 m: the ceiling.)"""
    lo = xyz[:, :2].min(0) - reach
    shape = tuple(np.ceil((xyz[:, :2].max(0) + reach - lo) / cell).astype(int) + 1)
    top = np.full(shape, -np.inf)
    r = int(np.ceil(reach / cell))
    t = track[::10]
    ti = np.floor((t[:, :2] - lo) / cell).astype(int)
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > r * r:
                continue
            a = np.clip(ti[:, 0] + di, 0, shape[0] - 1)
            b = np.clip(ti[:, 1] + dj, 0, shape[1] - 1)
            np.maximum.at(top, (a, b), t[:, 2])
    pi = np.floor((xyz[:, :2] - lo) / cell).astype(int)
    return xyz[:, 2] < top[pi[:, 0], pi[:, 1]] - 0.2


def walked_heights(track, band=0.15, step=0.05):
    """Metres the robot walked at each floor height (feet 0.38 m below the
    body), within `band` of it: (heights, metres)."""
    feet = track[:, 2] - 0.38
    seg = np.r_[np.linalg.norm(np.diff(track[:, :2], axis=0), axis=1), 0.0]
    hs = np.arange(np.floor(feet.min() / step) * step, feet.max() + step, step)
    return hs, np.array([seg[np.abs(feet - h) < band].sum() for h in hs])


def levels(xyz, track, min_area=5.0, gap=1.2, min_walk=10.0):
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=20))
    up = (np.abs(np.asarray(pc.normals)[:, 2]) > 0.95) & seen_from_above(xyz, track)
    z = xyz[up, 2]
    # floor area per height bin: distinct 10 cm cells with up-facing points
    zb = np.floor(z / 0.05).astype(np.int64)
    cell = np.floor(xyz[up, :2] / 0.1).astype(np.int64)
    key = np.unique(np.stack([zb, cell[:, 0], cell[:, 1]], 1), axis=0)
    bins, area = np.unique(key[:, 0], return_counts=True)
    area = area * 0.01                                     # m2
    centres = (bins + 0.5) * 0.05
    # where the robot walked: a peak per stretch of heights above min_walk
    hs, walked = walked_heights(track)
    found = []
    for i in np.argsort(-walked):
        if walked[i] < min_walk:
            break
        h_robot = hs[i]
        if any(abs(h_robot - f["robot_height"]) < gap for f in found):
            continue
        near = np.abs(centres - h_robot) <= 0.3
        if near.any() and area[near].max() >= min_area:
            j = np.flatnonzero(near)[np.argmax(area[near])]
            h = centres[j]
            a = area[np.abs(bins - bins[j]) <= 2].sum()
        else:
            h, a = h_robot, float(area[near].sum()) if near.any() else 0.0
        found.append({"height": round(float(h), 3), "floor_area_m2": round(float(a), 1),
                      "robot_height": round(float(h_robot), 3), "walked_m": round(float(walked[i]), 1)})
    # what was left out, so a missing level can be understood from the log
    told = [f["robot_height"] for f in found]
    for i in np.argsort(-walked):
        if walked[i] < 3.0:
            break
        if walked[i] < min_walk and all(abs(hs[i] - h) >= 0.5 for h in told):
            told.append(hs[i])
            print("  not a level: %+.2f m, walked only %.1f m there (a landing or stairs; --min-walk %.0f)"
                  % (hs[i], walked[i], min_walk))
    return sorted(found, key=lambda f: f["height"]), up


def plan(xyz, up, level, below, extent):
    lo, hi = extent
    shape = tuple(np.ceil((hi - lo) / RES).astype(int) + 1)
    h = xyz[:, 2] - level
    ij = np.floor((xyz[:, :2] - lo) / RES).astype(np.int64)
    occ = np.zeros(shape, np.int32)
    s = (h > 0.2) & (h < 1.8)
    np.add.at(occ, (ij[s, 0], ij[s, 1]), 1)
    occupied = occ >= 3
    free = np.zeros(shape, bool)
    f = up & (np.abs(h) < 0.10)
    if below is not None:
        f &= xyz[:, 2] > below + 1.2
    free[ij[f, 0], ij[f, 1]] = True
    from scipy import ndimage
    free = ndimage.binary_closing(free, iterations=1)
    # Obstacles only near this level's own floor: the ground floor's ceiling
    # is 0.5 m above the top landing and would otherwise fill its plan.
    occupied &= ndimage.binary_dilation(free, iterations=int(round(1.0 / RES)))
    free &= ~occupied
    grid = np.full(shape, -1, np.int8)
    grid[free] = 0
    grid[occupied] = 100
    return grid


def write_ros(path_base, grid, lo):
    img = np.full(grid.shape, 205, np.uint8)
    img[grid == 0] = 254
    img[grid == 100] = 0
    img = img.T[::-1]                                      # rows: +y up
    with open(path_base + ".pgm", "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        fh.write(img.tobytes())
    with open(path_base + ".yaml", "w") as fh:
        fh.write("image: %s.pgm\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\nnegate: 0\n"
                 "occupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
                 % (os.path.basename(path_base), RES, lo[0], lo[1]))


def draw(path, grid, lo, track_xy, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rgb = np.full(grid.shape + (3,), 1.0)
    rgb[grid == 0] = 0.82
    rgb[grid == 100] = 0.0
    ext = [lo[0], lo[0] + grid.shape[0] * RES, lo[1], lo[1] + grid.shape[1] * RES]
    w = (ext[1] - ext[0]) / 4 + 2
    hgt = (ext[3] - ext[2]) / 4 + 1
    fig, ax = plt.subplots(figsize=(min(max(w, 6), 20), min(max(hgt, 6), 24)))
    ax.imshow(np.transpose(rgb, (1, 0, 2)), origin="lower", extent=ext, interpolation="nearest")
    if len(track_xy):
        ax.plot(track_xy[:, 0], track_xy[:, 1], "r.", ms=1)
    ax.set_xticks(np.arange(np.floor(ext[0]), ext[1], 1), minor=True)
    ax.set_yticks(np.arange(np.floor(ext[2]), ext[3], 1), minor=True)
    ax.grid(which="minor", alpha=0.15)
    ax.grid(which="major", alpha=0.35)
    ax.set_xlabel("m")
    ax.set_ylabel("m")
    ax.set_title(title)
    plt.tight_layout()
    plt.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="2D floor plans per level from the dense cloud")
    ap.add_argument("walk")
    ap.add_argument("--min-walk", type=float, default=10.0,
                    help="metres the robot must walk at a height for it to be a level")
    args = ap.parse_args()
    walk = os.path.abspath(args.walk)
    name = os.path.basename(walk)
    build = os.path.join(walk, "map3d")
    xyz = np.load(os.path.join(build, "dense_map.npy"))
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    track = np.stack([d["x"], d["y"], d["z"]], 1)
    from progress import Progress, phase
    phase("finding the floors")
    lv, up = levels(xyz, track, min_walk=args.min_walk)
    # an earlier build's plans may have had more levels
    import glob
    for old in glob.glob(os.path.join(build, glob.escape(name) + "_plan_L*.*")):
        os.remove(old)
    prog = Progress("drawing floor plans", len(lv), "levels")
    lo = xyz[:, :2].min(0) - 0.5
    hi = xyz[:, :2].max(0) + 0.5
    out = []
    for n, L in enumerate(lv):
        below = lv[n - 1]["height"] if n else None
        above = lv[n + 1]["height"] if n + 1 < len(lv) else L["height"] + 2.5
        sel = (xyz[:, 2] > L["height"] - 0.3) & (xyz[:, 2] < min(above - 0.1, L["height"] + 2.5))
        grid = plan(xyz[sel], up[sel], L["height"], below, (lo, hi))
        base = os.path.join(build, "%s_plan_L%d" % (name, n))
        write_ros(base, grid, lo)
        # track on this level: body 0.38 m above the floor, within 0.3 m
        on = np.abs(track[:, 2] - 0.38 - L["height"]) < 0.3
        L.update(level=n, free_m2=round(float((grid == 0).sum()) * RES * RES, 1),
                 occupied_m2=round(float((grid == 100).sum()) * RES * RES, 1),
                 track_share=round(float(on.mean()), 3), files=base + ".{png,pgm,yaml}")
        draw(base + ".png", grid, lo, track[on, :2],
             "%s, level %d: floor at %.2f m (%.0f m2 of floor seen); black: 0.2-1.8 m above it"
             % (name, n, L["height"], L["floor_area_m2"]))
        out.append(L)
        prog.step()
        print("  level %d: floor at %+.2f m, %.0f m2 floor, plan free %.0f m2, occupied %.0f m2, "
              "track %.0f%% of the walk; wrote %s.png/.pgm/.yaml"
              % (n, L["height"], L["floor_area_m2"], L["free_m2"], L["occupied_m2"],
                 100 * L["track_share"], base))
    prog.end()
    with open(os.path.join(build, "floor_plans.json"), "w") as fh:
        json.dump(out, fh, indent=1)


if __name__ == "__main__":
    main()
