#!/usr/bin/env python3
"""
Propose, save and show a walk's crop outline: the part of the map to keep.

    python3 crop_outline.py ~/go2-lidar-mapping/walks/odom-test-2          propose, save, draw
    python3 crop_outline.py <walk> --show                               draw the saved one

Writes <walk>/crop.json (polygons in the map frame, metres) and
<walk>/crop_outline.png (the outline over the map, for checking). Edit the
json by hand to move a corner; dense_map.py then keeps only the points inside
the polygons, instead of guessing "outdoors" (dense_map.outdoors_mask). The
map frame is the robot's pose at the start of the walk, which loop closure
keeps fixed, so an outline stays valid across rebuilds of the same walk.

WHY
---
dense_map.py's outdoors test (lines of sight that meet no wall) kept trees
within about 3 m of the building and removed real surfaces: the far end of
odom-test-2's long corridor (y 33 to 42, a roof over it all the way to 57 m)
and stretches of courtyard walls on both walks. What a building has and a
garden has not is a roof.

HOW THE PROPOSAL IS MADE
------------------------
From map3d/lidar_map.npy (the LiDAR keyframe map, not yet cleaned) and the
track, on a 0.25 m grid:
  1. roof: horizontal surfaces 1.6 to 5 m above the floor the robot walked
     on, flat within a cell (middle half of heights within 10 cm; 6 cm cut
     off the far end of a corridor whose ceiling was seen from 20-30 m).
     Tree canopies are horizontal-ish too, but not flat.
  2. walls: vertical, flat surfaces 0.3 to 1.6 m up (trees are neither);
  3. roof and walls, gaps up to 0.75 m closed (doors, lights, missing
     returns), with the ground the robot walked, connected to the track;
  4. holes filled: a courtyard enclosed by the building's walls is kept,
     since its floor and inner walls belong to the building (as the
     outdoors test kept it). Cut it by hand if unwanted;
  5. widened by 0.5 m for the outer walls and eaves, and traced as polygons.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

CELL = 0.25
FLOOR_BELOW_BODY = 0.38


def propose(xyz, track):
    import cv2
    import open3d as o3d
    from scipy import ndimage
    from scipy.spatial import cKDTree
    _, i = cKDTree(track[:, :2]).query(xyz[:, :2])
    h = xyz[:, 2] - (track[i, 2] - FLOOR_BELOW_BODY)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=20))
    nz = np.abs(np.asarray(pc.normals)[:, 2])
    up = nz > 0.9
    band = up & (h > 1.6) & (h < 5.0)
    # walls: vertical and flat (a tree is neither)
    pc.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(20))
    ev = np.linalg.eigvalsh(np.asarray(pc.covariances))
    planar = ev[:, 0] / np.maximum(ev[:, 2], 1e-12) < 0.05
    wall_pts = (nz < 0.3) & planar & (h > 0.3) & (h < 1.6)
    lo = np.minimum(xyz[:, :2].min(0), track[:, :2].min(0)) - 2.0
    ij = np.floor((xyz[:, :2] - lo) / CELL).astype(np.int64)
    shape = tuple(ij.max(0) + 3)
    # flat roof cells: enough points, narrow spread of heights
    key = ij[band, 0] * shape[1] + ij[band, 1]
    z = xyz[band, 2]
    o = np.argsort(key)
    key, z = key[o], z[o]
    s = np.flatnonzero(np.r_[True, np.diff(key) != 0])
    e = np.r_[s[1:], len(key)]
    roof = np.zeros(shape, bool)
    for a, b in zip(s, e):
        if b - a >= 4 and np.percentile(z[a:b], 75) - np.percentile(z[a:b], 25) < 0.10:
            roof[key[a] // shape[1], key[a] % shape[1]] = True
    walls = np.zeros(shape, bool)
    np.add.at(walls, (ij[wall_pts, 0], ij[wall_pts, 1]), True)
    counts = np.zeros(shape, np.int32)
    np.add.at(counts, (ij[wall_pts, 0], ij[wall_pts, 1]), 1)
    walls = counts >= 3
    solid = roof | walls
    solid = ndimage.binary_closing(solid, iterations=3) | solid
    # connected to the track
    tj = np.floor((track[:, :2] - lo) / CELL).astype(np.int64)
    walked = np.zeros(shape, bool)
    walked[tj[:, 0], tj[:, 1]] = True
    walked = ndimage.binary_dilation(walked, iterations=4)
    lab, n = ndimage.label(solid | walked)
    keep_labels = np.unique(lab[walked & (lab > 0)])
    area = np.isin(lab, keep_labels)
    area = ndimage.binary_fill_holes(area)
    area = ndimage.binary_dilation(area, iterations=2)
    # polygons, in metres
    cs, _ = cv2.findContours(area.astype(np.uint8).T.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polys = []
    for c in cs:
        if cv2.contourArea(c) * CELL * CELL < 4.0:
            continue
        c = cv2.approxPolyDP(c, 1.0, True)[:, 0, :].astype(float)    # within a cell (25 cm)
        # contour points are (column, row) of the transposed mask, which is (i, j)
        pts = (c + 0.5) * CELL + lo
        polys.append([[round(float(x), 3), round(float(y), 3)] for x, y in pts])
    return polys, dict(roof_cells=int(roof.sum()), wall_cells=int(walls.sum()),
                       kept_area_m2=round(float(area.sum()) * CELL * CELL, 1))


def inside(xy, polys):
    from matplotlib.path import Path
    keep = np.zeros(len(xy), bool)
    for p in polys:
        keep |= Path(np.array(p)).contains_points(xy)
    return keep


def load(walk):
    path = os.path.join(walk, "crop.json")
    if not os.path.isfile(path):
        return None
    with open(path) as fh:
        return json.load(fh)


def draw(walk, crop, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    build = os.path.join(walk, "map3d")
    m = np.load(os.path.join(build, "lidar_map.npy"))
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    keep = inside(m[:, :2], crop["polygons"])
    fig, ax = plt.subplots(figsize=(12, 14))
    ax.scatter(m[keep, 0], m[keep, 1], s=0.05, c="0.3")
    ax.scatter(m[~keep, 0], m[~keep, 1], s=0.05, c="tab:orange")
    for k, p in enumerate(crop["polygons"]):
        p = np.array(p + [p[0]])
        ax.plot(p[:, 0], p[:, 1], "b-", lw=1.5)
        for n, (x, y) in enumerate(p[:-1]):
            ax.annotate("%d.%d" % (k, n), (x, y), fontsize=6, color="b")
    ax.plot(d["x"], d["y"], "r-", lw=1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_title("%s crop outline (blue; corners numbered polygon.corner): kept grey %d, "
                 "cut orange %d" % (os.path.basename(walk), keep.sum(), (~keep).sum()))
    plt.tight_layout()
    plt.savefig(out, dpi=70)


def main():
    ap = argparse.ArgumentParser(description="propose and draw a walk's crop outline")
    ap.add_argument("walk")
    ap.add_argument("--show", action="store_true", help="draw the saved outline only")
    args = ap.parse_args()
    walk = os.path.abspath(args.walk)
    path = os.path.join(walk, "crop.json")
    if not args.show:
        if os.path.isfile(path):
            kept = path + time.strftime(".%Y-%m-%d_%H%M%S")
            os.rename(path, kept)
            print("  previous outline kept as %s" % kept)
        build = os.path.join(walk, "map3d")
        m = np.load(os.path.join(build, "lidar_map.npy")).astype(np.float64)
        d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
        track = np.stack([d["x"], d["y"], d["z"]], 1)
        polys, st = propose(m, track)
        crop = {"polygons": polys, "frame": "map (the walk's start pose)",
                "source": "proposed by crop_outline.py from the roof seen over the track; edit freely",
                "proposal": st, "date": time.strftime("%Y-%m-%d")}
        with open(path, "w") as fh:
            json.dump(crop, fh, indent=1)
        print("  %d polygon(s), %s m2 kept; wrote %s" % (len(polys), st["kept_area_m2"], path))
    crop = load(walk)
    if crop is None:
        sys.exit("no crop.json in %s" % walk)
    out = os.path.join(walk, "crop_outline.png")
    draw(walk, crop, out)
    print("  wrote %s" % out)


if __name__ == "__main__":
    main()
