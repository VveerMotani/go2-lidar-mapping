#!/usr/bin/env python3
"""
A triangle mesh of a walk, from its dense cloud.

    python3 mesh_map.py ~/go2-lidar-mapping/walks/odom-test-2
    python3 mesh_map.py <walk> --depth 11 --smooth 10 --out /tmp/x.ply

Runs after dense_map.py. Writes map3d/<walk>_mesh.ply: triangles with vertex
colours (the camera's where colour_map.py found it, height colours
elsewhere) and a per-vertex colour_source (1 camera, 0 height colours), and
map3d/mesh_report.json. MeshLab, CloudCompare and Blender open it.

HOW
---
1. The dense cloud, thinned to 3 cm (a 2 cm cloud gives the mesher no more
   than its octree resolves), normals from 30 neighbours, turned to face
   the track: every surface was seen from where the robot was.
2. Poisson surface reconstruction (Open3D), octree depth --depth.
3. Trimmed: Poisson closes every hole with invented surface (over doorways,
   the unseen ceiling, the courtyard sky). Every vertex more than --trim
   (5 cm) from a real point is removed, with its triangles.
4. Smoothed: --smooth iterations of Taubin smoothing, which evens out the
   sensor's noise without shrinking the surfaces as plain averaging does.
5. Coloured: camera colour from the nearest camera-coloured point within
   5 cm, height colours elsewhere.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_map as L  # noqa: E402


def build_mesh(xyz, track, depth=10, trim=0.05, smooth=5, log=print):
    import open3d as o3d
    from scipy.spatial import cKDTree
    started = time.time()
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
    pc = pc.voxel_down_sample(0.03)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    p = np.asarray(pc.points)
    n = np.asarray(pc.normals)
    # face the robot: the nearest track pose, at the LiDAR's height
    eyes = track[::5] + [0, 0, 0.1]
    _, i = cKDTree(eyes).query(p)
    flip = np.einsum("ij,ij->i", n, eyes[i] - p) < 0
    n[flip] *= -1
    pc.normals = o3d.utility.Vector3dVector(n)
    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pc, depth=depth, n_threads=-1)
    t_poisson = time.time() - started
    v = np.asarray(mesh.vertices)
    dist, _ = cKDTree(p).query(v)
    far = dist > trim
    mesh.remove_vertices_by_mask(far)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    if smooth:
        mesh = mesh.filter_smooth_taubin(number_of_iterations=smooth)
    mesh.compute_vertex_normals()
    log("  mesh: Poisson depth %d, %d points in, %.0f s; trimmed %.0f%% of vertices (more than %.0f cm "
        "from a real point); Taubin x%d; %d vertices, %d triangles, %.0f s"
        % (depth, len(p), t_poisson, 100 * far.mean(), 100 * trim, smooth,
           len(mesh.vertices), len(mesh.triangles), time.time() - started))
    return mesh


def mesh_quality(mesh, xyz, samples=2000000):
    """accuracy: distance from the mesh's surface to the nearest real point;
    coverage: share of real points within 3 cm of the mesh."""
    from scipy.spatial import cKDTree
    s = np.asarray(mesh.sample_points_uniformly(samples).points)
    d_acc, _ = cKDTree(xyz).query(s)
    sub = xyz[np.random.default_rng(0).choice(len(xyz), min(len(xyz), 500000), replace=False)]
    d_cov, _ = cKDTree(s).query(sub)
    return dict(accuracy_median_cm=round(100 * float(np.median(d_acc)), 2),
                accuracy_90_cm=round(100 * float(np.percentile(d_acc, 90)), 2),
                coverage_3cm=round(float((d_cov < 0.03).mean()), 3)), s


def write_mesh_ply(path, mesh, rgb, source):
    v = np.asarray(mesh.vertices).astype("<f4")
    f = np.asarray(mesh.triangles).astype("<i4")
    with open(path, "wb") as fh:
        fh.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
                  "property float x\nproperty float y\nproperty float z\n"
                  "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                  "property uchar colour_source\n"
                  "comment colour_source: 1 camera colour, 0 height colours (not seen by the camera)\n"
                  "element face %d\nproperty list uchar int vertex_indices\nend_header\n"
                  % (len(v), len(f))).encode("ascii"))
        rec = np.empty(len(v), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                      ("r", "u1"), ("g", "u1"), ("b", "u1"), ("s", "u1")])
        rec["x"], rec["y"], rec["z"] = v[:, 0], v[:, 1], v[:, 2]
        rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        rec["s"] = source
        fh.write(rec.tobytes())
        faces = np.empty(len(f), dtype=[("n", "u1"), ("a", "<i4"), ("b", "<i4"), ("c", "<i4")])
        faces["n"] = 3
        faces["a"], faces["b"], faces["c"] = f[:, 0], f[:, 1], f[:, 2]
        fh.write(faces.tobytes())


def main():
    from scipy.spatial import cKDTree
    ap = argparse.ArgumentParser(description="triangle mesh of a walk from its dense cloud")
    ap.add_argument("walk")
    ap.add_argument("--depth", type=int, default=11, help="Poisson octree depth")
    ap.add_argument("--trim", type=float, default=0.05, help="metres from a real point")
    ap.add_argument("--smooth", type=int, default=5, help="Taubin smoothing iterations (0: none)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    walk = os.path.abspath(args.walk)
    name = os.path.basename(walk)
    build = os.path.join(walk, "map3d")
    xyz = np.load(os.path.join(build, "dense_map.npy"))
    d = np.genfromtxt(os.path.join(build, "trajectory.csv"), delimiter=",", names=True)
    track = np.stack([d["x"], d["y"], d["z"]], 1)
    from progress import phase
    started = time.time()
    phase("building the mesh surface")
    mesh = build_mesh(xyz, track, args.depth, args.trim, args.smooth)
    phase("colouring the mesh")
    colour_path = os.path.join(build, "dense_colour.npy")
    v = np.asarray(mesh.vertices)
    rgb, source = L.height_colours(v[:, 2]), np.zeros(len(v), np.uint8)
    if os.path.isfile(colour_path):
        # Camera colour from the nearest point the camera saw, if within
        # 5 cm: on walls, points the camera saw lie among LiDAR points it did
        # not, and the single nearest point gave 1 vertex in 2 height colours.
        c = np.load(colour_path)
        cam = c[:, 3] == 1
        if cam.any():
            dist, near = cKDTree(xyz[cam]).query(v)
            ok = dist < 0.05
            rgb[ok] = c[cam][near[ok], :3]
            source[ok] = 1
    print("  mesh vertices in camera colour: %.0f%%" % (100 * source.mean()))
    out = args.out or os.path.join(build, name + "_mesh.ply")
    phase("writing the mesh")
    write_mesh_ply(out, mesh, rgb, source)
    phase("checking the mesh")
    q, samples = mesh_quality(mesh, xyz)
    import map_quality
    fthick, fcells = map_quality.floor_only(samples.astype(np.float32), track)
    q.update(floor_only_cm=round(100 * fthick, 2), depth=args.depth, trim_m=args.trim, smooth=args.smooth,
             vertices=len(mesh.vertices), triangles=len(mesh.triangles),
             megabytes=round(os.path.getsize(out) / 1e6, 1), seconds=round(time.time() - started, 1))
    print("  surface within %.1f cm of a real point (median; 90%%: %.1f cm); %.0f%% of the cloud within "
          "3 cm of it; floor only %.1f cm" % (q["accuracy_median_cm"], q["accuracy_90_cm"],
                                               100 * q["coverage_3cm"], q["floor_only_cm"]))
    print("  wrote %s (%.0f MB)" % (out, q["megabytes"]))
    if not args.out:
        with open(os.path.join(build, "mesh_report.json"), "w") as fh:
            json.dump(q, fh, indent=1)


if __name__ == "__main__":
    main()
