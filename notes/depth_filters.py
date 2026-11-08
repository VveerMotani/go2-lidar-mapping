#!/usr/bin/env python3
"""
Intel's spatial filter for D400 depth frames, reimplemented from Intel's own
description, for recorded frames (no camera, no pyrealsense2).

    from depth_filters import spatial_filter
    depth_mm = spatial_filter(frame.image, fx)

Source: Intel, "Depth Post-Processing for Intel RealSense D400 Depth
Cameras" (dev.realsenseai.com/docs/depth-post-processing). Their pipeline
is decimation -> depth to disparity -> spatial -> temporal -> disparity to
depth -> hole filling. What is used here, and why:

  spatial filter  yes. An edge-preserving smoothing: raster-scan the image
                  along x and y, forwards and back, twice, keeping an
                  exponential moving average S = a*Y + (1-a)*S_prev, which
                  restarts (a = 1) wherever neighbouring pixels differ by
                  more than delta. Done on disparity (1/distance), since
                  stereo noise grows with distance squared. Intel's defaults:
                  alpha 0.6, delta 8 (in 1/32 of a pixel of disparity), 2
                  iterations. No hole filling (Intel's default).
  temporal filter no. It averages each pixel over frames, and Intel say it
                  "should be used sparingly on scenes with high-speed
                  motion": the camera is on a walking robot.
  decimation      no: for speed only; frames are already thinned when used.
  hole filling    no: it invents depth where the camera measured none.

The D435i's stereo baseline is 50 mm (Intel's datasheet); disparity in
pixels is fx * baseline / distance.

RESULT (odom-test-2, 2026-09-25): NOT USED. Nothing calls this.
------
With Intel's defaults it moved pixels by a median 1.1 mm and left the
floor's scatter around its own plane where it was (1-1.5 m: 0.85 -> 0.82 cm;
2-2.5 m: 3.87 -> 3.89 cm). Stronger settings made it worse (alpha 0.25,
delta 20: 3.8 -> 4.2 cm at 2-2.5 m; alpha 0.1: 5.3 cm). The reason: the
camera's error is not pixel-to-pixel speckle but broad waves. The floor's
residual correlates 1.00 between neighbouring pixels and still 0.95 between
pixels 8 apart, and a smoother a few pixels wide cannot remove what every
pixel around shares. It also costs 70 ms a frame here. Kept so the finding
is not repeated.
"""
import numpy as np

BASELINE_M = 0.050


def _pass(d, alpha, delta, axis, reverse):
    """One raster scan of the recursive filter, in place, along `axis`."""
    n = d.shape[axis]
    idx = range(n - 2, -1, -1) if reverse else range(1, n)
    step = 1 if reverse else -1
    for i in idx:
        prev = d.take(i + step, axis=axis)
        cur = d.take(i, axis=axis)
        ok = (cur > 0) & (prev > 0) & (np.abs(cur - prev) < delta)
        new = np.where(ok, alpha * cur + (1 - alpha) * prev, cur)
        if axis == 1:
            d[:, i] = new
        else:
            d[i, :] = new


def spatial_filter(depth_mm, fx, alpha=0.6, delta=8.0, iterations=2, baseline=BASELINE_M):
    """Intel's edge-preserving spatial filter. depth_mm: uint16 millimetres
    (0 = no reading). Returns filtered depth in millimetres, float32, holes
    still 0."""
    z = depth_mm.astype(np.float32) / 1000.0
    valid = z > 0
    k = 32.0 * fx * baseline                      # disparity in 1/32 pixel = k / z
    d = np.zeros_like(z)
    d[valid] = k / z[valid]
    for _ in range(iterations):
        for axis in (1, 0):                       # along x, then along y
            _pass(d, alpha, delta, axis, reverse=False)
            _pass(d, alpha, delta, axis, reverse=True)
    out = np.zeros_like(z)
    good = d > 0
    out[good] = 1000.0 * k / d[good]
    return out
