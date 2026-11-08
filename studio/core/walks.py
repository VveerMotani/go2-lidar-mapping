"""The walks on this laptop, and how far each has got.

A walk is a folder in the walks folder holding lidar.pcap. Its five stages
are read off the files, never stored anywhere else, so what Studio shows is
always what is actually on disk:

    on the robot   in the robot's /api/recordings list
    downloaded     <walk>/lidar.pcap
    bag            <walk>/bag/bag_0.mcap, closed properly
    map            <walk>/map3d/<walk>_map.ply
    clean cloud    <walk>/map3d/<walk>_dense.ply

and after the clean cloud, made by the same run but extras (a walk without
them is still finished): the camera-coloured cloud (_colour.ply, only for a
walk recorded with colour), a floor plan per level (_plan_L<n>.png) and the
mesh (_mesh.ply).

The recording checks come from notes/walk_library.py, the same code the robot
uses for its walk report, so the laptop and the robot never disagree.
"""
import csv
import glob
import json
import math
import os
import re
import threading

from . import settings
from .applog import log

try:
    import walk_library                      # notes/walk_library.py
except ImportError:                          # pragma: no cover
    walk_library = None

SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_cache = {}
_cache_lock = threading.Lock()

# What each file is, for the details panel and Save a copy.
FILES = [
    ("map", "map3d/{n}_map/{n}_map_0.mcap", "Map growing along the walk, for Foxglove"),
    ("dense", "map3d/{n}_dense.ply", "Whole walk as one clean cloud, for CloudCompare"),
    ("colour", "map3d/{n}_colour.ply", "Clean cloud in the camera's own colours, for CloudCompare"),
    ("mesh", "map3d/{n}_mesh.ply", "Surfaces as a coloured triangle mesh, for CloudCompare or Blender"),
    ("densemcap", "map3d/{n}_dense/{n}_dense_0.mcap", "Clean cloud (and colour) for Foxglove"),
    ("mapply", "map3d/{n}_map.ply", "Keyframe map with the robot's track"),
    ("bag", "bag/bag_0.mcap", "Every message: LiDAR, IMU, odometry, depth"),
]
RAW = ["lidar.pcap", "imu.csv", "odom.csv", "session.json", "depth.stream",
       "status.json", "depth_summary.json", "depth_status.json"]


def root():
    return settings.get("walks_dir")


def folder(name):
    """The walk's folder, or None if the name is not a plain walk name."""
    if not name or not SAFE_NAME.match(name):
        return None
    return os.path.join(root(), name)


def _size(path):
    try:
        if os.path.isdir(path):
            return sum(os.path.getsize(os.path.join(r, f))
                       for r, _, fs in os.walk(path) for f in fs)
        return os.path.getsize(path)
    except OSError:
        return 0


def _stamp(d):
    """Changes whenever anything that matters in the walk changes."""
    parts = []
    for rel in ("session.json", "odom.csv", "bag/metadata.yaml", "map3d",
                "map3d/dense_report.json"):
        try:
            parts.append(os.path.getmtime(os.path.join(d, rel)))
        except OSError:
            parts.append(0)
    return tuple(parts)


def _track(d):
    """Length and end-to-start distance of the scan-matched track."""
    p = os.path.join(d, "map3d", "trajectory.csv")
    try:
        with open(p, newline="") as fh:
            rows = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(fh)]
    except (OSError, ValueError, KeyError):
        return None
    if len(rows) < 2:
        return None
    length = sum(math.dist(a, b) for a, b in zip(rows, rows[1:]))
    return {"length_m": round(length, 1), "closure_m": round(math.dist(rows[0], rows[-1]), 2)}


def _dense_points(d):
    try:
        with open(os.path.join(d, "map3d", "dense_report.json")) as fh:
            return json.load(fh).get("points_kept")
    except (OSError, ValueError):
        return None


def _plans(d, n):
    """The floor plans, one per level, lowest first, with floor_plan.py's numbers."""
    meta = {}
    try:
        with open(os.path.join(d, "map3d", "floor_plans.json")) as fh:
            meta = {int(p["level"]): p for p in json.load(fh)}
    except (OSError, ValueError, KeyError, TypeError):
        pass
    out = []
    for png in glob.glob(os.path.join(d, "map3d", glob.escape(n) + "_plan_L*.png")):
        m = re.search(r"_plan_L(\d+)\.png$", png)
        if not m:
            continue
        lv = int(m.group(1))
        p = meta.get(lv, {})
        out.append({"level": lv, "rel": "map3d/" + os.path.basename(png), "bytes": _size(png),
                    "height": p.get("height"), "floor_m2": p.get("floor_area_m2")})
    return sorted(out, key=lambda p: p["level"])


def describe(name):
    """Everything the Walks page shows for one local walk."""
    d = folder(name)
    if d is None or not os.path.isfile(os.path.join(d, "lidar.pcap")):
        return None
    key = _stamp(d)
    with _cache_lock:
        hit = _cache.get(d)
        if hit and hit[0] == key:
            return dict(hit[1])

    info = walk_library.describe(d) if walk_library else {"verdict": {}}
    n = name
    has = lambda rel: os.path.isfile(os.path.join(d, rel.format(n=n)))  # noqa: E731
    odom_csv = os.path.join(d, "odom.csv")
    can_map = os.path.isfile(odom_csv) and os.path.getsize(odom_csv) > 200
    bag = info.get("bag") or {}
    meta = {}
    try:
        with open(os.path.join(d, "session.json")) as fh:
            meta = json.load(fh)
    except (OSError, ValueError):
        pass
    notes = {}
    try:
        with open(os.path.join(d, "notes.json")) as fh:
            notes = json.load(fh)
    except (OSError, ValueError):
        pass

    out = {
        "name": n,
        "path": d,
        "started": info.get("started_unix"),
        "seconds": info.get("seconds") or 0,
        "bytes": _size(d),
        "verdict": info.get("verdict"),
        "can_map": can_map,
        "depth": os.path.isfile(os.path.join(d, "depth.stream")),
        "clock_ok": bool(meta.get("clock_was_synced")),
        "notes": notes,
        "stages": {
            "downloaded": True,
            "bag": bool(bag.get("closed") and bag.get("has_metadata")),
            "bag_broken": bool(bag.get("mcap")) and not (bag.get("closed") and bag.get("has_metadata")),
            "map": has("map3d/{n}_map.ply"),
            "dense": has("map3d/{n}_dense.ply"),
            "colour": has("map3d/{n}_colour.ply"),
            "mesh": has("map3d/{n}_mesh.ply"),
        },
        "plans": _plans(d, n),
        "track": _track(d),
        "points": _dense_points(d),
        "files": [
            {"key": k, "rel": rel.format(n=n), "what": what,
             "bytes": _size(os.path.join(d, rel.format(n=n)))}
            for k, rel, what in FILES if has(rel)
        ],
        "raw_files": [f for f in RAW if os.path.isfile(os.path.join(d, f))],
        "raw_bytes": sum(_size(os.path.join(d, f)) for f in RAW),
    }
    with _cache_lock:
        _cache[d] = (key, out)
    return dict(out)


def local():
    r = root()
    try:
        names = sorted(os.listdir(r))
    except OSError:
        return []
    out = []
    for n in names:
        if SAFE_NAME.match(n) and os.path.isfile(os.path.join(r, n, "lidar.pcap")):
            try:
                w = describe(n)
            except Exception as e:                       # noqa: BLE001
                log("err", "disk", "could not read walk %s: %s" % (n, e))
                continue
            if w:
                out.append(w)
    return out


def merged(robot_walks, robot_reachable):
    """Local walks plus the robot's, one entry per name, newest first."""
    loc = {w["name"]: w for w in local()}
    out = []
    for rw in robot_walks or []:
        n = rw.get("name")
        if not n:
            continue
        w = loc.pop(n, None)
        if w is None:
            w = {"name": n, "path": None, "started": rw.get("modified"),
                 "seconds": rw.get("seconds") or 0, "bytes": rw.get("bytes_all") or rw.get("bytes") or 0,
                 "clock_ok": rw.get("clock_ok"), "can_map": None, "depth": None,
                 "stages": {"downloaded": False}, "files": [], "verdict": None}
        w["on_robot"] = True
        w["robot_complete"] = rw.get("complete", False)
        out.append(w)
    for w in loc.values():
        w["on_robot"] = False if robot_reachable else None
        out.append(w)
    out.sort(key=lambda w: w.get("started") or 0, reverse=True)
    return out


def save_notes(name, notes):
    d = folder(name)
    if d is None or not os.path.isdir(d):
        return False
    keep = {k: str(v)[:500] for k, v in (notes or {}).items()
            if k in ("building", "floor", "notes", "operator")}
    # Markers dropped during the walk: [{"t": seconds into the walk, "label": ...}]
    marks = (notes or {}).get("markers")
    if isinstance(marks, list):
        keep["markers"] = [{"t": float(m.get("t", 0)), "label": str(m.get("label", ""))[:60]}
                           for m in marks[:200] if isinstance(m, dict)]
    with open(os.path.join(d, "notes.json"), "w", encoding="utf-8") as fh:
        json.dump(keep, fh, indent=1)
    return True
