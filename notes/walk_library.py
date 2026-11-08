"""
What is in a folder of recorded walks, and whether each one is any good.

    import walk_library
    walks = walk_library.list_walks("~/go2-lidar-mapping/walks")
    for w in walks:
        print(w["name"], w["verdict"]["passed"], "of", w["verdict"]["total"])

WHY THIS IS A SEPARATE FILE
---------------------------
The same questions get asked in three places and must not be answered three
different ways:

  * the dashboard, listing recordings so you can see at a glance which are
    sound -- running on the Jetson, Python 3.8, standard library only;
  * inspect_bag.py, after a conversion, alongside its checks of the bag itself;
  * anything later that wants to know whether a walk is worth processing.

inspect_bag.py cannot be imported by the dashboard: it needs numpy, rosbag2_py
and rclpy, none of which exist on the Jetson and none of which are needed to
read a csv. So the session-level checks live here, in the standard library, and
inspect_bag.py calls into them for its own report.

WHAT A CHECK IS
---------------
A dict with `ok`, a short `headline`, and a `detail` that says what to do about
it in plain words. Nothing here prints; the caller decides how to show it.

The checks exist because every fault this project has hit looked completely
plausible on screen. A flat point cloud is still a point cloud. A walk with no
odometry still draws. So the rule is: measure the file, do not look at it.
"""
import csv
import io
import json
import os
import time

# The XT16 sends 2,535 packets a second. Materially fewer means dropped
# packets; materially more means the recording contains each packet twice,
# which is what happened before the recorder filtered by destination address.
LIDAR_PPS_NOMINAL = 2535.0

# A walk shorter than this is a test, not a walk. Not a failure -- just worth
# saying, because a 3-second recording that "looks fine" usually means the stop
# button was pressed by accident.
SHORT_WALK_SECONDS = 20.0

MCAP_MAGIC = b"\x89MCAP0\r\n"


def _check(ok, headline, detail=""):
    return {"ok": bool(ok), "headline": headline, "detail": detail}


# ---------------------------------------------------------------------------
# The finished bag
# ---------------------------------------------------------------------------
def bag_state(bag_dir):
    """Is there an mcap, and did it finish being written?

    A conversion that gets killed leaves a large, entirely plausible-looking
    file that plays back as nothing: rosbag2 writes the closing magic and
    metadata.yaml only during shutdown. Both are checked because they fail
    separately -- observed on First-Walk-CB-1s-Floor, 625 MB with neither.
    """
    out = {"dir": bag_dir, "mcap": None, "bytes": 0,
           "has_metadata": False, "closed": False}
    if not os.path.isdir(bag_dir):
        return out

    for entry in sorted(os.listdir(bag_dir)):
        if entry.endswith(".mcap"):
            out["mcap"] = os.path.join(bag_dir, entry)
            break
    if out["mcap"] is None:
        return out

    out["bytes"] = os.path.getsize(out["mcap"])
    out["has_metadata"] = os.path.isfile(os.path.join(bag_dir, "metadata.yaml"))

    # An mcap opens and closes with the same 8-byte magic. If the trailing copy
    # is absent the writer never shut down cleanly.
    try:
        with open(out["mcap"], "rb") as fh:
            head = fh.read(8)
            if out["bytes"] > 16 and head == MCAP_MAGIC:
                fh.seek(-8, os.SEEK_END)
                out["closed"] = fh.read(8) == MCAP_MAGIC
    except OSError:
        pass
    return out


# ---------------------------------------------------------------------------
# The recording itself
# ---------------------------------------------------------------------------
def imu_continuity(path, limit=None):
    """Repeated and out-of-order ticks in imu.csv.

    The IMU's `tick` is the robot's own counter, so it is the only field that
    can prove two samples are the same sample. Repeats mean the recorder was
    subscribed to both rt/lowstate and rt/lf/lowstate -- "lf" is the same data
    sent more slowly, not a second sensor -- and interleaved them into one file.

    `limit` caps how many rows are read, because the dashboard lists many walks
    at once and a two-minute walk is 70,000 rows. The counts are then a sample
    rather than a total, which is enough to answer yes or no; inspect_bag.py
    passes no limit and gets the real numbers.
    """
    out = {"rows": 0, "repeats": 0, "backwards": 0, "sampled": False}
    if not os.path.isfile(path):
        return out
    prev = None
    try:
        with io.open(path, "r", newline="") as fh:
            for row in csv.DictReader(fh):
                try:
                    tick = int(float(row["tick"]))
                except (KeyError, TypeError, ValueError):
                    continue
                out["rows"] += 1
                if prev is not None:
                    d = tick - prev
                    if d == 0:
                        out["repeats"] += 1
                    # A large negative jump is the 32-bit counter wrapping,
                    # which is legitimate. A small one is a stale sample.
                    elif -1000 < d < 0:
                        out["backwards"] += 1
                prev = tick
                if limit and out["rows"] >= limit:
                    out["sampled"] = True
                    break
    except OSError:
        pass
    return out


def bag_frames(bag_dir, topic="/lidar_points"):
    """How many messages the bag holds on a topic, from its metadata.yaml.

    Scanned line by line rather than parsed as YAML: there is no yaml module on
    the Jetson, the file is machine-written so its shape is stable, and the only
    thing wanted is one integer. Returns 0 when it cannot be found, which the
    caller treats as "no progress bar" rather than as an error.
    """
    meta = os.path.join(bag_dir, "metadata.yaml")
    try:
        with io.open(meta, encoding="utf-8") as fh:
            lines = fh.read().splitlines()
    except (OSError, UnicodeDecodeError):
        return 0
    for i, line in enumerate(lines):
        if line.strip().endswith(topic) and "name:" in line:
            for follow in lines[i:i + 8]:
                if "message_count:" in follow:
                    try:
                        return int(follow.split(":", 1)[1].strip())
                    except ValueError:
                        return 0
    return 0


def csv_has_rows(path):
    """Does this csv hold at least one row of data, not just its header?

    Size alone is not enough. record_session.py writes the header the moment it
    opens the file, so an odometry recording that captured nothing at all is a
    109-byte file that any "is it bigger than nothing" test happily passes --
    which is exactly the fault the odometry check exists to catch, so it must
    not be the fault the check has itself.
    """
    try:
        with io.open(path, "r", newline="") as fh:
            fh.readline()                     # header
            return bool(fh.readline().strip())
    except (OSError, UnicodeDecodeError):
        return False


def _dir_bytes(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def is_walk_dir(path):
    """A folder is a walk if the recorder's own outputs are in it."""
    if not os.path.isdir(path):
        return False
    for marker in ("session.json", "lidar.pcap", "imu.csv"):
        if os.path.isfile(os.path.join(path, marker)):
            return True
    return False


def describe(path, imu_limit=20000):
    """Everything the dashboard needs about one walk, plus its verdict."""
    path = os.path.abspath(path)
    meta = {}
    meta_path = os.path.join(path, "session.json")
    if os.path.isfile(meta_path):
        try:
            with io.open(meta_path, encoding="utf-8") as fh:
                meta = json.load(fh)
        except (OSError, ValueError):
            meta = {}

    lidar = meta.get("lidar") or {}
    imu = meta.get("imu") or {}
    odom = meta.get("odom") or {}

    odom_csv = os.path.join(path, "odom.csv")
    has_odom = csv_has_rows(odom_csv)

    bag = bag_state(os.path.join(path, "bag"))
    cont = imu_continuity(os.path.join(path, "imu.csv"), limit=imu_limit)

    seconds = float(meta.get("seconds") or 0.0)
    started = meta.get("started_unix")
    if not started:
        try:
            started = os.path.getmtime(
                meta_path if os.path.isfile(meta_path) else path)
        except OSError:
            started = None

    checks = []

    # 1. Without a synced clock the LiDAR and IMU cannot be lined up at all,
    #    so nothing downstream can be trusted. This one is fatal.
    synced = meta.get("clock_was_synced")
    checks.append(_check(
        synced, "clock was synced",
        "yes" if synced else
        "NO. The robot booted at 1970 and nothing set its clock, so the LiDAR "
        "and IMU cannot be lined up. This recording cannot be used for "
        "mapping."))

    # 2. Odometry is what makes the scans accumulate into a room rather than
    #    pile up in one place. Its absence is invisible in the finished file.
    checks.append(_check(
        has_odom, "odometry recorded",
        "%d rows" % odom.get("rows", 0) if has_odom else
        "odom.csv is missing or empty. Either no motion data reached the "
        "recorder, or this walk predates the odometry recorder. Every scan "
        "will be drawn at the same spot, so the walk comes out as a smear. It "
        "cannot be added later."))

    # 3. LiDAR arrived, and arrived once.
    pps = float(lidar.get("packets_per_second") or 0.0)
    packets = int(lidar.get("packets") or 0)
    if packets == 0:
        checks.append(_check(False, "lidar packets captured",
                             "none. The sensor was not sending, or the relay "
                             "had the port."))
    elif pps > LIDAR_PPS_NOMINAL * 1.5:
        checks.append(_check(
            False, "lidar packets captured",
            "%d packets at %.0f/s, about twice the XT16's %.0f/s. Every packet "
            "was recorded twice: the recorder caught both the sensor's packets "
            "and the relay's forwarded copies. walk_to_bag.sh removes the "
            "copies during conversion." % (packets, pps, LIDAR_PPS_NOMINAL)))
    elif pps < LIDAR_PPS_NOMINAL * 0.8:
        checks.append(_check(
            False, "lidar packets captured",
            "%d packets at %.0f/s, below the XT16's %.0f/s. Packets were "
            "dropped." % (packets, pps, LIDAR_PPS_NOMINAL)))
    else:
        checks.append(_check(True, "lidar packets captured",
                             "%d packets at %.0f/s" % (packets, pps)))

    # 4. One IMU stream, not two interleaved.
    if cont["rows"] == 0:
        checks.append(_check(False, "imu is a single stream",
                             "imu.csv has no readable rows"))
    else:
        clean = cont["repeats"] == 0 and cont["backwards"] == 0
        where = (" in the first %d rows" % cont["rows"]) if cont["sampled"] else ""
        checks.append(_check(
            clean, "imu is a single stream",
            ("no repeated or out-of-order ticks" + where) if clean else
            "%d repeated and %d out-of-order ticks%s. The recorder was "
            "subscribed to both rt/lowstate and rt/lf/lowstate. Fixed for "
            "recordings made after 2026-09-05; older ones keep it."
            % (cont["repeats"], cont["backwards"], where)))

    # 5. Long enough to be a walk.
    checks.append(_check(
        seconds >= SHORT_WALK_SECONDS, "long enough to be useful",
        ("%.0f s" % seconds) if seconds >= SHORT_WALK_SECONDS else
        "%.0f s. Fine as a test; too short to map anything." % seconds))

    # 6. The depth camera, only if it was on for this walk. A walk without
    #    depth is normal and gets no check at all.
    depth = meta.get("depth") or {}
    if "frames" in depth:
        n = int((depth.get("frames") or {}).get("depth", 0))
        dropped = int((depth.get("dropped") or {}).get("depth", 0))
        fps = float(depth.get("depth_fps") or 0.0)
        good = n > 0 and dropped <= max(5, 0.02 * (n + dropped))
        checks.append(_check(
            good, "depth camera recorded",
            ("%d frames at %.1f fps, %s, %.0f MB" % (
                n, fps, depth.get("mode") or "?", (depth.get("bytes") or 0) / 1e6)
             + (", %d dropped" % dropped if dropped else "")) if good else
            ("no depth frames were written. The camera was on but nothing "
             "arrived; see depth_recorder.log in the walk folder." if n == 0 else
             "%d frames kept but %d dropped: the robot could not compress fast "
             "enough. Use a lower depth resolution." % (n, dropped))))
    elif depth.get("error"):
        checks.append(_check(False, "depth camera recorded", depth["error"]))

    # 7. The finished file, if one has been built. A walk that has not been
    #    converted yet is not failing -- it simply has not been done, so no
    #    check is added at all in that case.
    if bag["mcap"]:
        good = bag["closed"] and bag["has_metadata"]
        missing = []
        if not bag["closed"]:
            missing.append("no closing marker")
        if not bag["has_metadata"]:
            missing.append("no metadata.yaml")
        checks.append(_check(
            good, "bag finished writing",
            ("%.0f MB, closed properly" % (bag["bytes"] / 1e6)) if good else
            "the file is there but was cut off (%s). It will open and play "
            "back as nothing. Re-run the conversion, or recover it with "
            "recover_bag.py." % ", ".join(missing)))

    passed = sum(1 for c in checks if c["ok"])
    return {
        "name": meta.get("name") or os.path.basename(path),
        "path": path,
        "started_unix": started,
        "started_human": meta.get("started_human"),
        "seconds": seconds,
        "bytes": _dir_bytes(path),
        "stopped_because": meta.get("stopped_because"),
        "lidar": {"packets": packets, "pps": pps,
                  "duplicates_filtered": "not_addressed_to_us" in lidar},
        "imu": {"rows": int(imu.get("rows") or cont["rows"]),
                "hz": float(imu.get("hz") or 0.0)},
        "odom": {"present": has_odom, "rows": int(odom.get("rows") or 0)},
        "bag": bag,
        "verdict": {"checks": checks, "passed": passed, "total": len(checks),
                    "ok": passed == len(checks)},
    }


def list_walks(root, imu_limit=20000):
    """Every walk under `root`, newest first."""
    if not root or not os.path.isdir(root):
        return []
    out = []
    for entry in sorted(os.listdir(root)):
        full = os.path.join(root, entry)
        if is_walk_dir(full):
            try:
                out.append(describe(full, imu_limit=imu_limit))
            except OSError:
                continue
    out.sort(key=lambda w: w.get("started_unix") or 0, reverse=True)
    return out


def human_size(n):
    n = float(n)
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return ("%.0f %s" % (n, unit)) if unit == "B" else "%.1f %s" % (n, unit)
        n /= 1024.0


def human_when(unix):
    if not unix:
        return "unknown"
    return time.strftime("%Y-%m-%d %H:%M", time.localtime(unix))
