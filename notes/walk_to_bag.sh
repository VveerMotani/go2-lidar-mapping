#!/usr/bin/env bash
# Turn a recorded walk into one file you can open in Foxglove and scrub.
#
#   ~/go2-lidar-mapping/notes/walk_to_bag.sh ~/walks/Lab-loop-1
#
# Produces  ~/walks/Lab-loop-1/bag/<name>_0.mcap  -- open that in Foxglove
# Desktop (File -> Open local file). You get a timeline with play, pause and a
# slider, and can step through frame by frame. No robot, no network, no live
# connection.
#
# WHY A BAG RATHER THAN A LIVE REPLAY
# -----------------------------------
# Replaying the pcap through the driver works, but it only ever streams "now" --
# there is no going back, no scrubbing, and it has to be re-run to see a moment
# again. A bag is a file: Foxglove reads it directly and gives you the whole
# recording on a timeline.
#
# WHAT IT DOES
# ------------
#   1. removes the duplicated packets, if this recording still has them
#   2. decodes the LiDAR pcap with the Hesai driver, with both the angle
#      and the firing-time calibrations applied, as fast as it will go
#   3. writes the bag (retime_bag.py): the LiDAR on the robot's clock, plus
#      odometry, IMU and the frame tree straight from the session's files
#   4. checks the file finished and no frames were lost
#
# A 315-second walk converted in about 8 minutes before 2026-09-17; the
# odometry used to be replayed at real speed. Now the time is the driver
# decoding plus writing the file.
#
# ROS DOMAIN
# ----------
# Everything runs on its own ROS domain (WALK_BAG_DOMAIN, default 87), so the
# conversion never appears in a live Foxglove session and the live driver does
# not have to be stopped. Before 2026-09-17 it shared the live domain: Foxglove
# showed the walk replaying as if it were live, and the live LiDAR was stopped
# for the length of the conversion.
#
# TIMESTAMPS COME FROM THE RECORDING, NOT FROM THE REPLAY (since 2026-09-17)
# -------------------------------------------------------------------------
# This used to stamp everything "now" as it replayed and assume the driver
# plays the pcap in real time. It does not: odom-test-1 replayed at 0.4x from
# E: and about 8x from WSL's own disk, so its LiDAR spread over 1,290 s
# against 315 s of odometry and scans landed at the wrong poses. Now the
# driver stamps frames with the LiDAR's packet time and retime_bag.py moves
# them onto the robot's clock with an offset measured from the pcap.
#
# DECODING IS THE VENDOR'S JOB, DELIBERATELY
# ------------------------------------------
# It would be faster to parse the pcap ourselves and write the mcap directly --
# no driver, no DDS, no waiting for the replay to play out in real time. We do
# not, because Hesai's driver is the authoritative implementation of their own
# packet format, and a second implementation of it in this repo would be one
# more thing to be subtly wrong. --speed is the compromise: the driver's own
# play_rate_ setting, which is a supported knob rather than a reimplementation.
#
# THE DUPLICATE PACKETS
# ---------------------
# record_session.py captured every LiDAR packet twice: once travelling from the
# sensor to the Jetson, and once again as lidar_relay.py forwarded a copy to the
# laptop. Both use UDP port 2368, and the recorder filtered on the port alone.
# Keeping only packets addressed to the Jetson leaves exactly the real ones.
# The recorder itself has since been fixed, but old recordings still need this,
# and it is harmless to run on a clean one.

set -o pipefail

SESSION=""
SPEED=1.0
while [ $# -gt 0 ]; do
  case "$1" in
    --speed) SPEED="${2:-1.0}"; shift 2 ;;
    --speed=*) SPEED="${1#*=}"; shift ;;
    *) SESSION="$1"; shift ;;
  esac
done

if [ -z "$SESSION" ] || [ ! -d "$SESSION" ]; then
  echo "usage: walk_to_bag.sh <session-directory> [--speed N]"
  echo "   eg: walk_to_bag.sh ~/walks/Lab-loop-1"
  echo
  echo "  --speed N   replay faster than real time. The driver's own"
  echo "              play_rate_ setting, with the IMU and odometry replays"
  echo "              scaled to match, so the three stay aligned. Conversion"
  echo "              takes as long as the walk at the default 1.0."
  echo
  echo "              UNVERIFIED above 1.0. The frame-count check at the end"
  echo "              is the guard: if the recorder cannot keep up it will"
  echo "              say the bag is SHORT. Start at 2 and check."
  exit 1
fi
SESSION="$(cd "$SESSION" && pwd)"
NAME="$(basename "$SESSION")"
WS="${GO2_WS:-$HOME/unitree_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_IP="${JETSON_IP:-192.168.123.18}"

LOCK="/tmp/walk_to_bag.lock"

ok()   { printf "  \033[32m/\033[0m %s\n" "$1"; }
bad()  { printf "  \033[31mX\033[0m %s\n" "$1"; }
step() { printf "\n\033[1m%s\033[0m\n" "$1"; }

for f in lidar.pcap imu.csv session.json; do
  [ -f "$SESSION/$f" ] || { bad "$SESSION has no $f"; exit 1; }
done

# Two of these running at once is not merely slow, it corrupts the result.
# Each starts its own driver publishing to /lidar_points and its own recorder
# writing to the same bag, and each one's cleanup kills the other's processes
# by name -- so whichever finishes first truncates the other. Observed on
# 2026-09-04: a 138-second walk produced a bag with 53% of its lidar frames.
if [ -e "$LOCK" ]; then
  OTHER=$(cat "$LOCK" 2>/dev/null)
  if kill -0 "$OTHER" 2>/dev/null; then
    bad "another walk_to_bag.sh is already running (pid $OTHER)"
    echo "    Wait for it to finish. Running two at once corrupts both."
    exit 1
  fi
  rm -f "$LOCK"          # stale lock from a run that was killed
fi
echo $$ > "$LOCK"

# Everything this script starts runs under `setsid`, in its own session, so
# that a signal aimed at the shell cannot leave a half-killed pipeline behind.
# The cost of that is that killing the shell does not stop any of them either.
# So teardown has to be explicit, and it has to happen on the interrupted path
# as well as the finished one.
#
# Armed only after the lock is taken. Before that point another conversion may
# legitimately own these processes, and a trap that fired on the "already
# running" exit would kill the other run's driver and recorder.
cleanup() {
  # SIGTERM, never SIGKILL, for the recorder: rosbag2 writes the mcap's closing
  # marker and its metadata.yaml during shutdown. Killed outright it leaves a
  # large file that plays back as nothing and reports no error.
  # Matched on this script's own arguments, so a recorder or driver the
  # user started is never touched.
  pkill -TERM -f "ros2 bag record .*walk_to_bag" 2>/dev/null
  pkill -f "hesai_ros_driver_node .*config_path" 2>/dev/null
  rm -f "$LOCK"
}

on_signal() {
  echo ""
  bad "interrupted -- stopping the replay and cleaning up"
  cleanup
  echo "  The partly written bag in $SESSION/bag is not usable."
  exit 130
}

trap on_signal INT TERM
trap cleanup EXIT

# Sourcing a script from inside a script hands it THIS script's positional
# arguments, and setup.env.sh treats its $1 as the network interface to use.
# Called as `walk_to_bag.sh ~/walks/Lab-loop-1`, that made CycloneDDS try to
# bind to a network interface named "/home/me/walks/Lab-loop-1" and every
# ROS node then failed with "rmw handle is invalid". Pass an explicit empty
# argument, and clear the inherited value so it re-detects rather than reusing
# an interface that may since have gone away.
unset UNITREE_IFACE
# shellcheck source=/dev/null
source "$WS/setup.env.sh" "" >/dev/null 2>&1
# shellcheck source=/dev/null
source "$WS/install/setup.bash" >/dev/null 2>&1

# Everything this script does is local: the driver replays a file and the
# recorder writes another, both on this machine. So pin DDS to loopback rather
# than to whichever cable happens to be plugged in.
#
# This matters more than it sounds. ROS 2 uses DDS even to talk to itself, and
# setup.env.sh points DDS at the robot-facing interface. Unplug the robot and
# that interface disappears -- then every node here dies with
# "eth1: does not match an available interface", despite the robot playing no
# part in the conversion. Observed 2026-09-04, mid-conversion, with the cable
# pulled. On loopback it runs anywhere, cable or no cable.
#
# The receive buffer is asked for (max=) rather than demanded (min=). In the
# Docker image (docker/) the kernel is the host's, and on Docker Desktop its
# net.core.rmem_max is the 208 KB default: min="10MB" there makes every node
# refuse to start. Here in WSL rmem_max is 16 MB, so both get the same 10 MB.
# A replay that did drop frames is caught by the frame count in step 6.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true" /></Interfaces><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1344B</FragmentSize></General><Internal><SocketReceiveBufferSize max="10MB" /></Internal></Domain></CycloneDDS>'
export ROS_DOMAIN_ID="${WALK_BAG_DOMAIN:-87}"
ok "DDS on loopback, ROS domain $ROS_DOMAIN_ID (invisible to live Foxglove)"

SECONDS_LONG=$(python3 -c "
import json
d = json.load(open('$SESSION/session.json'))
print(int(d.get('seconds', 60) / max(float('$SPEED'), 0.01)) + 8)
" 2>/dev/null || echo 60)

# ---------------------------------------------------------------------------
step "1. removing duplicated packets"
# ---------------------------------------------------------------------------
CLEAN="$SESSION/lidar_clean.pcap"

# Recordings made after the recorder was fixed are already filtered at capture
# time, and say so: session.json carries "not_addressed_to_us", the count of
# packets the sniffer threw away. Cleaning those again copies the largest file
# in the session -- 215 MB on First-Walk -- to produce a byte-for-byte
# duplicate, and spends a full tcpdump pass doing it. Ask the file instead.
ALREADY_FILTERED=$(python3 -c "
import json
d = json.load(open('$SESSION/session.json'))
print(1 if (d.get('lidar') or {}).get('not_addressed_to_us') is not None else 0)
" 2>/dev/null || echo 0)

if [ "$ALREADY_FILTERED" = "1" ] && [ ! -f "$CLEAN" ]; then
  SOURCE_PCAP="$SESSION/lidar.pcap"
  ok "recorder already filtered this one -- using lidar.pcap directly"
elif [ -f "$CLEAN" ]; then
  SOURCE_PCAP="$CLEAN"
  ok "already done: $(basename "$CLEAN")"
else
  SOURCE_PCAP="$CLEAN"
  sudo tcpdump -r "$SESSION/lidar.pcap" -w "$CLEAN" "dst host $JETSON_IP" 2>/dev/null
  sudo chown "$USER" "$CLEAN" 2>/dev/null
  BEFORE=$(sudo tcpdump -r "$SESSION/lidar.pcap" -nn 2>/dev/null | wc -l)
  AFTER=$(sudo tcpdump -r "$SOURCE_PCAP" -nn 2>/dev/null | wc -l)
  ok "$BEFORE packets -> $AFTER kept"
fi

# ---------------------------------------------------------------------------
step "2. pointing the driver at the recording"
# ---------------------------------------------------------------------------
# The replay reads from, and the raw bag is written to, WSL's own disk. Read
# across the E: drive share the driver managed 2 frames a second of 10, and
# writing the raw bag there competes with it. Only the finished, retimed bag
# goes to the session folder.
WORK="$HOME/.cache/walk_to_bag/$NAME"
rm -rf "$WORK"
mkdir -p "$WORK"
cp "$SOURCE_PCAP" "$WORK/lidar.pcap" || { bad "could not copy the pcap to $WORK"; exit 1; }
REPLAY_PCAP="$WORK/lidar.pcap"
ok "pcap copied to WSL's disk for the replay"

CFG="$SESSION/replay_config.yaml"
# The walk's own copy first, then the walks folder beside it (where
# get_lidar_calibration.py saves it), then this laptop's usual place, then the
# copy baked into the Docker image (GO2_CONFIG_DIR).
CORRECTION="${GO2_LIDAR_CORRECTION:-}"
if [ -z "$CORRECTION" ]; then
  REPO_WALKS="$(cd "$HERE/.." && pwd)/walks"
  for c in "$SESSION/xt16_correction.csv" "$(dirname "$SESSION")/xt16_correction.csv"            "$REPO_WALKS/xt16_correction.csv"            "${GO2_CONFIG_DIR:-/nonexistent}/xt16_correction.csv"; do
    [ -f "$c" ] && { CORRECTION="$c"; break; }
  done
  CORRECTION="${CORRECTION:-$REPO_WALKS/xt16_correction.csv}"
fi
if [ ! -f "$CORRECTION" ]; then
  bad "no angle calibration at $CORRECTION"
  echo "    Without it every laser is treated as horizontal and the replayed"
  echo "    cloud comes out flat -- one layer, no height."
  echo "    Fetch it while the sensor is connected:"
  echo "      python3 $HERE/get_lidar_calibration.py"
  exit 1
fi

# The per-laser FIRING TIME offsets. Different from the angle calibration, and
# also needed for replay: the sixteen lasers do not fire simultaneously, so
# each point in a block was measured a few microseconds after the last, while
# the sensor was still turning. Without this the driver assumes they all fired
# at once, which smears each block slightly.
#
# Unlike the angle file, this one is generic to the model rather than to our
# unit, and Hesai ship it inside the SDK -- so there is nothing to fetch and no
# reason for it to have been left as the placeholder "Your firetime file path"
# that the stock config carries.
FIRETIMES="${GO2_LIDAR_FIRETIMES:-$WS/src/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/correction/firetime_correction/PandarXT-16_Firetime Correction File.csv}"
if [ -f "$FIRETIMES" ]; then
  ok "firetimes $(basename "$FIRETIMES")"
else
  FIRETIMES=""
  bad "no firetime file found -- continuing without it (points within each"
  echo "    block will share one timestamp; harmless for viewing, slightly"
  echo "    wrong for SLAM)"
fi

python3 - "$WS/src/HesaiLidar_ROS_2.0/config/config.yaml" "$REPLAY_PCAP" "$CFG" "$CORRECTION" "$FIRETIMES" "$SPEED" <<'PY'
import re, sys
src, pcap, out, correction = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
firetimes, speed = sys.argv[5], sys.argv[6]

lines = open(src).read().splitlines(True)
block = None
result = []
for line in lines:
    stripped = line.strip()
    # The config repeats correction_file_path under several source blocks --
    # udp, pcap, rosbag, serial. Only the pcap one matters here, so track which
    # block we are inside rather than replacing the first match and silently
    # editing the live-sensor settings instead.
    if stripped.endswith(":") and not stripped.startswith("#"):
        block = stripped[:-1]

    # The LiDAR's own packet time, not arrival time: retime_bag.py maps it
    # onto the robot's clock afterwards. Arrival time depends on how fast
    # the replay runs, which is not real time.
    if re.match(r"\s*use_timestamp_type:", line):
        line = re.sub(r"use_timestamp_type:\s*\d", "use_timestamp_type: 0", line)
    elif re.match(r"\s*source_type:", line):
        line = re.sub(r"source_type:\s*\d", "source_type: 2", line)
    elif block == "pcap_type":
        if re.match(r"\s*pcap_path:", line):
            line = re.sub(r'pcap_path:\s*"[^"]*"', 'pcap_path: "%s"' % pcap, line)
        elif re.match(r"\s*correction_file_path:", line):
            line = re.sub(r'correction_file_path:\s*"[^"]*"',
                          'correction_file_path: "%s"' % correction, line)
        elif re.match(r"\s*firetimes_path:", line) and firetimes:
            line = re.sub(r'firetimes_path:\s*"[^"]*"',
                          'firetimes_path: "%s"' % firetimes, line)
        elif re.match(r"\s*play_rate_:", line):
            line = re.sub(r"play_rate_:\s*[0-9.]+", "play_rate_: %s" % speed, line)
        elif re.match(r"\s*pcap_play_synchronization:", line):
            # No pacing: decode as fast as the machine allows. Timestamps come
            # from the packets (use_timestamp_type: 0), so speed is irrelevant.
            line = re.sub(r"pcap_play_synchronization:\s*\w+",
                          "pcap_play_synchronization: false", line)
        elif re.match(r"\s*pcap_play_in_loop:", line):
            line = re.sub(r"pcap_play_in_loop:\s*\w+", "pcap_play_in_loop: false", line)
    result.append(line)

open(out, "w").write("".join(result))
PY
ok "$(basename "$CFG")"
ok "calibration $(basename "$CORRECTION")"

# ---------------------------------------------------------------------------
step "3. clearing anything left from an earlier conversion"
# ---------------------------------------------------------------------------
# Only this script's own leftovers: a replay driver (matched by its config_path)
# and a raw recorder (matched by its output under ~/.cache/walk_to_bag). The
# live driver and odom_publisher.py are left alone. They are on another ROS
# domain (see "ROS domain" above) and cannot reach this bag.
#
# A replay driver that has reached the end of its pcap can ignore SIGTERM and
# linger (seen 2026-09-17), so escalate.
pkill -f "hesai_ros_driver_node .*config_path" 2>/dev/null
pkill -TERM -f "ros2 bag record .*walk_to_bag" 2>/dev/null
sleep 2
pkill -KILL -f "hesai_ros_driver_node .*config_path" 2>/dev/null
if pgrep -f "hesai_ros_driver_node .*config_path" >/dev/null 2>&1; then
  bad "an earlier replay driver will not stop:"
  pgrep -af "hesai_ros_driver_node .*config_path" | sed 's/^/      /'
  exit 1
fi
ok "clear"

# ---------------------------------------------------------------------------
step "4. decoding the LiDAR"
# ---------------------------------------------------------------------------
# Hesai's driver turns the packets into point clouds, as fast as it can read
# the file. Only /lidar_points is recorded here; the odometry, IMU and frame
# tree are written straight from their files in step 5.
#
# WHY NOT REPLAY EVERYTHING IN REAL TIME, AS BEFORE 2026-09-17
# The old way replayed odometry and IMU at real speed next to the driver and
# stamped everything as it arrived. That made conversion take as long as the
# walk, and it silently depended on the driver keeping real time, which it
# does not (0.4x from E:, about 8x from WSL's own disk). Timestamps now come
# from the recording (retime_bag.py), so nothing here needs to run at any
# particular speed.
BAGDIR="$SESSION/bag"
RAWBAG="$WORK/raw"
# Reliable, deep queue: at several times real speed the driver can outpace the
# recorder, and a dropped frame here is a hole in the map. The frame count in
# step 6 is the check.
cat > "$WORK/qos.yaml" <<'QOS'
/lidar_points:
  reliability: reliable
  history: keep_last
  depth: 2000
QOS

# 250 packets per turn of the sensor; each record in this pcap is a 16-byte
# pcap header plus the frame, so count records by walking the headers.
EXPECTED_FRAMES=$(python3 -c "
import struct
n = 0
with open('$REPLAY_PCAP', 'rb') as fh:
    fh.seek(24)
    while True:
        h = fh.read(16)
        if len(h) < 16:
            break
        fh.seek(struct.unpack('<IIII', h)[2], 1)
        n += 1
print(max(1, n // 250))
" 2>/dev/null || echo 1)

# One pass of the driver over the pcap, recorded into $RAWBAG. The driver exits
# at the end of the pcap (pcap_play_in_loop is false). The backstop is
# generous: four times the walk, which it has never needed.
decode() {
  rm -rf "$RAWBAG"
  setsid nohup ros2 bag record --storage mcap -o "$RAWBAG" \
    --qos-profile-overrides-path "$WORK/qos.yaml" /lidar_points \
    > /tmp/walk_bag_record.log 2>&1 < /dev/null &
  sleep 4

  setsid nohup ros2 run hesai_ros_driver hesai_ros_driver_node \
    --ros-args -p config_path:="$CFG" \
    > /tmp/walk_bag_lidar.log 2>&1 < /dev/null &

  for _ in $(seq 1 30); do
    pgrep -f "config_path:=$CFG" >/dev/null 2>&1 && break
    sleep 1
  done
  if ! pgrep -f "config_path:=$CFG" >/dev/null 2>&1; then
    bad "the replay driver did not start -- see /tmp/walk_bag_lidar.log"
    head -6 /tmp/walk_bag_lidar.log 2>/dev/null | sed 's/^/      /'
    exit 1
  fi

  LIMIT=$(( SECONDS_LONG * 4 ))
  ELAPSED=0
  while [ "$ELAPSED" -lt "$LIMIT" ]; do
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    pgrep -f "config_path:=$CFG" >/dev/null 2>&1 || break
    FRAMES=$(grep -c "raw frame" /tmp/walk_bag_lidar.log 2>/dev/null)
    printf "\r  %ds  frame %s of about %s   " "$ELAPSED" "${FRAMES:-0}" "$EXPECTED_FRAMES"
  done
  echo
  if [ "$ELAPSED" -ge "$LIMIT" ]; then
    bad "the driver did not finish within ${LIMIT}s -- the bag may be short"
    pkill -KILL -f "config_path:=$CFG" 2>/dev/null
  else
    ok "decoded in ${ELAPSED}s"
  fi
  sleep 3     # let the last frames reach the recorder

  # SIGTERM, not SIGKILL: rosbag2 writes the mcap's closing marker and its
  # metadata.yaml during shutdown. A recorder killed outright leaves a file that
  # plays back as nothing (First-Walk-CB-1s-Floor, 2026-09-04).
  pkill -TERM -f "ros2 bag record .*walk_to_bag" 2>/dev/null
  for _ in $(seq 1 30); do
    pgrep -f "ros2 bag record .*walk_to_bag" >/dev/null 2>&1 || break
    sleep 1
  done
  if [ -z "$(find "$RAWBAG" -name "*.mcap" 2>/dev/null | head -1)" ]; then
    bad "no LiDAR was recorded -- see /tmp/walk_bag_record.log"
    tail -5 /tmp/walk_bag_record.log 2>/dev/null | sed 's/^/    /'
    exit 1
  fi
}

decode
# Unpaced, the driver can outrun the recorder when the machine is busy: frames
# the recorder cannot take in time are dropped, and the map gets holes. Seen
# 2026-09-25 in the Docker image, odom-test-1 decoded while another job had all
# eight cores: 2,338 of 3,160 frames; the same run on an idle machine kept
# 3,150. A slower laptop could do the same, so check here and, if short,
# decode once more paced to the recording's own speed. That takes as long as
# the walk, and is only paid when it is needed.
RAW_FRAMES=$(grep -m1 "message_count:" "$RAWBAG/metadata.yaml" 2>/dev/null | tr -dc '0-9')
if [ "${RAW_FRAMES:-0}" -lt $(( EXPECTED_FRAMES * 97 / 100 )) ]; then
  bad "only ${RAW_FRAMES:-0} of about $EXPECTED_FRAMES frames kept up -- the machine was too busy"
  echo "    decoding again at the recording's own pace; this takes as long as the walk"
  sed -i -E 's/pcap_play_synchronization:\s*\w+/pcap_play_synchronization: true/' "$CFG"
  decode
fi

# ---------------------------------------------------------------------------
step "5. writing the bag, everything on the recording's clock"
# ---------------------------------------------------------------------------
# retime_bag.py writes the final bag: the LiDAR moved onto the robot's clock,
# and odometry, IMU and the frame tree straight from the session's files.
#
#     map --(static)--> odom --(odom.csv)--> base_link --(static)--> hesai_lidar
#
# base_link -> hesai_lidar is Unitree's documented mounting: 0.171 m forward,
# 0.0908 m up, no rotation (SLAM and Navigation Services Interface page), the
# same numbers go2-start.sh uses.
# Keep the bag this replaces, as bag-previous (one generation: a bag with depth
# is several GB). A re-process must never silently delete the last good file.
if [ -d "$BAGDIR" ]; then
  rm -rf "$SESSION/bag-previous"
  mv "$BAGDIR" "$SESSION/bag-previous" && ok "previous bag kept as bag-previous"
fi
if [ ! -f "$SESSION/odom.csv" ]; then
  bad "no odom.csv in this session: the cloud will NOT accumulate"
  echo "    Every scan will be drawn at the same pose (recordings before"
  echo "    2026-09-05). A frozen odom -> base_link is written so the frame"
  echo "    tree still joins up and Foxglove shows the data."
fi
python3 "$HERE/retime_bag.py" "$SOURCE_PCAP" "$RAWBAG" "$BAGDIR" --session "$SESSION"
RETIME=$?
if [ "$RETIME" = 0 ]; then
  ok "written; LiDAR clock saved to lidar_clock.json"
  rm -rf "$WORK"
elif [ "$RETIME" = 2 ]; then
  bad "written, but LiDAR and odometry do not cover the same time (see above)"
  rm -rf "$WORK"
else
  bad "writing the bag failed; the decoded LiDAR is kept at $RAWBAG"
  exit 1
fi

# ---------------------------------------------------------------------------
step "6. result"
# ---------------------------------------------------------------------------
MCAP=$(find "$BAGDIR" -name "*.mcap" 2>/dev/null | head -1)
if [ -z "$MCAP" ]; then
  bad "no mcap produced -- see /tmp/walk_bag_record.log"
  tail -5 /tmp/walk_bag_record.log 2>/dev/null | sed 's/^/    /'
  exit 1
fi

ok "$MCAP  ($(du -h "$MCAP" | cut -f1))"

# Did the file actually finish? A truncated mcap looks entirely normal from the
# outside -- right name, plausible size -- and then plays back as nothing.
# Two independent signs, because they fail separately:
#   * the closing magic, written as the writer shuts down
#   * metadata.yaml, written by rosbag2 after that
FINISHED=$(python3 -c "
import sys
try:
    with open('$MCAP','rb') as f:
        f.seek(-8, 2)
        sys.stdout.write('1' if f.read(8) == b'\x89MCAP0\r\n' else '0')
except Exception:
    sys.stdout.write('0')
" 2>/dev/null || echo 0)

if [ "$FINISHED" != "1" ]; then
  bad "the mcap has no closing marker -- it was cut off mid-write"
  echo "    Nothing will play it. Recover what is there with:"
  echo "      python3 $HERE/recover_bag.py $BAGDIR"
  echo "    or simply re-run this script; the source files are untouched."
elif [ ! -f "$BAGDIR/metadata.yaml" ]; then
  bad "no metadata.yaml -- the mcap closed but rosbag2 did not finish"
  echo "      python3 $HERE/recover_bag.py $BAGDIR"
else
  ok "file closed properly (end marker and metadata.yaml both present)"
fi

ros2 bag info "$BAGDIR" 2>/dev/null | sed 's/^/    /'

# Compare what arrived against what the recording holds. A short bag looks
# entirely normal in Foxglove -- it just ends early -- so it has to be checked
# rather than eyeballed.
# Count the pcap that was actually replayed, NOT session.json's packet count.
# On a recording made before the recorder filtered duplicates, session.json
# holds every packet the sniffer saw -- each real one twice -- so a perfect
# conversion of Lab-loop-1 was reported as "245 of about 487 (50%) -- SHORT".
# It was 245 of 244. The scary number was the check, not the bag.
#
# Counted by walking the pcap record headers rather than by shelling out to
# tcpdump: no sudo, no second pass over 200 MB of payload, and it works the
# same whether stage 1 deduplicated the file or used it as it was.
EXPECTED=$(python3 -c "
import struct, sys

path = '$SOURCE_PCAP'
with open(path, 'rb') as fh:
    magic = fh.read(4)
    if magic in (b'\xd4\xc3\xb2\xa1', b'\x4d\x3c\xb2\xa1'):
        end = '<'
    elif magic in (b'\xa1\xb2\xc3\xd4', b'\xa1\xb2\x3c\x4d'):
        end = '>'
    else:
        print(0); sys.exit()
    fh.seek(24)                      # past the file header
    n = 0
    first = last = None
    while True:
        head = fh.read(16)           # ts_sec, ts_usec, incl_len, orig_len
        if len(head) < 16:
            break
        sec, usec, incl, _ = struct.unpack(end + 'IIII', head)
        fh.seek(incl, 1)
        first = sec if first is None else first
        last = sec
        n += 1
# The XT16 turns 10 times a second. Expected frames come from the recording's
# length, not its packet count: a recording that lost packets still has every
# turn, each with fewer points (odom-test-2 lost 13.5% and reported 115%).
span = (last - first + 1) if first is not None else 0
print(max(1, int(span * 10)), int(100 * n / max(span * 2500, 1)))
" 2>/dev/null || echo "0 0")
PACKETS_PCT=${EXPECTED#* }
EXPECTED=${EXPECTED%% *}
GOT=$(ros2 bag info "$BAGDIR" 2>/dev/null \
      | grep "/lidar_points" | grep -oE "Count: [0-9]+" | grep -oE "[0-9]+")
if [ -n "$EXPECTED" ] && [ -n "$GOT" ] && [ "$EXPECTED" -gt 0 ]; then
  PCT=$(( GOT * 100 / EXPECTED ))
  if [ -n "$PACKETS_PCT" ] && [ "$PACKETS_PCT" -lt 95 ]; then
    bad "packets    about ${PACKETS_PCT}% of what the LiDAR sent reached the recorder"
    echo "               every frame is there, with fewer points. Recordings made before"
    echo "               the recorder's kernel filter (2026-09-17) lost packets while"
    echo "               the depth camera streamed to the laptop."
  fi
  if [ "$PCT" -ge 90 ]; then
    ok "frames     $GOT of about $EXPECTED (${PCT}%)"
  else
    bad "frames     only $GOT of about $EXPECTED (${PCT}%) -- the bag is SHORT"
    echo "               the replay was cut off. Re-run this, alone, with"
    echo "               nothing else heavy on the machine."
  fi
fi

WINPATH="$(wslpath -w "$MCAP" 2>/dev/null || echo "$MCAP")"
echo
echo "  Open it in Foxglove Desktop:  File -> Open local file"
echo "    $WINPATH"
echo
echo "  Then add a 3D panel: display frame 'odom', Follow mode Fixed, turn on"
echo "  /lidar_points, and use the timeline to play, pause and scrub."
echo
echo "  For a sharp map and the robot's track, build the scan-matched map:"
echo "    $HERE/process_walk.sh $SESSION --map-only"
echo "  (or the dashboard's Walk library, Build map)."
echo
echo "  Check it properly:  python3 $HERE/inspect_bag.py $BAGDIR"
