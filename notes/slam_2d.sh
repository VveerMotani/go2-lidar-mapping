#!/usr/bin/env bash
# Build a 2D floor plan with slam_toolbox, from a recorded walk or live.
#
#   ~/go2-lidar-mapping/notes/slam_2d.sh ~/go2-lidar-mapping/walks/<walk>        from a recording
#   ~/go2-lidar-mapping/notes/slam_2d.sh ~/go2-lidar-mapping/walks/<walk> --speed 2
#   ~/go2-lidar-mapping/notes/slam_2d.sh --live                             while connected
#
# From a recording it writes, into <walk>/map/:
#   map.pgm, map.yaml   the floor plan, in the standard ROS map format
#   map.png             the same picture, for opening on Windows
#   posegraph.*         slam_toolbox's own save, for continuing or localising later
#
# Live, it maps while you drive and Foxglove shows /map growing (add a Map panel,
# or a 3D panel with /map turned on). Press Ctrl + C to stop; the map is saved to
# ~/go2-lidar-mapping/walks/live-map-<time>/.
#
# WHAT IT DOES
# ------------
#   1. slices the 3D LiDAR cloud into a flat 2D scan (pointcloud_to_laserscan),
#      keeping only points between just above the floor and just above the body
#   2. feeds the scans and the robot's odometry to slam_toolbox, which matches
#      scans against each other, corrects the odometry's drift, and closes loops
#   3. saves the map
#
# NEEDS ODOMETRY
# --------------
# A walk without odom.csv has no moving odom -> base_link, so slam_toolbox is
# told the robot never moved. It will produce a map, but of one spot. The script
# says so and carries on, which is useful for checking the pipeline runs and
# useless as a map. Walks recorded from 2026-09-17 with the robot standing are
# the first that should work.
#
# Deliberately NOT `set -u`: ROS's setup scripts reference unset variables and
# exit 127 under it. Same note as in go2-start.sh.
set -o pipefail

LIVE=0
SPEED=1.0
WALK=""
while [ $# -gt 0 ]; do
  case "$1" in
    --live) LIVE=1; shift ;;
    --speed) SPEED="${2:-1.0}"; shift 2 ;;
    --speed=*) SPEED="${1#*=}"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) WALK="$1"; shift ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/unitree_ws"
PARAMS="$HERE/slam_2d.yaml"

ok()   { printf "  \033[32m/\033[0m %s\n" "$1"; }
bad()  { printf "  \033[31mX\033[0m %s\n" "$1"; }
step() { printf "\n\033[1m%s\033[0m\n" "$1"; }

if [ "$LIVE" = 0 ]; then
  if [ -z "$WALK" ] || [ ! -d "$WALK" ]; then
    echo "usage: slam_2d.sh <walk-directory> [--speed N]    or    slam_2d.sh --live"
    exit 1
  fi
  WALK="$(cd "$WALK" && pwd)"
  BAG="$WALK/bag"
  if ! ls "$BAG"/*.mcap >/dev/null 2>&1; then
    bad "$WALK has no bag yet. Convert it first:"
    echo "      $HERE/walk_to_bag.sh $WALK"
    exit 1
  fi
  OUT="$WALK/map"
else
  OUT="$(cd "$HERE/.." && pwd)/walks/live-map-$(date +%Y%m%d-%H%M%S)"
fi

# One mapping run at a time, and never alongside a conversion: both publish
# transforms, and two sets of transforms for the same frames confuse everything.
LOCK=/tmp/slam_2d.lock
for other in /tmp/slam_2d.lock /tmp/walk_to_bag.lock; do
  if [ -e "$other" ] && kill -0 "$(cat "$other" 2>/dev/null)" 2>/dev/null; then
    bad "another mapping run or conversion is running (pid $(cat "$other")). Wait for it."
    exit 1
  fi
done
echo $$ > "$LOCK"

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
unset UNITREE_IFACE
# shellcheck source=/dev/null
source "$WS/setup.env.sh" "" >/dev/null 2>&1
# shellcheck source=/dev/null
source "$WS/install/setup.bash" >/dev/null 2>&1

if [ "$LIVE" = 0 ]; then
  # Everything is local when mapping from a file, so pin DDS to loopback: the
  # robot cable may not be plugged in. Same reasoning as walk_to_bag.sh.
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true" /></Interfaces><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1344B</FragmentSize></General><Internal><SocketReceiveBufferSize min="10MB" /></Internal></Domain></CycloneDDS>'
  # Its own ROS domain, so a map being built from a file never shows up in a
  # live Foxglove session or collides with the live /tf. walk_to_bag.sh uses 87.
  export ROS_DOMAIN_ID="${SLAM_DOMAIN:-88}"
  SIM=true
else
  # Live, the LiDAR driver and odometry are already running under go2-start.sh's
  # DDS settings. Use the same ones, or the nodes would not find each other.
  SIM=false
fi

# ---------------------------------------------------------------------------
# Helpers are started in their own process groups and stopped by group, so the
# `ros2 run` wrapper and the real node it starts both go. Killing only the
# wrapper's pid leaves the node running: found the hard way in walk_to_bag.sh.
# ---------------------------------------------------------------------------
GROUPS_STARTED=()
start() {                      # start <logname> <command...>
  local log="/tmp/slam_2d_$1.log"; shift
  setsid "$@" > "$log" 2>&1 < /dev/null &
  GROUPS_STARTED+=("$!")
}

RESTORE_MAP_ODOM=0
cleanup() {
  for pg in "${GROUPS_STARTED[@]}"; do
    kill -TERM -- "-$pg" 2>/dev/null
  done
  sleep 1
  for pg in "${GROUPS_STARTED[@]}"; do
    kill -KILL -- "-$pg" 2>/dev/null
  done
  if [ "$RESTORE_MAP_ODOM" = 1 ]; then
    # Put back the static map -> odom that go2-start.sh runs, so the live
    # session looks exactly as it did before mapping started.
    setsid nohup ros2 run tf2_ros static_transform_publisher \
      --frame-id map --child-frame-id odom > /tmp/tf_map_odom.log 2>&1 < /dev/null &
    ok "restored the static map -> odom transform"
  fi
  rm -f "$LOCK"
}
trap cleanup EXIT
trap 'echo; exit 130' INT TERM

mkdir -p "$OUT"

save_map() {
  step "saving the map"
  local base="$OUT/map"
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$base" \
    --ros-args -p map_subscribe_transient_local:=true > /tmp/slam_2d_saver.log 2>&1
  if [ -f "$base.pgm" ]; then
    ok "$base.pgm and map.yaml"
  else
    bad "no map was saved; see /tmp/slam_2d_saver.log"
    tail -5 /tmp/slam_2d_saver.log | sed 's/^/      /'
    return 1
  fi

  timeout 20 ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/posegraph'}" >/dev/null 2>&1 \
    && ok "posegraph saved, for continuing this map later" \
    || echo "    (posegraph not saved; the map above is still good)"

  # PGM is the ROS standard but Windows will not open it. Write a PNG beside it,
  # standard library only.
  python3 - "$base.pgm" "$base.png" <<'PY' && ok "$base.png"
import struct, sys, zlib
src, dst = sys.argv[1], sys.argv[2]
data = open(src, "rb").read()
parts, pos = [], 0
while len(parts) < 4:                       # magic, width, height, maxval
    while data[pos:pos + 1].isspace():
        pos += 1
    if data[pos:pos + 1] == b"#":
        pos = data.index(b"\n", pos) + 1
        continue
    end = pos
    while not data[end:end + 1].isspace():
        end += 1
    parts.append(data[pos:end]); pos = end
pos += 1
w, h = int(parts[1]), int(parts[2])
pix = data[pos:pos + w * h]
raw = b"".join(b"\x00" + pix[y * w:(y + 1) * w] for y in range(h))
def chunk(k, d):
    return struct.pack(">I", len(d)) + k + d + struct.pack(">I", zlib.crc32(k + d) & 0xFFFFFFFF)
open(dst, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
                      + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
PY
}

# ---------------------------------------------------------------------------
step "1. checking the input"
# ---------------------------------------------------------------------------
if [ "$LIVE" = 0 ]; then
  ok "walk $(basename "$WALK")"
  if [ -f "$WALK/odom.csv" ] && [ "$(wc -l < "$WALK/odom.csv")" -gt 1 ]; then
    ok "odometry present: $(( $(wc -l < "$WALK/odom.csv") - 1 )) rows"
  else
    bad "this walk has NO odometry. slam_toolbox will be told the robot never"
    echo "    moved, so the map shows one spot. Useful to check the pipeline runs,"
    echo "    useless as a map. Record a walk with the robot standing."
  fi
else
  if ! pgrep -f "hesai_ros_driver_node" >/dev/null 2>&1; then
    bad "the LiDAR driver is not running. Run go2-start.sh first."
    exit 1
  fi
  if ! pgrep -f "odom_publisher.py" >/dev/null 2>&1; then
    bad "odometry is not running. Run go2-start.sh first."
    exit 1
  fi
  ok "LiDAR driver and odometry are running"
  # slam_toolbox publishes map -> odom itself. go2-start.sh publishes a static
  # one. Two publishers of the same link is exactly the kind of fault that
  # shows up as a map jumping between two places, so stop the static one for
  # as long as mapping runs, and put it back afterwards.
  if pgrep -f "static_transform_publisher.*--frame-id map --child-frame-id odom" >/dev/null 2>&1; then
    pkill -f "static_transform_publisher.*--frame-id map --child-frame-id odom"
    RESTORE_MAP_ODOM=1
    ok "paused the static map -> odom (slam_toolbox provides it while mapping)"
  fi
fi

# ---------------------------------------------------------------------------
step "2. starting the mapping nodes"
# ---------------------------------------------------------------------------
if [ "$LIVE" = 0 ]; then
  # Only the fixed LiDAR mount. The bag's own /tf_static is NOT replayed: it
  # also holds a static map -> odom, which would fight slam_toolbox's.
  start tf_lidar ros2 run tf2_ros static_transform_publisher \
    --x 0.171 --y 0 --z 0.0908 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id hesai_lidar \
    --ros-args -p use_sim_time:=true
  ok "LiDAR mount transform (Unitree's published position)"

  # A walk with odometry replays a moving odom -> base_link on /tf. A walk
  # without it stored a frozen one on /tf_static, which is not replayed, and
  # slam_toolbox then drops every scan waiting for a robot position that never
  # comes ("discarding message because the queue is full"). Supply the frozen
  # one here so the pipeline still runs end to end.
  if ! { [ -f "$WALK/odom.csv" ] && [ "$(wc -l < "$WALK/odom.csv")" -gt 1 ]; }; then
    start tf_frozen ros2 run tf2_ros static_transform_publisher \
      --frame-id odom --child-frame-id base_link \
      --ros-args -p use_sim_time:=true
    ok "frozen odom -> base_link (no odometry in this walk)"
  fi
fi

# The slice: base_link sits inside the body, roughly 0.3 m above the floor when
# standing. -0.05 .. 0.35 m around it keeps walls, doors and furniture and drops
# the floor, which would otherwise be drawn as an obstacle all around the robot.
start scan ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r cloud_in:=/lidar_points -r scan:=/scan \
  -p target_frame:=base_link -p transform_tolerance:=0.1 \
  -p min_height:=-0.05 -p max_height:=0.35 \
  -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.0087 \
  -p range_min:=0.3 -p range_max:=20.0 -p use_inf:=true \
  -p use_sim_time:=$SIM
ok "3D cloud to 2D scan"

start slam ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file "$PARAMS" -p use_sim_time:=$SIM
ok "slam_toolbox"
sleep 4

if ! kill -0 "${GROUPS_STARTED[-1]}" 2>/dev/null; then
  bad "slam_toolbox exited straight away; see /tmp/slam_2d_slam.log"
  tail -8 /tmp/slam_2d_slam.log | sed 's/^/      /'
  exit 1
fi

# ---------------------------------------------------------------------------
if [ "$LIVE" = 1 ]; then
  step "3. mapping live"
  echo "  Drive the robot slowly. In Foxglove, add a Map panel for /map, or turn"
  echo "  on /map in the 3D panel with the display frame set to map."
  echo "  Press Ctrl + C to stop and save."
  trap 'echo; save_map; exit 0' INT
  while true; do sleep 1; done
fi

step "3. replaying the walk"
# ---------------------------------------------------------------------------
echo "  at speed $SPEED. Takes about as long as the walk at speed 1."
ros2 bag play "$BAG" --clock --rate "$SPEED" \
  --topics /lidar_points /tf /odom /imu/data > /tmp/slam_2d_play.log 2>&1
ok "replay finished"
sleep 3                 # let the last scans be matched and the map published

save_map || exit 1

echo
echo "  Open the floor plan on Windows:"
echo "    $(wslpath -w "$OUT/map.png" 2>/dev/null || echo "$OUT/map.png")"
