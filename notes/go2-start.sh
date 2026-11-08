#!/usr/bin/env bash
# Bring the whole Go2 stack up. One command, run it after powering the robot on.
#
#   wsl -d unitree
#   ~/go2-lidar-mapping/notes/go2-start.sh
#
# Safe to run twice -- anything already running is left alone.
#
# This replaces the startup half of go2-up.sh (since removed), which had a bug: it started the
# robot-side services with `sshpass` plus `ssh -f`, which do not work together,
# and sent that step's errors to /dev/null. So it failed silently, brought the
# laptop side up anyway, and looked like it had worked while nothing at all was
# running on the robot. Observed 2026-09-04.

# Deliberately NOT `set -u`. ROS's own setup scripts reference unset variables
# all over the place, so with -u the shell dies the instant setup.env.sh is
# sourced -- silently, exit 127, no message, right after the first tick of
# output. Cost an afternoon on 2026-09-04. Do not add it back.
set -o pipefail

JETSON=192.168.123.18
ROBOT=192.168.123.161
LIDAR=192.168.123.20
LAPTOP=192.168.123.99
WS="$HOME/unitree_ws"

# Where the XT16 sits relative to the robot body, in metres and radians.
#
# THESE ARE UNITREE'S PUBLISHED NUMBERS, not a guess. From the SDK docs, SLAM
# and Navigation Services Interface:
#
#   "The coordinates of the XT-16 lidar in the IMU coordinate system inside the
#    Go2 body are (0.1710, 0, 0.0908), without relative rotation."
#
# (For contrast the Mid-360 is (0.1870, 0, 0.0803) with a 13 degree pitch about
# Y, so the XT16 sitting level is deliberate rather than an omission.)
#
# They replaced a guess of x=0, z=0.30 -- 17 cm short in X and 3.3x too high.
# Next time the robot is here, measure to CONFIRM these rather than to derive
# them: about 17 cm forward of the body IMU and 9 cm above it. If the tape says
# something materially different, the XT16 is not in Unitree's factory mount
# and this whole block has to be re-derived. See docs/sdk-doc-review.md section 2.
LIDAR_X=0.171
LIDAR_Y=0
LIDAR_Z=0.0908
LIDAR_YAW=0
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
SSH_OPTS="$SSH_OPTS -o ConnectTimeout=8 -o LogLevel=ERROR"
SSH="sshpass -p $GO2_PASSWORD ssh $SSH_OPTS unitree@$JETSON"

ok()   { printf "  \033[32m/\033[0m %s\n" "$1"; }
bad()  { printf "  \033[31mX\033[0m %s\n" "$1"; }
step() { printf "\n\033[1m%s\033[0m\n" "$1"; }

# ---------------------------------------------------------------------------
step "1. laptop address"
# ---------------------------------------------------------------------------
# Windows forgets 192.168.123.99 on every restart, and setup.env.sh chooses its
# network interface by looking for it. Without it, everything below binds to
# the wrong interface and silently receives nothing.
if ip -o -4 addr show | grep -q "$LAPTOP"; then
  ok "$LAPTOP present on $(ip -o -4 addr show | awk -v ip="$LAPTOP" '$4 ~ ip {print $2}')"
else
  bad "$LAPTOP is missing. In PowerShell as Administrator:"
  echo "      New-NetIPAddress -InterfaceAlias 'Ethernet' -IPAddress $LAPTOP -PrefixLength 24"
  echo
  echo "    If it says the address already exists but the robot still will not"
  echo "    answer, the SkipAsSource flag is set and blocks outgoing traffic:"
  echo "      Get-NetIPAddress -IPAddress $LAPTOP | Set-NetIPAddress -SkipAsSource \$false"
  exit 1
fi

# shellcheck source=/dev/null
source "$WS/setup.env.sh" >/dev/null 2>&1

# setup.env.sh sources ROS and the unitree_ros2 workspace, but NOT the main
# workspace -- and that is where hesai_ros_driver is built. Without this line
# the driver fails with "Package 'hesai_ros_driver' not found", listing a
# search path that pointedly does not include ~/unitree_ws/install.
# The old go2-up.sh sourced both; this script originally sourced only the first.
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$WS/install/setup.bash" >/dev/null 2>&1
else
  bad "$WS/install/setup.bash is missing -- has the workspace been built?"
fi

if [ -z "${UNITREE_IFACE:-}" ]; then
  bad "setup.env.sh did not set an interface -- run it by hand to see the error:"
  echo "      source $WS/setup.env.sh"
  exit 1
fi
ok "environment ready, interface $UNITREE_IFACE"

# ---------------------------------------------------------------------------
step "2. waiting for the robot"
# ---------------------------------------------------------------------------
for _ in $(seq 1 40); do
  ping -c1 -W2 "$JETSON" >/dev/null 2>&1 && break
  sleep 3
done
ping -c1 -W2 "$JETSON" >/dev/null 2>&1 || { bad "Jetson $JETSON never came up"; exit 1; }

for pair in "$ROBOT robot" "$JETSON jetson" "$LIDAR lidar"; do
  set -- $pair
  if ping -c1 -W2 "$1" >/dev/null 2>&1; then ok "$2 $1"; else bad "$2 $1 not answering"; fi
done

# ---------------------------------------------------------------------------
step "3. robot clock"
# ---------------------------------------------------------------------------
# The Jetson has no battery-backed clock and boots at 1970. That breaks pip,
# makes every log timestamp useless, and makes a walk recording unusable,
# because the LiDAR and IMU are lined up by their timestamps.
$SSH "echo $GO2_PASSWORD | sudo -S date -s @$(date +%s) >/dev/null 2>&1" >/dev/null 2>&1
ok "robot clock: $($SSH 'date' 2>/dev/null)"

# ---------------------------------------------------------------------------
step "4. services on the robot"
# ---------------------------------------------------------------------------
# Piped in as a real script rather than squeezed into an ssh argument. That is
# the fix for the bug described at the top of this file.
if [ -f "$HERE/jetson/start_services.sh" ]; then
  $SSH 'bash -s' < "$HERE/jetson/start_services.sh" "$LAPTOP" 2>&1 | sed 's/^/  /'
else
  bad "missing $HERE/jetson/start_services.sh"
  exit 1
fi

# ---------------------------------------------------------------------------
step "5. services on the laptop"
# ---------------------------------------------------------------------------
# start_local <label> <pattern-that-identifies-it> <command...>
#
# The pattern is separate from the label because several of these are the same
# program with different arguments -- two static_transform_publishers, say.
# Matching on the program name alone would see one running and skip the other,
# which is exactly how the lidar transform kept going missing.
start_local() {
  local name="$1"; local match="$2"; shift 2
  if pgrep -f "$match" >/dev/null 2>&1; then
    ok "already running   $name"
    return
  fi
  setsid nohup "$@" > "/tmp/$name.log" 2>&1 < /dev/null &
  sleep 2
  if pgrep -f "$match" >/dev/null 2>&1; then
    ok "started           $name"
  else
    bad "FAILED            $name"
    # Show the reason here instead of leaving it in a log nobody reads. A
    # missing package is by far the most common cause and is one line long.
    sed -n '1,4p' "/tmp/$name.log" 2>/dev/null | sed 's/^/                    /'
  fi
}

# The robot publishes no /tf at all, so the frame tree has to be built here.
#
#     map --(static)--> odom --(MOVING)--> base_link --(static)--> hesai_lidar
#
# The middle link is the one that matters and the one this stack got wrong
# until 2026-09-05. It used to be another static transform, straight from odom
# to hesai_lidar -- which says the LiDAR never moves. Every frame of a walk
# then gets drawn at the same world pose, so ten metres of walking piles up in
# one place instead of spreading out into a room. The first recorded walk has
# exactly that shape: /tf_static with two transforms and no dynamic /tf at all.
start_local tf_map_odom "child-frame-id odom" \
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom

# The sensor is bolted to the BODY, not to the world. Vendor numbers, above.
start_local tf_base_lidar "child-frame-id hesai_lidar" \
  ros2 run tf2_ros static_transform_publisher \
    --x "$LIDAR_X" --y "$LIDAR_Y" --z "$LIDAR_Z" \
    --roll 0 --pitch 0 --yaw "$LIDAR_YAW" \
    --frame-id base_link --child-frame-id hesai_lidar

# And the body moves. This is the link that makes the cloud accumulate.
# It comes from Unitree's own legged + IMU fusion odometry on rt/sportmodestate
# -- free, always published, no extra load on the robot. It drifts, as all dead
# reckoning does, but a drifting pose is incomparably better than a frozen one.
start_local odom_publisher "odom_publisher.py" \
  python3 "$HERE/odom_publisher.py"

# stdbuf keeps the driver's log line-buffered; without it output arrives in
# clumps and looks like the driver has stalled when it has not.
# Match on "start.py", not just the package name. A driver replaying a
# recorded pcap runs as `ros2 run ... -p config_path:=<somewhere>` and would
# otherwise look identical -- so after building a bag, this script would decide
# the live driver was already running and quietly leave you with a finished
# replay publishing nothing.
start_local hesai_ros_driver "hesai_ros_driver start.py" \
  stdbuf -oL ros2 launch hesai_ros_driver start.py

# The depth camera. The client waits quietly for the robot's relay, so it is
# harmless while the camera is off; switch the camera on from the dashboard's
# Sensors panel and depth appears without anything else to start here.
start_local realsense_client "realsense_clien[t].py" \
  python3 "$HERE/realsense_client.py" --host "$JETSON"

# The depth cloud Foxglove draws, from the decimated preview image. The full
# resolution cloud is about 5 MB a message, too heavy for the bridge.
start_local depth_points "point_cloud_xyz_nod[e]" \
  ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
    -r image_rect:=/camera/depth_preview/image_rect_raw \
    -r camera_info:=/camera/depth_preview/camera_info \
    -r points:=/depth/points

# Frame rate and delay of every sensor as /go2/rate/* and /go2/delay/*, for
# the plots in the "Go2 sensors" Foxglove layout.
start_local sensor_monitor "sensor_monito[r].py" \
  python3 "$HERE/sensor_monitor.py"

# The 3D view lives in Foxglove on Windows. foxglove_bridge is what it talks to.
start_local foxglove_bridge foxglove_bridge \
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml

sleep 8

# ---------------------------------------------------------------------------
step "6. check"
# ---------------------------------------------------------------------------
curl -s -m 5 -o /dev/null -w "  cameras    :8080  http %{http_code}\n" \
  "http://$JETSON:8080/stats" 2>/dev/null || bad "cameras not answering"

HEALTH=$(curl -s -m 5 "http://$JETSON:8090/health" 2>/dev/null)
if [ -n "$HEALTH" ]; then
  ok "dashboard  http://$JETSON:8090"
  echo "$HEALTH" | python3 -c "
import json,sys
d = json.load(sys.stdin)
s = d['streams']
print('    lowstate        %6.0f Hz on %s' % (s['lowstate']['hz'], s['lowstate']['topic']))
print('    sportmodestate  %6.0f Hz on %s' % (s['sportmodestate']['hz'], s['sportmodestate']['topic']))
if not s['sportmodestate']['hz']:
    print('    (no sportmodestate: the robot is probably not standing yet)')
" 2>/dev/null
else
  bad "dashboard not answering -- ssh in and read /tmp/dashboard.log"
fi

# Check that both transform publishers are running, rather than trying to
# observe the transforms on the wire.
#
# Two earlier attempts were worse than no check. `tf2_ros tf2_echo` has to
# start a node and wait for latched messages, and reported "frame does not
# exist" for transforms that were demonstrably being published. `ros2 topic
# echo --once` returns nothing on a latched topic, with or without
# --qos-durability transient_local. Both produced confident false alarms, which
# is worse than silence: it sends you looking for a problem that is not there.
#
# These processes are the transforms. If they are up, the frames are published.
if pgrep -f "child-frame-id odom" >/dev/null 2>&1 \
   && pgrep -f "child-frame-id hesai_lidar" >/dev/null 2>&1; then
  ok "frames     map -> odom -> base_link -> hesai_lidar (static links)"
else
  bad "frames     a static transform publisher is missing -- Foxglove will show"
  echo "               an empty 3D panel even though points are arriving"
  pgrep -af "child-frame-id" | sed 's/^/               /'
fi

# The moving link is checked separately and loudly, because its absence is the
# failure that looks like success: you still get a point cloud, it is just
# drawn in one place forever. Nothing about the picture says why.
if pgrep -f "odom_publisher.py" >/dev/null 2>&1; then
  ok "frames     odom -> base_link is MOVING (odom_publisher.py)"
else
  bad "frames     no odom -> base_link publisher"
  echo "               This BREAKS THE TREE, it does not merely freeze it:"
  echo "               map->odom and base_link->hesai_lidar become two"
  echo "               disconnected halves, and a 3D panel set to 'odom' shows"
  echo "               NOTHING AT ALL rather than a smear."
  echo "               To see the cloud meanwhile, set the display frame to"
  echo "               'hesai_lidar' instead of 'odom'."
  sed -n '1,4p' /tmp/odom_publisher.log 2>/dev/null | sed 's/^/               /'
fi

echo
echo "  open       http://$JETSON:8090          dashboard"
echo "             http://$JETSON:8080          cameras only"
echo "             Foxglove -> ws://localhost:8765"
echo "               3D panel, display frame 'odom', topic /lidar_points"
echo "               Decay Time 0 to start (decisions.md D2). Now that"
echo "               odom -> base_link moves, a small decay -- half a second --"
echo "               is what makes the map build up on screen. Raise it slowly;"
echo "               5 s at this point rate is what killed rviz2."
echo
echo "             Layouts -> Import from file: E:\\Unitree\\notes\\foxglove\\go2_sensors.json"
echo "               LiDAR and depth together, depth image, rates and delays"
echo "  depth      dashboard, Sensors panel -> Start (the laptop side is already up)"
echo
echo "  optional   python3 $HERE/camera_bridge.py          dock cameras into Foxglove"
echo "             python3 $HERE/camera_bridge.py --head   + the robot's own head camera"
echo "                                                        (multicast H.264; may not survive WSL)"
echo "  stop all   $HERE/go2-stop.sh"
