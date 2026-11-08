#!/usr/bin/env bash
# Start, stop or check the depth camera on the robot.
#
#     bash ~/go2dev/realsense_start.sh            start camera and relay
#     bash ~/go2dev/realsense_start.sh --color    start with colour too
#     bash ~/go2dev/realsense_start.sh stop       stop both, give the camera back
#     bash ~/go2dev/realsense_start.sh stop --cameras
#                                                 ...and restart camera_server.py
#     bash ~/go2dev/realsense_start.sh status
#
# Runs Intel's ROS 1 camera node (library 2.50.0, preinstalled) with the only
# settings proven to stream depth on this robot, and realsense_ros1_relay.py,
# which forwards depth and the factory calibration to the laptop on port 8093.
# On the laptop: notes/realsense_client.py.
#
# PROVEN SETTINGS, 2026-09-17
#   initial_reset, infrared off, IMU off, depth 640x480 at 15.
#   With the node's defaults (both infrared cameras at 90 fps, depth 848x480 at
#   30) colour streamed but not a single depth frame arrived.
#
# The camera server (camera_server.py, the dashboard's video) holds the
# camera's video devices and has to be stopped first. `stop` does not restart
# it unless given --cameras (the dashboard's Sensors panel passes it).
#
# Deliberately not `set -u`: ROS's setup scripts reference unset variables.

cd "$HOME/go2dev" || exit 1
COLOR=0
CMD=start
CAMERAS=0
# Depth mode as WIDTHxHEIGHTxFPS. 640x480x15 is the one proven on 2026-09-17;
# the D435i also offers 848x480 (Intel's recommended depth resolution for
# accuracy) and 1280x720.
DEPTH=640x480x15
for a in "$@"; do
  case "$a" in
    --color) COLOR=1 ;;
    --cameras) CAMERAS=1 ;;
    --depth=*) DEPTH="${a#--depth=}" ;;
    stop|status|start) CMD="$a" ;;
  esac
done
IFS=x read -r DW DH DF <<< "$DEPTH"

# The [x] in each pattern stops pgrep and pkill matching their own command line.
running() { pgrep -af "realsense2_camer[a]/RealSenseNodeFactory|realsense_ros1_rela[y].py|rosmaste[r]"; }

if [ "$CMD" = status ]; then
  running || echo "not running"
  exit 0
fi

if [ "$CMD" = stop ]; then
  pkill -f "realsense_ros1_rela[y].py"
  pkill -f "roslaunch realsense2_camer[a]"
  sleep 3
  pkill -f "rosmaste[r]"
  running || echo "stopped"
  if [ "$CAMERAS" = 1 ] && [ -f "$HOME/go2dev/camera_server.py" ]       && ! pgrep -f "camera_serve[r].py" >/dev/null; then
    sleep 2
    PYTHONUNBUFFERED=1 setsid nohup python3 "$HOME/go2dev/camera_server.py"       > /tmp/camera_server.log 2>&1 < /dev/null &
    sleep 2
    if pgrep -f "camera_serve[r].py" >/dev/null; then
      echo "camera server back on (Cameras panel)"
    else
      echo "camera server did not start; /tmp/camera_server.log:"
      tail -n 5 /tmp/camera_server.log
    fi
  fi
  exit 0
fi

source /opt/ros/noetic/setup.bash

pkill -f "camera_serve[r].py" && echo "stopped the dashboard's camera server (it holds the camera)"
pkill -f "realsense_ros1_rela[y].py"
pkill -f "roslaunch realsense2_camer[a]"
sleep 2

if [ "$COLOR" = 1 ]; then
  COLOR_ARGS="enable_color:=true color_width:=640 color_height:=480 color_fps:=15"
  RELAY_ARGS="--color"
else
  COLOR_ARGS="enable_color:=false"
  RELAY_ARGS=""
fi

setsid nohup roslaunch realsense2_camera rs_camera.launch \
  initial_reset:=true $COLOR_ARGS \
  enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
  depth_width:=$DW depth_height:=$DH depth_fps:=$DF \
  > /tmp/realsense_camera.log 2>&1 < /dev/null &
echo "camera starting at ${DW}x${DH} ${DF} fps (about 20 s, it resets the camera first)"

for _ in $(seq 1 40); do
  grep -q "RealSense Node Is Up" /tmp/realsense_camera.log 2>/dev/null && break
  sleep 1
done
if grep -q "RealSense Node Is Up" /tmp/realsense_camera.log 2>/dev/null; then
  echo "camera up"
else
  echo "camera did not come up; last lines of /tmp/realsense_camera.log:"
  tail -n 8 /tmp/realsense_camera.log
  exit 1
fi

setsid nohup python3 "$HOME/go2dev/realsense_ros1_relay.py" $RELAY_ARGS \
  > /tmp/realsense_relay.log 2>&1 < /dev/null &
sleep 3
if pgrep -f "realsense_ros1_rela[y].py" >/dev/null; then
  echo "relay up on port 8093"
else
  echo "relay failed; /tmp/realsense_relay.log:"
  tail -n 8 /tmp/realsense_relay.log
  exit 1
fi

rate=$(timeout 8 rostopic hz /camera/depth/image_rect_raw 2>&1 | grep "average rate" | tail -1)
echo "depth on the robot: ${rate:-no frames yet}"
