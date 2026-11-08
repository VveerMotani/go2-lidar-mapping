#!/usr/bin/env bash
# Stop everything go2-start.sh started.
#
#   ~/go2-lidar-mapping/notes/go2-stop.sh            stop the laptop side only
#   ~/go2-lidar-mapping/notes/go2-stop.sh --all      stop the robot side too
#
# The laptop side is what matters for the memory leak, so that is the default.
# Leaving the robot's own services running costs nothing and means the dashboard
# is still there next time you connect.
#
# Afterwards, from PowerShell:   wsl --shutdown

set -u

JETSON=192.168.123.18
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
SSH_OPTS="$SSH_OPTS -o ConnectTimeout=8 -o LogLevel=ERROR"

echo "stopping the laptop side"
for p in camera_bridge.py depth_client.py realsense_client.py sensor_monitor.py \
         point_cloud_xyz_node imu_publisher.py odom_publisher.py \
         hesai_ros_driver foxglove_bridge static_transform_publisher rviz2; do
  if pgrep -f "$p" >/dev/null 2>&1; then
    pkill -f "$p" 2>/dev/null
    echo "  stopped  $p"
  fi
done

if [ "${1:-}" = "--all" ]; then
  echo "stopping the robot side"
  sshpass -p "$GO2_PASSWORD" ssh $SSH_OPTS unitree@$JETSON \
    "pkill -f camera_server.py; pkill -f lidar_relay.py; pkill -f dashboard.py; \
     bash ~/go2dev/realsense_start.sh stop >/dev/null 2>&1; \
     pkill -f record_session.py; echo '  robot services stopped'" 2>/dev/null
fi

echo
echo "now run  wsl --shutdown  from PowerShell -- that is what stops the"
echo "Windows memory leak accruing."
