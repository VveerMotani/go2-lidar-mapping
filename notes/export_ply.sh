#!/usr/bin/env bash
# Run export_ply.py with ROS on the path.
#
#   ~/go2-lidar-mapping/notes/export_ply.sh ~/go2-lidar-mapping/walks/Lab-loop-1/bag
#   ~/go2-lidar-mapping/notes/export_ply.sh <bag> --voxel 0.05 --max-range 30
#
# WHY A WRAPPER
# -------------
# export_ply.py reads a bag, which needs rosbag2_py and the message packages.
# Those only exist once ROS's setup scripts have been sourced, and a program
# cannot source them for itself -- sourcing changes the shell's environment, and
# by the time Python is running the shell has already been decided.
#
# The dashboard starts the export as a subprocess of a long-running server that
# may itself have been started from a shell with no ROS on it. Without this
# wrapper the export failed instantly with an ImportError, which the progress
# bar reported as a one-line traceback and nothing else.
#
# Deliberately NOT `set -u`: ROS's own setup scripts reference unset variables
# all over the place and exit 127 under it. Same note as in go2-start.sh; do not
# add it back.
set -o pipefail

WS="$HOME/unitree_ws"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# setup.env.sh reads its own $1 as the network interface to use, and a script
# sourced from a script inherits THIS script's arguments -- which here is a bag
# path. Passing an explicit empty argument stops CycloneDDS being told to bind
# to a network interface named ~/go2-lidar-mapping/walks/... , the same trap
# walk_to_bag.sh documents.
unset UNITREE_IFACE
# shellcheck source=/dev/null
[ -f "$WS/setup.env.sh" ] && source "$WS/setup.env.sh" "" >/dev/null 2>&1
# shellcheck source=/dev/null
[ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck source=/dev/null
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" >/dev/null 2>&1

if ! python3 -c "import rosbag2_py" 2>/dev/null; then
  echo "rosbag2_py is still not importable after sourcing ROS."
  echo "  tried: $WS/setup.env.sh, /opt/ros/humble/setup.bash,"
  echo "         $WS/install/setup.bash"
  echo "Reading a bag is not possible without it."
  exit 1
fi

# Unbuffered, so the dashboard's progress bar moves while this runs rather than
# arriving in one lump when the pipe is flushed at exit.
exec python3 -u "$HERE/export_ply.py" "$@"
