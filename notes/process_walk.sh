#!/usr/bin/env bash
# Everything a walk needs, in one run: the bag, then the scan-matched map.
#
#   ~/go2-lidar-mapping/notes/process_walk.sh ~/go2-lidar-mapping/walks/odom-test-1
#   ~/go2-lidar-mapping/notes/process_walk.sh <session> --map-only
#   ~/go2-lidar-mapping/notes/process_walk.sh <session> --colour
#
# The dashboard's Walk library "Process" button runs this.
#
#   1-6  walk_to_bag.sh   <session>/bag/bag_0.mcap
#   7    lidar_map.py     <session>/map3d/<name>_map.ply        (CloudCompare)
#   8    dense_map.py     <session>/map3d/<name>_dense.ply      every scan, cleaned
#                         <session>/map3d/<name>_map/*.mcap     (Foxglove)
#                         <session>/map3d/trajectory.csv
#                         <session>/map3d/<name>_colour.ply     camera colour (colour_map.py),
#                                                               only with --colour
#   9    floor_plan.py    <session>/map3d/<name>_plan_L<n>.png  a 2D plan per floor (+ .pgm/.yaml)
#   10   mesh_map.py      <session>/map3d/<name>_mesh.ply       triangle mesh, coloured
#
# --map-only skips the bag, for rebuilding the map from a bag already made.
# --colour also colours the clean cloud from the camera's colour frames. An
# experiment, off by default since 2026-09-26: 5-10 minutes more per walk
# and the result is not good enough yet (docs: why the colour looks wrong).
#
# Before 2026-09-17 the second step was export_ply.py, which placed every scan
# by leg odometry alone and gave a smeared cloud. lidar_map.py aligns each scan
# to the map built so far instead.
#
# Deliberately NOT `set -u`: ROS's setup scripts reference unset variables.

SESSION=""
MAP_ONLY=0
COLOUR=--no-colour
for a in "$@"; do
  case "$a" in
    --map-only) MAP_ONLY=1 ;;
    --colour) COLOUR= ;;
    *) SESSION="$a" ;;
  esac
done
if [ -z "$SESSION" ] || [ ! -d "$SESSION" ]; then
  echo "usage: process_walk.sh <session-directory> [--map-only] [--colour]"
  exit 1
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$MAP_ONLY" = 0 ]; then
  bash "$HERE/walk_to_bag.sh" "$SESSION" || exit $?
fi

printf "\n\033[1m%s\033[0m\n" "7. building the scan-matched map"
if [ -z "$(find "$SESSION/bag" -name "*.mcap" 2>/dev/null | head -1)" ]; then
  printf "  \033[31mX\033[0m %s\n" "no bag in $SESSION/bag -- process the walk first"
  exit 1
fi
if [ ! -f "$SESSION/odom.csv" ]; then
  printf "  \033[31mX\033[0m %s\n" "no odom.csv: the map needs the robot's odometry as a starting guess"
  exit 1
fi
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash >/dev/null 2>&1
PYTHONUNBUFFERED=1 python3 -W ignore "$HERE/lidar_map.py" "$SESSION"
STATUS=$?

# The dense, cleaned cloud from every scan, using the track just built.
if [ "$STATUS" = 0 ]; then
  printf "\n\033[1m%s\033[0m\n" "8. building the dense, cleaned point cloud"
  PYTHONUNBUFFERED=1 python3 -W ignore "$HERE/dense_map.py" "$SESSION" $COLOUR
  STATUS=$?
fi

# Floor plans and the mesh, from the dense cloud. Extras: if one fails the
# map above is still good, so it warns and the run still succeeds.
if [ "$STATUS" = 0 ]; then
  printf "
[1m%s[0m
" "9. drawing the floor plans"
  PYTHONUNBUFFERED=1 python3 -W ignore "$HERE/floor_plan.py" "$SESSION"     || printf "  [33m![0m %s
" "floor plans failed; the maps above are unaffected"
  printf "
[1m%s[0m
" "10. building the mesh"
  PYTHONUNBUFFERED=1 python3 -W ignore "$HERE/mesh_map.py" "$SESSION"     || printf "  [33m![0m %s
" "mesh failed; the maps above are unaffected"
fi

# Depth recorded, but the camera's place on the robot not known yet: say what
# to do, since the map above has the depth only as images.
if [ "$STATUS" = 0 ] && [ -f "$SESSION/depth.stream" ] \
    && [ ! -f "$SESSION/camera_mount.json" ] && [ ! -f "$(dirname "$SESSION")/camera_mount.json" ]; then
  echo
  echo "  This walk has depth, but the camera's position on the robot is not known yet."
  echo "  Measure it (Launchpad: Measure where the camera is), then:"
  echo "    python3 $HERE/depth_mount.py $SESSION --from-lidar DX,DY,DZ,TILT"
  echo "    $HERE/process_walk.sh $SESSION --map-only"
fi
exit $STATUS
