#!/usr/bin/env bash
# The image's stand-in for the laptop's ~/unitree_ws/setup.env.sh. That one
# also picks the robot-facing network interface and points CycloneDDS at it;
# nothing in this image talks to the robot, and walk_to_bag.sh pins DDS to
# loopback itself.
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
