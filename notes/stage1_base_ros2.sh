#!/usr/bin/env bash
# Stage 1: base dev tools + ROS 2 Humble
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "### apt update/upgrade"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "### base dev tools"
sudo apt-get install -y \
  build-essential cmake ninja-build git curl wget unzip zip \
  python3-pip python3-venv python3-dev python3-setuptools \
  libeigen3-dev libboost-all-dev libyaml-cpp-dev pkg-config \
  net-tools iputils-ping iproute2 iperf3 tcpdump \
  usbutils v4l-utils ffmpeg libssl-dev libusb-1.0-0-dev \
  software-properties-common lsb-release gnupg2 ca-certificates \
  htop tree jq vim

echo "### universe repo"
sudo add-apt-repository -y universe

echo "### ROS 2 apt source"
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
echo "ros-apt-source version: ${ROS_APT_SOURCE_VERSION}"
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo apt-get install -y /tmp/ros2-apt-source.deb
sudo apt-get update -y

echo "### ROS 2 Humble desktop"
sudo apt-get install -y ros-humble-desktop ros-dev-tools

echo "### ROS 2 extras (nav, slam, perception, teleop)"
sudo apt-get install -y \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan ros-humble-rtabmap-ros \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-vision-msgs ros-humble-image-tools ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy ros-humble-joy ros-humble-foxglove-bridge \
  ros-humble-tf-transformations

echo "### rosdep init"
sudo rosdep init 2>/dev/null || true
rosdep update || true

echo "### STAGE 1 COMPLETE"
