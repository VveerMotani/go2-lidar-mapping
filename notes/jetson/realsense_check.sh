#!/usr/bin/env bash
# What does this robot computer have, for the depth camera and for 4G?
#
#     bash ~/go2dev/realsense_check.sh
#
# Run on the robot's onboard computer (192.168.123.18). Read-only: it lists and
# reports, and changes nothing. Safe to run as often as you like.
#
# Deliberately not `set -e`: several of these tools may be missing, and a check
# that stops at the first missing one tells you nothing.

hr() { printf "\n\033[1m%s\033[0m\n" "$1"; }

hr "Robot computer"
head -1 /etc/nv_tegra_release 2>/dev/null || echo "  not a Jetson?"
echo "  $(lsb_release -ds 2>/dev/null)  $(uname -m)"
# Intel's prebuilt Jetson packages need JetPack 5.0.2 or newer, which is
# L4T R35.1. The first line above says R35, R36 ... or older.
echo "  clock: $(date)"
[ "$(date +%Y)" -lt 2024 ] && echo "  !! clock is not set. Run go2-start.sh from the laptop first: downloads fail with a wrong clock."

hr "Camera on USB"
cam=$(lsusb | grep -i -E "8086:|intel corp.*realsense|realsense")
if [ -n "$cam" ]; then
  echo "$cam" | sed 's/^/  /'
else
  echo "  no Intel RealSense found on USB"
  echo "  everything on USB:"; lsusb | sed 's/^/    /'
fi
# A RealSense needs USB 3 for depth. 5000M is USB 3; 480M is USB 2, which
# works only at low resolution and often drops depth altogether.
echo "  USB speed of video devices (5000M = USB 3, 480M = USB 2):"
lsusb -t 2>/dev/null | grep -i uvcvideo | sed 's/^/    /' || echo "    none"

hr "Intel RealSense driver"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  echo "  installed: $(dpkg -s librealsense2-utils 2>/dev/null | sed -n 's/^Version: //p')"
  rs-enumerate-devices -s 2>&1 | sed 's/^/  /'
else
  echo "  not installed. Next: realsense_install.sh (needs internet on this computer)."
fi

hr "ROS camera package"
found=0
for d in /opt/ros/*/share/realsense2_camera; do
  [ -d "$d" ] && { echo "  installed for $(echo "$d" | cut -d/ -f4)"; found=1; }
done
[ "$found" = 0 ] && echo "  not installed"
ls -d /opt/ros/*/ 2>/dev/null | sed 's#/opt/ros/#  ROS installed: #; s#/$##'
if [ -f /opt/ros/foxy/lib/librmw_cyclonedds_cpp.so ]; then
  echo "  Foxy can use CycloneDDS, the same as the laptop"
else
  echo "  Foxy has no CycloneDDS plugin (it would use Fast DDS; the laptop uses CycloneDDS)"
fi

hr "Internet on this computer"
if timeout 6 curl -sSfI https://librealsense.realsenseai.com >/dev/null 2>&1; then
  echo "  yes, Intel's package server answers"
else
  echo "  no. For the driver install, plug a phone into the robot's USB and turn on USB tethering."
fi
ip -br addr 2>/dev/null | grep -E "^(usb|enx|wwan|wlan|rndis)" | sed 's/^/  /'

hr "4G modem"
if lsusb | grep -i -q -E "2c7c:|quectel"; then
  lsusb | grep -i -E "2c7c:|quectel" | sed 's/^/  /'
  echo "  visible to this computer"
else
  echo "  not visible to this computer (it may belong to Unitree's own main board)"
fi
ip -br link 2>/dev/null | grep -E "wwan" | sed 's/^/  /'
echo
