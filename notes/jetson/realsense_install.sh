#!/usr/bin/env bash
# Install Intel's RealSense driver on the robot's onboard computer.
#
#     bash ~/go2dev/realsense_install.sh
#
# Run on the robot (192.168.123.18) over `ssh -t`, so sudo can ask for the
# password. Needs internet on the robot: plug a phone into the robot's
# USB and turn on USB tethering first.
#
# WHAT IT DOES
# ------------
# Exactly Intel's documented Debian install for Jetson, nothing more:
#   doc/installation_jetson.md and doc/distribution_linux.md in librealsense.
#   1. registers Intel's package signing key
#   2. adds Intel's package repository for this Ubuntu release
#   3. installs librealsense2-utils and librealsense2-dev
#
# Intel's Jetson guide installs these two packages only. librealsense2-dkms, the
# kernel module from the desktop guide, is deliberately NOT installed: it does
# not apply to Jetson kernels.
#
# The package list is refreshed for Intel's repository alone. If the robot's
# other repositories (ROS, for one) have expired signing keys, a normal
# `apt-get update` fails on them and stops before Intel's packages are seen.
# Refreshing only Intel's avoids that without touching the others.
#
# Deliberately not `set -u`.
set -o pipefail

ok()   { printf "  \033[32m/\033[0m %s\n" "$1"; }
bad()  { printf "  \033[31mX\033[0m %s\n" "$1"; }
step() { printf "\n\033[1m%s\033[0m\n" "$1"; }

KEY_URL="https://librealsense.realsenseai.com/Debian/librealsenseai.asc"
KEYRING="/etc/apt/keyrings/librealsenseai.gpg"
LIST="/etc/apt/sources.list.d/librealsense.list"

step "1. checking this computer is ready"
if [ "$(date +%Y)" -lt 2024 ]; then
  bad "the clock reads $(date). Downloads fail certificate checks with a wrong clock."
  echo "    Run go2-start.sh from the laptop, which sets it, then run this again."
  exit 1
fi
ok "clock $(date +%F)"

REL=$(head -1 /etc/nv_tegra_release 2>/dev/null | sed -n 's/^# R\([0-9]*\).*REVISION: \([0-9.]*\).*/\1.\2/p')
MAJOR=${REL%%.*}
if [ -z "$REL" ]; then
  bad "could not read /etc/nv_tegra_release. Is this the Jetson?"
  exit 1
elif [ "$MAJOR" -lt 35 ]; then
  bad "L4T R$REL. Intel's prebuilt Jetson packages need JetPack 5.0.2 (L4T R35.1) or newer."
  echo "    Stopping rather than installing packages that will not work."
  exit 1
fi
ok "L4T R$REL (JetPack 5 or newer)"

if ! timeout 10 curl -sSf "$KEY_URL" -o /tmp/librealsenseai.asc; then
  bad "no internet. Plug a phone into the robot's USB, turn on USB tethering, and try again."
  ip -br addr | grep -E "^(usb|enx|rndis)" | sed 's/^/    /'
  exit 1
fi
ok "Intel's package server answers"

sudo -v || { bad "sudo needs the password"; exit 1; }

step "2. adding Intel's repository"
sudo mkdir -p /etc/apt/keyrings
gpg --dearmor < /tmp/librealsenseai.asc | sudo tee "$KEYRING" > /dev/null
ok "signing key $KEYRING"
echo "deb [signed-by=$KEYRING] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee "$LIST" > /dev/null
ok "repository $LIST ($(lsb_release -cs))"

sudo apt-get update \
  -o Dir::Etc::sourcelist="sources.list.d/librealsense.list" \
  -o Dir::Etc::sourceparts="-" \
  -o APT::Get::List-Cleanup="0" || { bad "could not read Intel's repository"; exit 1; }
ok "package list refreshed"

step "3. installing"
sudo apt-get install -y librealsense2-utils librealsense2-dev || { bad "install failed; the lines above say why"; exit 1; }
ok "librealsense2-utils and librealsense2-dev"

step "4. checking the camera"
if rs-enumerate-devices -s 2>/dev/null | grep -q -i "intel"; then
  rs-enumerate-devices -s | sed 's/^/  /'
  ok "the driver sees the camera"
else
  bad "the driver is installed but sees no camera yet."
  echo "    Intel's guide says to reconnect the camera after installing. Unplug"
  echo "    and replug its USB cable, or restart the robot, then run:"
  echo "      rs-enumerate-devices -s"
fi
echo
