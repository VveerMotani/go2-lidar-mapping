#!/usr/bin/env bash
# Unitree Go2 development environment.
# Usage:  source ~/unitree_ws/setup.env.sh   [interface]
# The interface is the one that can reach the robot at 192.168.123.161.
# Under WSL mirrored networking this mirrors the Windows adapter (often eth0/eth2).

WS="$HOME/unitree_ws"

# --- ROS 2 -----------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# --- Unitree ROS 2 bridge (its own CycloneDDS build) -----------------------
if [ -f "$WS/src/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
  source "$WS/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

# --- Standalone CycloneDDS (for unitree_sdk2 / python bindings) ------------
export CYCLONEDDS_HOME="$WS/sdk/cyclonedds/install"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"

# --- Pick the robot-facing interface ---------------------------------------
UNITREE_IFACE="${1:-${UNITREE_IFACE:-}}"
if [ -z "$UNITREE_IFACE" ]; then
  # prefer an interface already on the robot's subnet, else the default route
  UNITREE_IFACE=$(ip -o -4 addr show | awk '/192\.168\.123\./ {print $2; exit}')
  [ -z "$UNITREE_IFACE" ] && UNITREE_IFACE=$(ip route show default | awk '{print $5; exit}')
fi
export UNITREE_IFACE

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
# MaxMessageSize: the default is 14720 B, ~10x the 1500-byte Ethernet MTU, so every
# large sample is IP-fragmented. Only the FIRST fragment carries a UDP port number,
# which is why a port-scoped Hyper-V firewall rule passed discovery but dropped all
# data (reasm_reqds=6345, reasm_oks=0). Capping below the MTU stops us generating
# fragments at all. FragmentSize is already the default; stated here for intent.
# SocketReceiveBufferSize: 1 MiB default drops packets on a 320k point/s stream.
# Needs net.core.rmem_max=16777216 in the distro or CycloneDDS logs an error.
# See docs/why-it-failed.md section 3.2.
# The "lo" (loopback) interface is REQUIRED, not optional. Verified 2026-09-03:
# with only ${UNITREE_IFACE} listed, two ROS nodes on THIS machine cannot discover
# each other at all -- local discovery relies on multicast looping back, and WSL's
# mirrored networking does not loop multicast back to local processes.
#
# That single fault caused every mystery of 2026-09-03: `ros2 topic list` hanging,
# foxglove_bridge deadlocking silently at startup, and rosbridge reporting
# "Service /rosapi/topics_and_raw_types does not exist" while rosapi ran beside it.
# It would also have made `ros2 bag record` produce EMPTY bags.
#
# Robot traffic still goes over ${UNITREE_IFACE}, which is listed first.
# Do not remove the lo entry. Test with talker/listener (see docs/decisions.md D9).
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${UNITREE_IFACE}\" priority=\"default\" multicast=\"default\" /><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"true\" /></Interfaces><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1344B</FragmentSize></General><Internal><SocketReceiveBufferSize min=\"10MB\" /></Internal></Domain></CycloneDDS>"

# --- Socket buffers ---------------------------------------------------------
# CycloneDDS asks the kernel for the SocketReceiveBufferSize above, and the
# kernel silently caps it at net.core.rmem_max. Ours is 16 MB; Unitree set BOTH
# directions to 52428800 (50 MB) on the dock as a prerequisite for their SLAM
# service -- see the SDK docs, SLAM and Navigation Services Interface, step 4.
# Their number is the one to match: the XT16 is a heavier stream than the L1
# this stack was sized for.
#
# Warn rather than change anything. This is a system setting, it needs root,
# and it survives a reboot only if written to /etc/sysctl.conf.
_rmem=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
_wmem=$(sysctl -n net.core.wmem_max 2>/dev/null || echo 0)
if [ "${_rmem:-0}" -lt 52428800 ] || [ "${_wmem:-0}" -lt 52428800 ]; then
  echo "  note: socket buffers are below Unitree's values (rmem ${_rmem}, wmem ${_wmem})."
  echo "        To match the dock, once:"
  echo "          sudo tee -a /etc/sysctl.conf <<'EOF'"
  echo "          net.core.rmem_max=52428800"
  echo "          net.core.wmem_max=52428800"
  echo "          EOF"
  echo "          sudo sysctl -p"
fi
unset _rmem _wmem

# --- Robot constants --------------------------------------------------------
export GO2_IP=192.168.123.161          # robot, wired
export GO2_LOCAL_IP=192.168.123.99     # what your laptop should be, wired

echo "Unitree env ready  |  iface=${UNITREE_IFACE}  rmw=${RMW_IMPLEMENTATION}  robot=${GO2_IP}"

# --- GUI apps: prefer the discrete NVIDIA GPU under WSLg ---
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export LIBGL_ALWAYS_INDIRECT=0
