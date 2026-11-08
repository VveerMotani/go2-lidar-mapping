#!/usr/bin/env bash
# Health check: prove the toolchain is sane, and (optionally) that the dog answers.
WS="$HOME/unitree_ws"
pass(){ printf "  \033[32mOK\033[0m   %s\n" "$1"; }
fail(){ printf "  \033[31mFAIL\033[0m %s\n" "$1"; }
warn(){ printf "  \033[33m--\033[0m   %s\n" "$1"; }

echo "== toolchain =="
command -v colcon >/dev/null && pass "colcon" || fail "colcon"
[ -f /opt/ros/humble/setup.bash ] && pass "ROS 2 Humble" || fail "ROS 2 Humble"
[ -d "$WS/sdk/cyclonedds/install" ] && pass "CycloneDDS 0.10.x built" || fail "CycloneDDS"
[ -d "$WS/sdk/unitree_sdk2/install" ] && pass "unitree_sdk2 (C++)" || fail "unitree_sdk2"
python3 -c "import unitree_sdk2py" 2>/dev/null && pass "unitree_sdk2py imports" || fail "unitree_sdk2py"
python3 -c "import cyclonedds, importlib.metadata as m; print('   cyclonedds', m.version('cyclonedds'))" 2>/dev/null || warn "cyclonedds python binding"
[ -d "$WS/src/unitree_ros2/cyclonedds_ws/install" ] && pass "unitree_ros2 built" || fail "unitree_ros2"
[ -d "$WS/src/go2_ros2_sdk" ] && pass "go2_ros2_sdk cloned" || warn "go2_ros2_sdk"
[ -d "$WS/sim/unitree_mujoco" ] && pass "unitree_mujoco" || warn "unitree_mujoco"
[ -d "$WS/venv-rl" ] && pass "RL venv" || warn "RL venv"

echo
echo "== ROS 2 sanity =="
source "$WS/setup.env.sh" >/dev/null 2>&1
timeout 10 ros2 topic list >/dev/null 2>&1 && pass "ros2 daemon + DDS local discovery" || fail "ros2 topic list"

echo
echo "== robot =="
if ping -c 2 -W 2 "${GO2_IP:-192.168.123.161}" >/dev/null 2>&1; then
  pass "robot reachable at ${GO2_IP:-192.168.123.161}"
  timeout 8 ros2 topic list 2>/dev/null | grep -q lowstate && pass "lowstate topic visible" || warn "no /lowstate yet (is the bridge running?)"
else
  warn "robot not reachable -- not connected, or laptop not on 192.168.123.0/24"
fi
