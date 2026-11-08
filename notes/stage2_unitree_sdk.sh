#!/usr/bin/env bash
# Stage 2: CycloneDDS 0.10.2 + Unitree SDK2 (C++ & Python) + ROS 2 bridges
set -euo pipefail
WS="$HOME/unitree_ws"
mkdir -p "$WS/sdk" "$WS/src" "$WS/sim"

# ---------------------------------------------------------------- CycloneDDS
echo "### CycloneDDS 0.10.x (standalone, for unitree_sdk2 + python bindings)"
cd "$WS/sdk"
[ -d cyclonedds ] || git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="$WS/sdk/cyclonedds/install" -DBUILD_EXAMPLES=OFF
cmake --build . --target install -j"$(nproc)"
export CYCLONEDDS_HOME="$WS/sdk/cyclonedds/install"

# ------------------------------------------------------------- unitree_sdk2 C++
echo "### unitree_sdk2 (C++)"
cd "$WS/sdk"
[ -d unitree_sdk2 ] || git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="$WS/sdk/unitree_sdk2/install"
make -j"$(nproc)"
make install

# ---------------------------------------------------------- unitree_sdk2_python
echo "### unitree_sdk2_python"
cd "$WS/sdk"
[ -d unitree_sdk2_python ] || git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
python3 -m pip install --upgrade pip setuptools wheel
CYCLONEDDS_HOME="$WS/sdk/cyclonedds/install" python3 -m pip install --user -e .
python3 -m pip install --user numpy opencv-python

# ------------------------------------------------------------- unitree_ros2
echo "### unitree_ros2 (official DDS <-> ROS2 bridge)"
cd "$WS/src"
[ -d unitree_ros2 ] || git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2/cyclonedds_ws/src
[ -d rmw_cyclonedds ] || git clone https://github.com/ros2/rmw_cyclonedds -b humble
[ -d cyclonedds ] || git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
# NOTE: cyclonedds must be built WITHOUT ROS sourced (per Unitree README)
colcon build --packages-select cyclonedds
set +u; source /opt/ros/humble/setup.bash; set -u
colcon build

# ------------------------------------------------------------- unitree_ros (URDF)
echo "### unitree_ros (URDF + meshes)"
cd "$WS/src"
[ -d unitree_ros ] || git clone https://github.com/unitreerobotics/unitree_ros.git

# ------------------------------------------------------------- go2_ros2_sdk
echo "### go2_ros2_sdk (community, WebRTC + DDS)"
cd "$WS/src"
[ -d go2_ros2_sdk ] || git clone --recurse-submodules https://github.com/abizovnuralem/go2_ros2_sdk.git
cd go2_ros2_sdk
python3 -m pip install --user -r requirements.txt || true

echo "### STAGE 2 COMPLETE"
