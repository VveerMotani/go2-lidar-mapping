# Go2 walk processor: everything needed to turn a recorded walk into a
# Foxglove file and a point cloud, on any machine that runs Docker.
#
#   docker build -t go2-walks -f docker/Dockerfile .        (from the repo root)
#   docker run --rm -v <folder with walks>:/walks go2-walks process <walk>
#
# It is the laptop's WSL distro cut down to the offline half: ROS 2 Humble,
# Hesai's driver at the exact version we use, and the Python the map builders
# need. None of the robot-facing parts (CycloneDDS on the robot's network,
# unitree_sdk2, the Hesai live connection) are here. Processing runs on
# loopback and never touches a network.
#
# Versions are pinned to what the WSL distro runs, so the image gives the same
# maps: Hesai ROS driver 2.0.12, Open3D 0.19.0, NumPy 1.26.4, SciPy 1.8 (apt).

FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-rmw-cyclonedds-cpp \
        python3-pip python3-scipy \
        libyaml-cpp-dev libboost-thread-dev libssl-dev libpcap-dev \
        libgl1 libgomp1 \
        git procps tcpdump sudo unzip \
    && rm -rf /var/lib/apt/lists/*

# numpy first and pinned: open3d would otherwise pull in numpy 2, which the
# apt SciPy and ROS's Python bindings are not built for.
RUN pip3 install --no-cache-dir numpy==1.26.4 \
    && pip3 install --no-cache-dir open3d==0.19.0 opencv-python-headless==4.11.0.86 numpy==1.26.4

# Hesai's driver, at the commit the laptop builds (release 2.0.12). Decoding is
# the vendor's job: see "DECODING IS THE VENDOR'S JOB" in notes/walk_to_bag.sh.
ARG HESAI_REF=e7e112f0809f0eed5e3c81c55a1a0376474db234
# In /opt, not /root: go2-walks.sh runs the container as the calling user on
# Linux, so the results are not owned by root, and that user cannot read /root.
WORKDIR /opt/unitree_ws
RUN git clone https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git src/HesaiLidar_ROS_2.0 \
    && cd src/HesaiLidar_ROS_2.0 \
    && git checkout "$HESAI_REF" \
    && git submodule update --init --recursive \
    && rm -rf .git
COPY docker/hesai_config.yaml src/HesaiLidar_ROS_2.0/config/config.yaml
RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# The scripts, and this robot's calibrations (per-unit LiDAR angles, the LiDAR's
# fitted mount, the camera's if it has been measured). A walk folder, or the
# folder of walks beside it, can carry its own copies, which win.
COPY docker/setup.env.sh /opt/unitree_ws/setup.env.sh
COPY notes/ /opt/go2/notes/
COPY walks/xt16_correction.csv walks/*.json /opt/go2/config/
COPY docker/go2 /usr/local/bin/go2
RUN chmod +x /usr/local/bin/go2 /opt/go2/notes/*.sh

ENV GO2_CONFIG_DIR=/opt/go2/config \
    GO2_WS=/opt/unitree_ws \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1

WORKDIR /walks
ENTRYPOINT ["/usr/local/bin/go2"]
CMD ["help"]
