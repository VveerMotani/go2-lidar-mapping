# What to do next: ideas, ranked (2026-09-25)

Written the day before the robot came back, from a research pass over vendor
docs and open-source projects, checked against what this repo already does.
Labels: **[vendor]** Unitree, Hesai or Intel documentation; **[OSS]** an
open-source project or paper; **[ours]** measured on this robot or already in
this repo; **[reasoning]** not checked against a source. **(?)** means unsure.

Already done, so not on this list: per-point deskew (`lidar_map.py`), height
from odometry with automatic "free" on slopes (`--vertical auto`), RGB and
depth recorded with every walk (`odom-test-2`: 4,012 depth and 4,011 colour
frames), the camera-mount fitter (`depth_mount.py`), the dense cleaned cloud
(`dense_map.py`).

---

## Tomorrow (robot day), in this order

| # | What | Why now | Time | Risk |
| --- | --- | --- | --- | --- |
| 1 | **The handover test.** Install autostart on the robot, then from a *second* laptop with only a browser: set the clock, record a 1-minute walk, download it, process it with Docker, open it in Foxglove. | This is what the professor will do. It has never been done end to end on the robot. | 45 min | Low |
| 2 | **Hardware facts.** Jetson model (`cat /proc/device-tree/model`), RealSense model (`rs-enumerate-devices -s`), firmware version (Unitree app), and whether `rt/wirelesscontroller` is published. | Decides what the robot can run (Orin Nano or NX), whether the camera has an IMU, and whether the remote can start a walk. | 15 min | None |
| 3 | **Measure the camera** with a tape (four numbers from the LiDAR), then fit it on `odom-test-2`, which already has depth. | Unblocks every depth-on-the-map idea. The fit is offline; only the tape needs the robot. | 20 min | Low |
| 4 | **A loop-closure walk.** A figure-8 that passes the start two or three times, near structure. Optionally 4 to 6 printed AprilTags (tag36h11) at taped positions. | Gives a benchmark to compare SLAM back-ends against, and a real accuracy number for the map. | 20 min | Low |
| 5 | **Two floors as two walks.** Floor A ending on the stair landing; move the robot (carry it, or the lift); floor B starting from the same landing. | Multi-floor data without risking the robot on stairs. The landing is the overlap that lets the two maps be joined offline. | 30 min | Low |
| 6 | **A cautious stair test**: one to three shallow steps, spotter beside it, obstacle avoidance off, LiDAR cable strain-relieved. | Tells us whether stairs are viable at all with the XT16 on top. | 20 min | **High**: a fall lands on the LiDAR |
| 7 | **Sport API smoke test** over the cable: stand up, `Move(0.2, 0, 0)` for 2 s, `StopMove`. Spotter holds the remote. | The first step of any autonomy. Confirms commands, timeouts and that the remote overrides. | 30 min | Medium |

Stairs [vendor, Go2 manual]: *RIGHT (long) + START* is stair mode 1 (up
forwards, down backwards), *LEFT (long) + START* is mode 2 (down forwards).
Newer firmware folded this into "A.I. mode" in the app, and a robot in
obstacle-avoidance mode stops in front of stairs (?). Resellers quote a 16 cm
maximum step; building stairs are 15 to 18 cm, so we are at the limit.

---

## Next 2 to 4 weeks, mostly on the laptop

**1. Loop closure and a SLAM comparison.** Our map closes to 0.26 m on a
315 s walk but has no loop closure [ours]. Try, in order, on the retimed
`.mcap` (not the pcap):

| Tool | Needs an IMU? | Why try it |
| --- | --- | --- |
| KISS-SLAM (`pip install kiss-slam`) | No | KISS-ICP + loop closure, ten minutes to a baseline [OSS] |
| Our pipeline + MapClosures + an Open3D pose graph | No | Keeps the leg-odometry seed nobody else uses [reasoning] |
| GLIM, LiDAR-only config | No | Best quality ceiling, merges sessions (floors!) [OSS] |
| RTAB-Map `icp_odometry` | No | Multi-session is built in; can add camera loop closures [OSS] |
| FAST-LIO2 / Leg-KILO | **Yes, same clock** | Only after the IMU and LiDAR clocks agree [OSS] |

The IMU problem: the XT16 has none, and `rt/lowstate` is stamped on arrival
while the LiDAR has its own clock. `retime_bag.py` already maps the LiDAR onto
the robot's clock to 0.03 ms [ours], which may be enough; the stricter fix is
PTP on the XT16 (the manual lists IEEE 1588v2) [vendor].

**2. Depth on the map, properly.** Once the mount is fitted: integrate depth
frames posed by the LiDAR track into an Open3D TSDF volume (depth only out to
about 3 m, where it beats the XT16's 2-degree gaps), then **colour the LiDAR
cloud from RGB**, using depth as a z-buffer so hidden points are not painted
[reasoning]. NVIDIA's nvblox is the GPU route if Open3D is too slow [OSS].

**3. Multi-floor maps.** One walk per floor, joined at the shared landing
(ICP between the two maps around the landing, or GLIM's session merge). Floors
separate cleanly afterwards by a histogram of floor heights [reasoning]. A
floor plan per level: slice at 1 to 1.5 m, occupancy grid, SVG.

**4. Object detection, offline first.** YOLO on the recorded RGB on the
laptop's RTX 4070, boxes projected through depth and the track into the 3D map:
a map with "chair", "door", "extinguisher" on it. On the robot later, if the
Jetson is an Orin NX (Ultralytics support JetPack 5 with cp38 wheels, but
installing them needs an offline transfer) [OSS].

**5. Wireless.** The robot's own Wi-Fi is for the app and WebRTC; nothing
shows it reaching the 192.168.123 network (?). The simplest real fix is a small
**travel router on the robot** (e.g. GL.iNet), cabled into the dock and given
an address on 192.168.123.x: the laptop joins its Wi-Fi and sees the dashboard,
the Jetson and the main board exactly as over the cable [reasoning; one
reseller's Go2 setup already puts a router at .100]. Keep recording on the
robot; lossless depth is ~74 Mbit/s. Needs buying, and a power source on the
dock (?).

**6. Recording without the cable, sooner.** Start and stop a walk from a
button combination on the Unitree remote (`rt/wirelesscontroller`, read on the
Jetson), with a beep or the head light as confirmation. Needs a combination
that the robot does not already use [vendor topic; reasoning].

**7. First autonomy.** In this order: an obstacle stop (XT16 points in a box
ahead of the robot trigger `StopMove`); go to a clicked point
(`ObstaclesAvoidClient.MoveToAbsolutePosition`, frame to be tested (?));
replay a recorded route; person following with YOLO and depth. Speed capped at
0.3 m/s, spotter on the remote. Avoid the soft e-stop (`L2+B`) unless it is a
real emergency: it drops the robot onto the LiDAR [vendor manual].

---

## Bigger projects

- **A building's digital twin**: every floor, merged, with a floor plan per
  level, re-walked weekly and compared (change detection by cloud-to-cloud
  distance).
- **A published dataset**: legged robot + XT16 + 500 Hz IMU + leg odometry +
  RGB-D, with a DOI. Legged datasets of this kind are rare (legkilo-dataset is
  the nearest precedent) [OSS].
- **Real to sim**: the TSDF mesh as a MuJoCo or Isaac Sim scene; the Go2 model
  and RL code are already in `~/unitree_ws/sim` [ours].
- **Semantic navigation**: "walk to the fire extinguisher", on top of 4 and 7.

## Sources

Unitree: [unitree_sdk2 sport client](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp),
[obstacles_avoid_client](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp),
[Go2 user manual](https://static.generation-robots.com/media/Go2-User-Manual.pdf).
SLAM: [KISS-SLAM](https://github.com/PRBonn/kiss-slam),
[MapClosures](https://github.com/PRBonn/MapClosures),
[GLIM](https://koide3.github.io/glim/),
[RTAB-Map multi-session](https://github.com/introlab/rtabmap/wiki/Multi-session),
[FAST-LIO ROS2](https://github.com/hku-mars/FAST_LIO/tree/ROS2),
[Leg-KILO](https://github.com/ouguangjun/Leg-KILO).
Depth: [nvblox](https://github.com/nvidia-isaac/nvblox),
[direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration).
Community: [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk),
[SLAM_navigation_go2](https://github.com/HenryDong-AI/SLAM_navigation_go2),
[MYBOTSHOP stair thread](https://forum.mybotshop.de/t/unitree-go2-no-more-stair-climbing-mode-after-update/1459).
