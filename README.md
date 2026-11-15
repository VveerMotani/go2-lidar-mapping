# Go2 LiDAR mapping

**Walk a Unitree Go2 quadruped through a building and get a clean, measurable 3D map of every floor it visited.**

![Two floors of a university building, mapped in one 5-minute walk. Point cloud coloured by height above each floor; the upper floor is lifted 7 m to show both. White line: the robot's path, including both staircases.](docs/figures/hero-two-floors.png)

*One 5-minute walk through two floors of a university building, including both staircases. 9.55 million points on a 2 cm grid, coloured by height. The white line is the robot's path.*

## Hardware

| | |
| --- | --- |
| Robot | Unitree Go2 EDU with its onboard Jetson |
| LiDAR | Hesai XT16 (16 beams, 360°, 10 Hz) on the robot's back |
| Depth camera | Intel RealSense, 848×480 at 15 fps |
| Also recorded | The Go2's IMU (500 Hz), leg odometry and body height |

## How it works

![Pipeline: the robot records LiDAR, IMU, odometry and depth on one clock; a Docker image decodes, retimes, scan-matches, loop-closes, and builds the dense map, floor plans and mesh.](docs/figures/pipeline.svg)

1. **Record on the robot.** The robot serves a web page. Plug in a laptop, run the pre-flight checks, press Start, walk the robot with its remote, press Stop, download a `.zip`.
2. **Process in Docker.** One command turns the zip into a map: `./go2-walks.sh process walk.zip`. No ROS needed on the laptop.
3. **Look at it.** Foxglove shows the map growing over time; CloudCompare lets you measure the dense cloud in metres. Floor plans and a mesh come out too.

A Windows desktop app does all three steps with buttons, for people who don't want a terminal.

## Results on the two-floor walk

| | |
| --- | --- |
| Walk | 303 s, 130 m, two floors 3.6 m apart, no sensor frames dropped |
| Loop closure | Track returns to 0.16 m of its start; leg odometry alone ends 2.56 m away |
| Map | 9.55 M points on a 2 cm grid after cleaning |
| Mesh | 5.1 M triangles, within 1.3 cm of a real point (median) |

![Leg odometry alone vs scan matching with loop closure. Leg odometry drifts sideways and stays flat while the robot climbs a floor.](docs/figures/odometry-vs-scan-matching.png)

*Why the map can't come from the robot's own odometry: it drifts up to 6 m sideways and never notices the robot climbing a floor.*

## Problems solved along the way

- **One clock for four sensors.** The robot boots thinking it's 1970 and the LiDAR keeps its own clock. Every packet is re-timed onto the robot's clock.
- **Leg odometry drifts.** Each LiDAR scan is aligned to the map so far (ICP scan matching), not placed where the legs say the robot is.
- **16 beams are weak in tilt.** The corridor ceiling came out twice until a pose graph pulled the outbound and return passes together.
- **Stairs.** Height comes from the scans and the depth camera on stairs, where the odometry gets it wrong.
- **Measuring, not eyeballing.** `map_quality.py` compares repeated passes over the same place, so every change is judged by numbers.

The full story of each problem, with measurements, is in [`docs/recording-a-walk.md`](docs/recording-a-walk.md).

## Status and next

**Works today:** recording, processing, multi-floor maps with loop closure, floor plans, meshes and quality checks, on walks up to about 5 minutes.

**Next:** live mapping on the robot, then autonomous exploration: the robot walks into a building it has never seen and comes back with the full map.

## Running it

You need Docker and a recorded walk.

```bash
./go2-walks.sh --build selftest      # build the image once, about 15 minutes
./go2-walks.sh process my-walk.zip
```

On Windows: `.\go2-walks.ps1 -Build selftest`, then `.\go2-walks.ps1 process my-walk.zip`.

| Folder | What's in it |
| --- | --- |
| [`notes/`](notes/) | The pipeline: `retime_bag.py`, `lidar_map.py`, `pose_graph.py`, `dense_map.py`, `map_quality.py` |
| [`notes/jetson/`](notes/jetson/) | What runs on the robot: the recorder and its web page |
| [`docker/`](docker/) | The processing image |
| [`studio/`](studio/) | The Windows desktop app |
| [`docs/`](docs/) | The full workflow, design decisions and next ideas |

## License

MIT. See [LICENSE](LICENSE).
