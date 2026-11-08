# Recording a walk, start to finish

From pressing a button on the robot to scrubbing through the result in
Foxglove. Written 2026-09-04, the first day this was done for real.

Everything lands in `E:\Unitree\walks\`.

---

## 1. Bring everything up

```bash
wsl -d unitree
~/go2-lidar-mapping/notes/go2-start.sh
```

It checks the laptop's address, waits for the robot, syncs the robot's clock,
starts the cameras, LiDAR relay and dashboard on the robot, and the LiDAR
driver, transforms and foxglove_bridge on the laptop. Safe to run twice.

**The clock sync is not optional.** The robot has no battery-backed clock and
boots at 1970. The LiDAR and IMU in a recording are lined up by their
timestamps, so a recording made before the clock is set cannot be used.

**With only a browser** — any laptop, static address `192.168.123.x` on the
cable, no WSL — skip this step. Once `install_autostart.sh` has been run on the
robot (see `dashboard.md`, "Starting at boot"), the services start by
themselves; open `http://192.168.123.18:8090` about a minute after power-on and
set the clock from the pre-flight strip, step 3.0.

## 2. Get the angle calibration — once, ever

```bash
python3 ~/go2-lidar-mapping/notes/get_lidar_calibration.py
```

Saves `E:\Unitree\walks\xt16_correction.csv`.

**Do this while the sensor is connected, and keep the file.** Each of the
XT16's sixteen lasers sits at a different vertical angle, measured per unit at
the factory. Live, the driver asks the sensor for those angles. Replaying a
recording there is nothing to ask, and without them every laser is treated as
horizontal: the cloud looks fine from above and is **completely flat from the
side — one layer, no height**. That is what happens if this file is missing.

It only needs doing once. It is read-only, and sends nothing except a "get
calibration" request.

## 3. Record

On the dashboard at `http://192.168.123.18:8090`, the **Record a walk** panel.

0. **Check the pre-flight strip is green**, or read what is red. It will refuse
   to start on a blocking failure and tell you which — clock not set, robot not
   standing, sensor not answering, no room. You can override it, and mostly you
   should not. A red **clock is set** chip has a **Set from this computer**
   button beside it, which gives the robot this laptop's time; check the
   laptop's own clock is right first.
1. Type a name, press **Start walk**
2. Wait ten seconds and check **the counters are climbing** — LiDAR packets,
   IMU rows and odometry rows. If LiDAR stays at zero, stop and fix that
   before walking.

   **The robot must be standing.** Odometry comes from `rt/sportmodestate`,
   which the motion control service only publishes while the robot is up. Lying
   down, `odom.csv` will be empty — and without it the walk replays as a smear
   rather than a map, because nothing says where each scan was taken from. The
   recorder says so on its own output after five seconds, and `session.json`
   records it, but it is far easier to stand the robot up now than to notice
   afterwards.
3. **Unplug the ethernet cable.** The recorder runs on the robot and writes to
   the robot's own disk; the laptop was only the button
4. Walk it, about two minutes
5. Plug the cable back in, wait ~15 seconds, refresh the page
6. Press **Stop walk**

The page then checks the recording and says whether it is any good — clock,
odometry, LiDAR packets, IMU continuity, length — in about a second, before you
have gone anywhere. It must not say `CLOCK WAS NOT SYNCED`.

### What makes a walk useful for mapping

- **Start and finish in the same place.** SLAM can then recognise it has
  returned somewhere and correct its accumulated drift. This is the difference
  between a clean map and a banana-shaped one
- **Move slowly and smoothly.** Fast rotation is the main cause of failure
- **Stay near structure** — corners, furniture, doorways. A blank corridor
  gives the algorithm nothing to hold onto

About 160 MB a minute. The robot has 429 GB free.

## 4. Copy it to the laptop

**From the page:** pick the walk in the Record panel's list and press
**Download**. It arrives as `<name>.zip` holding a `<name>` folder; unzip it
into `E:\Unitree\walks\`. Nothing to install, so this works from any laptop.
Keep the cable in until it finishes: about 800 MB for five minutes of walk.

**From WSL**, the same thing with scp:

```bash
~/go2-lidar-mapping/notes/fetch_walk.sh
```

With no arguments it lists what is on the robot and what you already have.
Then:

```bash
~/go2-lidar-mapping/notes/fetch_walk.sh Lab-loop-1
```

Lands in `E:\Unitree\walks\Lab-loop-1\`. Use `--all` for everything.

## 5. Process it: the bag and the sharp map

```bash
~/go2-lidar-mapping/notes/process_walk.sh ~/go2-lidar-mapping/walks/odom-test-1
```

Two parts, in one run:

1. **The bag** (`walk_to_bag.sh`, steps 1 to 6). Removes duplicated packets if
   the recording still has them, decodes the LiDAR with Hesai's driver and both
   calibrations as fast as it will go, then writes one `.mcap` with the LiDAR,
   odometry, IMU and frame tree, every message on the recording's own clock
   (`retime_bag.py`). A 315-second walk takes about 2.5 minutes.
2. **The scan-matched map** (`lidar_map.py`, step 7). Aligns each scan to the
   map built so far. About as long as the walk. Writes into `map3d/`:
   `<name>_map.ply` for CloudCompare, a Foxglove file, and `trajectory.csv`.

It runs on its own ROS domain (87), so the live LiDAR and a live Foxglove
session carry on untouched while it runs.

`--map-only` rebuilds just the map from a bag already made.

### Or press the button

The dashboard's **Walk library** panel lists every walk on the laptop with its
checks. **Process** runs `process_walk.sh` with a progress bar; **Build map**
runs it with `--map-only`. It needs the dashboard started with `--walks`, which
is the default on the laptop:

```bash
python3 ~/go2-lidar-mapping/notes/dashboard.py --walks ~/go2-lidar-mapping/walks
```

### Why timestamps come from the recording (2026-09-17)

Until then the conversion replayed the LiDAR, the odometry and the IMU together
and stamped every message with the time it was replayed. That silently assumed
Hesai's driver replays a pcap in real time. It does not, whatever its
`pcap_play_synchronization` setting says: on `odom-test-1` it ran at 0.4x
reading from E: and about 8x from WSL's own disk. The LiDAR spread over 1,290 s
against 315 s of odometry, so scans were drawn at the wrong poses, and the
replay was cut off at 86% of its frames.

Now:

* the driver stamps each frame with the **LiDAR's own packet time**
  (`use_timestamp_type: 0`). The XT16's clock is never set on this robot and
  reads May 2020, but it runs steadily;
* `retime_bag.py` measures that clock against the robot's from the pcap, where
  the recorder stamped every packet on arrival: the smallest arrival-minus-LiDAR
  offset in each 5 s window, and a straight line through those. On
  `odom-test-1`: drift 33.5 ppm, fit within 0.03 ms over 787,185 packets;
* odometry and IMU are written straight from `odom.csv` and `imu.csv` with their
  recorded times, using the same message code as the live publishers.

Replay speed no longer affects the result, so the driver runs unpaced.

### If it says the file was cut off

A conversion that gets killed leaves a large, plausible-looking `.mcap` that
plays back as nothing: rosbag2 writes the file's end marker and its
`metadata.yaml` only during shutdown. The script now checks for both, and if
they are missing:

```bash
python3 ~/go2-lidar-mapping/notes/recover_bag.py ~/go2-lidar-mapping/walks/<name>/bag
```

It keeps everything written before the cut, closes the file properly into
`<name>/bag-recovered/`, and rebuilds `metadata.yaml`. **The original is never
touched.** Recovering the first long walk kept 99.86% of the file.

**Prefer re-running `walk_to_bag.sh` when the recording is still on disk**, and
it is not a close call. Two reasons:

- Recovery gets back what was *written*, not what should have been. If the
  conversion died a third of the way through, you get a third of a walk.
- A recovered file **has no index**. The index is written at shutdown, along
  with everything else recovery is replacing, and it cannot be reconstructed
  from outside. `ros2 bag play` and `inspect_bag.py` cope — they scan linearly
  — but **Foxglove refuses any unindexed file over 600 MB**:

  > File has exceeded the maximum unindexed read size of 600.0 MB bytes.

  Under 600 MB it opens fine. Over it, rebuild with `walk_to_bag.sh`, or index
  the file with Foxglove's own `mcap recover` tool, which is what that error
  message is pointing at.

Recovery is for when the source files are gone, or when you want to see what a
damaged bag contains before deciding.

Note what it cannot do: it recovers what was *written*, not what should have
been. If the conversion was killed a third of the way through, you get a third
of a walk, and re-running `walk_to_bag.sh` is what gets the rest.

## 6. Look at it

### The map (what you usually want)

In Foxglove, once: **Layouts → Import from file →**
`E:\Unitree\notes\foxglove\go2_walk_map.json`. Then **File → Open local file**:

```
E:\Unitree\walks\odom-test-1\map3d\odom-test-1_map\odom-test-1_map_0.mcap
```

Scrub anywhere and you see everything mapped up to that moment:

| Topic | What |
| --- | --- |
| `/map_so_far` | the map up to now, one every second, 10 cm spacing |
| `/track` | the scan-matched track so far |
| `/track_odom` | the leg odometry's track, off by default, to see its drift |
| `/scan` | the current scan |

The `.ply` (`map3d/odom-test-1_map.ply`) is the whole walk in one cloud,
coloured by height, with the track in white. In CloudCompare, Top view, then
zoom.

### The bag

```
E:\Unitree\walks\Lab-loop-1\bag\bag_0.mcap
```

Add a 3D panel, set **display frame to `odom`** and **Follow mode to Fixed**,
enable **`/lidar_points`**, and use the timeline to play, pause and scrub.

Two settings cause most confusion here. **Follow mode** left on "pose" keeps the
robot centred, so the world appears to slide along with it. **Decay time** only
keeps scans that have played since the last seek: jump 15 s ahead and the trail
starts again. For the whole walk at any moment, use the map file above.

### If the panel is empty

Not "the recording is empty" — almost certainly the frame tree is **broken in
two**. The cloud arrives in frame `hesai_lidar`, which hangs off `base_link`;
if nothing publishes `odom -> base_link` then `map -> odom` and
`base_link -> hesai_lidar` are two disconnected trees, and Foxglove asked to
display in `odom` cannot transform the cloud into it at all. So it draws
nothing, and it looks exactly like a failed recording.

**To see it immediately:** change the display frame from `odom` to
`hesai_lidar`. The data is there.

**To fix the file:** re-run `walk_to_bag.sh`. For a session with `odom.csv` it
publishes the real moving transform; for one without, it publishes a frozen
identity so the tree at least joins up. Either way the panel works.

`inspect_bag.py` reports this as **"frame tree joins up: DISCONNECTED"** and
names the frame to fall back to.

No robot, no network, no live connection. The file is self-contained.

## 6b. Why the map is sharp: scan matching

Placing every scan by leg odometry alone (what `export_ply.sh` did for bags,
and still does) smears the map. Leg odometry drifts, mostly in heading, and
one degree of heading error moves a wall 20 m away by 35 cm. On `odom-test-1`
the odometry-only cloud was a blur of shifted copies of the same rooms.

`lidar_map.py` starts each scan where the leg odometry says the robot moved,
then slides and turns it until it lines up with the map built so far
(point-to-plane ICP, Open3D). A match is only accepted if it covers most of the
scan and stays within 0.5 m and 8 degrees of the odometry's guess. On
`odom-test-1`: 3,136 of 3,149 scans matched, and the track ends 0.26 m from its
start.

Standing still, the pose is held from the last match rather than re-matched:
matching a stationary robot only adds jitter, and on `odom-test-1` it cut the
map from about 280 s to under 2 minutes (2,099 of 3,151 scans were standing).

Height comes from the legs, everything else from scan matching
(`--vertical height`, the default). Measured on `odom-test-1`, floor under
the track:

| `--vertical` | floor thickness (1 m cells) | floor height across the walk |
| --- | --- | --- |
| `free` (scan matching sets everything) | 17.9 cm | 116 cm |
| `gravity` (roll, pitch and height from odometry) | 27.1 cm | 79 cm |
| `height` (height from odometry) | 17.4 cm | 26 cm |

Taking roll and pitch from the IMU made the floor thicker, which suggests the
LiDAR sits slightly tilted relative to the body despite the documentation's
"no rotation"; a floor fit from standing scans read about 1 and 3 degrees but
varied too much between scans (the robot was getting up) to apply. Use
`free` on stairs and ramps.

Since then: each scan is deskewed (the robot moves 10 cm during one turn of
the sensor), the LiDAR's tilt on the body was measured (`lidar_mount.py`),
height on stairs comes from scan matching (`--vertical auto`), and the track
is loop-closed (6b-1).

## 6b-1. Loop closure (2026-09-25)

The scan matcher compares each scan with its last 25 keyframes only, so what
it gets slightly wrong is never corrected by what it saw earlier. Where a walk
passes the same place twice, the two passes disagreed: on `odom-test-1`'s
corridor the ceiling came out twice, 30 to 40 cm apart, on the way out and the
way back, and the floor twice about 6 to 12 cm apart. Heading and position were
already good (0.1 to 0.2 degrees, 2 to 5 cm between passes); **tilt** was
the damage, 0.4 to 0.6 degrees typically and up to 2, because 1 degree at 8 m
moves a floor 14 cm. The XT16 sees no floor within 1.5 m and the floor and
ceiling beyond only at grazing angles, so tilt is its weakest direction.

`pose_graph.py` now runs inside `lidar_map.py` once every scan is matched.
Keyframes that come back within 3 m of an earlier one after at least 8 m of
walking are aligned to it (a keyframe and its four neighbours either side,
about 3 m of walk, point-to-plane ICP), and the whole track is adjusted by
robust least squares so the loops agree while each step stays close to what
the scan matcher measured. Only then are the map and the Foxglove file
written. About 10 s per walk. `--no-loop-closure` builds without it;
`trajectory_frontend.csv` and `/track_frontend` keep the uncorrected track.

### Measuring it: `map_quality.py`

Two new measures, both re-reading the scans with the build's track:

* **passes agree**: the track is cut into 2 m stretches, and every pair of
  stretches over the same place with 6 m or more of walking between them is
  aligned by ICP. The correction it finds is how far apart the passes put the
  same place: height (doubled floors and ceilings), sideways, heading, tilt.
* **end vs start**: the last standing scans aligned to the first. That is
  where the robot *really* ended, which is not where it started:
  `odom-test-1` ended 17 cm away. The difference from where the track says it
  ended is the track's error after the whole walk. "Track ends 0.35 m from its
  start" was mostly the robot lying down at the end (26 cm of body height).

### Result

Both walks rebuilt with `process_walk.sh --map-only`; the builds before are in
`map3d/previous/2026-09-25_112551` and `..._112554`.

| | odom-test-1 before | after | odom-test-2 before | after |
| --- | --- | --- | --- | --- |
| track error at the end | 4.2 cm, 1.28° | **2.2 cm, 0.17°** | 9.9 cm, 1.20° | **1.0 cm, 0.30°** |
| passes: height, median / worst | 3.0 / 12.3 cm | **1.2 / 5.4 cm** | 1.7 / 12.1 cm | **0.8 / 4.8 cm** |
| passes: sideways, median | 1.7 cm | **0.5 cm** | 5.3 cm | **0.6 cm** |
| passes: tilt, median / worst | 0.6 / 1.6° | **0.4 / 1.4°** | 0.4 / 2.0° | **0.2 / 0.8°** |
| floor thickness, dense cloud | 14.6 cm | **8.5 cm** | 17.9 cm | 17.1 cm |
| floor thickness, keyframe map | 18.6 cm | **14.4 cm** | 20.7 cm | 21.2 cm |
| ceiling, far minus near | −12 cm | **−3 cm** | +1 cm | +1 cm |
| dense cloud | 1.77 M points | 1.63 M | 3.17 M | 3.04 M |

`map3d/loop_closure_section.png` in each walk is a 30 cm slice through a
revisited corridor, coloured by pass: two ceilings and two floors before, one
each after (`odom-test-2`: the passes' floors 6.1 cm apart before, 0.1 cm
after). `map3d/loop_closure_floor_thickness.png` maps the floor thickness per
1 m cell.

**Why `odom-test-2`'s floor thickness hardly moved.** Its thick cells are the
corridor's edges: the measure takes the spread of everything within 60 cm of
the lowest point in a 1 m cell, and a cell with a wall in it counts the wall's
bottom 60 cm. Cells on the track are 5 to 10 cm in both builds. The same
contamination explains "floor wander" rising from 44 to 53 cm there; over
cells with no wall in them it fell from 8 to 5 cm. Fewer points in the dense
cloud is expected: surfaces that were drawn twice are now drawn once.

### Tried and rejected

* **The IMU's tilt as a soft prior** (gravity, 1 degree). The track's tilt
  differs from the IMU's by 1.5 to 2.5 degrees for 10 to 40 s at a time, which
  looked like drift in the scan matcher. It is at least partly the IMU's own
  estimate: pulled towards it, `odom-test-1`'s passes disagreed more (tilt 0.6
  to 0.8 degrees, height 90th percentile 7.5 to 28 cm). At 3 degrees it was no
  better than none. Same lesson as `--vertical gravity`, now with the mount
  calibrated.
* **Steps kept in the keyframe's tilted frame**: correcting tilt then turned
  every step, and 10 m of walking gained 35 cm of height nobody measured.
* **Loop matches accepted only below 8 cm RMS**: that is the typical RMS at a
  10 cm grid, so half the good matches were thrown away.
* **Submaps of a keyframe and two neighbours** (1.5 m of walk): not enough to
  pin tilt down with this sensor; `odom-test-1`'s loop around its start came
  out worse than without loop closure. Four neighbours fixed it; six was no
  better and four times slower.

### What it does not fix

The fast tilt jitter of single scans (about 0.45 degrees against the IMU)
remains, and so does the XT16's blindness to the floor within 1.5 m: the
floor on the track is 5 to 10 cm thick. See 6b-3 for what the depth camera
did and did not change.

Measured later the same day (6b-3): a change that should not matter (LiDAR
range 29.5 instead of 30 m) moves several of these numbers by more than some
of the differences above. Four against six neighbours was within that noise;
loop closure itself, and the rejections of gravity, tilted steps and the 8 cm
RMS cut-off, were far outside it.

## 6b-3. Height, stairs and the depth camera (2026-09-25)

Goal: use the depth camera for working out where the robot is, not just for
drawing the map: a thinner floor, and no jump in height where the stairs
start (`odom-test-2`). New measures in `map_quality.py` for this: **floor
only** (floor thickness from points whose surface faces up, so wall bottoms
do not count) and **stairs** (how far the track goes below the floor at the
foot of each flight, above the landing at its head, and its height noise on
the flight).

### What the measurements showed first

* **The jump at the stairs was not a matching problem.** As the robot pitched
  nose-up for the first step (74.0 s), Unitree's odometry height (`pz`) fell
  10 cm while the legs' own `body_height` stayed at 0.30 m. On the flat the
  map builder took height changes from `pz`, and it switches to scan-matched
  height on stairs only 0.3 s later (a 1 s median of pitch), by when the
  track had copied a 6-7 cm dip. Now height on the flat follows
  `body_height` (`--flat-height body`; `odom` is the old way).
* **Depth on the map made the floor thicker, not thinner**: `odom-test-2`'s
  dense floor was 2.3 cm without depth and 6.7 cm with it (floor only). It was
  not the calibration (depth and LiDAR agree within 1 cm out to 3 m, floors
  and walls), nor a clock offset (best fit at 0 ms), nor the lying-down
  frames (now left out anyway, as LiDAR scans are). It is the camera: within
  one frame, with no pose involved, the floor scatters around its own plane by
  0.4 cm at 0.5-1 m, 0.9 cm at 1-1.5 m, 1.8 cm at 1.5-2 m, 3.7 cm at 2-2.5 m,
  5.8 cm at 3-4 m (stereo error grows with distance squared; a plain, glossy
  floor seen at a grazing angle is its worst case). The dense cloud now uses
  depth out to 1.5 m only, where it beats the LiDAR.
* **Glossy floor**: where `odom-test-2` starts, the depth "floor" runs 10-20
  cm below the real one and bends down near the wall ahead: the camera
  measures the room's reflection.

### The depth camera in the scan matching

Depth points added to the LiDAR's in the ICP made floors thicker (LiDAR
floor 2.4 to 2.9-3.4 cm, ceiling far-minus-near 0 to +6 cm): the camera sees
a patch of floor about 1 m deep, which fixes height to a centimetre but tilt
only to half a degree, and tilt errors grow with the LiDAR's range. So the
LiDAR decides the pose and the camera then moves the scan **up or down
only**, on stairs and ramps (`--depth-matching height`, the default;
`full` and `off` for comparison). On every scan instead of stairs only it was
bad (ceiling +64 cm).

### Result

Both walks rebuilt; step 1's builds are in `map3d/previous/2026-09-25_130945`
(`odom-test-1`) and `..._130855` (`odom-test-2`).

| | odom-test-1 step 1 | step 2 | odom-test-2 step 1 | step 2 |
| --- | --- | --- | --- | --- |
| floor thickness, dense | 8.5 cm | **7.5 cm** | 17.1 cm | **12.2 cm** |
| floor only, dense | 4.0 cm | **2.7 cm** | 6.6 cm | **5.1 cm** |
| floor only, keyframe map | 3.5 cm | **3.0 cm** | 2.4 cm | 2.9 cm |
| stairs: below the floor at the foot, up / down | | | 5.7 / 3.3 cm | **2.1 / 2.8 cm** |
| stairs: height noise, 90th percentile, up / down | | | 2.6 / 4.2 cm | **1.8 / 3.8 cm** |
| track error at the end | 2.2 cm, 0.17° | 2.3 cm, 0.82° | 1.0 cm, 0.30° | 0.5 cm, 1.05° |
| passes: height, median / worst | 1.2 / 5.4 cm | 1.4 / 6.4 cm | 0.8 / 4.8 cm | 1.3 / 4.7 cm |
| dense cloud | 1.63 M | 1.61 M | 3.04 M | 2.37 M |

`odom-test-2/map3d/depth_height_stairs.png`: the track's height at the foot
of the stairs, both ways, before and after. `depth_height_stairs_section.png`:
the stairs from the side (the stairs themselves looked clean in both).

**How sure these are.** Rebuilt unchanged, builds differ only slightly (up
to 0.2 cm floor-only, 0.2° end tilt), but they are sensitive: rebuilt
with the LiDAR's range at 29.5, 30 and 30.5 m, the same settings gave floor
only 2.3-2.9 cm (keyframe map), 4.3-5.0 cm (dense), a dip at the foot of 2.7-3.9
cm without the camera and 0.6-2.1 cm with it, and an end error tilt anywhere
from 0.3 to 1.1 degrees. So: the thinner dense floor (mostly the 1.5 m depth
range) and the smaller dip on the stairs are real; the end tilt errors and the
keyframe map's 2.4 against 2.9 cm are within the noise.

**`odom-test-1`'s floor wander rose from 38 to 88 cm** without the floor
moving: one cell has about 110 points 1.2 m below the floor (in both builds;
a reflection or a gap), and they crossed the 2% the measure takes as a cell's
lowest surface. A weakness of the measure, noted in `map_quality.py`.

Tried and rejected, besides depth points in the ICP and height corrected
everywhere: height from matching everywhere with depth (`--vertical free`:
floors 2.1 to 3.2 cm, ceiling +8 cm); the IMU's fast tilt blended with the
track's slow tilt, 0.5-2 s (no thinner floor, passes worse, worst 4 to 10 cm).

`export_ply.sh --live --seconds 15` is still the way to save the cloud around
the robot right now, in the sensor's own frame.

## 6b-2. Dense, cleaned cloud (step 8 of Process)

`dense_map.py` places **every** scan (the map builder only keeps one about every
0.4 m as a keyframe) into a 3 cm grid using the track the map builder found,
then cleans it. Writes `map3d/<walk>_dense.ply` and `map3d/<walk>_removed.ply`
(what cleaning took out: grey seen too rarely, red seen through, magenta
isolated specks; open both together in CloudCompare to check).

Cleaning, in order:

1. **Seen by at least 3 scans**, counted in 10 cm cells. Counted per 3 cm voxel
   it removed 1.5 million voxels of `odom-test-1`, 44% of them ceiling, which
   the XT16 only sees from far away with widely spread points.
2. **Free-space carving.** Rays from the sensor to a sample of points in every
   fifth scan mark the cells they pass through as empty at that moment. Cells
   seen through three times more often than hit held something that moved.
   Rays within 15 cm of the floor under the sensor are not counted: at first
   they carved the floor itself (46% of the carved points on `odom-test-1`,
   16% after).
3. **Statistical outlier removal** (Open3D, 20 neighbours, 2 standard
   deviations).

Measured 2026-09-17, points within 2 m of the track:

| walk | map | ceiling | floor | walls | floor thickness |
| --- | --- | --- | --- | --- | --- |
| odom-test-1 | keyframe map | 49,575 | 96,031 | 76,241 | 18.6 cm |
| odom-test-1 | dense | 204,278 | 439,090 | 213,958 | 15.2 cm |
| odom-test-2 | keyframe map | 92,884 | 124,728 | 113,599 | 20.7 cm |
| odom-test-2 | dense | 372,555 | 554,912 | 365,078 | 17.4 cm |

About 40 s per walk: scans are placed on all cores but one.

## 6b-4. The colour cloud (2026-09-25)

`dense_map.py` now also colours the cleaned cloud from the colour frames in
`depth.stream` (`colour_map.py`), when the walk has them:

| File | What |
| --- | --- |
| `map3d/<walk>_colour.ply` | camera colour where a colour frame saw the point, height colours elsewhere, and a per-point `colour_source` (1 camera, 0 height colours): a grey point can always be told from grey paint. In CloudCompare it is a scalar field. |
| `map3d/<walk>_dense/` | `/colour_map` beside `/dense_map`: fields `rgb` and `colour_source` |
| `map3d/dense_colour.npy` | r g b colour_source per point, in `dense_map.npy`'s order |

In Foxglove, the walk-map layout (`notes/foxglove/go2_walk_map.json`) now
shows `/colour_map` in colour mode **RGB**, and hides `/dense_map` (same
points, height colours; tick it to compare). To see which points have camera
colour, set `/colour_map`'s colour mode to *Color map* on field
`colour_source`. `/track_frontend` (the track before loop closure) is there
too, off.

Each colour frame (5 a second) is placed with the track, the camera mount
and Intel's factory position of the colour lens; points within 10 m take its
colour if the depth frame recorded with it shows nothing nearer along the
same ray. Each point's colour is the mean over the frames that saw it,
weighted towards close and steady views.

Checked on `odom-test-2`:

* **Held-out photos**: colouring without the frames near 12 test frames, then
  looking at the coloured cloud from each test frame's pose, it matches the
  real photo (brightness correlation 0.62; 0.56 with the view turned 2°).
  `map3d/colour_views.png` shows the result from above and from inside.
* **Timing and calibration**: colours agree best with the frame times as
  recorded and with Intel's factory calibration of the colour lens (1° off in
  any direction was worse).
* **Coverage**: 73% of points in camera colour: floor 94%, walls 69%, ceiling
  56%.

Dropped on the way: deciding what hides what from the cloud alone (only 26%
of the floor coloured: seen at a grazing angle, nearer floor "hid" farther
floor), and a 6 m reach (ceiling 3%).

The colour is what the camera saw: the glossy floor carries its reflections,
exposure changes between frames, and anything the cleaning kept is coloured
too (a person-shaped shape remains in `odom-test-2`'s corridor at about 160 s).
`odom-test-1` has no camera recording, so no colour cloud.

## 6b-5. Crop outline, 2 cm grid, floor plans and mesh (2026-09-25)

### Crop outline per walk: `crop_outline.py`, `<walk>/crop.json`

`dense_map.py` guessed what was outdoors (lines of sight that meet no wall).
It kept trees within about 3 m of the building and removed real surfaces:
the far end of `odom-test-2`'s long corridor and stretches of courtyard walls
on both walks. What a building has and a garden has not is a **roof**:
`crop_outline.py <walk>` proposes an outline from flat ceilings 1.6-5 m up and
flat walls 0.3-1.6 m up, connected to the track, with enclosed courtyards
filled in (the old guess kept them too), and saves it as `<walk>/crop.json`
with `crop_outline.png` to check it. The corners are numbered in the picture;
edit the json to move them. When a walk has a `crop.json`, `dense_map.py`
crops to it and skips the guess (`--no-crop` for the guess).

Both proposals needed one hand edit, recorded in the json's `edits`:
`odom-test-2`'s corridor continues past an unroofed gap at y 34-38 to y 58
(added as a second polygon), and `odom-test-1`'s outline had a notch cutting
the building's corner walls at x -13, y 0-6. Compared with the guess
(`odom-test-2/map3d/crop_vs_outdoors.png` shows it): the far corridor, the
courtyard walls and a building corner came back; the tree clumps south of
`odom-test-2` and north of `odom-test-1` went.

### 2 cm grid

`dense_map.py --voxel` is now 0.02 (was 0.03): twice the points, floors no
thicker (floor only 5.0 to 5.1 cm on `odom-test-2`, 2.7 to 2.5 cm on
`odom-test-1`), and 80% of `odom-test-2` in camera colour instead of 74%.
The cost: the dense step took 364 s instead of about 230 s on `odom-test-2`
and peaked at 5.0 GB of the 7 GB WSL has. **A walk twice as long may not
fit; use `--voxel 0.03` then.**

### Intel's depth filters: tried, not used

Intel's spatial filter (edge-preserving smoothing in disparity), rebuilt from
Intel's own description in `depth_filters.py`, changed the floor's scatter by
nothing measurable at Intel's defaults and made it worse when stronger. This
camera's error is broad waves, not speckle: it correlates 0.95 between pixels
8 apart. Intel's temporal filter was not tried: Intel say it is for slow
scenes, and the camera walks. See the docstring.

### Floor plans: `floor_plan.py` (step 9 of Process)

One 2D plan per floor: `map3d/<walk>_plan_L<n>.png` (walls and obstacles
0.2-1.8 m above that floor black, floor seen grey, the track in red, 5 cm
pixels, 1 m grid) and the same as a ROS `map_server` map (`.pgm` + `.yaml`).
Floors are found as heights with at least 5 m2 of horizontal surface that
the robot was above somewhere nearby: `odom-test-1` has one, `odom-test-2`
two (the ground at -0.07 m and the stair landing at +1.62 m, 22 m2). Two
mistakes on the way, both fixed: the ceiling counted as a floor (it is
horizontal too), and the ground floor's ceiling, 0.5 m above the landing,
filled the landing's plan with "walls". The courtyards are white (not seen):
the LiDAR saw their ground only as sparse rings.

### Mesh: `mesh_map.py` (step 10 of Process)

`map3d/<walk>_mesh.ply`: Poisson surface reconstruction (Open3D, depth 11)
of the dense cloud with normals turned towards where the robot was, trimmed
wherever the surface strays more than 5 cm from a real point (Poisson closes
every hole with invented surface), 5 passes of Taubin smoothing, coloured
from the camera where it saw (with `colour_source` per vertex).

| | odom-test-1 | odom-test-2 |
| --- | --- | --- |
| surface to the nearest real point, median / 90% | 1.2 / 2.7 cm | 1.2 / 2.4 cm |
| cloud within 3 cm of the surface | 87% | 79% |
| floor only (flatness) | 1.4 cm | 1.5 cm |
| triangles, file | 4.4 M, 94 MB | 2.7 M, 58 MB |
| vertices in camera colour | (no camera) | 66% |

The mesh's floor is far flatter than the cloud's (5.3 cm): fitting a surface
averages out the camera's noise. Depth 10 covered only 62% of the cloud;
smoothing beyond 5 passes changed nothing measurable. Walls the camera never
saw keep height colours (upper walls, most of the ceiling).

`process_walk.sh` runs steps 9 and 10 after the dense cloud; if either fails
it warns and the run still succeeds. Go2 Studio's progress bar knows steps 1-8
only, so it stays on "dense" for the last minute or so.

After all of this, `map_quality.py` on both walks is where step 3 left it,
within the rebuild-to-rebuild noise.

## 6c. Depth camera in the walk

Turn the depth camera on in the dashboard's **Sensors** panel before pressing
**Start walk**. `record_session.py` sees the camera running and starts
`depth_recorder.py` beside it, which writes `depth.stream` into the walk
folder: every depth frame, lossless (16-bit PNG, millimetres), stamped by the
camera node with the robot's clock, plus the factory calibration and Intel's
camera frames. Colour too, if the camera was started with colour. About 3 MB
a second at 640 x 480, 15 fps. The walk's progress shows a **depth** line and
the report adds **depth camera recorded**.

Processing then puts depth in both files:

| File | Depth |
| --- | --- |
| `bag/bag_0.mcap` | `/camera/depth/image_rect_raw` (full resolution), `camera_info`, colour, the camera's frames |
| `map3d/<walk>_map_0.mcap` | `/depth_preview` heat map, `/depth_scan` points, `/depth_so_far` on the map, `/color/compressed` |
| `map3d/<walk>_map.ply` | depth points merged with the LiDAR |

### Where the camera is: camera_mount.json

Unitree publish the LiDAR's position, not the RealSense's. Until it is known
the depth goes into the files as images only. Measure it with a tape (the
Launchpad step gives four numbers from the LiDAR), then fit it exactly:

```bash
python3 ~/go2-lidar-mapping/notes/depth_mount.py ~/go2-lidar-mapping/walks/<walk> --from-lidar DX,DY,DZ,TILT
~/go2-lidar-mapping/notes/process_walk.sh ~/go2-lidar-mapping/walks/<walk> --map-only
```

`depth_mount.py` places ~60 depth frames through the scan-matched track and
moves the mount until their points lie on the LiDAR's surfaces (point to
plane, robust weights, coarse to fine). It writes `walks/camera_mount.json`
with the measurement kept alongside, and refuses to write a poor fit.

Tested on 2026-09-17 with synthetic depth rendered from `odom-test-1`'s LiDAR
map at a known mount (0.285, 0, 0.02 m, 12 degrees down): from three wrong
starts up to 13 cm and 12 degrees off, it returned 0.286, 0.000, 0.020 m and
12.0 degrees each time. A first, shorter schedule stopped at 8 degrees and
-2 cm: height and tilt trade off when the camera mostly sees floor, which is
why the fine stages now run long. Real depth is noisier than the test.

## 7. Check it is sound

```bash
python3 ~/go2-lidar-mapping/notes/inspect_bag.py ~/go2-lidar-mapping/walks/Lab-loop-1/bag
```

It no longer just prints numbers — it ends with a pass/fail line per check, and
it also looks at the recording the bag came from, because some faults are only
visible there.

The checks, and what each one catches:

| Check | Fails when |
| --- | --- |
| **moving transform (`/tf`)** | the bag has only static transforms — every scan drawn at the same pose, so a walk is a smear. Needs `odom.csv` and `odom_publisher.py` |
| **cloud has height** | the per-laser angle calibration was missing; the cloud is one flat layer |
| per-point timestamps / ring | the driver did not emit them (ours does — `timestamp` float64 and `ring` uint16) |
| frames are full | dropped UDP packets, or a frame split at the wrong azimuth |
| steady frame rate | a gap much longer than the 0.1 s median |
| **`imu.csv` is a single stream** | repeated or out-of-order ticks — the recorder was subscribed to both `rt/lowstate` and `rt/lf/lowstate` |
| clock was synced | the robot booted at 1970 and nothing set its clock |
| odometry reached the bag | `odom.csv` exists but `/odom` is missing from the bag |

The two in bold are the failures that look completely plausible on screen. Both
were found by hand on day one and both are now caught automatically — running
the inspector on `Lab-loop-1` reports 8 of 10, failing exactly those two.

Rough numbers to expect alongside: 25,000–32,000 points per frame, 10 frames a
second of walk, ~500 IMU samples a second, ~50 odometry samples a second.

---

## Where the files come from

| File | What |
| --- | --- |
| `lidar.pcap` | raw LiDAR packets as recorded |
| `lidar_clean.pcap` | the same with duplicates removed. Only for recordings made before the recorder filtered at capture time — newer ones skip this and use `lidar.pcap` directly, which `session.json` says by carrying `not_addressed_to_us` |
| `imu.csv` | one row per IMU sample, ~500/s |
| `odom.csv` | one row per odometry sample, ~50/s — **where the robot was** |
| `session.json` | counts, rates, which topics were used, what the duplicate filter rejected, and whether the clock was synced |
| `replay_config.yaml` | generated driver config pointing at this recording |
| `bag/bag_0.mcap` | the finished file for Foxglove |
| `xt16_correction.csv` | the sensor's angle calibration, shared by all walks |

---

## Two bugs this process found on day one

**Every LiDAR packet was recorded twice.** The recorder captured both the
sensor's packets arriving at the robot and `lidar_relay.py`'s forwarded copies
heading for the laptop — both use UDP port 2368, and it filtered on the port
alone. 5,070 packets a second where the XT16 sends 2,535. The recorder now
keeps only packets addressed to the robot itself; `walk_to_bag.sh` still cleans
older recordings.

**The replayed cloud was flat.** The missing angle calibration, above. Found by
checking the z range rather than by looking at the picture, which looked
entirely reasonable from the top-down view.

Both were caught by a 24-second test recording. Recording something short and
inspecting it properly is worth doing before any long walk.

## Three more found on 2026-09-05, from the files rather than the robot

**Every scan was drawn in the same place.** The frame tree bolted the LiDAR
rigidly to the world, so a walk came out as a smear. Nothing in the picture
says so — it looks like a point cloud, because it is one. Fixed by recording
the robot's own odometry and publishing it as a *moving* transform. See
`decisions.md` D11.

**The LiDAR extrinsic was a guess, and the vendor publishes it.** It was
x=0, z=0.30; Unitree's docs give (0.171, 0, 0.0908) with no rotation.

**`imu.csv` had 285 duplicate and 27 backwards samples**, because the recorder
subscribed to both `rt/lowstate` and `rt/lf/lowstate` and wrote both into one
file. `lf` means low frequency — same data, sent less often, not a second
sensor. It now uses the fast topic only, with the slow one as a fallback if the
fast one is silent, and says in `session.json` which one it used.

## And one on 2026-09-05 that was the check, not the bag

**The frame-count check said a perfect conversion was 50% short.** It compared
the frames in the bag against `session.json`'s packet count divided by 250 — but
on a recording made before the recorder filtered duplicates, that count holds
every packet the sniffer saw, each real one twice. Converting `Lab-loop-1`
reported

```
X frames     only 245 of about 487 (50%) -- the bag is SHORT
```

It was 245 of 244. `walk_to_bag.sh` now counts the pcap it actually replays, by
walking the record headers, which is right for old and new recordings alike and
needs neither sudo nor a second pass over the payload. The same conversion now
says `frames 245 of about 243 (100%)`.

Worth noting as its own class of fault: a check that cries wolf is not harmless.
This one would have sent you re-running a two-minute conversion that had already
worked.

## One on 2026-09-06, found by cancelling on purpose

**Cancelling a conversion left the frozen transform running.** The dashboard's
Cancel sends SIGTERM to the script's process group — and that killed the script
and nothing else, because every helper it starts is put in its own session with
`setsid` precisely so a stray signal cannot take one down. The lock went, the
recorder went, and the frozen `odom -> base_link` publisher carried on
publishing: the exact thing this script's own comments say must never outlive
it, since it then fights `odom_publisher.py` for the same link in the next live
session and every lookup lands on whichever arrived last.

`walk_to_bag.sh` now traps INT and TERM as well as EXIT, and tears down the
replay driver, both replays, the recorder (with SIGTERM, so the mcap still
closes) and the frozen transform. The trap is armed only *after* the lock is
taken — armed before, the "another conversion is already running" exit would
have killed the other run's processes.

Found by cancelling a real conversion 22% of the way in and counting what was
still alive afterwards. Nothing about the dashboard looked wrong: the job said
cancelled, the bar went red, the lock was gone.
