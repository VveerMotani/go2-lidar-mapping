# Decisions — read this before changing anything

One entry per decision. Each says what was chosen, what was rejected and why,
and what would make us revisit. The reasoning and citations live in
[`why-it-failed.md`](why-it-failed.md); this file is the short version.

Status key: **DECIDED** · **PROVISIONAL** (decided, but resting on something
untested) · **OPEN**.

---

## D1 — Architecture: WSL2 stays the primary DDS participant

**Date:** 2026-09-03 · **Status: DECIDED**

The laptop's WSL2 distro `unitree` remains a full ROS 2 / DDS participant on the
192.168.123 wire, holding `192.168.123.99`. Native Ubuntu 22.04 is the fallback.

**Why.** This is not a workaround — it is Unitree's own documented setup.
`unitree_ros2` specifies Ubuntu 22.04 + ROS 2 Humble + CycloneDDS 0.10.2 +
`rmw_cyclonedds_cpp` + a static `192.168.123.99`, and that is exactly what we
have built. Mirrored networking's multicast support is a documented WSL feature.
It already works: `/lowstate` was observed at ~500 Hz. Cost to keep: zero.

**Rejected — B, everything on the Jetson, laptop as display.** The Jetson is
Ubuntu 20.04 / ROS 2 Foxy / Python 3.8 with **no internet and no Wi-Fi**, and a
clock that resets to 1970 each boot and thereby breaks pip. Unitree's Humble path
is simply unavailable there. Every install becomes an offline `scp` + 
`--no-build-isolation` exercise. We have paid that tax once already.

**Rejected — C, native Ubuntu 22.04 dual-boot.** Technically the best platform
and kept as the fallback, but it means repartitioning a laptop with a nearly-full
C:, rebuilding three workspaces, and losing Windows during robot work. Cannot be
ready for tomorrow.

**Rejected — D, Docker Desktop or a bridged VM.** Docker Desktop on Windows runs
its engine *inside a WSL2 VM*, so `network_mode: host` binds to that VM, not to
Windows — it is strictly worse than what we already have. A bridged VirtualBox/
Hyper-V guest would carry multicast cleanly but conflicts with the Hyper-V that
WSL needs, has no GPU, and would mean rebuilding everything for no gain.

**Revisit if:** `poolmon` confirms the `NtFC` leak and the reboot cadence becomes
intolerable, **or** the work moves to low-level / high-rate control. Do not run
1 kHz torque control through WSL — that goes on the Jetson or on option C.

---

## D2 — The 3D view moves to Foxglove on Windows; rviz2 under WSLg is demoted

**Date:** 2026-09-03 · **Status: PROVISIONAL** — depends on pre-flight P4

`foxglove_bridge` runs in WSL; **Foxglove Desktop runs natively on Windows** and
connects to `ws://localhost:8765`. `rviz2` stays installed as a fallback only.

**Why.** RViz died with Ogre `Vertex Buffer: Out of memory` and was auto-restarted
by the launch file — that is the blinking. Decay Time 5 s at 20 Hz asks the GPU to
hold ~1.6 M points at once. Beyond that, rviz2 under WSLg is not a supported path
in any vendor sense: it goes WSLg → a Direct3D 12 translation layer → the vGPU
driver, and any of those failing takes the window down. A visualiser has to be
the trustworthy part of the stack. Foxglove renders on the RTX 4070 through the
normal Windows driver and takes WSLg out of the critical path.
**`ros-humble-foxglove-bridge 3.4.3` is already installed** — no internet needed.

**Rejected — keep rviz2, just lower the decay time.** Fixes the immediate crash
but leaves the whole WSLg/D3D12/vGPU chain in the path of every debugging session.

**Rejected — a web viewer served from WSL.** Same unresolved Windows→WSL
question as Foxglove, with more to build.

**Rejected — run the 3D view on the Jetson.** 4 cores, no GPU, no internet.

**This is PROVISIONAL because** it needs Windows to reach a TCP port inside WSL —
the very thing we concluded (probably wrongly) was impossible. **Run pre-flight
P4 before relying on it.** If P4 fails, fall back to rviz2 in WSL with **Decay
Time 0** and a small point size.

**Kept regardless:** `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` in `setup.env.sh`,
so any WSLg rendering uses the RTX 4070 rather than the Intel iGPU.

---

## D3 — LiDAR: 10 Hz, single return, filtering on; destination IP never touched

**Date:** 2026-09-03 · **Status: DECIDED**

Web console at `http://192.168.123.20`:

| Setting | Value |
| --- | --- |
| Destination IP / Port | `192.168.123.18` / `2368` — **never change** |
| Spin Rate | **600 rpm (10 Hz)** — down from 1200 |
| Return Mode | **Single Return → Strongest** — was dual |
| Interstitial Points Filtering | **ON** |
| Reflectivity Mapping | Linear · **Sync Angle** off · **Trigger** Angle-Based |
| Clock Source | GPS (default), unused · **VLAN** off · **FOV** full 360° |

**Why 10 Hz.** The point *rate* is fixed at 320,000/s no matter the spin. Faster
spinning does not add data, it spreads the same data thinner: ~64,000 points per
frame at 5 Hz, **~32,000 at 10 Hz**, ~16,000 at 20 Hz. Raising the spin to 20 Hz
"because more is better" halved what each frame showed. 10 Hz is also the factory
default and what SLAM pipelines expect. 5 Hz is denser per frame but smears badly
on a walking robot (200 ms per scan).

**Why single return.** Dual return doubles the stream to 640,000 points/s —
double the UDP load through the Jetson relay and through DDS — for a second echo
indoor mapping does not use.

**Why the destination IP is untouchable.** Repointing it to `.99` stopped the
sensor transmitting entirely until a full power cycle. `lidar_relay.py` on the
Jetson exists solely so this never has to change. **Verified this is sound:**
the driver's `device_ip_address` feeds only the PTC/TCP client
(`libhesai/Lidar/lidar.h:223,245`), *not* a UDP source filter, so packets relayed
from `.18` are accepted normally. Keep `device_udp_src_port: 0` or that stops
being true.

**Rejected — 20 Hz.** The original "more is better" reasoning. Wrong, see above.
**Rejected — nonlinear reflectivity mapping.** Boosts contrast for human viewing
and confuses anything expecting standard intensity.
**Rejected — a restricted azimuth FOV.** The dog turns; a body-fixed FOV window
would rotate with it and lose coverage unpredictably.

**Also decided: the sensor is not faulty.** Wide vertical banding at range is
geometry — 2° between channels means the gap between laser lines is ~0.035 × distance
(3.5 cm at 1 m, 35 cm at 10 m). Every 16-line LiDAR looks like this. Stop
suspecting the hardware.

---

## D4 — `config.yaml`: one real bug fixed, timestamps stay on host time

**Date:** 2026-09-03 · **Status: DECIDED**

- `frame_start_azimuth: 0` → **`-1`**. A genuine error: the file's own comment
  gives the valid range as 1–359 and says to use a negative value to disable it.
  `0` is out of range.
- `send_imu_ros: true` → **`false`**. The XT-16 datasheet lists no IMU. Flip back
  if `/lidar_imu` turns out to publish.
- **`use_timestamp_type: 1` stays.** Host receive time, not sensor time. The
  LiDAR's clock free-runs from 2020 with no GPS or PTP, so sensor timestamps
  would be years off and every TF lookup downstream would fail.
- `speed: -1` stays — the web console remains authoritative on spin rate; the
  driver must not silently re-spin the sensor over PTC.
- `enable_packet_loss_tool: true` stays — `/lidar_packets_loss` is the instrument
  that settles D7.

---

## D5 — Windows settings: keep three, add one, revert none (they are already reverted)

**Date:** 2026-09-03 · **Status: DECIDED**

**Kept.** `networkingMode=mirrored` (the only mode giving the distro the real
`.99` and carrying multicast); `firewall=true`; and the Hyper-V rule
**`WSL-Unitree-Link`** — inbound, **any protocol**, remote `192.168.123.0/24`.

The rule must stay **address**-scoped. The earlier port-scoped rule
(UDP 7400–7700) failed because **only the first fragment of a fragmented IP
datagram carries a port number** — later fragments cannot match a port rule and
are dropped, which is exactly the `reasm_reqds=6345, reasm_oks=0` we measured.
Small discovery packets fit in one fragment, which is why discovery worked and
data did not.

**Added: cap CycloneDDS below the MTU.** `MaxMessageSize=1400B` (default is
14720 B, ~10× the 1500-byte MTU), `FragmentSize=1344B`, and
`SocketReceiveBufferSize min="10MB"` with `net.core.rmem_max=16777216` in the
distro. The firewall rule makes fragmentation *survivable*; this stops us
generating fragments at all.

**Rejected — `Set-NetFirewallHyperVVMSetting -DefaultInboundAction Allow`.**
Microsoft offers it, but it opens WSL to all inbound traffic on every network the
laptop ever joins. The subnet-scoped rule achieves the same thing for the robot.

**Already resolved, no action needed.** Verified read-only on 2026-09-03:
- The old `WSL-Unitree-DDS` port rule is **gone**.
- The Ethernet adapter is **back on DHCP** (`Dhcp: Enabled`).
  The session history lists this as unresolved item #1 — **it is fixed.** Do not
  run the "repair" commands; there is nothing to repair.

**Consequence:** `192.168.123.99` is *not* currently configured and must be
re-added each robot session, as a **secondary** address that leaves DHCP intact:

```powershell
New-NetIPAddress -InterfaceAlias 'Ethernet' -IPAddress 192.168.123.99 -PrefixLength 24 -SkipAsSource $true
```

`-SkipAsSource $true` keeps normal internet traffic on the DHCP address. **Caveat:**
it can also stop Windows choosing `.99` as the source for traffic *to* the robot
subnet. If the robot is unreachable despite the address being present, flip it:
`Get-NetIPAddress -IPAddress 192.168.123.99 | Set-NetIPAddress -SkipAsSource $false`.

---

## D6 — The RAM leak is `ntfs.sys` / VHDX I/O, not networking and not the GPU

**Date:** 2026-09-03 · **Status: PROVISIONAL** — strong match, tag not yet captured

**Previous suspects were wrong.** `laptop-ram-leak-2026-09-02.md` lists mirrored
networking / Hyper-V firewall first and WSLg GPU passthrough second. Neither fits.

[microsoft/WSL#40804](https://github.com/microsoft/WSL/issues/40804) reports an
`ntfs.sys` non-paged pool leak, pool tag **`NtFC`**, at ~21–23 GB/hr, driven by
**block-level I/O against the WSL2 `.vhdx`** — explicitly *not* the 9p `/mnt`
bridge and not networking — on **Windows 11 build 26200.x**. Microsoft closed it
**"not planned"**; only a reboot frees the pool.

This laptop runs **10.0.26200.9168**, has a **23.8 GB** `ext4.vhdx` that was
written hard all day (three workspaces, `colcon build`, package installs), and saw
**22.4 GB** of nonpaged pool invisible to Task Manager and cleared only by reboot.
Tag class, magnitude, build, invisibility and recovery all match.

**Consequence: this is not an argument against D1.** It would afflict any heavy
WSL2 use on this Windows build. It is managed, not fixed:
- `vmIdleTimeout=60000` is now the **most important** setting, not a memory tweak —
  the leak accrues only while the VM is doing disk I/O.
- Run `wsl --shutdown` at the end of every session.
- `memory=8GB` does **not** cap the pool; the pool is host kernel memory.
- Past ~20 GB mid-session, reboot Windows. Nothing else releases it.

**PROVISIONAL because** the tag was never captured (needs elevation). Before the
next reboot after a robot day, run `poolmon -b` from an **admin** shell:
`NtFC` confirms this; `NDIS`/`vmsw` would put networking back in the frame;
`Dxgk`/`VmBs` would implicate WSLg.

**Separate issue, already correctly diagnosed:** the same day's 100% RAM episode
was ordinary user-process exhaustion (an orphaned 8.4 GB Python child, another
stack running twice) with `vmmemWSL` at ~100 MB. Two different problems. Do not
conflate them.

---

## D7 — The ~3 s driver log gap: one candidate ruled out, question still open

**Date:** 2026-09-03 · **Status: OPEN** — not blocking

**Ruled out.** The driver contains a loop that sleeps exactly **3 seconds** and
then runs `pkill -f rviz2` (`src/manager/node_manager.cc:101-105`) — a seductive
fit for both the gap and the RViz deaths. It is **not** the cause: the flag it
tests, `is_pcap_end`, is set `true` only in `pcap_source.cc` (lines 637, 861), so
with `source_type: 1` it never fires. Recorded here so nobody rediscovers it.

**Also ruled out:** the UDP path (`SO_RCVTIMEO` and `select()` both use **20 ms**,
so a stall would log warnings at ~50 Hz, not go silent); any ~20 s periodic timer
(none exists in the SDK); a matching upstream issue (none filed).

**Leading hypothesis:** a logging artefact. `go2-up.sh` redirects the driver's
stdout to a file, which switches the C runtime from line- to **4 KB block-buffered**
output — producing exactly "silence, then a clump", paced by log volume rather
than by anything in the driver.

**How to close it (pre-flight R6):** run the driver in the **foreground,
unredirected**, and watch `ros2 topic hz /lidar_points` beside the log. Steady
10 Hz through a log gap ⇒ buffering, close the question. A genuine 3 s dropout ⇒
it is real; capture `/lidar_packets_loss` across the event. If you must redirect,
prefix with `stdbuf -oL`.

---

## D7b — Verification uses `cyclonedds ls` and Foxglove, not the `ros2` CLI

**Date:** 2026-09-03 · **Status: DECIDED** (from measurements, robot absent)

**`ros2 topic list` / `hz` / `echo` are not trustworthy instruments in this
distro.** Measured today: under `rmw_cyclonedds_cpp` they hang indefinitely —
with the new config, with the old config, and with **no `CYCLONEDDS_URI` at
all**; with the stock apt CycloneDDS as well as the custom build; with multicast
on or off; and even with a live C++ publisher on the wire, where `ros2 topic
echo` reports the misleading "topic does not appear to be published yet". Under
FastRTPS the same command works, and with `lo` pinned CycloneDDS works too.

**Root cause: DDS discovery is not working over `eth1`** (the office
network's DHCP address, which is what `setup.env.sh` falls back to when no
`192.168.123.x` interface exists). CycloneDDS traces cleanly through interface
selection and then goes silent; `cyclonedds ls` reports **"Entities discovered:
0"** while a C++ talker is publishing. A publisher publishes regardless of
listeners, so the talker "working" proves the transmit path, not discovery.
rclpy then hangs waiting on a graph query that never completes.

**Almost certainly an office-network property, not a setup fault** — `/lowstate`
was seen at ~500 Hz over the `192.168.123.0/24` robot link yesterday, which is
the easy case for multicast. Not cured; not narrowed further, because that would
mean changing Windows network settings.

**Consequences, and they are practical:**
- **Never run DDS checks before the robot is connected and `.99` is up.** A hang
  there means nothing. The pre-flight originally had a no-robot `talker`/`echo`
  dry-run that would have hung and wasted the morning; it is rewritten.
- **Instruments, in order:** Foxglove (C++ bridge, shows topics *and* rates —
  so D2 doubles as the introspection tool) → `cyclonedds ls`
  (`~/.local/bin/cyclonedds`; returns instead of hanging, so a zero count is a
  real answer) → `ddsperf` → `tcpdump -i any -n udp port 2368` (no DDS at all,
  and what actually proved the LiDAR link yesterday).
- **If `cyclonedds ls` sees the robot but the CLI still hangs**, the first thing
  to try is `<Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery>`.
  The trace shows `ParticipantIndex: none`, which disables unicast discovery —
  with multicast unavailable that leaves no discovery path at all.

**Side effect: the §3.2 CycloneDDS change is exonerated** — it behaves
identically to no config at all here. It remains unverified as a *fix* for the
fragmentation until the robot streams (R4).

---

## D8 — Where each service runs

**Date:** 2026-09-03 · **Status: DECIDED**

| Service | Runs on | Why |
| --- | --- | --- |
| `camera_server.py` (:8080), `dashboard.py` (:8090) | **Jetson** | Plain TCP, unaffected by DDS/WSL issues, and next to the cameras. Reachable from the Windows browser today. |
| `lidar_relay.py` | **Jetson** | The XT16's destination IP is `.18` and must never change. The relay repeats 2368/10110 to `.99`. |
| `hesai_ros_driver`, `static_transform_publisher`, `foxglove_bridge` | **WSL (laptop)** | ROS 2 Humble, where the workspace is built. |
| Foxglove Desktop (3D view) | **Windows, natively** | See D2. |
| 1 kHz torque control | **neither — Jetson or native Linux** | Never through WSL. |

**Note.** The services were moved to the Jetson because nothing served from WSL
seemed reachable from the Windows browser. That conclusion is now doubted — the
Hyper-V firewall has `LoopbackEnabled: True` by design, and Microsoft's own
guidance points at services bound to `127.0.0.1` instead of `0.0.0.0`. Pre-flight
P4 tests it. **Even if P4 passes, leave these on the Jetson** — the placement is
right on its own merits.

---

## Open questions carried forward

| # | Question | Where it gets answered |
| --- | --- | --- |
| 1 | Can Windows reach a TCP service inside WSL? **D2 depends on it.** | Pre-flight P4 — no robot needed |
| 2 | Does `MaxMessageSize=1400B` actually eliminate the fragmentation? | R4, with the robot streaming |
| 2b | Why does DDS discovery fail over the office network — and will it work over the robot link? | R4's `cyclonedds ls`. See D7b. |
| 3 | Is the 3 s gap real or buffering? | R6 |
| 4 | Is the pool tag really `NtFC`? | `poolmon -b`, admin, before the next reboot |
| 5 | Does the XT16 console offer 300 rpm (5 Hz)? Manual says 600/1200 only; the brochure implies 5 Hz exists. | Look at the dropdown |
| 6 | Does the Go2's 4G/eSIM give the Jetson an internet route? | `ip route` on the Jetson. Does not change D1. |
| 7 | Can the XT16 feed Unitree's `/uslam/*`? Almost certainly not — closed firmware tied to the built-in L1. | Untouched since day one |
| 8 | Root password on the Jetson never changed. | Housekeeping |

---

## D9 — CYCLONEDDS_URI must list the loopback interface `lo`

**Date:** 2026-09-03 · **Status: DECIDED** — verified by test, no robot needed

`CYCLONEDDS_URI` in `setup.env.sh` now lists **two** interfaces: the robot-facing
one (first) and **`lo` with `multicast="true"`** (second).

**This was the root cause of every unexplained failure on 2026-09-03.** With only
the robot interface listed, **two ROS 2 nodes on this same laptop could not
discover each other at all.** Local discovery relies on multicast looping back on
that interface, and WSL's mirrored networking does not loop multicast back to
local processes.

One fault, four symptoms — all of which looked like separate problems:

| Symptom | Actually was |
| --- | --- |
| `ros2 topic list` / `hz` / `echo` hang forever | no node could be discovered |
| `foxglove_bridge` deadlocks silently at startup (zero log output, stuck in `futex_do_wait`, never binds 8765) | blocked waiting on the node graph |
| rosbridge: `Service /rosapi/topics_and_raw_types does not exist` | rosbridge could not see `rosapi_node` running beside it |
| *(would have happened)* `ros2 bag record` producing **empty bags** | recorder could not discover the driver's topics |

**Why it stayed hidden.** Robot traffic (`/lowstate` at ~500 Hz, 2026-09-02)
crosses the physical wire and worked fine. Only *laptop-local* node-to-node
communication was broken, and nothing had needed that until the visualiser.

**Verified fix** (talker in one shell, listener in another — listener receives):

```xml
<Interfaces>
  <NetworkInterface name="${UNITREE_IFACE}" priority="default" multicast="default" />
  <NetworkInterface name="lo"               priority="default" multicast="true" />
</Interfaces>
```

Robot interface first so it stays primary for robot traffic.

**Rejected — `<Peers><Peer address="localhost"/></Peers>` with `AllowMulticast=false`.**
Tested; still broken. The loopback interface entry is what works.

**Rejected — switching to FastDDS.** foxglove_bridge worked under FastDDS, which is
what first (wrongly) suggested a CycloneDDS/foxglove_bridge incompatibility.
Unitree requires CycloneDDS, and once `lo` was added, foxglove_bridge worked
under CycloneDDS too. There was never an incompatibility.

**How to re-test in one minute** (no robot):

```bash
source ~/unitree_ws/setup.env.sh
ros2 run demo_nodes_cpp talker      # terminal 1
ros2 run demo_nodes_cpp listener    # terminal 2 -- must print "I heard"
```

If the listener hears nothing, `lo` has gone missing from `CYCLONEDDS_URI`.
**Supersedes D7b**, which advised avoiding `ros2 topic list` because it hung —
it hung for this reason, and now works.

---

## D10 — foxglove_bridge stays primary; rosbridge is the documented fallback

**Date:** 2026-09-03 · **Status: DECIDED** — connection verified, no robot needed

`go2-up.sh` starts **`foxglove_bridge`** (port **8765**, "Foxglove WebSocket"
connection type). `ros-humble-rosbridge-suite` is installed and works as a
fallback (port **9090**, "Rosbridge" connection type).

**Why foxglove_bridge.** It speaks Foxglove's binary protocol. rosbridge
serialises every message to JSON, which would choke on a 320,000 point/s cloud.
rosbridge was adopted mid-session only because foxglove_bridge appeared broken —
see D9; it was not.

**If the fallback is ever needed**, the two parameters are not optional: with
rosbridge's defaults Foxglove reports *"Failed to fetch topics from rosbridge"*.

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  call_services_in_new_thread:=true default_call_service_timeout:=5.0
```

**Confirmed working 2026-09-03:** Foxglove Desktop on Windows connected to WSL and
rendered its 3D panel. This also settles **open question 1 / pre-flight P4** —
Windows *can* reach a TCP service inside WSL. D2 is no longer provisional.

---

## D11 — The frame tree gets a moving link, and the LiDAR extrinsic comes from Unitree

**Date:** 2026-09-05 · **Status: DECIDED** — from the vendor docs and from measurement

The tree was two static transforms:

```
map --(static)--> odom --(static, z=0.30)--> hesai_lidar
```

It is now:

```
map --(static)--> odom --(MOVING)--> base_link --(static, vendor)--> hesai_lidar
```

**Why.** `odom` is a fixed world frame; `hesai_lidar` is bolted to a walking
dog. Fixing one rigidly to the other tells every consumer that the sensor never
moves, so every frame of a walk is drawn at the same world pose. Ten metres of
walking piles up in one spot. Both recorded walks have exactly this shape —
`/tf_static` with two transforms and no dynamic `/tf` anywhere — and it is why
the first recordings "were not the best". The failure is invisible: you still
get a point cloud, it is just the same one, 240 times.

**The static link is now the vendor's number.** From the SDK docs, SLAM and
Navigation Services Interface: *"The coordinates of the XT-16 lidar in the IMU
coordinate system inside the Go2 body are (0.1710, 0, 0.0908), without relative
rotation."* That replaces a guess of x=0, z=0.30 — 17 cm short in X and 3.3×
too high in Z. See `sdk-doc-review.md` §2. Next time the robot is here, measure
to **confirm** rather than to derive.

**The moving link is the vendor's odometry.** `rt/sportmodestate` carries a
fused legged + IMU pose at ~50 Hz, free and always published — the Odometer
Services Interface calls it the interface that "fuses legged (motor) data with
IMU integration data". `odom_publisher.py` republishes it as
`nav_msgs/Odometry` plus the `odom -> base_link` transform, live or from a
recording. `record_session.py` now saves it as `odom.csv`.

**Rejected — deriving the moving link by scan matching first.** RTAB-Map ICP on
the existing bag would also produce `odom -> base_link`, and it remains the
plan for walks recorded before 2026-09-05 that have no `odom.csv`. But it is
work, it drifts on a legged gait, and the robot hands us a usable answer for
nothing. Use the free one as the prior; scan-match on top of it later.

**The trap this introduced, found the same day.** Adding a link to the middle
of the tree means that when the middle link is *missing*, the tree is no longer
frozen — it is **broken in two**. `map -> odom` and `base_link -> hesai_lidar`
with nothing between them are two valid, disconnected trees, and a viewer asked
to display in `odom` cannot transform the cloud into it at all. It shows an
**empty panel**, which looks like the recording failed rather than like a
missing transform. The old two-static tree at least always joined up.

So: `walk_to_bag.sh` now publishes a **frozen identity `odom -> base_link`**
when a session has no `odom.csv`, loudly labelled, and kills it afterwards so it
cannot leak into a later live session and fight the real publisher. A
wrong-but-connected tree shows you your data; a broken one shows you nothing.
`inspect_bag.py` gained a **connectivity** check — counting transforms was never
enough, and the old "are there transforms?" check passed happily on the broken
tree. Live, `go2-start.sh` and `odom_publisher.py` both now say plainly that the
symptom is an empty panel and that `hesai_lidar` is the display frame to fall
back to.

**Consequence for Decay Time.** D2 kept it at 0 because 5 s at this point rate
killed rviz2. That reasoning stands, but with a *moving* transform a small
decay — around half a second — is now what makes the map build up on screen
rather than what floods the GPU. Raise it slowly.

**Also amended: D5's socket buffers.** Unitree set `net.core.rmem_max` **and**
`net.core.wmem_max` to `52428800` (50 MB) on the dock as a prerequisite for
their SLAM service. We set `rmem_max=16777216` and never set `wmem_max` at all.
`setup.env.sh` now warns when either is below the vendor value and prints the
command; it deliberately does not change a system setting by itself.

---

## D12 — Hesai's driver stays the only thing that decodes XT16 packets

**Date:** 2026-09-05 · **Status: DECIDED**

`walk_to_bag.sh` builds a bag by replaying the pcap through the vendor driver
over DDS and recording the result. That is slower and more fragile than parsing
the pcap ourselves and writing an mcap directly — the replay runs in real time,
and it has already failed four ways in one day (a live driver contaminating the
bag; two conversions truncating each other; the DDS interface vanishing when
the cable was pulled; a recorder killed before it wrote the file's terminator).

**We keep the vendor driver anyway.** A second implementation of Hesai's packet
format in this repo is one more thing to be subtly and silently wrong, and the
format is theirs, not ours. Instead the replay path is hardened — a lock, a
SIGTERM shutdown that waits, and a check that the finished file actually ends —
and the speed problem is addressed with the driver's **own** `play_rate_`
setting, exposed as `walk_to_bag.sh --speed N`, with the IMU and odometry
replays scaled to match.

**Revisit if** conversion time becomes the bottleneck in practice and `--speed`
turns out not to help. The gate for any replacement is that it must reproduce
the driver's own output on `Lab-loop-1` to float precision.

**Also from this:** the replay config now points at the SDK's shipped
`PandarXT-16_Firetime Correction File.csv`. The stock config leaves
`firetimes_path` as the placeholder string, so per-laser firing-time correction
was simply off. The angle calibration was already known to matter — a missing
one produced a completely flat cloud — and this is the same class of thing, one
level finer.
