# Go2 Studio

The Windows app for the Go2: connect, record a walk, download it, make the
map, open it, save a copy. No terminal.

Double-click **`Go2 Studio.vbs`** (or run `install_shortcut.ps1` once for a
desktop and Start-menu shortcut). A window opens; that is all.

## What it needs

| | |
| --- | --- |
| Python for Windows 3.10+ | from python.org, with *Add python.exe to PATH* ticked. Studio uses the standard library only |
| Microsoft Edge | already on every Windows 11 machine; Studio's window is Edge in app mode |
| A converting engine | **Docker Desktop** on any laptop (uses the image from `docker/`), or the developer WSL distro `unitree` on the development laptop. Settings, Converting engine, picks automatically |
| Foxglove, CloudCompare | optional, for the Open buttons. Found automatically once installed |
| The robot's dashboard, 2026-09-25 or later | for Download, Set clock and Robot logs. Copy it to the robot once with `notes/deploy_dashboard.sh` |

## Pages

| Page | For |
| --- | --- |
| Connect | the five connection checks plus the clock, each with its fix. Sets the laptop's robot address with one Windows permission prompt, or shows how by hand |
| Record | the robot's pre-flight checks, Start walk with name, building, floor and notes, the depth camera switch with its resolution, frame rate and colour stream (Apply restarts a running camera at a new setting), live counters, the walk's trail with distance back to the start, markers, Stop and the walk report |
| Walks | every walk on the robot and on this laptop, with its five stages: on the robot, downloaded, bag, map, clean cloud. Download, Make the map, Cancel, Show in folder, Save a copy, Open in Foxglove or CloudCompare, delete. After a run: Map in Foxglove, Colour cloud (or Clean cloud), Floor plans (pictures in Details and files) and Mesh |
| Live | the robot's own dashboard (`/dashboard-v2`) inside Studio: 3D model, sensors, actions, logs. Its Record tab is hidden there, because Studio records |
| Help | the problems this project has met, searchable |
| Settings | walks folder, engine, what to keep, programs, robot address, developer logs |

**The robot pill and the Next step bar.** The status bar starts with *Robot
connected*, *Robot not connected* or *Out of reach, still recording*. Under it, on
every page, the Next step bar follows the session and offers the one thing to do:
plug in, fix the address, (the clock is set automatically), record, stop (a walk
started anywhere, including the robot's own page, is followed), download and
make the map of a walk still only on the robot, watch the conversion, open the
result, or retry a failure. Settings, After a walk, can make Studio download and
convert new walks by itself.
The x on the bar hides that one message until it changes; Settings, Next
step bar, chooses Always, Only when it needs me (no plain status), or Never.

**Developer logs** (the `<>` button, or Ctrl+Shift+D) opens a console at the
bottom: everything Studio does, every request to the robot with its timing, the
live output of a conversion, the robot's own service logs, and system info.
**Export debug bundle** zips all of it into Downloads.

**Camera colour is opt-in.** Colouring the clean cloud from the camera is an
experiment that adds 5-10 minutes a walk, so it is off unless the *Camera
colour* check mark beside Make the map (or Settings, Camera colour) is ticked.
Studio then passes `--colour` to `notes/process_walk.sh`; without it the colour
of an earlier build is removed, since it no longer lines up with the new cloud.

**Progress.** The processing scripts print one kind of progress line
(`notes/progress.py`): `~ colouring the cloud: 600/1500 frames`, or `~ building
the mesh surface` for a phase with no count. Studio turns them into the bar and
the line under the walk (`DENSE_PARTS` in `core/jobs.py` places each part in its
stage). A new long step only needs to print those lines to show up.

## How it works

```
Edge window (static/)  ->  studio.py on 127.0.0.1:8421 (core/)  ->  robot dashboard :8090
                                        |
                                        +-> Docker or WSL: notes/process_walk.sh
                                        +-> Explorer, folder dialog, Foxglove, CloudCompare
```

| File | Does |
| --- | --- |
| `studio.py` | starts the server and the window; quits a minute after the last window closes, unless a job is still running |
| `core/server.py` | the HTTP API; local only, and every POST needs the `X-Go2-Studio` header |
| `core/robot.py` | talks to the robot's dashboard; polls it once a second in the background |
| `core/jobs.py` | download, convert, save a copy; reads progress from the scripts' own output |
| `core/walks.py` | what is on this laptop, read off the files; checks from `notes/walk_library.py` |
| `core/winos.py` | Explorer, the folder dialog, programs, the admin prompt, engine detection |
| `core/applog.py` | the developer log, in memory and in `%LOCALAPPDATA%\Go2Studio\logs` |
| `static/` | the page: `index.html`, `app.css`, `app.js` |
| `mockup/` | the approved design mockup, fake data |

Settings live in `%APPDATA%\Go2Studio\settings.json`; logs, job history and
Studio's Edge profile in `%LOCALAPPDATA%\Go2Studio`.

For development: `python studio.py --no-window --stay`, then open
http://127.0.0.1:8421 in any browser.

## Tested (2026-09-25, without the robot)

* Make the map through WSL on `odom-test-1`: bag, map and clean cloud in 391 s,
  track closing to 0.17 m, progress read live from the scripts.
* Download from a stand-in robot (the real dashboard running in WSL): 78 MB at
  104 MB/s, unpacked to a normal walk folder with the LiDAR calibration.
* Save a copy; refusal of paths outside the walk; refusal of POSTs without the
  header; robot logs, and refusal of unknown log names.
* Pre-flight checks drawn from the stand-in's real answers.

**Not yet tested, needs the robot:** Start and Stop walk, the live counters
and trail, Set clock, the depth camera switch, the admin address prompt, the
Docker engine on a second laptop.
