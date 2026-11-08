#!/usr/bin/env bash
# Copy dashboard v2 to the robot's Jetson and restart it there.
#
#   deploy_dashboard.sh              copy everything, restart, verify
#   deploy_dashboard.sh --code-only  skip the 4 MB mesh blob and three.js
#   deploy_dashboard.sh --restart    restart what is already there
#   deploy_dashboard.sh --stop       stop it
#
# Follows the same shape as go2-start.sh: ssh in with sshpass, nothing is
# permanently installed, the robot stays as the factory shipped it.
#
# WHAT GETS TRANSFERRED, and why each file is needed:
#
#   dashboard.py                     the server
#   dashboard_static/index.html      the page
#   dashboard_static/camera.html     the standalone camera page (/camera)
#   dashboard_static/app.css         styles for both
#   dashboard_static/app.js          panel wiring, polling, settings
#   dashboard_static/robot3d.js      the three.js scene
#   dashboard_static/charts.js       the history graphs
#   record_session.py                the walk recorder the page starts and stops
#   start_services.sh                starts cameras, LiDAR relay and dashboard
#   install_autostart.sh             run once: makes start_services.sh run at boot
#   dashboard_static/go2_model.json  link tree, joint axes, mesh offsets
#   dashboard_static/go2_meshes.bin  4.1 MB of geometry
#   dashboard_static/vendor/three.module.js
#   dashboard_static/vendor/OrbitControls.js
#
# The two vendor files are why there is no CDN anywhere in the page: the Jetson
# has no internet, and the page must load with the laptop offline too.
#
# The Jetson runs Python 3.8 with no pip access, so dashboard.py imports only
# the standard library plus unitree_sdk2py, which is already installed there.

set -o pipefail

JETSON=192.168.123.18
DEST=/home/unitree/go2dev
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8"
SSH="sshpass -p $GO2_PASSWORD ssh $SSH_OPTS unitree@$JETSON"
SCP="sshpass -p $GO2_PASSWORD scp $SSH_OPTS"

ok()  { printf "  \033[32m✓\033[0m %s\n" "$1"; }
bad() { printf "  \033[31m✗\033[0m %s\n" "$1"; }

# The Jetson's clock resets to 1970 at every boot (no RTC, no NTP). scp itself
# copes, but a wrong clock makes log timestamps useless and breaks pip, so sync
# it here the same way go2-start.sh does.
sync_clock() {
  $SSH "echo $GO2_PASSWORD | sudo -S date -s @$(date +%s) >/dev/null 2>&1" >/dev/null 2>&1
  ok "robot clock: $($SSH 'date' 2>/dev/null)"
}

stop_it() {
  $SSH "pkill -f dashboard.py" >/dev/null 2>&1
  sleep 1
  ok "dashboard stopped"
}

start_it() {
  # The commands are piped to a shell on the robot, the same way go2-start.sh
  # runs jetson/start_services.sh. This used to be `sshpass ... ssh -f "..."`,
  # which is the combination that silently started nothing in go2-up.sh: on
  # 2026-09-17 a deploy stopped the running dashboard, "started" the new one,
  # and left no process and not even a log file behind.
  #
  # unitree_sdk2py needs the CycloneDDS that was built on the Jetson.
  # --iface eth0 is the robot-facing interface there; --domain 0 is the robot.
  # setsid and the redirects let ssh return without taking the process with it.
  $SSH "bash -s" <<EOF >/dev/null 2>&1
export CYCLONEDDS_HOME="\$HOME/cyclonedds_ws/install/cyclonedds"
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib:\${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1
cd "$DEST" || exit 1
setsid nohup python3 dashboard.py --iface eth0 --domain 0 > /tmp/dashboard.log 2>&1 < /dev/null &
EOF
  sleep 5
}

verify() {
  local body
  body=$($SSH "curl -s -m 5 http://localhost:8090/health" 2>/dev/null)
  if [ -z "$body" ]; then
    bad "dashboard is not answering on :8090"
    echo "  last lines of /tmp/dashboard.log on the Jetson:"
    $SSH "tail -n 15 /tmp/dashboard.log" 2>/dev/null | sed 's/^/    /'
    return 1
  fi
  ok "http://$JETSON:8090  (also /camera, /data, /health)"
  echo "$body" | sed 's/^/    /'

  # A running server with no DDS data is the most common half-broken state, so
  # call it out rather than letting it look like success.
  case "$body" in
    *'"status": "ok"'*|*'"status":"ok"'*) ok "receiving DDS from the robot" ;;
    *) bad "serving, but no DDS data yet -- is the robot on domain 0 and eth0 the right interface?" ;;
  esac
}

copy_all() {
  local code_only="$1"
  $SSH "mkdir -p $DEST/dashboard_static/vendor" >/dev/null 2>&1 || {
    bad "cannot reach the Jetson at $JETSON"; exit 1; }

  $SCP "$SRC/dashboard.py" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
  ok "dashboard.py"

  # The dashboard's "Record a walk" panel starts this, so it has to sit next to
  # dashboard.py. Without it the panel appears but says it is unavailable.
  if [ -f "$SRC/jetson/record_session.py" ]; then
    $SCP "$SRC/jetson/record_session.py" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
    ok "record_session.py"
  else
    bad "jetson/record_session.py missing -- walk recording will be unavailable"
  fi

  # Standard-library only, so it runs on the Jetson's Python 3.8. The walk
  # library panel reports itself unavailable there (processing needs the Hesai
  # driver, which is on the laptop), but the same checks are what the recording
  # panel will use to give a verdict the moment a walk is stopped.
  # The Sensors panel's switches. Without it the panel says it is unavailable.
  if [ -f "$SRC/sensors.py" ]; then
    $SCP "$SRC/sensors.py" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
    ok "sensors.py"
  fi

  if [ -f "$SRC/walk_library.py" ]; then
    $SCP "$SRC/walk_library.py" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
    ok "walk_library.py"
  fi

  # Read-only network probe, used to work out how the robot can be reached
  # without the cable. Copied here so it is on the Jetson when it is wanted,
  # rather than needing a second transfer at the moment of confusion.
  # The depth camera's robot-side tools. The check is read-only; the installer
  # follows Intel's documented Jetson steps and is only run by hand, with
  # internet on the robot. Copying them here means they are already on the
  # robot when the camera work starts.
  for tool in realsense_check.sh realsense_install.sh realsense_start.sh realsense_ros1_relay.py lidar_relay.py depth_recorder.py; do
    if [ -f "$SRC/jetson/$tool" ]; then
      $SCP "$SRC/jetson/$tool" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
      ok "$tool"
    fi
  done

  # What starts the services at boot, and the three things it starts that are
  # not already copied above. start_services.sh runs them from this folder, so
  # the go2dev.service that install_autostart.sh installs (run once, by hand)
  # needs every one of them here. camera_server.py used to be copied by hand.
  for tool in start_services.sh install_autostart.sh camera_server.py; do
    if [ -f "$SRC/jetson/$tool" ]; then
      $SCP "$SRC/jetson/$tool" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
      ok "$tool"
    fi
  done

  if [ -f "$SRC/jetson/probe_network.sh" ]; then
    $SCP "$SRC/jetson/probe_network.sh" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
    ok "probe_network.sh"
  fi

  # The XT16's per-laser angle calibration. The robot never uses it -- it is
  # needed on the laptop when a recording is replayed -- but the pre-flight
  # strip on the robot's dashboard checks for it, and without a copy beside
  # dashboard.py that chip is red on every walk for a file that exists and is
  # fine. A check that is always red teaches you to ignore it.
  if [ -f "$SRC/../walks/xt16_correction.csv" ]; then
    $SCP "$SRC/../walks/xt16_correction.csv" "unitree@$JETSON:$DEST/" >/dev/null || exit 1
    ok "xt16_correction.csv"
  fi

  $SCP "$SRC"/dashboard_static/*.html "$SRC"/dashboard_static/*.css \
       "$SRC"/dashboard_static/*.js \
       "unitree@$JETSON:$DEST/dashboard_static/" >/dev/null || exit 1
  ok "page, styles and scripts"

  # Dashboard v2 (/dashboard-v2) and the two small libraries it needs. The
  # robot has no internet, so they are served from here like three.js is.
  $SSH "mkdir -p $DEST/dashboard_static/v2" >/dev/null 2>&1
  $SCP "$SRC"/dashboard_static/v2/* "unitree@$JETSON:$DEST/dashboard_static/v2/" >/dev/null || exit 1
  $SCP "$SRC"/dashboard_static/vendor/gridstack* "$SRC"/dashboard_static/vendor/uPlot* \
       "unitree@$JETSON:$DEST/dashboard_static/vendor/" >/dev/null || exit 1
  ok "dashboard v2, GridStack and uPlot"

  # What is deployed, for the System page: the commit, and whether the files
  # differed from it. Written here, on the laptop side, because the robot has
  # no git and no copy of the repo.
  local ver
  ver="$(git -C "$SRC/.." rev-parse --short HEAD 2>/dev/null || echo unknown)"
  git -C "$SRC/.." diff --quiet -- notes 2>/dev/null || ver="$ver+changes"
  echo "$(date +%Y-%m-%d) $ver" | $SSH "cat > $DEST/VERSION" >/dev/null 2>&1 && ok "version $ver"

  if [ "$code_only" = "yes" ]; then
    ok "skipped the mesh blob and three.js (--code-only)"
    return
  fi

  # These two are large and never change unless make_model.py is re-run or
  # three.js is re-vendored, which is what --code-only is for.
  $SCP "$SRC/dashboard_static/go2_model.json" "$SRC/dashboard_static/go2_meshes.bin" \
       "unitree@$JETSON:$DEST/dashboard_static/" >/dev/null || exit 1
  ok "robot model  ($(du -h "$SRC/dashboard_static/go2_meshes.bin" | cut -f1))"

  $SCP "$SRC"/dashboard_static/vendor/three.module.js "$SRC"/dashboard_static/vendor/OrbitControls.js \
       "unitree@$JETSON:$DEST/dashboard_static/vendor/" >/dev/null || exit 1
  ok "three.js and OrbitControls"
}

case "${1:-}" in
  --stop)
    stop_it
    ;;
  --restart)
    stop_it; start_it; verify
    ;;
  --code-only|"")
    for f in dashboard.py dashboard_static/index.html dashboard_static/app.js \
             dashboard_static/robot3d.js dashboard_static/go2_model.json \
             dashboard_static/vendor/three.module.js; do
      [ -f "$SRC/$f" ] || { bad "missing $f -- run make_model.py first?"; exit 1; }
    done
    echo "deploying to $JETSON:$DEST"
    sync_clock
    copy_all "$([ "${1:-}" = "--code-only" ] && echo yes || echo no)"
    stop_it
    start_it
    verify
    ;;
  *)
    echo "usage: deploy_dashboard.sh [--code-only|--restart|--stop]"; exit 1
    ;;
esac
