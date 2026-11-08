#!/usr/bin/env bash
# Start the robot-side services on the Jetson.
#
# Run it from the laptop by piping it in, which avoids every shell-quoting
# problem that comes with putting a multi-line command inside an ssh argument:
#
#   sshpass -p "$GO2_PASSWORD" ssh -o StrictHostKeyChecking=no unitree@192.168.123.18 \
#       'bash -s' < notes/jetson/start_services.sh
#
# WHY THIS EXISTS
# ---------------
# go2-start.sh starts these three with `sshpass ... ssh -f "<multi-line command>"`
# and sends that step's errors to /dev/null. Two things go wrong there:
#
#   * sshpass and `ssh -f` do not combine reliably -- ssh puts itself into the
#     background around authentication time and sshpass loses the channel it
#     was going to type the password into.
#   * because stderr is discarded, when it fails it fails silently. The script
#     carries on and brings the laptop side up, so it looks like it worked
#     while nothing is actually running on the robot.
#
# Observed on 2026-09-04: laptop side fully up, robot side completely absent.
#
# Piping a real script in sidesteps both. Every service is started with setsid,
# nohup and all three streams redirected, so it survives the ssh session
# closing.

set -u

LAPTOP="${1:-192.168.123.99}"
DEST="$HOME/go2dev"

# unitree_sdk2py needs the CycloneDDS that was built on this machine.
# PYTHONUNBUFFERED keeps the logs readable -- without it Python block-buffers
# into the log file and output arrives in clumps minutes late.
export CYCLONEDDS_HOME="$HOME/cyclonedds_ws/install/cyclonedds"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1

cd "$DEST" || { echo "no $DEST on this machine"; exit 1; }

start() {
  local name="$1"; shift
  if pgrep -f "$name" >/dev/null 2>&1; then
    echo "  already running   $name"
    return 0
  fi
  # setsid detaches from this ssh session's process group, so closing the
  # connection does not take the service down with it.
  setsid nohup "$@" > "/tmp/${name%.py}.log" 2>&1 < /dev/null &
  sleep 1
  if pgrep -f "$name" >/dev/null 2>&1; then
    echo "  started           $name"
  else
    echo "  FAILED            $name -- last lines of /tmp/${name%.py}.log:"
    tail -n 6 "/tmp/${name%.py}.log" 2>/dev/null | sed 's/^/                    /'
  fi
}

echo "starting services in $DEST"
start camera_server.py python3 camera_server.py
start lidar_relay.py   python3 lidar_relay.py --dest "$LAPTOP"
start dashboard.py     python3 dashboard.py --iface eth0 --domain 0

sleep 3
echo
echo "running now:"
pgrep -af "camera_server|lidar_relay|dashboard" | sed 's/^/  /' || echo "  nothing"
echo
echo "answering:"
curl -s -m 4 -o /dev/null -w "  cameras   :8080 -> %{http_code}\n" http://localhost:8080/stats
curl -s -m 4 -o /dev/null -w "  dashboard :8090 -> %{http_code}\n" http://localhost:8090/health
