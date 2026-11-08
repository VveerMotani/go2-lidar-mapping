#!/usr/bin/env bash
# Copy a recorded walk off the robot into E:\Unitree\walks\.
#
#   ~/go2-lidar-mapping/notes/fetch_walk.sh                 list what is on the robot
#   ~/go2-lidar-mapping/notes/fetch_walk.sh Lab-loop-1      copy that one
#   ~/go2-lidar-mapping/notes/fetch_walk.sh --all           copy everything
#
# Walks live on the robot's own disk while recording -- that is what lets you
# unplug and walk away. This brings them back to the laptop, where they are on
# E: and therefore survive rebuilding the WSL distro.
#
# Afterwards, turn one into something Foxglove can scrub:
#
#   ~/go2-lidar-mapping/notes/walk_to_bag.sh ~/go2-lidar-mapping/walks/Lab-loop-1

set -o pipefail

JETSON=192.168.123.18
REMOTE=go2dev/sessions
# Default: the repo's walks/ folder, beside notes/, wherever the repo lives.
DEST="${GO2_WALKS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/walks}"

SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
SSH_OPTS="$SSH_OPTS -o ConnectTimeout=8 -o LogLevel=ERROR"
SSH="sshpass -p $GO2_PASSWORD ssh $SSH_OPTS unitree@$JETSON"
SCP="sshpass -p $GO2_PASSWORD scp $SSH_OPTS"

ok()  { printf "  \033[32m/\033[0m %s\n" "$1"; }
bad() { printf "  \033[31mX\033[0m %s\n" "$1"; }

ping -c1 -W2 "$JETSON" >/dev/null 2>&1 || {
  bad "cannot reach the robot at $JETSON -- is the cable in?"; exit 1; }

mkdir -p "$DEST" || { bad "cannot create $DEST"; exit 1; }

WALKS=$($SSH "ls -1 $REMOTE 2>/dev/null" 2>/dev/null)
if [ -z "$WALKS" ]; then
  echo "no walks on the robot yet."
  echo "record one from the dashboard: http://$JETSON:8090"
  exit 0
fi

# --- no argument: just report what is where -------------------------------
if [ $# -eq 0 ]; then
  echo "on the robot:"
  $SSH "cd $REMOTE && du -sh */ 2>/dev/null" 2>/dev/null | sed 's/^/  /'
  echo
  echo "already copied to $DEST:"
  ls -1 "$DEST" 2>/dev/null | sed 's/^/  /' || echo "  (none)"
  echo
  echo "copy one with:  fetch_walk.sh <name>     or  fetch_walk.sh --all"
  exit 0
fi

if [ "$1" = "--all" ]; then
  TARGETS="$WALKS"
else
  TARGETS="$*"
fi

for name in $TARGETS; do
  if ! echo "$WALKS" | grep -qx "$name"; then
    bad "$name is not on the robot"
    continue
  fi
  if [ -d "$DEST/$name" ]; then
    ok "$name already in $DEST -- skipping (delete it to re-copy)"
    continue
  fi

  SIZE=$($SSH "du -sh $REMOTE/$name 2>/dev/null | cut -f1" 2>/dev/null)
  echo "copying $name ($SIZE) ..."
  # The files are root-owned on the robot but world-readable, so an ordinary
  # scp as the unitree user can read them.
  if $SCP -r "unitree@$JETSON:$REMOTE/$name" "$DEST/" >/dev/null 2>&1; then
    ok "$DEST/$name"
    ls -la "$DEST/$name" | tail -n +2 | awk '{printf "      %8s  %s\n", $5, $9}'
  else
    bad "copy of $name failed"
  fi
done

echo
echo "next:  ~/go2-lidar-mapping/notes/walk_to_bag.sh $DEST/<name>"
