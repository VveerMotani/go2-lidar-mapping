#!/usr/bin/env bash
# Start the robot-side services at boot, so nobody has to ssh in.
#
# Run once, on the Jetson, piped in from the laptop the same way as
# start_services.sh:
#
#   sshpass -p "$GO2_PASSWORD" ssh -o StrictHostKeyChecking=no unitree@192.168.123.18 \
#       'bash -s' < notes/jetson/install_autostart.sh
#
#   ... 'bash -s -- --uninstall' < notes/jetson/install_autostart.sh
#   ... 'bash -s -- --status'    < notes/jetson/install_autostart.sh
#
# Or, once deploy_dashboard.sh has copied it across, on the Jetson itself:
#   bash ~/go2dev/install_autostart.sh [--uninstall|--status]
#
# WHY
# ---
# Until now the camera server, LiDAR relay and dashboard ran only because
# go2-start.sh started them over ssh, which needs the WSL laptop. With this
# unit they are up about half a minute after the robot is switched on, and a
# laptop with nothing but a browser and a cable can record a walk: the
# dashboard sets the robot's clock and hands walks back as zips.
#
# HOW, and why this shape
# -----------------------
# The unit runs start_services.sh unchanged, rather than one unit per service,
# so there is still exactly one description of how the services are started
# and go2-start.sh keeps working: start_services.sh skips anything already
# running, so a laptop that runs go2-start.sh after boot changes nothing.
#
# start_services.sh launches each service with `setsid nohup ... &` and exits
# after about fifteen seconds of checks. That rules out the obvious types:
#
#   Type=simple   systemd would treat the script's exit as the service ending,
#                 and clean up everything it started.
#   Type=forking  expects one daemon whose PID it can track. There are three,
#                 and systemd would have to guess which one is "main".
#
# Type=oneshot with RemainAfterExit=yes is the fit: systemd waits for the script
# to finish, then keeps the unit "active (exited)". Because the unit stays
# active, the processes the script left behind -- which setsid does NOT move out
# of the unit's cgroup -- are left alone, and `systemctl stop go2dev` still
# stops all three, because they are in that cgroup.
#
# What it does not give: restart on crash. A dead dashboard stays dead until
# the next boot or `sudo systemctl restart go2dev`, exactly as before.
#
# The unit waits for eth0 to have an address before starting, because the
# dashboard's DDS is bound to eth0 and cannot start without it. It waits up to
# a minute and then starts regardless, so the unit never blocks boot.
#
# Nothing else about the robot changes. --uninstall removes the unit and leaves
# the running services alone.

set -u

UNIT=go2dev.service
UNIT_PATH=/etc/systemd/system/$UNIT
RUN_AS="$(id -un)"
DEST="$HOME/go2dev"
PASSWORD="${GO2_PASSWORD:?set GO2_PASSWORD to the password of the robot computer}"

# `bash -s` means this script arrives on stdin, so `sudo -S` must not read the
# password from there -- it would eat the rest of the script. Give it its own.
root() { printf '%s\n' "$PASSWORD" | sudo -S -p "" "$@"; }

if [ "$RUN_AS" = "root" ]; then
  echo "run this as the unitree user, not as root: the unit runs as whoever installs it"
  exit 1
fi

root true 2>/dev/null || { echo "sudo refused the password (set GO2_PASSWORD)"; exit 1; }

case "${1:-}" in
  --uninstall)
    root systemctl disable "$UNIT" >/dev/null 2>&1
    root rm -f "$UNIT_PATH"
    root systemctl daemon-reload
    echo "removed $UNIT. The services running now are left running;"
    echo "they will not start at the next boot."
    exit 0
    ;;
  --status)
    systemctl status "$UNIT" --no-pager 2>&1 | head -n 20
    exit 0
    ;;
  "")
    ;;
  *)
    echo "usage: install_autostart.sh [--uninstall|--status]"; exit 1
    ;;
esac

if [ ! -f "$DEST/start_services.sh" ]; then
  echo "no $DEST/start_services.sh -- run notes/deploy_dashboard.sh from the laptop first"
  exit 1
fi

TMP="$(mktemp)"
cat > "$TMP" <<EOF
# Installed by install_autostart.sh -- see that file for why it looks like this.
[Unit]
Description=Go2 robot-side services: cameras, LiDAR relay, dashboard (:8090)
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
User=$RUN_AS
WorkingDirectory=$DEST
Environment=HOME=$HOME
# dashboard.py needs sudo for packet capture and reads the password from here.
Environment=GO2_PASSWORD=$PASSWORD
# eth0 carries DDS; give it up to a minute to get its address, then go anyway.
# "\$\$" is systemd's escape for a literal dollar sign.
ExecStartPre=/bin/sh -c 'for i in \$\$(seq 60); do ip -4 addr show eth0 2>/dev/null | grep -q inet && exit 0; sleep 1; done; exit 0'
# The leading "-": start_services.sh ends with a curl to each service, which
# fails if the dashboard is still starting its DDS. A failed oneshot is stopped,
# and stopping it would kill the very services it had just started.
ExecStart=-/bin/bash $DEST/start_services.sh
TimeoutStartSec=180
# No PrivateTmp: the services log to /tmp, the dashboard and the root walk
# recorder share a stop file there, and go2-start.sh reads those logs over ssh.

[Install]
WantedBy=multi-user.target
EOF

root install -m 644 -o root -g root "$TMP" "$UNIT_PATH" || { rm -f "$TMP"; echo "could not write $UNIT_PATH"; exit 1; }
rm -f "$TMP"
root systemctl daemon-reload
root systemctl enable "$UNIT" 2>&1 | sed 's/^/  /'
echo "installed $UNIT_PATH (runs $DEST/start_services.sh as $RUN_AS at boot)"

# Start it now too. start_services.sh skips whatever is already running, so
# this is harmless when go2-start.sh has brought things up already, and it
# proves the unit works without waiting for a reboot.
echo "starting it now ..."
root systemctl start "$UNIT"
systemctl --no-pager --lines=0 status "$UNIT" 2>&1 | head -n 5 | sed 's/^/  /'
echo
echo "the start script's own report:"
root journalctl -u "$UNIT" -b --no-pager -n 20 -o cat 2>/dev/null | sed 's/^/  /'
