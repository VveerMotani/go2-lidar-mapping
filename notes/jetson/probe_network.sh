#!/usr/bin/env bash
# What networks can this robot be reached on?
#
#     bash probe_network.sh
#
# Run this ON THE JETSON. It changes nothing -- every command below reads.
#
# WHY
# ---
# The dashboard is served by the Jetson and binds 0.0.0.0, so it answers on
# every address the Jetson has. The question this script exists to answer is how
# many addresses that is.
#
# Reaching the dashboard without the ethernet cable needs one of:
#
#   1. a wireless interface on the Jetson itself, either joined to a network or
#      running as an access point; or
#   2. a route from whatever network the robot's Wi-Fi puts you on (the phone
#      app's network) through to 192.168.123.18.
#
# Which of those is available is a property of this particular robot's wiring
# and firmware, not something worth guessing at from a laptop. So: measure.
#
# Deliberately NOT `set -u` or `set -e`. Several commands here are expected to
# be missing on a given image, and a probe that stops at the first absent tool
# tells you nothing. See the same note in go2-start.sh.

echo "=============================================================="
echo " hostname : $(hostname)"
echo " kernel   : $(uname -srm)"
echo " os       : $(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
echo "=============================================================="

echo
echo "--- interfaces and addresses ---------------------------------"
# The dashboard's own /health reports this list too; here it comes with the
# link state, which /health does not show.
ip -br addr 2>/dev/null || ifconfig -a 2>/dev/null || echo "  no ip/ifconfig"

echo
echo "--- routes ---------------------------------------------------"
ip route 2>/dev/null || echo "  no ip route"

echo
echo "--- wireless -------------------------------------------------"
# Three ways of asking, because which are installed varies by image. If all
# three come back empty the Jetson has no wireless interface of its own, and
# option 1 above is out.
if command -v iw >/dev/null 2>&1; then
  iw dev 2>/dev/null | sed 's/^/  /'
else
  echo "  iw: not installed"
fi
if command -v nmcli >/dev/null 2>&1; then
  echo "  nmcli devices:"
  nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device 2>/dev/null | sed 's/^/    /'
else
  echo "  nmcli: not installed"
fi
if command -v iwconfig >/dev/null 2>&1; then
  iwconfig 2>/dev/null | grep -v "no wireless extensions" | sed 's/^/  /'
fi
# A wireless chip may be present but with no driver bound, which looks the same
# as absent from userspace. This distinguishes them.
echo "  wireless devices known to the kernel:"
ls -1 /sys/class/net/*/wireless 2>/dev/null | sed 's/^/    /' || true
[ -z "$(ls -1 /sys/class/net/*/wireless 2>/dev/null)" ] && echo "    none"

echo
echo "--- who is listening -----------------------------------------"
# 8090 dashboard, 8080 cameras, 2368 LiDAR relay. All should show 0.0.0.0 or
# *, meaning "every interface". Anything bound to 192.168.123.18 specifically
# would be reachable over the cable only, which is exactly the fault this whole
# exercise is about.
if command -v ss >/dev/null 2>&1; then
  ss -tulnp 2>/dev/null | grep -E ':(8080|8090|2368|9347)\b' | sed 's/^/  /'
  echo "  (all of the above should say 0.0.0.0 or *, not 192.168.123.18)"
else
  netstat -tulnp 2>/dev/null | grep -E ':(8080|8090|2368|9347)\b' | sed 's/^/  /'
fi

echo
echo "--- firewall -------------------------------------------------"
# An open port that a filter drops looks identical to a closed one from outside.
command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | sed 's/^/  /'
iptables -S 2>/dev/null | head -20 | sed 's/^/  /' || echo "  iptables: not readable without root"

echo
echo "--- forwarding -----------------------------------------------"
# If the robot's Wi-Fi lands you on a different subnet, reaching 192.168.123.18
# from there needs forwarding enabled somewhere on the path.
echo "  net.ipv4.ip_forward = $(cat /proc/sys/net/ipv4/ip_forward 2>/dev/null)"

echo
echo "=============================================================="
echo " What to do with this"
echo "=============================================================="
echo " A wireless interface with an address  -> browse to that address:8090."
echo " A wireless interface with no address  -> it exists but is not joined;"
echo "                                          worth configuring."
echo " No wireless interface at all          -> the Jetson is wired-only. Reach"
echo "                                          it from the robot's own Wi-Fi"
echo "                                          network instead, which needs a"
echo "                                          route to 192.168.123.18."
echo
