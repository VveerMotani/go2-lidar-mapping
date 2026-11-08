"""
List and switch the Go2's internal services.

    python3 service_switch.py --list
    python3 service_switch.py --off obstacles_avoidance
    python3 service_switch.py --on  video_hub
    python3 service_switch.py --off chat_go --off vui_service --dry-run

Runs on the laptop, over DDS. Nothing is installed on the robot.

WHY YOU WANT THIS
-----------------
Three jobs, all from the Go2 SDK docs:

  * Unitree's own SLAM and navigation service requires the robot's built-in
    obstacle avoidance to be OFF ("Turn off the obstacle avoidance mode that
    comes with the robot, and the head light will turn blue" -- SLAM and
    Navigation Services Interface).

  * During a recording, services we are not using are competing for the
    robot's CPU and its radio. chat_go, vui_service and webrtc_bridge are the
    obvious candidates.

  * Before ANY low-level control, the main motion control service has to be
    stopped or two programs fight over the legs. The Quick Start page is blunt
    about the consequence: "If multiple motion controls are active
    simultaneously, the Go2 robot will receive two or more control commands,
    leading to confusion and potential loss of control."

    Note which name: mcf on software >= 1.1.6, sport_mode below it. Ours is
    above, so it is mcf. Switching the wrong one silently does nothing, and
    you find out when the robot fights you.

WHAT THIS DELIBERATELY DOES NOT DO
----------------------------------
It refuses to touch the motion control services unless you pass --i-mean-it,
and it prints what standing on three legs looks like first. Turning off mcf
while the robot is standing drops it. That is a real robot, on a real floor,
with real legs.

SAFETY: --dry-run
-----------------
Default behaviour for anything destructive is to say what it would do. The
robot does not have to be present for --dry-run to be useful; it is how you
check the spelling of a service name before the robot is in front of you.

The full service list, from the RobotStateClient Interface page:

    mcf                  main motion control  (software >= 1.1.6)
    sport_mode           main motion control  (software <  1.1.6)
    basic_service        underlying service
    obstacles_avoidance  obstacle avoidance
    unitree_lidar        the built-in L1 radar service
    video_hub            video source
    webrtc_bridge        WebRTC (what the phone app holds open)
    vui_service          offline voice
    chat_go              "clumsy dog" service
    net_switcher         network switch
    ota_box              firmware upgrade
    utrack               accompanying / follow service

Error 5202 from the robot means the service is protected and cannot be
switched. That is the robot's decision, not a fault here.
"""
import argparse
import sys

# Services that move the legs. Switching one of these off while the robot is
# standing is not a configuration change, it is a fall.
MOTION_SERVICES = {"mcf", "sport_mode", "wheeled_sport"}

# Documented in the RobotStateClient Interface page. Used only to catch typos
# early -- the robot remains the authority, and an unknown name is a warning
# rather than a refusal, because this list will age.
KNOWN_SERVICES = {
    "mcf", "sport_mode", "wheeled_sport", "basic_service", "obstacles_avoidance",
    "unitree_lidar", "video_hub", "webrtc_bridge", "vui_service", "chat_go",
    "net_switcher", "ota_box", "utrack",
}

ERROR_NOTES = {
    5201: "service switch execution error (returned by the robot)",
    5202: "the service is protected and cannot be turned on or off",
    3104: "request timed out -- is the robot on the wire and is --iface right?",
    3203: "the robot's software does not implement this call",
}


def describe(code):
    return ERROR_NOTES.get(code, "error code %d" % code)


def main():
    ap = argparse.ArgumentParser(
        description="list and switch the Go2's internal services")
    ap.add_argument("--iface", default=None,
                    help="network interface facing the robot; defaults to "
                         "$UNITREE_IFACE, which setup.env.sh sets")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--list", action="store_true", help="show every service")
    ap.add_argument("--on", action="append", default=[], metavar="NAME",
                    help="turn a service on (repeatable)")
    ap.add_argument("--off", action="append", default=[], metavar="NAME",
                    help="turn a service off (repeatable)")
    ap.add_argument("--dry-run", action="store_true",
                    help="say what would happen and change nothing")
    ap.add_argument("--i-mean-it", action="store_true",
                    help="permit switching a motion control service. The robot "
                         "will drop if it is standing. Legs off the ground or "
                         "lying down first.")
    args = ap.parse_args()

    if not args.list and not args.on and not args.off:
        ap.error("nothing to do -- pass --list, --on NAME or --off NAME")

    for name in args.on + args.off:
        if name not in KNOWN_SERVICES:
            print("warning: '%s' is not in the documented service list. "
                  "Check the spelling with --list." % name)

    dangerous = [n for n in args.on + args.off if n in MOTION_SERVICES]
    if dangerous and not args.i_mean_it:
        print("refusing to switch %s without --i-mean-it."
              % ", ".join(sorted(set(dangerous))))
        print()
        print("  These are the motion control services. Switching one off while")
        print("  the robot is standing drops it on the floor. Lie the robot down")
        print("  or suspend its legs first, then pass --i-mean-it.")
        print()
        print("  On software >= 1.1.6 the name is 'mcf'; below that it is")
        print("  'sport_mode'. Switching the wrong one does nothing at all and")
        print("  gives no error, which you discover when your own controller")
        print("  starts fighting Unitree's.")
        return 2

    if args.dry_run:
        print("dry run -- nothing will be sent to the robot")
        for n in args.off:
            print("  would turn OFF  %s" % n)
        for n in args.on:
            print("  would turn ON   %s" % n)
        if args.list:
            print("  would list the robot's services")
        return 0

    # Imported late so --dry-run and --help work on a machine without the SDK.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

    iface = args.iface
    if not iface:
        import os
        iface = os.environ.get("UNITREE_IFACE")
    if not iface:
        raise SystemExit(
            "no interface. Pass --iface, or source setup.env.sh first so "
            "UNITREE_IFACE is set.")

    ChannelFactoryInitialize(args.domain, iface)
    client = RobotStateClient()
    client.SetTimeout(3.0)
    client.Init()

    rc = 0

    for name in args.off + args.on:
        want_on = name in args.on
        code = client.ServiceSwitch(name, want_on)
        # ServiceSwitch returns either a bare code or (code, data) depending on
        # the SDK build; accept both rather than guessing.
        if isinstance(code, tuple):
            code = code[0]
        if code == 0:
            print("  %-20s -> %s" % (name, "on" if want_on else "off"))
        else:
            print("  %-20s FAILED: %s" % (name, describe(code)))
            rc = 1

    if args.list or args.off or args.on:
        code, services = client.ServiceList()
        if code != 0 or services is None:
            print("could not read the service list: %s" % describe(code))
            return 1
        print("\n  %-22s %-8s %s" % ("service", "status", ""))
        for s in sorted(services, key=lambda s: s.name):
            flags = []
            if s.protect:
                flags.append("protected")
            if s.name in MOTION_SERVICES:
                flags.append("MOTION CONTROL")
            # The docs define status as 0: on, 1: off -- the opposite way round
            # to how it reads. Spelled out here so nobody has to remember.
            print("  %-22s %-8s %s"
                  % (s.name, "on" if s.status == 0 else "off",
                     ", ".join(flags)))

    return rc


if __name__ == "__main__":
    sys.exit(main())
