"""
Headless Go2 simulator that publishes the same DDS streams as the real robot.

Why this exists instead of unitree_mujoco's own `unitree_mujoco.py`:

  1. The stock runner opens a MuJoCo viewer window. Under WSLg that goes
     through a Direct3D 12 translation layer that has already taken RViz down
     once (see docs/decisions.md D2). A test fixture must not be the flaky part.
  2. The stock runner also needs a gamepad (config.USE_JOYSTICK = 1).
  3. Most importantly: the vendor bridge leaves four things empty that the
     dashboard needs. It publishes motor_state and the raw IMU quaternion, but
     never fills `foot_force` (go2.xml has no touch sensors), never fills
     `imu_state.rpy`, never fills the battery fields, and never fills gait or
     body height on SportModeState_.

We reuse the vendor bridge unmodified for physics and publishing, and fill the
missing fields into the very same `low_state` / `high_state` objects it owns.
That works because the bridge's publish threads only ever overwrite the fields
they know about, so whatever we last wrote into the others goes out on the wire.
Nothing under sim/unitree_mujoco/ is edited.

    python3 sim_lowstate.py                    # domain 1, interface lo
    python3 sim_lowstate.py --domain 0 --iface eth0
    python3 sim_lowstate.py --motion still     # hold the stand pose

Run it inside the venv:

    source ~/unitree_ws/venv-rl/bin/activate
    python3 ~/unitree_ws/projects/sim_lowstate.py
"""
import argparse
import math
import os
import sys
import time

import mujoco
import numpy as np

SIM_DIR = os.path.expanduser("~/unitree_ws/sim/unitree_mujoco/simulate_python")
SCENE = os.path.expanduser("~/unitree_ws/sim/unitree_mujoco/unitree_robots/go2/scene.xml")

sys.path.insert(0, SIM_DIR)

# The vendor bridge does `import pygame` at module scope, but only ever uses it
# for the optional gamepad. We never call SetupJoystick, so satisfy the import
# with an empty stub rather than installing a package we would not use.
if "pygame" not in sys.modules:
    import types
    sys.modules["pygame"] = types.ModuleType("pygame")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
from unitree_sdk2py_bridge import UnitreeSdk2Bridge  # noqa: E402

# ---------------------------------------------------------------------------
# Joint order. This is THE convention that everything downstream depends on.
#
# The Go2 SDK indexes its twelve motors 0..11 in the order
#
#     0  FR_hip   1  FR_thigh   2  FR_calf
#     3  FL_hip   4  FL_thigh   5  FL_calf
#     6  RR_hip   7  RR_thigh   8  RR_calf
#     9  RL_hip  10  RL_thigh  11  RL_calf
#
# F/R = front/rear, R/L = right/left. go2.xml lists its actuators and its
# jointpos sensors in exactly this order, so mj_data.ctrl[i] and
# low_state.motor_state[i] refer to the same joint. dashboard.py repeats this
# list; if you change one, change both.
# ---------------------------------------------------------------------------
STAND = [0.0, 0.9, -1.8] * 4          # the "home" keyframe in go2.xml
FOOT_GEOMS = ["FR", "FL", "RR", "RL"]  # named in go2.xml, SDK foot_force order

KP, KD = 60.0, 5.0                     # position gains, enough to hold a stand


def quat_to_rpy(w, x, y, z):
    """MuJoCo gives the IMU as a quaternion; the real robot also reports rpy.

    The dashboard reads rpy, so derive it here. Standard ZYX (yaw-pitch-roll)
    convention, the same one the Unitree firmware uses.
    """
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def target_pose(t, motion):
    """Joint targets at time t. Deliberately not a neutral pose.

    'wave'  slow squat plus a per-leg phase offset, so every one of the twelve
            joints moves and the four foot forces rise and fall out of phase.
            This is what you want on screen when checking that the 3D model
            tracks the data.
    'still' hold the stand pose, for reading steady numbers.
    """
    q = list(STAND)
    if motion == "still":
        return q

    squat = 0.25 * math.sin(2.0 * math.pi * t / 6.0)          # whole body, 6 s
    for leg in range(4):
        phase = 2.0 * math.pi * (t / 3.0 + leg / 4.0)          # legs out of phase
        lift = 0.18 * math.sin(phase)
        q[leg * 3 + 0] += 0.10 * math.sin(phase)               # hip  abduct/adduct
        q[leg * 3 + 1] += squat + lift                         # thigh
        q[leg * 3 + 2] += -1.6 * squat - 0.8 * lift            # calf keeps foot down
    return q


def foot_forces(model, data, geom_ids):
    """Sum the normal contact force on each foot geom, in newtons.

    go2.xml has no touch sensors, so read the contacts directly. Note the units
    differ from the real robot, whose foot_force is a raw sensor count in the
    low hundreds -- documented in docs/dashboard.md.
    """
    out = [0.0, 0.0, 0.0, 0.0]
    buf = np.zeros(6)
    for c in range(data.ncon):
        con = data.contact[c]
        mujoco.mj_contactForce(model, data, c, buf)
        normal = abs(buf[0])
        for i, gid in enumerate(geom_ids):
            if con.geom1 == gid or con.geom2 == gid:
                out[i] += normal
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=1, help="DDS domain (sim default 1)")
    ap.add_argument("--iface", default="lo", help="network interface (sim default lo)")
    ap.add_argument("--scene", default=SCENE)
    ap.add_argument("--motion", choices=["wave", "still"], default="wave")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)   # the "home" stand keyframe

    geom_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_GEOMS]
    if any(g < 0 for g in geom_ids):
        raise SystemExit("foot geoms %s not found in %s" % (FOOT_GEOMS, args.scene))

    ChannelFactoryInitialize(args.domain, args.iface)
    bridge = UnitreeSdk2Bridge(model, data)       # starts its own publish threads

    print("publishing rt/lowstate + rt/sportmodestate on domain %d via %s"
          % (args.domain, args.iface))
    print("motion: %s  (ctrl-c to stop)" % args.motion)

    dt = model.opt.timestep
    t0 = time.time()
    try:
        while True:
            step_start = time.perf_counter()
            t = time.time() - t0

            # --- control: PD toward the animated target -----------------------
            q = target_pose(t, args.motion)
            for i in range(12):
                pos = data.sensordata[i]
                vel = data.sensordata[i + 12]
                data.ctrl[i] = KP * (q[i] - pos) + KD * (0.0 - vel)

            mujoco.mj_step(model, data)

            # --- fill what the vendor bridge leaves empty ---------------------
            # These land in the same objects the bridge's threads publish from.
            forces = foot_forces(model, data, geom_ids)
            for i in range(4):
                bridge.low_state.foot_force[i] = int(round(forces[i]))

            w, x, y, z = (data.sensordata[36 + k] for k in range(4))   # imu_quat
            roll, pitch, yaw = quat_to_rpy(w, x, y, z)
            bridge.low_state.imu_state.rpy[0] = roll
            bridge.low_state.imu_state.rpy[1] = pitch
            bridge.low_state.imu_state.rpy[2] = yaw

            # MuJoCo models no motor thermals, so motor_state[i].temperature
            # would sit at 0 and the dashboard's temperature heat-map would be
            # a uniform cold blue. Fake a plausible one: a 30 C ambient that
            # each motor rises above in proportion to how hard it is working.
            # Invented, like the battery below, and documented as such.
            for i in range(12):
                load = abs(bridge.low_state.motor_state[i].tau_est)
                bridge.low_state.motor_state[i].temperature = int(
                    30 + min(45.0, 1.6 * load))

            # Plausible battery so the power panel has something to show. The
            # simulator models no electrical system; these are invented, and
            # docs/dashboard.md says so.
            bridge.low_state.bms_state.soc = 87
            bridge.low_state.power_v = 28.6 - 0.2 * math.sin(t / 20.0)
            bridge.low_state.power_a = 3.0 + 2.0 * abs(math.sin(t / 4.0))
            bridge.low_state.temperature_ntc1 = 41
            bridge.low_state.temperature_ntc2 = 39

            # SportModeState_ fields the bridge does not touch.
            bridge.high_state.mode = 1
            bridge.high_state.gait_type = 1
            bridge.high_state.body_height = float(data.qpos[2])
            bridge.high_state.yaw_speed = float(data.qvel[5])

            slack = dt - (time.perf_counter() - step_start)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
