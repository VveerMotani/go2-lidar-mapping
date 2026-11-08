"""
First contact: listen to the robot, command nothing.

Read-only. It subscribes to the Go2's state broadcast and prints a few fields.
Safe to run with the dog on the floor -- it cannot move anything.

    python3 read_state.py            # interface auto-detected from UNITREE_IFACE
    python3 read_state.py eth0       # or name it explicitly
"""
import os
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

seen = 0


def on_state(msg: LowState_) -> None:
    global seen
    seen += 1
    if seen % 10:            # ~500 Hz in, 2.5 Hz out
        return
    roll, pitch, yaw = msg.imu_state.rpy
    print(
        f"battery {msg.bms_state.soc:3d}%   "
        f"roll {roll:+.2f}  pitch {pitch:+.2f}  yaw {yaw:+.2f}   "
        f"hip[FR] {msg.motor_state[0].q:+.3f} rad   "
        f"foot force {[f for f in msg.foot_force]}"
    )


if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("UNITREE_IFACE", "eth0")
    # Over the wired link the Go2 publishes the low-frequency state; the full-rate
    # "rt/lowstate" is only on the robot's internal network.
    topic = sys.argv[2] if len(sys.argv) > 2 else "rt/lf/lowstate"
    print(f"listening on {iface}, topic {topic} -- Ctrl-C to stop")

    ChannelFactoryInitialize(0, iface)
    sub = ChannelSubscriber(topic, LowState_)
    sub.Init(on_state, 10)

    try:
        while True:
            time.sleep(1)
            if seen == 0:
                print("  ...nothing heard yet. Robot on? Cable in? Right interface?")
    except KeyboardInterrupt:
        print(f"\nstopped after {seen} messages")
