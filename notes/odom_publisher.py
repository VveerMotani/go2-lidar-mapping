"""
Publish the Go2's own odometry as standard ROS 2, and as the transform that
makes the point cloud accumulate into a map instead of a smear.

    # live, from a robot on the wire (needs unitree_ros2 running)
    python3 odom_publisher.py

    # from a recording made by jetson/record_session.py -- no robot needed
    python3 odom_publisher.py --from-file walks/Lab-loop-1/odom.csv

Publishes:

    /odom       nav_msgs/Odometry     pose and twist of base_link in odom
    /tf         odom -> base_link     the moving link in the frame tree

WHY THIS EXISTS -- THE SMEAR
----------------------------
Before this, go2-start.sh published TWO STATIC transforms:

    map --(static)--> odom --(static, z=0.30)--> hesai_lidar

"odom" is by definition a fixed world frame. "hesai_lidar" is bolted to a
walking dog. Fixing one rigidly to the other tells every consumer that the
LiDAR never moves -- so all 240 frames of a walk get drawn at the same world
pose. Walk ten metres and you get a ten-metre smear rather than a room. The
first recorded walk (2026-09-04) has exactly this shape: its bag contains
/tf_static with two transforms and no dynamic /tf at all.

The correct tree needs a moving link in the middle:

    map --> odom --> base_link --> hesai_lidar
                     ^ THIS FILE   ^ static, vendor-published extrinsic

WHERE THE POSE COMES FROM
-------------------------
Unitree's own legged + IMU fusion odometry, which the robot publishes for free
on "rt/sportmodestate" at around 50 Hz. It is documented in the ROS2 Services
Interface page of the Go2 SDK docs, which defines the message as:

    float32[3] position        // x, y, z in the odom frame
    float32[3] velocity        // body velocity
    float32    yaw_speed
    IMUState   imu_state       // quaternion is w, x, y, z
    float32    body_height
    uint8      gait_type, mode
    int16[4]   foot_force

and in the Odometer Services Interface page, which calls it the "legged fusion
odometry interface ... fuses legged (motor) data with IMU integration data".

For Go2 software > V1.1.6 the same value is reachable through
Motion Services Interface V2.0's position(); we take the topic because it comes
without an RPC round trip and at a useful rate.

WHAT IT IS AND IS NOT
---------------------
It is dead reckoning. It drifts, and it drifts worse on a legged robot than on
wheels, because a foot that slips is indistinguishable from a body that moved.
Over a two-minute walk expect metres of error by the end, and expect a loop not
to close.

That is fine for what it is for here: giving each LiDAR frame a *different*
pose so the cloud spreads out into something room-shaped and so a SLAM package
has a motion prior to start from. It is not a map, and nothing downstream
should treat "odom" as if it were "map".

QUATERNION ORDER -- the same trap as imu_publisher.py
-----------------------------------------------------
Unitree stores orientation as w, x, y, z. ROS wants x, y, z, w. Swapping them
raises no error; the robot simply appears to be facing the wrong way and any
map built on it comes out bent in a way that looks like a calibration fault.
"""
import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# Columns written by jetson/record_session.py. Kept here as the single
# definition of the file format so the writer and the reader cannot drift
# apart -- record_session.py imports nothing from the laptop, so this is a
# convention, and conventions need somewhere to be written down.
ODOM_COLUMNS = (
    "host_time", "px", "py", "pz", "vx", "vy", "vz", "yaw_speed",
    "qw", "qx", "qy", "qz", "body_height", "gait_type", "mode",
    "foot_fr", "foot_fl", "foot_rr", "foot_rl",
)

# The robot reports no uncertainty of its own. These say "roughly this good"
# rather than leaving zeros, which some consumers read as "exactly certain".
# Position is deliberately looser than orientation: the orientation comes from
# an IMU that is genuinely good, the position from integrating leg motion,
# which is genuinely not.
# Written with float() around the lists below, not bare numbers: ROS 2's
# message classes assert every covariance entry is a Python float, and a
# literal 0 is an int. The first real odometry message on 2026-09-17 raised
# AssertionError and killed this publisher, which left odom -> base_link
# missing and Foxglove's 3D panel empty. Same trap imu_publisher.py hit.
POSE_COV = [
    0.05, 0, 0, 0, 0, 0,
    0, 0.05, 0, 0, 0, 0,
    0, 0, 0.10, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0,
    0, 0, 0, 0, 0.01, 0,
    0, 0, 0, 0, 0, 0.02,
]
POSE_COV = [float(v) for v in POSE_COV]
TWIST_COV = [
    0.02, 0, 0, 0, 0, 0,
    0, 0.02, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0,
    0, 0, 0, 0, 0.01, 0,
    0, 0, 0, 0, 0, 0.02,
]
TWIST_COV = [float(v) for v in TWIST_COV]


def normalised(quat_wxyz):
    """Unitree w,x,y,z -> ROS x,y,z,w, unit length.

    float() on every value is not decoration: live from the robot these arrive
    as numpy.float32, and the ROS message classes assert on
    isinstance(value, float), which float32 fails. imu_publisher.py carries the
    same note for the same reason.
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return x / n, y / n, z / n, w / n


class OdomOutput(object):
    """Builds and sends the Odometry message and the matching transform.

    Both come from one place so they can never disagree. A transform that says
    one thing while /odom says another is a genuinely horrible bug to find:
    every number looks right on its own.
    """

    def __init__(self, node, odom_topic, odom_frame, base_frame):
        self.node = node
        self.odom_frame = odom_frame
        self.base_frame = base_frame

        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = node.create_publisher(Odometry, odom_topic, qos)

        # /tf is conventionally reliable with a deeper queue: a dropped
        # transform makes every lookup in that window fail, whereas a dropped
        # odometry message is just one missing sample.
        tf_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self.tf_pub = node.create_publisher(TFMessage, "/tf", tf_qos)
        self.count = 0

    def send(self, stamp, pos, quat_xyzw, vel, yaw_speed):
        x, y, z = (float(v) for v in pos)
        qx, qy, qz, qw = (float(v) for v in quat_xyzw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = POSE_COV
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        odom.twist.twist.angular.z = float(yaw_speed)
        odom.twist.covariance = TWIST_COV
        self.odom_pub.publish(odom)

        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = self.odom_frame
        tr.child_frame_id = self.base_frame
        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.translation.z = z
        tr.transform.rotation.x = qx
        tr.transform.rotation.y = qy
        tr.transform.rotation.z = qz
        tr.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage(transforms=[tr]))

        self.count += 1


class LiveBridge(Node):
    """Subscribe to the robot's SportModeState, republish as odometry + tf."""

    def __init__(self, args):
        super().__init__("go2_odom_publisher")
        from unitree_go.msg import SportModeState      # only needed live

        self.out = OdomOutput(self, args.topic, args.odom_frame, args.base_frame)
        self.t0 = time.time()
        self.warned_zero = False

        # Sensor-data QoS on the way in: a 50 Hz stream wants the newest
        # sample, not a replayed backlog.
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(SportModeState, args.source_topic,
                                 self.on_state, qos)
        self.get_logger().info(
            "republishing %s -> %s and %s -> %s on /tf"
            % (args.source_topic, args.topic, args.odom_frame, args.base_frame))
        self.create_timer(5.0, self._report)

    def on_state(self, msg):
        self.out.send(self.get_clock().now().to_msg(),
                      msg.position,
                      normalised(msg.imu_state.quaternion),
                      msg.velocity,
                      msg.yaw_speed)

    def _report(self):
        elapsed = time.time() - self.t0
        if self.out.count == 0:
            self.get_logger().warn(
                "nothing on the source topic yet. sportmodestate only "
                "publishes while the motion control service is running and the "
                "robot is standing -- lying down, there is nothing to report. "
                "Until it does, odom -> base_link is missing and the frame tree "
                "is in two disconnected halves, so a 3D panel set to 'odom' "
                "will be EMPTY, not merely frozen. Set the display frame to "
                "'hesai_lidar' to see the cloud meanwhile.")
            return
        self.get_logger().info("%d messages, %.0f Hz"
                               % (self.out.count, self.out.count / elapsed))


class FileBridge(Node):
    """Replay a recorded odom.csv in its original timing."""

    def __init__(self, args):
        super().__init__("go2_odom_publisher")
        self.out = OdomOutput(self, args.topic, args.odom_frame, args.base_frame)
        self.use_recorded_stamps = args.stamp == "recorded"
        self.rate_scale = args.rate
        self.loop = args.loop
        self.rows = self._load(args.from_file)
        if len(self.rows) < 2:
            raise SystemExit("%s has fewer than two rows" % args.from_file)

        span = self.rows[-1][0] - self.rows[0][0]
        hz = len(self.rows) / span if span > 0 else 0.0
        travelled = self._path_length()
        drift = self._start_to_finish()
        self.get_logger().info(
            "%s: %d samples, %.1f s, %.0f Hz, path %.1f m, "
            "start to finish %.2f m"
            % (os.path.basename(args.from_file), len(self.rows), span, hz,
               travelled, drift))

    def _load(self, path):
        rows = []
        with open(path) as fh:
            for r in csv.DictReader(fh):
                try:
                    rows.append((
                        float(r["host_time"]),
                        (float(r["px"]), float(r["py"]), float(r["pz"])),
                        (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])),
                        (float(r["vx"]), float(r["vy"]), float(r["vz"])),
                        float(r["yaw_speed"]),
                    ))
                except (KeyError, ValueError):
                    continue        # header repeat, or a truncated last line
        return rows

    def _path_length(self):
        total = 0.0
        for i in range(1, len(self.rows)):
            a, b = self.rows[i - 1][1], self.rows[i][1]
            total += math.dist(a, b)
        return total

    def _start_to_finish(self):
        return math.dist(self.rows[0][1], self.rows[-1][1])

    def play(self):
        passes = 0
        while rclpy.ok():
            first = self.rows[0][0]
            wall_start = time.time()
            for recorded_t, pos, quat, vel, yaw_speed in self.rows:
                if not rclpy.ok():
                    return
                # Same pacing rule as imu_publisher.py: below about a
                # millisecond, sleep() overshoots more than it helps, so
                # publish immediately and let the next sample's deadline
                # absorb the difference.
                due = wall_start + (recorded_t - first) / self.rate_scale
                delay = due - time.time()
                if delay > 0.0008:
                    time.sleep(delay)

                if self.use_recorded_stamps:
                    stamp = rclpy.time.Time(
                        seconds=int(recorded_t),
                        nanoseconds=int((recorded_t % 1) * 1e9)).to_msg()
                else:
                    stamp = self.get_clock().now().to_msg()
                self.out.send(stamp, pos, normalised(quat), vel, yaw_speed)

            passes += 1
            wall = time.time() - wall_start
            self.get_logger().info(
                "pass %d: %d messages in %.1f s -- %.0f Hz published"
                % (passes, len(self.rows), wall,
                   len(self.rows) / wall if wall else 0))
            if not self.loop:
                return


def main():
    ap = argparse.ArgumentParser(
        description="publish the Go2's legged odometry as nav_msgs/Odometry "
                    "and the odom -> base_link transform")
    ap.add_argument("--from-file", default=None,
                    help="replay an odom.csv from record_session.py; "
                         "omit for live from the robot")
    ap.add_argument("--topic", default="/odom")
    ap.add_argument("--source-topic", default="/sportmodestate",
                    help="live mode: the unitree_go/msg/SportModeState topic. "
                         "The DDS name is rt/sportmodestate; ROS 2 drops the "
                         "rt/ prefix.")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--stamp", choices=["now", "recorded"], default="now",
                    help="replay mode: stamp with the current time (default, "
                         "matching how the Hesai driver stamps replayed "
                         "frames) or with the recorded time")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="replay speed multiplier")
    ap.add_argument("--loop", action="store_true", help="replay repeatedly")
    args = ap.parse_args()

    rclpy.init()
    try:
        if args.from_file:
            node = FileBridge(args)
            node.play()
        else:
            node = LiveBridge(args)
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
