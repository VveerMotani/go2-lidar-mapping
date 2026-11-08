"""
Publish the Go2's IMU as a standard ROS 2 sensor_msgs/Imu.

Every SLAM package -- LIO-SAM, FAST-LIO2, Point-LIO -- wants motion data in the
one standard message type. The robot does not provide it. Unitree ships its own
message format, and go2_ros2_sdk publishes a different custom one again, so
nothing on this machine speaks the language the SLAM packages expect. This is
the translator.

Runs on the laptop, inside the `unitree` WSL distro. Two modes:

    # live, from a robot on the wire (needs unitree_ros2 running)
    python3 imu_publisher.py

    # from a recording made by jetson/record_session.py -- no robot needed
    python3 imu_publisher.py --from-file sessions/corridor1/imu.csv

Then check it:

    ros2 topic hz /imu/data
    ros2 topic echo /imu/data --once

WHY REPLAY MODE IS THE ONE YOU WILL USE MOST
--------------------------------------------
SLAM is tuned by running the same data through it over and over with different
settings. Doing that against a live robot means the robot must be switched on
and walking for every attempt, and no two attempts see the same thing. Against
a recording, every run is identical and the robot can be in its box.

Pair it with the LiDAR half of the same recording, which the Hesai driver
replays directly:

    # config.yaml:  source_type: 2   pcap_path: ".../lidar.pcap"
    ros2 launch hesai_ros_driver start.py

Both halves were recorded from one clock on the Jetson, so they line up.

TIMESTAMPS
----------
By default published messages are stamped with the time they are published,
not the time they were recorded. That is deliberate: the Hesai driver is
configured with use_timestamp_type: 1, meaning it stamps replayed LiDAR frames
with the host's current time too. Both therefore live on the same replay clock
and stay aligned. Use --stamp recorded if you have changed that.
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

# ---------------------------------------------------------------------------
# THE QUATERNION ORDER -- the one thing in this file that is easy to get wrong
# and hard to notice.
#
# Unitree stores orientation as   quaternion = [w, x, y, z]
# ROS sensor_msgs/Imu wants        x, y, z, w
#
# Swap them and nothing errors. The robot simply appears to be oriented wrongly,
# and a SLAM map built on it comes out bent in a way that looks like a
# calibration problem. Both places that read a Unitree quaternion in this repo
# say so: this file and jetson/record_session.py.
# ---------------------------------------------------------------------------

# Rough covariances. The robot does not report its own uncertainty, so these
# are stated as "known to be roughly this good" rather than left at zero --
# zero means "exactly certain", which no sensor is, and some SLAM packages
# treat a zero orientation covariance as a reason to distrust the message.
# The first element being -1 would mean "no orientation at all", which would
# disqualify this IMU from LIO-SAM entirely, so it must not be that.
ORIENTATION_COV = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
ANGULAR_COV = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
LINEAR_COV = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]


def fill(msg, frame_id, quat_wxyz, gyro, accel):
    """Populate a sensor_msgs/Imu from Unitree's fields.

    Every value is cast with float() on the way in. That is not decoration:
    live from the robot these arrive as numpy.float32, and the ROS message
    classes assert on isinstance(value, float), which numpy.float32 fails.
    It is easy to miss because numpy.float64 IS a float subclass and passes --
    so an expression that happens to divide (like the normalisation below)
    works while a plain assignment of the same data throws.
    """
    msg.header.frame_id = frame_id

    w, x, y, z = (float(v) for v in quat_wxyz)
    # Normalise: the robot's quaternion is float32 and drifts off unit length
    # by a little, which some solvers object to.
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    msg.orientation.x = x / n
    msg.orientation.y = y / n
    msg.orientation.z = z / n
    msg.orientation.w = w / n

    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    msg.linear_acceleration.x = float(accel[0])
    msg.linear_acceleration.y = float(accel[1])
    msg.linear_acceleration.z = float(accel[2])

    msg.orientation_covariance = ORIENTATION_COV
    msg.angular_velocity_covariance = ANGULAR_COV
    msg.linear_acceleration_covariance = LINEAR_COV
    return msg


class LiveBridge(Node):
    """Subscribe to the robot's own LowState topic, republish the IMU part."""

    def __init__(self, args):
        super().__init__("go2_imu_publisher")
        from unitree_go.msg import LowState        # only needed in live mode

        self.frame_id = args.frame_id
        self.count = 0
        self.t0 = time.time()

        # Sensor data QoS: best-effort and shallow, which is what a 500 Hz
        # stream wants. A reliable queue would build a backlog and deliver
        # stale samples, which is worse than dropping one.
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Imu, args.topic, qos)
        self.create_subscription(LowState, args.source_topic, self.on_lowstate, qos)

        self.get_logger().info(
            "republishing %s -> %s as sensor_msgs/Imu (frame %s)"
            % (args.source_topic, args.topic, self.frame_id))
        self.create_timer(5.0, self._report)

    def on_lowstate(self, msg):
        imu = msg.imu_state
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        fill(out, self.frame_id, imu.quaternion, imu.gyroscope, imu.accelerometer)
        self.pub.publish(out)
        self.count += 1

    def _report(self):
        elapsed = time.time() - self.t0
        if self.count == 0:
            self.get_logger().warn(
                "nothing received on the source topic yet -- is unitree_ros2 "
                "running and is the robot on the wire?")
        else:
            self.get_logger().info("%d messages, %.0f Hz"
                                   % (self.count, self.count / elapsed))


class FileBridge(Node):
    """Replay a recorded imu.csv in its original timing."""

    def __init__(self, args):
        super().__init__("go2_imu_publisher")
        self.frame_id = args.frame_id
        self.use_recorded_stamps = args.stamp == "recorded"
        self.loop = args.loop
        self.rate_scale = args.rate

        self.rows = self._load(args.from_file)
        if len(self.rows) < 2:
            raise SystemExit("%s has fewer than two rows" % args.from_file)

        span = self.rows[-1][0] - self.rows[0][0]
        hz = len(self.rows) / span if span > 0 else 0.0
        self.get_logger().info(
            "%s: %d samples, %.1f s, %.0f Hz"
            % (os.path.basename(args.from_file), len(self.rows), span, hz))

        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Imu, args.topic, qos)

    def _load(self, path):
        """Read the CSV into (host_time, quat, gyro, accel) tuples."""
        rows = []
        with open(path) as fh:
            for r in csv.DictReader(fh):
                try:
                    rows.append((
                        float(r["host_time"]),
                        (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])),
                        (float(r["gx"]), float(r["gy"]), float(r["gz"])),
                        (float(r["ax"]), float(r["ay"]), float(r["az"])),
                    ))
                except (KeyError, ValueError):
                    continue          # header repeat, or a truncated last line
        return rows

    def play(self):
        """Publish every sample, spaced as it was recorded."""
        passes = 0
        while rclpy.ok():
            first = self.rows[0][0]
            wall_start = time.time()
            for recorded_t, quat, gyro, accel in self.rows:
                if not rclpy.ok():
                    return
                # Sleep until this sample is due, so the stream comes out at
                # the rate it went in rather than as fast as the disk allows.
                # Samples are only 2 ms apart at 500 Hz, which is near the
                # granularity of sleep() -- asking for a sub-millisecond sleep
                # reliably overshoots and the stream comes out slow. Below that
                # threshold, publish immediately and let the next sample's
                # deadline absorb the difference, so timing stays correct on
                # average instead of drifting later and later.
                due = wall_start + (recorded_t - first) / self.rate_scale
                delay = due - time.time()
                if delay > 0.0008:
                    time.sleep(delay)

                out = Imu()
                if self.use_recorded_stamps:
                    out.header.stamp.sec = int(recorded_t)
                    out.header.stamp.nanosec = int((recorded_t % 1) * 1e9)
                else:
                    out.header.stamp = self.get_clock().now().to_msg()
                fill(out, self.frame_id, quat, gyro, accel)
                self.pub.publish(out)

            passes += 1
            wall = time.time() - wall_start
            self.get_logger().info(
                "pass %d: %d messages in %.1f s -- %.0f Hz published"
                % (passes, len(self.rows), wall, len(self.rows) / wall if wall else 0))
            if not self.loop:
                return


def main():
    ap = argparse.ArgumentParser(
        description="publish the Go2 IMU as sensor_msgs/Imu")
    ap.add_argument("--from-file", default=None,
                    help="replay an imu.csv from record_session.py; "
                         "omit for live from the robot")
    ap.add_argument("--topic", default="/imu/data",
                    help="what to publish (SLAM configs usually call this imuTopic)")
    ap.add_argument("--source-topic", default="/lowstate",
                    help="live mode: the unitree_go/msg/LowState topic")
    ap.add_argument("--frame-id", default="imu_link",
                    help="frame the IMU is rigidly attached to")
    ap.add_argument("--stamp", choices=["now", "recorded"], default="now",
                    help="replay mode: stamp with the current time (default, "
                         "matches how the Hesai driver stamps replayed frames) "
                         "or with the recorded time")
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
