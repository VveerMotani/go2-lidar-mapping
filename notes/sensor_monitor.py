#!/usr/bin/env python3
"""
Frame rate and delay of each sensor, as topics Foxglove can plot.

    python3 sensor_monitor.py                 (go2-start.sh starts it)

Once a second, for each stream below, publishes:

    /go2/rate/<name>      std_msgs/Float32   messages a second arriving here
    /go2/delay/<name>     std_msgs/Float32   milliseconds from the message's own
                                             timestamp to its arrival here
    /go2/sensors          std_msgs/String    one line for all of them, for a
                                             Foxglove Raw Message or Log panel

Streams:
    lidar        /lidar_points                    XT16 cloud, from the laptop's driver
    depth        /camera/depth/image_rect_raw     full-resolution depth, from the robot
    depth_cloud  /depth/points                    the preview cloud Foxglove draws
    color        /camera/color/image_raw/compressed

HOW IT MEASURES WITHOUT SLOWING ANYTHING
---------------------------------------
Messages are taken serialized and never decoded. The timestamp is read straight
from the first bytes (every one of these types starts with a std_msgs/Header),
so a 1.8 MB depth frame costs a few microseconds here rather than a copy.

WHAT THE DELAY INCLUDES
-----------------------
Depth is stamped by the camera node on the robot, so its delay is capture to
arrival on the laptop: camera, relay, network and client together. It is only
as right as the two clocks agree; go2-start.sh sets the robot's clock from the
laptop at the start, which is typically good to a few milliseconds.
The LiDAR is stamped by the driver on the laptop, so its delay is mostly
the driver assembling a full turn of the sensor.
A delay that is negative, or seconds large, means the clocks disagree, not that
the data is from the future.
"""
import argparse
import struct
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Float32, String

STREAMS = [
    ("lidar", "/lidar_points", PointCloud2),
    ("depth", "/camera/depth/image_rect_raw", Image),
    ("depth_cloud", "/depth/points", PointCloud2),
    ("color", "/camera/color/image_raw/compressed", CompressedImage),
]


def header_stamp(raw):
    """sec, nanosec of a serialized message whose first field is a Header.

    CDR: 4 bytes of encapsulation (byte 1 says the byte order), then the
    Header's stamp as int32 sec and uint32 nanosec.
    """
    if len(raw) < 12:
        return None
    little = raw[1] == 1
    return struct.unpack("<iI" if little else ">iI", raw[4:12])


class Stream(object):
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.delays = []
        self.last_arrival = None


class Monitor(Node):
    def __init__(self, period):
        super().__init__("sensor_monitor")
        self.period = period
        self.streams = {}
        self.pubs = {}
        # Best effort subscribes to both reliable and best-effort publishers,
        # and never makes a publisher wait for this node.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        for name, topic, mtype in STREAMS:
            st = Stream(name)
            self.streams[name] = st
            self.create_subscription(mtype, topic,
                                     lambda raw, st=st: self.on_msg(st, raw),
                                     qos, raw=True)
            self.pubs[name] = (self.create_publisher(Float32, "/go2/rate/" + name, 10),
                               self.create_publisher(Float32, "/go2/delay/" + name, 10))
        self.summary = self.create_publisher(String, "/go2/sensors", 10)
        self.last = time.time()
        self.create_timer(period, self.report)

    def on_msg(self, st, raw):
        now = time.time()
        st.count += 1
        st.last_arrival = now
        stamp = header_stamp(raw)
        if stamp and stamp[0] > 0:
            st.delays.append((now - (stamp[0] + stamp[1] * 1e-9)) * 1000.0)

    def report(self):
        now = time.time()
        dt = max(now - self.last, 1e-3)
        self.last = now
        parts = []
        for name, _, _ in STREAMS:
            st = self.streams[name]
            rate = st.count / dt
            rate_pub, delay_pub = self.pubs[name]
            rate_pub.publish(Float32(data=float(rate)))
            if st.delays:
                delay = sorted(st.delays)[len(st.delays) // 2]      # median
                delay_pub.publish(Float32(data=float(delay)))
                parts.append("%s %.1f/s %.0f ms" % (name, rate, delay))
            elif st.last_arrival and now - st.last_arrival < 5:
                parts.append("%s %.1f/s" % (name, rate))
            else:
                parts.append("%s off" % name)
            st.count = 0
            st.delays = []
        self.summary.publish(String(data="   ".join(parts)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--period", type=float, default=1.0,
                    help="seconds between reports")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Monitor(args.period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
