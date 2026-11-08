#!/usr/bin/env python3
"""
Receive the robot's depth camera and publish it as ROS 2 topics.

    python3 realsense_client.py
    python3 realsense_client.py --host 192.168.123.18 --port 8093

Pairs with notes/jetson/realsense_ros1_relay.py, started on the robot by
realsense_start.sh. See that file for why the camera goes through a relay.

PUBLISHES
---------
  /camera/depth/image_rect_raw          sensor_msgs/Image, 16UC1, millimetres
  /camera/depth/camera_info             the camera's factory calibration
  /camera/depth_preview/image_rect_raw  every Nth pixel (--preview N, default 4)
  /camera/depth_preview/camera_info     the same calibration, scaled to match
  /camera/color/image_raw/compressed    sensor_msgs/CompressedImage, when relayed
  /camera/color/camera_info
  /tf_static                            the camera's own frame positions

Names and frame ids match Intel's camera node, so anything written for a
RealSense on ROS finds what it expects.

Timestamps are the robot's, as the camera node stamped them. go2-start.sh sets
the robot's clock from the laptop, so they line up with the LiDAR and odometry.

Reliable QoS on the publishers, deliberately: depth_image_proc and Foxglove
both subscribe reliably, and a best-effort publisher would silently never
connect to them.
"""
import argparse
import array
import json
import socket
import struct
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import StaticTransformBroadcaster


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(1 << 20, n - len(buf)))
        if not chunk:
            raise ConnectionError("robot closed the connection")
        buf += chunk
    return bytes(buf)


def as_uint8(data):
    """Image bytes in the one form rclpy stores without checking each byte.

    Handed `bytes`, a generated message's uint8[] setter validates every
    element in a Python loop: 1.8 million iterations per 1280x720 depth frame,
    which capped this client at about 8 frames a second on a 15 fps stream. An
    array.array of typecode 'B' is stored as it is. Same bytes either way.
    """
    return array.array("B", data)


def ros_time(stamp):
    return Time(sec=int(stamp[0]), nanosec=int(stamp[1]))


def scaled_info(msg, n):
    """The calibration for an image made of every n-th pixel of the original.

    Exact, not estimated: keeping pixel u*n of the original means pixel u of
    the preview, so focal lengths divide by n and the optical centre moves
    with the pixel-centre convention, c' = (c + 0.5) / n - 0.5.
    """
    out = CameraInfo()
    out.header = msg.header
    out.width, out.height = msg.width // n, msg.height // n
    out.distortion_model = msg.distortion_model
    out.d = list(msg.d)
    k = list(msg.k)
    k[0] /= n; k[4] /= n
    k[2] = (k[2] + 0.5) / n - 0.5
    k[5] = (k[5] + 0.5) / n - 0.5
    out.k = [float(v) for v in k]
    out.r = list(msg.r)
    p = list(msg.p)
    p[0] /= n; p[5] /= n
    p[2] = (p[2] + 0.5) / n - 0.5
    p[6] = (p[6] + 0.5) / n - 0.5
    p[3] /= n; p[7] /= n
    out.p = [float(v) for v in p]
    return out


def camera_info(info, stamp):
    msg = CameraInfo()
    msg.header.stamp = ros_time(stamp)
    msg.header.frame_id = info["frame_id"]
    msg.width, msg.height = int(info["width"]), int(info["height"])
    msg.distortion_model = info["distortion_model"]
    # ROS 2 asserts every entry is a Python float; the relay's JSON may carry ints.
    msg.d = [float(v) for v in info["D"]]
    msg.k = [float(v) for v in info["K"]]
    msg.r = [float(v) for v in info["R"]]
    msg.p = [float(v) for v in info["P"]]
    return msg


class Client(Node):
    def __init__(self, args):
        super().__init__("realsense_client")
        self.args = args
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_rect_raw", qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", qos)
        self.color_pub = self.create_publisher(CompressedImage, "/camera/color/image_raw/compressed", qos)
        self.color_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", qos)
        # A light copy for viewing. Full resolution as 3D points is ~4.9 MB a
        # message, ~49 MB/s, which Foxglove cannot draw through the bridge: the
        # 3D panel simply stayed empty. Every 4th pixel is ~0.3 MB.
        self.preview_pub = self.create_publisher(Image, "/camera/depth_preview/image_rect_raw", qos)
        self.preview_info_pub = self.create_publisher(CameraInfo, "/camera/depth_preview/camera_info", qos)
        self.static = StaticTransformBroadcaster(self)
        self.counts = {"depth": 0, "color": 0}

    # Not named handle: rclpy's Node already has a `handle` property, and
    # overriding it breaks the node's own constructor.
    def on_packet(self, header, payload):
        kind = header.get("kind")
        if kind == "tf":
            msgs = []
            for t in header.get("transforms", []):
                m = TransformStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = t["parent"]
                m.child_frame_id = t["child"]
                m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in t["t"])
                (m.transform.rotation.x, m.transform.rotation.y,
                 m.transform.rotation.z, m.transform.rotation.w) = (float(v) for v in t["q"])
                msgs.append(m)
            if msgs:
                self.static.sendTransform(msgs)
            self.get_logger().info("camera frames: %s" % ", ".join(
                "%s->%s" % (t["parent"], t["child"]) for t in header.get("transforms", [])) or "none received")
            return

        if kind == "depth":
            msg = Image()
            msg.header.stamp = ros_time(header["stamp"])
            msg.header.frame_id = header["frame_id"]
            msg.height, msg.width = int(header["height"]), int(header["width"])
            msg.encoding = "16UC1"
            msg.is_bigendian = 1 if header.get("bigendian") else 0
            msg.step = int(header["step"])
            # array.array, not bytes: see as_uint8 below.
            msg.data = as_uint8(payload)
            self.depth_pub.publish(msg)
            info = camera_info(header["info"], header["stamp"]) if header.get("info") else None
            if info is not None:
                self.depth_info_pub.publish(info)

            n = self.args.preview
            if n > 1 and info is not None:
                dtype = ">u2" if msg.is_bigendian else "<u2"
                full = np.frombuffer(payload, dtype=dtype).reshape(msg.height, msg.step // 2)
                small = np.ascontiguousarray(full[::n, :msg.width:n])
                p = Image()
                p.header = msg.header
                p.height, p.width = small.shape
                p.encoding = "16UC1"
                p.is_bigendian = msg.is_bigendian
                p.step = p.width * 2
                p.data = as_uint8(small.tobytes())
                self.preview_pub.publish(p)
                self.preview_info_pub.publish(scaled_info(info, n))
            self.counts["depth"] += 1

        elif kind == "color":
            msg = CompressedImage()
            msg.header.stamp = ros_time(header["stamp"])
            msg.header.frame_id = header["frame_id"]
            msg.format = "jpeg"
            msg.data = as_uint8(payload)
            self.color_pub.publish(msg)
            if header.get("info"):
                self.color_info_pub.publish(camera_info(header["info"], header["stamp"]))
            self.counts["color"] += 1

    def run(self):
        last = time.time()
        prev = dict(self.counts)
        while rclpy.ok():
            try:
                sock = socket.create_connection((self.args.host, self.args.port), timeout=5)
            except OSError as e:
                self.get_logger().warn("cannot reach the relay at %s:%d (%s). Is realsense_start.sh running on the robot? Retrying."
                                       % (self.args.host, self.args.port, e))
                time.sleep(3)
                continue
            sock.settimeout(10)
            self.get_logger().info("connected to the robot's camera relay")
            try:
                while rclpy.ok():
                    hlen = struct.unpack(">I", recv_exact(sock, 4))[0]
                    header = json.loads(recv_exact(sock, hlen))
                    plen = struct.unpack(">I", recv_exact(sock, 4))[0]
                    payload = recv_exact(sock, plen) if plen else b""
                    self.on_packet(header, payload)
                    rclpy.spin_once(self, timeout_sec=0)
                    now = time.time()
                    if now - last >= 5:
                        rates = {k: (self.counts[k] - prev[k]) / (now - last) for k in self.counts}
                        self.get_logger().info("depth %.1f/s  colour %.1f/s" % (rates["depth"], rates["color"]))
                        last, prev = now, dict(self.counts)
            except (OSError, ConnectionError, ValueError) as e:
                self.get_logger().warn("lost the relay: %s. Reconnecting." % e)
            finally:
                sock.close()
            time.sleep(1)


def main():
    ap = argparse.ArgumentParser(description="publish the robot's depth camera as ROS 2 topics")
    ap.add_argument("--host", default="192.168.123.18")
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--preview", type=int, default=4,
                    help="also publish every Nth pixel as a light preview; 1 turns it off")
    args = ap.parse_args()
    rclpy.init()
    node = Client(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
