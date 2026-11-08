#!/usr/bin/env python3
"""
Forward the RealSense's ROS 1 topics to the laptop, over one TCP connection.

    python3 realsense_ros1_relay.py            (started by realsense_start.sh)

Runs on the robot's onboard computer, next to Intel's ROS 1 camera node.
The laptop runs notes/realsense_client.py, which turns the stream back into
ROS 2 topics for Foxglove, depth_image_proc and mapping.

WHY THIS EXISTS
---------------
On 2026-09-17 two camera nodes were tried on this robot:
  * ROS 1 Noetic, Intel library 2.50.0 (preinstalled): depth streams at
    640x480x15, with the factory calibration.
  * ROS 2 Foxy, Intel library 2.51.1 (installed that day): allocated about
    15 GB within 20 seconds on every start, with any settings, and was killed
    by the kernel. Probably the camera's firmware (5.15) being newer than the
    library.
The laptop runs ROS 2, and ROS 1 and ROS 2 cannot talk to each other directly.
Rather than update the camera software or its firmware, this relays the node
that works. Nothing is estimated: the calibration and the camera's internal
frame positions are Intel's own, forwarded exactly as the node publishes them.

WIRE FORMAT
-----------
Repeated packets:  4-byte big-endian header length, JSON header,
                   4-byte big-endian payload length, payload bytes.
Header "kind" is one of:
  depth   16UC1 raw pixels, plus that camera's calibration
  color   JPEG, plus that camera's calibration
  tf      the camera's static transforms, no payload (sent on connect)

Only the newest frame of each kind is sent. If the laptop falls behind, older
frames are dropped rather than queued, so what it shows stays current.

Python 3.8 and ROS 1 Noetic, as installed on the robot.
"""
import argparse
import json
import os
import socket
import struct
import threading
import time

import rospy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

try:
    import cv2
    import numpy as np
    HAVE_CV = True
except ImportError:
    HAVE_CV = False


def stamp_of(header):
    return [int(header.stamp.secs), int(header.stamp.nsecs)]


def info_dict(info):
    if info is None:
        return None
    return {
        "width": info.width, "height": info.height,
        "distortion_model": info.distortion_model,
        "D": list(info.D), "K": list(info.K), "R": list(info.R), "P": list(info.P),
        "frame_id": info.header.frame_id,
    }


class Relay(object):
    def __init__(self, args):
        self.args = args
        self.lock = threading.Condition()
        self.latest = {}          # kind -> (header dict, payload bytes)
        self.infos = {}           # kind -> CameraInfo
        self.statics = {}         # child frame -> transform dict
        self.sent = {"depth": 0, "color": 0}
        # Frames the camera node delivered, and what the laptop is being sent,
        # written once a second to STATUS_FILE for the dashboard's Sensors panel.
        self.received = {"depth": 0, "color": 0}
        self.connected = False
        self.mode = {}

        rospy.Subscriber(args.depth_topic, Image, self.on_depth, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(args.depth_info, CameraInfo, lambda m: self.infos.__setitem__("depth", m), queue_size=1)
        if args.color:
            rospy.Subscriber(args.color_topic, Image, self.on_color, queue_size=1, buff_size=2 ** 24)
            rospy.Subscriber(args.color_info, CameraInfo, lambda m: self.infos.__setitem__("color", m), queue_size=1)
        rospy.Subscriber("/tf_static", TFMessage, self.on_static, queue_size=10)

    # --- inputs -----------------------------------------------------------
    def on_depth(self, msg):
        self.received["depth"] += 1
        self.mode["depth"] = "%dx%d" % (msg.width, msg.height)
        if msg.encoding not in ("16UC1", "mono16"):
            rospy.logwarn_throttle(10, "unexpected depth encoding %s" % msg.encoding)
            return
        header = {"kind": "depth", "stamp": stamp_of(msg.header), "frame_id": msg.header.frame_id,
                  "width": msg.width, "height": msg.height, "encoding": "16UC1",
                  "step": msg.step, "bigendian": bool(msg.is_bigendian),
                  "info": info_dict(self.infos.get("depth"))}
        self.put("depth", header, bytes(msg.data))

    def on_color(self, msg):
        self.received["color"] += 1
        self.mode["color"] = "%dx%d" % (msg.width, msg.height)
        if not HAVE_CV:
            return
        channels = 3 if msg.encoding in ("rgb8", "bgr8") else None
        if channels is None:
            rospy.logwarn_throttle(10, "unexpected colour encoding %s" % msg.encoding)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]                # JPEG encoder expects BGR
        ok, jpg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality])
        if not ok:
            return
        header = {"kind": "color", "stamp": stamp_of(msg.header), "frame_id": msg.header.frame_id,
                  "width": msg.width, "height": msg.height, "format": "jpeg",
                  "info": info_dict(self.infos.get("color"))}
        self.put("color", header, jpg.tobytes())

    def on_static(self, msg):
        for t in msg.transforms:
            tr, rot = t.transform.translation, t.transform.rotation
            self.statics[t.child_frame_id] = {
                "parent": t.header.frame_id, "child": t.child_frame_id,
                "t": [tr.x, tr.y, tr.z], "q": [rot.x, rot.y, rot.z, rot.w]}

    def put(self, kind, header, payload):
        with self.lock:
            self.latest[kind] = (header, payload)
            self.lock.notify()

    # --- output -----------------------------------------------------------
    @staticmethod
    def send_packet(sock, header, payload=b""):
        h = json.dumps(header).encode("utf-8")
        sock.sendall(struct.pack(">I", len(h)) + h + struct.pack(">I", len(payload)))
        if payload:
            sock.sendall(payload)

    def serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self.args.port))
        srv.listen(1)
        srv.settimeout(1.0)
        rospy.loginfo("relay listening on port %d" % self.args.port)
        while not rospy.is_shutdown():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            rospy.loginfo("laptop connected from %s" % addr[0])
            self.connected = True
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                self.stream(conn)
            except (OSError, socket.error) as e:
                rospy.loginfo("laptop disconnected: %s" % e)
            finally:
                conn.close()
                self.connected = False

    def stream(self, conn):
        # The static transforms first, waiting briefly for the latched message.
        deadline = time.time() + 3.0
        while not self.statics and time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.1)
        self.send_packet(conn, {"kind": "tf", "transforms": list(self.statics.values())})

        last_report = time.time()
        while not rospy.is_shutdown():
            with self.lock:
                while not self.latest and not rospy.is_shutdown():
                    self.lock.wait(0.5)
                batch, self.latest = self.latest, {}
            for kind, (header, payload) in batch.items():
                self.send_packet(conn, header, payload)
                self.sent[kind] = self.sent.get(kind, 0) + 1
            if time.time() - last_report > 10:
                rospy.loginfo("sent %s" % self.sent)
                last_report = time.time()


STATUS_FILE = "/tmp/realsense_relay_status.json"


def write_status(relay):
    """Once a second: frame rates in and out, for the dashboard to read."""
    prev_in, prev_out, prev_t = dict(relay.received), dict(relay.sent), time.time()
    while not rospy.is_shutdown():
        time.sleep(1.0)
        now = time.time()
        dt = max(now - prev_t, 1e-3)
        status = {
            "time": now,
            "connected": relay.connected,
            "mode": relay.mode,
            "fps_in": {k: round((relay.received[k] - prev_in.get(k, 0)) / dt, 1) for k in relay.received},
            "fps_out": {k: round((relay.sent.get(k, 0) - prev_out.get(k, 0)) / dt, 1) for k in relay.sent},
        }
        prev_in, prev_out, prev_t = dict(relay.received), dict(relay.sent), now
        try:
            tmp = STATUS_FILE + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(status, fh)
            os.replace(tmp, STATUS_FILE)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description="relay the RealSense ROS 1 topics to the laptop")
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    ap.add_argument("--depth-info", default="/camera/depth/camera_info")
    ap.add_argument("--color", action="store_true", help="also relay colour, as JPEG")
    ap.add_argument("--color-topic", default="/camera/color/image_raw")
    ap.add_argument("--color-info", default="/camera/color/camera_info")
    ap.add_argument("--jpeg-quality", type=int, default=85)
    args, _ = ap.parse_known_args()             # tolerate roslaunch-style extras

    rospy.init_node("realsense_relay", anonymous=False, disable_signals=False)
    relay = Relay(args)
    t = threading.Thread(target=relay.serve, daemon=True)
    t.start()
    threading.Thread(target=write_status, args=(relay,), daemon=True).start()
    rospy.spin()


if __name__ == "__main__":
    main()
