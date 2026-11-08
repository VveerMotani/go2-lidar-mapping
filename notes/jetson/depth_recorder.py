#!/usr/bin/env python3
"""
Record the RealSense depth (and optionally colour) into a walk's folder.

    python3 depth_recorder.py --out ~/go2dev/sessions/<walk>        (record_session.py starts it)

Runs on the robot, next to Intel's ROS 1 camera node started by
realsense_start.sh. record_session.py starts it automatically when that node
is running, so turning the camera on from the dashboard's Sensors panel before
a walk is all it takes.

WRITES
------
  depth.stream          every depth frame, lossless, with the camera's factory
                        calibration and its internal frame positions
  depth_status.json     frames, rate, drops and size, once a second
  depth_summary.json    the same, final, written on stop

FILE FORMAT (the same packets realsense_ros1_relay.py sends over TCP)
-----------
Repeated:  4-byte big-endian header length, JSON header,
           4-byte big-endian payload length, payload.
The first packet is kind "info" (format name and version). Then:
  depth   PNG, 16-bit, millimetres ("encoding": "png16"), or zlib-compressed
          raw little-endian uint16 ("zlib16") if OpenCV is missing. Lossless
          either way.
  color   JPEG (only with --color)
  tf      the camera's static transforms, whenever they arrive
Header "stamp" is [secs, nsecs] as the camera node stamped the frame: the
robot's clock, the same one the LiDAR, IMU and odometry are recorded with.

KEEPING UP
----------
Frames are compressed on a worker thread from a short queue. If compression
falls behind, the newest frame replaces the oldest waiting one and the drop is
counted, so the recording never lags or grows memory; the summary says how
many were dropped.

Python 3.8 and ROS 1 Noetic, as installed on the robot.
"""
import argparse
import array
import json
import os
import signal
import struct
import threading
import time
import zlib
from collections import deque

import rospy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

try:
    import cv2
    import numpy as np
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

FORMAT = {"kind": "info", "format": "go2-depth-stream", "version": 1}


def info_dict(info):
    if info is None:
        return None
    return {"width": info.width, "height": info.height,
            "distortion_model": info.distortion_model,
            "D": list(info.D), "K": list(info.K), "R": list(info.R), "P": list(info.P),
            "frame_id": info.header.frame_id}


class Recorder(object):
    def __init__(self, args):
        self.args = args
        self.path = os.path.join(args.out, "depth.stream")
        self.fh = open(self.path, "wb")
        self.lock = threading.Lock()                 # file writes
        self.cond = threading.Condition()
        self.queue = deque()
        self.infos = {}
        self.statics = {}
        self.frames = {"depth": 0, "color": 0}
        self.dropped = {"depth": 0, "color": 0}
        self.received = {"depth": 0, "color": 0}
        self.bytes = 0
        self.mode = None
        self.started = time.time()
        self.first_stamp = self.last_stamp = None
        self.running = True
        self.write_packet(dict(FORMAT, encoding="png16" if HAVE_CV else "zlib16",
                               started=self.started))

        rospy.Subscriber(args.depth_topic, Image, self.on_depth, queue_size=2, buff_size=2 ** 24)
        rospy.Subscriber(args.depth_info, CameraInfo,
                         lambda m: self.infos.__setitem__("depth", m), queue_size=1)
        if args.color:
            rospy.Subscriber(args.color_topic, Image, self.on_color, queue_size=2, buff_size=2 ** 24)
            rospy.Subscriber(args.color_info, CameraInfo,
                             lambda m: self.infos.__setitem__("color", m), queue_size=1)
        rospy.Subscriber("/tf_static", TFMessage, self.on_static, queue_size=10)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    # --- file -------------------------------------------------------------
    def write_packet(self, header, payload=b""):
        h = json.dumps(header).encode("utf-8")
        with self.lock:
            self.fh.write(struct.pack(">I", len(h)) + h + struct.pack(">I", len(payload)))
            if payload:
                self.fh.write(payload)
            self.bytes += 8 + len(h) + len(payload)

    # --- inputs -----------------------------------------------------------
    def enqueue(self, kind, msg):
        self.received[kind] += 1
        with self.cond:
            # Keep at most a few frames of each kind waiting. Replace the
            # oldest rather than block the ROS callback.
            same = [i for i, (k, _) in enumerate(self.queue) if k == kind]
            if len(same) >= self.args.queue:
                del self.queue[same[0]]
                self.dropped[kind] += 1
            self.queue.append((kind, msg))
            self.cond.notify()

    def on_depth(self, msg):
        self.enqueue("depth", msg)

    def on_color(self, msg):
        self.enqueue("color", msg)

    def on_static(self, msg):
        changed = False
        for t in msg.transforms:
            tr, rot = t.transform.translation, t.transform.rotation
            entry = {"parent": t.header.frame_id, "child": t.child_frame_id,
                     "t": [tr.x, tr.y, tr.z], "q": [rot.x, rot.y, rot.z, rot.w]}
            if self.statics.get(t.child_frame_id) != entry:
                self.statics[t.child_frame_id] = entry
                changed = True
        if changed:
            self.write_packet({"kind": "tf", "transforms": list(self.statics.values())})

    # --- compression --------------------------------------------------------
    def work(self):
        while True:
            with self.cond:
                while not self.queue and self.running:
                    self.cond.wait(0.5)
                if not self.queue:
                    if not self.running:
                        return
                    continue
                kind, msg = self.queue.popleft()
            try:
                if kind == "depth":
                    self.write_depth(msg)
                else:
                    self.write_color(msg)
            except Exception as e:                       # never kill the recording
                rospy.logwarn_throttle(10, "could not write a %s frame: %s" % (kind, e))

    def write_depth(self, msg):
        if msg.encoding not in ("16UC1", "mono16"):
            rospy.logwarn_throttle(10, "unexpected depth encoding %s" % msg.encoding)
            return
        w, h = msg.width, msg.height
        raw = bytes(msg.data)
        if HAVE_CV:
            img = np.frombuffer(raw, dtype=">u2" if msg.is_bigendian else "<u2")
            img = img.reshape(h, msg.step // 2 if msg.step else w)[:, :w]
            ok, png = cv2.imencode(".png", img.astype(np.uint16),
                                   [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
            if not ok:
                return
            payload, encoding = png.tobytes(), "png16"
        else:
            if msg.is_bigendian:
                swapped = array.array("H", raw)
                swapped.byteswap()
                raw = swapped.tobytes()
            payload, encoding = zlib.compress(raw, 1), "zlib16"
        stamp = [int(msg.header.stamp.secs), int(msg.header.stamp.nsecs)]
        self.write_packet({"kind": "depth", "stamp": stamp, "frame_id": msg.header.frame_id,
                           "width": w, "height": h, "encoding": encoding,
                           "info": info_dict(self.infos.get("depth"))}, payload)
        t = stamp[0] + stamp[1] * 1e-9
        self.first_stamp = t if self.first_stamp is None else self.first_stamp
        self.last_stamp = t
        self.mode = "%dx%d" % (w, h)
        self.frames["depth"] += 1

    def write_color(self, msg):
        if not HAVE_CV or msg.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        ok, jpg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality])
        if not ok:
            return
        stamp = [int(msg.header.stamp.secs), int(msg.header.stamp.nsecs)]
        self.write_packet({"kind": "color", "stamp": stamp, "frame_id": msg.header.frame_id,
                           "width": msg.width, "height": msg.height, "format": "jpeg",
                           "info": info_dict(self.infos.get("color"))}, jpg.tobytes())
        self.frames["color"] += 1

    # --- status -------------------------------------------------------------
    def summary(self, running=True):
        span = (self.last_stamp - self.first_stamp) if self.first_stamp else 0.0
        return {"time": time.time(), "running": running, "file": "depth.stream",
                "mode": self.mode, "encoding": "png16" if HAVE_CV else "zlib16",
                "frames": dict(self.frames), "dropped": dict(self.dropped),
                "received": dict(self.received),
                "depth_fps": round((self.frames["depth"] - 1) / span, 1) if span > 0 else 0.0,
                "seconds": round(time.time() - self.started, 1),
                "bytes": self.bytes, "color": bool(self.args.color),
                "tf_frames": sorted(self.statics)}

    def write_json(self, name, running):
        path = os.path.join(self.args.out, name)
        tmp = path + ".tmp"
        try:
            with open(tmp, "w") as fh:
                json.dump(self.summary(running), fh, indent=1)
            os.replace(tmp, path)
            os.chmod(path, 0o644)
        except OSError:
            pass

    def stop(self):
        with self.cond:
            self.running = False
            self.cond.notify()
        self.worker.join(timeout=10.0)
        with self.lock:
            self.fh.flush()
            self.fh.close()
        try:
            os.chmod(self.path, 0o644)
        except OSError:
            pass
        self.write_json("depth_status.json", False)
        self.write_json("depth_summary.json", False)


def main():
    ap = argparse.ArgumentParser(description="record RealSense depth into a walk folder")
    ap.add_argument("--out", required=True, help="the walk's folder")
    ap.add_argument("--color", action="store_true", help="also record colour, as JPEG")
    ap.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    ap.add_argument("--depth-info", default="/camera/depth/camera_info")
    ap.add_argument("--color-topic", default="/camera/color/image_raw")
    ap.add_argument("--color-info", default="/camera/color/camera_info")
    ap.add_argument("--jpeg-quality", type=int, default=85)
    ap.add_argument("--queue", type=int, default=4,
                    help="frames of each kind allowed to wait for compression")
    args, _ = ap.parse_known_args()
    if not os.path.isdir(args.out):
        raise SystemExit("no such folder: %s" % args.out)

    rospy.init_node("depth_recorder", anonymous=True, disable_signals=True)
    rec = Recorder(args)
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    print("recording depth to %s (%s)" % (rec.path, "png16" if HAVE_CV else "zlib16"), flush=True)
    while not stopping.is_set() and not rospy.is_shutdown():
        stopping.wait(1.0)
        rec.write_json("depth_status.json", True)
    rec.stop()
    s = rec.summary(False)
    print("depth: %d frames (%d dropped), colour: %d, %.1f MB"
          % (s["frames"]["depth"], s["dropped"]["depth"], s["frames"]["color"],
             s["bytes"] / 1e6), flush=True)
    rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
