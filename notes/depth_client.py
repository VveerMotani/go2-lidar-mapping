"""
The laptop half of the depth camera: turn depth_relay.py's stream into ROS 2.

    python3 depth_client.py                        # 192.168.123.18:8091
    python3 depth_client.py --cloud                # also publish a point cloud
    python3 depth_client.py --host 192.168.123.18 --port 8091

Publishes:

    /depth/image_raw     sensor_msgs/Image      16UC1, millimetres
    /depth/camera_info   sensor_msgs/CameraInfo
    /depth/points        sensor_msgs/PointCloud2  (only with --cloud)

HOW THIS FITS TOGETHER
----------------------
    robot:   depth_relay.py  reads the camera's depth device, sends raw frames
    laptop:  depth_client.py reads those frames, publishes them as ROS topics
    windows: Foxglove displays them

The split is the same one used everywhere else in this project: the Jetson does
the minimum, because it has no internet and cannot easily gain packages, and
everything that needs ROS happens on the laptop.

STATUS -- READ THIS BEFORE TRUSTING ANYTHING BELOW
--------------------------------------------------
The far end of this, depth_relay.py, has never been run against the real
camera. Nobody has yet confirmed the robot's depth camera exposes a plain Z16
depth device at all. Run this first, on the robot:

    python3 depth_relay.py --probe

If that reports no depth node, this program has nothing to talk to and the
camera needs its manufacturer's driver instead -- see
docs/downloads-and-storage.md, Plan B.

This program itself has been tested only against a synthetic sender producing
the same wire format, not against a camera.

ABOUT THE LENS NUMBERS
----------------------
Turning depth into a point cloud needs the camera's focal length and centre
point. Those come from the factory calibration, which is exactly what
depth_relay.py cannot read -- it deliberately avoids the manufacturer's driver.
So they are ESTIMATED here from an assumed field of view, and the estimate is
printed at startup. Distances will be roughly right and the shape will be
roughly right, but do not measure anything important against it until the real
values are known. Pass --hfov, or --fx/--fy/--cx/--cy, once you have them.
"""
import argparse
import math
import socket
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

# The wire format, matching depth_relay.py exactly. Change one, change both.
MAGIC = b"GO2D"
HEADER = "!4sIHHQ"                 # magic, payload bytes, width, height, micros
HEADER_LEN = struct.calcsize(HEADER)


def recv_exactly(sock, n):
    """Read exactly n bytes, or return None if the connection ends first.

    A plain recv() returns whatever has arrived so far, which for a 600 kB
    depth frame is almost never the whole thing.
    """
    chunks = []
    remaining = n
    while remaining > 0:
        block = sock.recv(min(remaining, 1 << 20))
        if not block:
            return None
        chunks.append(block)
        remaining -= len(block)
    return b"".join(chunks)


class DepthClient(Node):
    def __init__(self, args):
        super().__init__("go2_depth_client")
        self.args = args
        self.frames = 0
        self.t0 = time.time()
        self.intrinsics = None          # filled once the first frame arrives
        self._grid = None               # cached pixel grid for the point cloud

        qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_image = self.create_publisher(Image, args.topic, qos)
        self.pub_info = self.create_publisher(
            CameraInfo, args.topic.rsplit("/", 1)[0] + "/camera_info", qos)
        self.pub_cloud = (self.create_publisher(PointCloud2, args.cloud_topic, qos)
                          if args.cloud else None)

    # -- intrinsics --------------------------------------------------------

    def _make_intrinsics(self, w, h):
        """Focal length and centre, from the arguments or estimated from FOV."""
        fx = self.args.fx
        fy = self.args.fy
        if fx is None:
            # A pinhole camera with horizontal field of view a and width w has
            # fx = (w/2) / tan(a/2). Rearranged from the usual projection.
            fx = (w / 2.0) / math.tan(math.radians(self.args.hfov) / 2.0)
            estimated = True
        else:
            estimated = False
        if fy is None:
            fy = fx                      # square pixels, true of most depth sensors
        cx = self.args.cx if self.args.cx is not None else w / 2.0
        cy = self.args.cy if self.args.cy is not None else h / 2.0

        if estimated:
            self.get_logger().warn(
                "lens numbers ESTIMATED from a %.0f degree field of view: "
                "fx=fy=%.1f, cx=%.1f, cy=%.1f. Shapes and distances will be "
                "approximately right, not exactly. Pass --fx/--fy/--cx/--cy "
                "once the real calibration is known."
                % (self.args.hfov, fx, cx, cy))
        else:
            self.get_logger().info("lens numbers given: fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
                                   % (fx, fy, cx, cy))
        return fx, fy, cx, cy

    def _camera_info(self, stamp, w, h):
        fx, fy, cx, cy = self.intrinsics
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.args.frame_id
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    # -- publishing --------------------------------------------------------

    def publish(self, depth_mm, w, h):
        stamp = self.get_clock().now().to_msg()

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.args.frame_id
        img.height, img.width = h, w
        # 16UC1 is the standard encoding for a depth image in millimetres,
        # which is what depth_image_proc and Foxglove both expect.
        img.encoding = "16UC1"
        img.is_bigendian = 0
        img.step = w * 2
        img.data = depth_mm.tobytes()
        self.pub_image.publish(img)
        self.pub_info.publish(self._camera_info(stamp, w, h))

        if self.pub_cloud is not None:
            self.pub_cloud.publish(self._to_cloud(depth_mm, w, h, stamp))

    def _to_cloud(self, depth_mm, w, h, stamp):
        """Project the depth image into 3D points.

        depth_image_proc can do this too, and is installed. Doing it here as
        well means one command produces something visible in Foxglove's 3D
        panel with nothing else running, which is what you want the first time
        you are trying to find out whether the camera works at all.
        """
        fx, fy, cx, cy = self.intrinsics
        step = max(1, self.args.cloud_decimate)

        d = depth_mm[::step, ::step].astype(np.float32) * 0.001   # mm -> metres
        rows, cols = d.shape

        # The pixel grid never changes, so build it once rather than per frame.
        # Worth doing, but do not expect much: measured against a 30 fps
        # source, caching it changed nothing. What actually limits the rate is
        # the sheer volume of point data being published, and the knob for that
        # is --cloud-decimate:
        #
        #     decimate 2   76,800 points   10 fps
        #     decimate 4   19,200 points   17 fps
        #     decimate 8    4,800 points   22 fps
        #     no cloud at all               22 fps
        #
        # 22 fps is where publishing the depth image alone tops out, so that is
        # the ceiling regardless. Running slower than the camera does not build
        # up lag: the connection simply pushes back on the robot, which then
        # skips camera frames rather than queueing them.
        if self._grid is None or self._grid[0].shape != d.shape:
            u = (np.arange(cols, dtype=np.float32) * step - cx) / fx
            v = (np.arange(rows, dtype=np.float32) * step - cy) / fy
            self._grid = np.meshgrid(u, v)
        uu, vv = self._grid

        # Zero means "no reading" on every depth sensor, not "at the lens", so
        # those pixels must be dropped rather than plotted at the origin.
        valid = (d > 0) & (d < self.args.max_range)
        z = d[valid]
        x = uu[valid] * z
        y = vv[valid] * z

        # Optical frame convention: x right, y down, z forward.
        points = np.stack([x, y, z], axis=-1).astype(np.float32)

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.args.frame_id
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.is_dense = True
        cloud.data = points.tobytes()
        return cloud

    # -- the connection ----------------------------------------------------

    def run(self):
        address = (self.args.host, self.args.port)
        while rclpy.ok():
            sock = None
            try:
                self.get_logger().info("connecting to %s:%d" % address)
                sock = socket.create_connection(address, timeout=10.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info("connected")
                self._read_forever(sock)
            except (OSError, socket.timeout) as e:
                self.get_logger().warn(
                    "%s -- is depth_relay.py running on the robot? "
                    "(python3 depth_relay.py --device N --port %d)"
                    % (e, self.args.port))
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
            if rclpy.ok():
                time.sleep(2.0)

    def _read_forever(self, sock):
        while rclpy.ok():
            head = recv_exactly(sock, HEADER_LEN)
            if head is None:
                self.get_logger().warn("stream ended")
                return

            magic, length, w, h, micros = struct.unpack(HEADER, head)
            if magic != MAGIC:
                # Nothing sensible can be done once the stream is misaligned;
                # reconnecting resynchronises from a known point.
                self.get_logger().error(
                    "bad frame marker %r -- reconnecting" % magic)
                return
            expected = w * h * 2
            if length != expected:
                self.get_logger().error(
                    "frame says %d bytes but %dx%d needs %d -- reconnecting"
                    % (length, w, h, expected))
                return

            payload = recv_exactly(sock, length)
            if payload is None:
                self.get_logger().warn("stream ended mid-frame")
                return

            if self.intrinsics is None:
                self.intrinsics = self._make_intrinsics(w, h)
                self.get_logger().info("first frame: %dx%d" % (w, h))

            depth = np.frombuffer(payload, dtype=np.uint16).reshape(h, w)
            self.publish(depth, w, h)

            self.frames += 1
            if self.frames % 60 == 0:
                elapsed = time.time() - self.t0
                nz = depth[depth > 0]
                self.get_logger().info(
                    "%d frames, %.1f fps, %.0f%% valid, %s"
                    % (self.frames, self.frames / elapsed,
                       100.0 * nz.size / depth.size,
                       ("%d..%d mm" % (nz.min(), nz.max())) if nz.size
                       else "ALL ZERO -- lens covered?"))


def main():
    ap = argparse.ArgumentParser(
        description="publish the Go2's depth stream as ROS 2 topics")
    ap.add_argument("--host", default="192.168.123.18")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--topic", default="/depth/image_raw")
    ap.add_argument("--cloud-topic", default="/depth/points")
    ap.add_argument("--frame-id", default="depth_camera")
    ap.add_argument("--cloud", action="store_true",
                    help="also publish a point cloud, viewable straight away "
                         "in Foxglove's 3D panel")
    ap.add_argument("--cloud-decimate", type=int, default=4,
                    help="take every Nth pixel for the cloud. Default 4, which "
                         "measured 19,200 points at 17 fps; 2 gives four times "
                         "the points at 10 fps, 8 gives a quarter at 22 fps")
    ap.add_argument("--max-range", type=float, default=10.0,
                    help="metres beyond which readings are dropped as noise")
    ap.add_argument("--hfov", type=float, default=87.0,
                    help="horizontal field of view in degrees, used only to "
                         "estimate the lens numbers when they are not given")
    ap.add_argument("--fx", type=float, default=None)
    ap.add_argument("--fy", type=float, default=None)
    ap.add_argument("--cx", type=float, default=None)
    ap.add_argument("--cy", type=float, default=None)
    args = ap.parse_args()

    rclpy.init()
    node = DepthClient(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
