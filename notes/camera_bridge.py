"""
Put the Go2's cameras into ROS 2, so they appear in Foxglove beside the LiDAR.

    python3 camera_bridge.py                          # both dock cameras
    python3 camera_bridge.py --cameras 2              # just one
    python3 camera_bridge.py --host 192.168.123.18:8080
    python3 camera_bridge.py --head                   # + the robot's own camera

Publishes, for each camera N:

    /camera<N>/image_raw/compressed     sensor_msgs/CompressedImage

and with --head, additionally:

    /camera/head/image_raw/compressed   sensor_msgs/CompressedImage

TWO DIFFERENT CAMERAS
---------------------
Worth being clear, because they are easy to confuse. The numbered ones are the
RealSense on the dock, read as plain /dev/video devices by camera_server.py and
served over HTTP. The head one is the Go2's own built-in forward camera, which
the robot broadcasts as multicast H.264 whether anyone is listening or not.
They are separate hardware on separate paths; --head adds a camera rather than
replacing anything. See GstHeadReader below.

WHY THIS IS NEEDED
------------------
camera_server.py on the robot serves video the way a webcam page does: an HTTP
stream of JPEG frames. A browser understands that, which is why the dashboard
shows the cameras. Foxglove does not -- it shows ROS topics arriving through
foxglove_bridge, and nothing was publishing the cameras as ROS topics. So you
could see the point cloud in Foxglove and the cameras in the dashboard, but
never both in one window. This closes that gap.

Nothing changes on the robot. This reads exactly the same stream the dashboard
reads, and republishes it.

WHY CompressedImage RATHER THAN Image
-------------------------------------
The frames arrive already JPEG-compressed. Publishing them as CompressedImage
passes the bytes straight through -- no decoding here, no re-encoding, and
about a tenth of the network traffic of raw images. Foxglove's Image panel
displays CompressedImage directly. It also means this program needs no OpenCV,
so it will run anywhere ROS 2 does.

If a node downstream needs raw images, run image_transport's republish:

    ros2 run image_transport republish compressed raw \\
        --ros-args -r in/compressed:=/camera2/image_raw/compressed \\
                   -r out:=/camera2/image_raw
"""
import argparse
import os
import shlex
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

DEFAULT_HOST = "192.168.123.18:8080"


class MjpegReader(object):
    """Pulls JPEG frames out of an HTTP multipart stream.

    The server sends, over and over:

        --frameboundary
        Content-Type: image/jpeg
        Content-Length: 12345
        <blank line>
        <12345 bytes of JPEG>

    Content-Length is what makes this reliable: we read exactly that many bytes
    rather than hunting for the next boundary, so a boundary string appearing
    by chance inside the JPEG data cannot desynchronise the stream.
    """

    def __init__(self, url, timeout=10.0):
        self.url = url
        self.timeout = timeout
        self.stream = None

    def open(self):
        self.stream = urllib.request.urlopen(self.url, timeout=self.timeout)
        ctype = self.stream.headers.get("Content-Type", "")
        if "multipart" not in ctype:
            raise ValueError("expected a multipart stream, got %r" % ctype)

    def close(self):
        try:
            if self.stream:
                self.stream.close()
        except Exception:
            pass
        self.stream = None

    def frames(self):
        return multipart_frames(self.stream)


def multipart_frames(stream):
    """Yield one JPEG payload at a time from a multipart stream.

    Shared by the robot's HTTP camera server and by the GStreamer pipeline
    below, because multipartmux emits the same framing an MJPEG server does.
    """
    while True:
        length = None
        # Read the part's headers, ending at the blank line.
        while True:
            line = stream.readline()
            if not line:
                return                       # stream closed
            line = line.strip()
            if not line:
                break                        # blank line: headers done
            if line.lower().startswith(b"content-length:"):
                try:
                    length = int(line.split(b":", 1)[1])
                except ValueError:
                    length = None

        if length is None:
            # Without a length we cannot know where the frame ends, and
            # guessing would eventually hand a truncated image downstream.
            raise ValueError("stream has no Content-Length per frame")

        data = stream.read(length)
        if len(data) < length:
            return                           # truncated: treat as closed
        yield data
        stream.readline()                    # the CRLF after the payload


class GstHeadReader(object):
    """The robot's own head camera, over multicast H.264.

    This is a different camera from the ones camera_server.py serves. Those are
    the RealSense on the dock, read as plain /dev/video devices. This one is the
    Go2's built-in forward camera, and the robot broadcasts it whether or not
    anything is listening -- so it costs the dock nothing at all.

    From the SDK docs, Multimedia Services Interface:

        address 230.1.1.1  port 1720   RTP/H264
        1280 x 720 @ 15 Hz, FOV 100 degrees horizontal by 56 vertical

        "It is recommended to use a directed UDP interface for image
         transmission ... and it is not recommended to use DDS (because DDS
         only transmits videos encoded with H264, and users still need to
         perform video decoding)."

    Requires the robot's video_hub service to be running. service_switch.py
    --list will say.

    WHY A SUBPROCESS RATHER THAN OPENCV
    -----------------------------------
    Unitree's example uses cv2.VideoCapture with CAP_GSTREAMER. That needs an
    OpenCV built with GStreamer support, and this distro's is not
    (cv2.getBuildInformation() reports "GStreamer: NO"), which would mean
    rebuilding OpenCV from source to gain nothing.

    Instead we run gst-launch-1.0 and have it hand us finished JPEG frames in
    exactly the multipart framing the rest of this file already parses. No
    OpenCV, no Python GStreamer bindings, and the pipeline is a string you can
    paste into a terminal when it misbehaves.

    The cost is a decode plus a JPEG encode on this machine. At 720p15 that is
    negligible, and it keeps CompressedImage end to end so Foxglove displays it
    with no further work.

    THE PART THAT MAY NOT WORK
    --------------------------
    This is MULTICAST, and multicast through WSL2's mirrored networking is the
    one thing this whole stack has repeatedly had trouble with (decisions.md
    D5, D9). Tested here only against a synthetic sender on loopback. If no
    frames arrive from the real robot, check first whether the packets reach
    the machine at all:

        tcpdump -i any -n host 230.1.1.1

    and if they do not, run this pipeline on the dock instead and relay -- the
    same "keep the dock dumb, forward bytes" pattern lidar_relay.py uses.
    """

    PIPELINE = (
        "udpsrc address={address} port={port}{iface} ! queue ! "
        "application/x-rtp, media=video, encoding-name=H264 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "jpegenc quality={quality} ! multipartmux ! fdsink fd=1"
    )

    def __init__(self, address, port, iface=None, quality=85):
        self.address = address
        self.port = port
        self.iface = iface
        self.quality = quality
        self.proc = None
        self.stream = None
        self.last_error = None

    def pipeline(self):
        """The gst-launch pipeline string. Printed at startup so it can be
        pasted into a terminal when this misbehaves."""
        return self.PIPELINE.format(
            address=self.address, port=self.port, quality=self.quality,
            iface=(" multicast-iface=%s" % self.iface) if self.iface else "")

    def open(self):
        pipeline = self.pipeline()
        # shlex.split keeps the caps string ("application/x-rtp, media=video,
        # ...") in one argument, which a naive split on spaces would break.
        argv = ["gst-launch-1.0", "-q"] + shlex.split(pipeline)
        try:
            self.proc = subprocess.Popen(argv, stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, bufsize=0)
        except FileNotFoundError:
            raise OSError(
                "gst-launch-1.0 not found. Install it with the command from "
                "the SDK docs (Multimedia Services Interface): sudo apt-get "
                "install gstreamer1.0-tools gstreamer1.0-plugins-good "
                "gstreamer1.0-plugins-bad gstreamer1.0-libav")
        self.stream = self.proc.stdout

    def frames(self):
        return multipart_frames(self.stream)

    def close(self):
        if self.proc is None:
            return
        try:
            self.proc.terminate()
            self.proc.wait(timeout=3)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        # gst writes its complaints to stderr and says nothing on stdout, so a
        # silent failure here is usually explained there.
        try:
            err = self.proc.stderr.read(2000).decode("utf-8", "replace").strip()
            if err:
                self.last_error = err.splitlines()[-1]
        except Exception:
            pass
        self.proc = None


class CameraBridge(Node):
    def __init__(self, args):
        super().__init__("go2_camera_bridge")
        self.host = args.host
        self.stop = False

        # Sensor-data QoS: best effort, shallow queue. A viewer that falls
        # behind should get the newest frame, not a backlog of stale ones.
        qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cameras = []
        for cam in args.cameras:
            topic = "%s%d/image_raw/compressed" % (args.topic_prefix, cam)
            url = "http://%s/stream?cam=%d" % (self.host, cam)
            entry = {
                "id": cam,
                "url": url,
                "make_reader": (lambda u=url: MjpegReader(u)),
                "frame_id": "%s%d" % (args.frame_prefix, cam),
                "pub": self.create_publisher(CompressedImage, topic, qos),
                "count": 0, "errors": 0, "t0": time.time(), "last_error": None,
            }
            self.cameras.append(entry)
            self.get_logger().info("camera %d: %s -> %s" % (cam, entry["url"], topic))

        if args.head:
            reader = GstHeadReader(args.head_address, args.head_port,
                                   args.head_iface, args.head_quality)
            topic = "%s/image_raw/compressed" % args.head_topic
            entry = {
                "id": "head",
                "url": "udp://%s:%d (multicast, H.264)"
                       % (args.head_address, args.head_port),
                "make_reader": (lambda r=reader: r),
                "frame_id": args.head_frame,
                "pub": self.create_publisher(CompressedImage, topic, qos),
                "count": 0, "errors": 0, "t0": time.time(), "last_error": None,
            }
            self.cameras.append(entry)
            self.get_logger().info("head camera -> %s" % topic)
            self.get_logger().info("  pipeline: %s" % reader.pipeline())

        for entry in self.cameras:
            t = threading.Thread(target=self._pump, args=(entry,),
                                 name="cam%s" % entry["id"])
            t.daemon = True
            t.start()

        self.create_timer(5.0, self._report)

    def _pump(self, entry):
        """One camera, forever: connect, forward frames, reconnect on failure."""
        reader = entry["make_reader"]()
        while not self.stop:
            try:
                reader.open()
                entry["last_error"] = None
                for jpeg in reader.frames():
                    if self.stop:
                        break
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = entry["frame_id"]
                    msg.format = "jpeg"
                    msg.data = jpeg
                    entry["pub"].publish(msg)
                    entry["count"] += 1
            except (urllib.error.URLError, socket.timeout, ValueError,
                    OSError) as e:
                entry["errors"] += 1
                entry["last_error"] = str(e)
            finally:
                reader.close()
                # GstHeadReader stashes gst's own last stderr line while
                # closing. It is far more informative than "stream closed".
                if getattr(reader, "last_error", None):
                    entry["last_error"] = reader.last_error
            if not self.stop:
                # The robot may simply not be up yet. Keep trying quietly
                # rather than exiting, so this can be started in any order.
                time.sleep(2.0)

    def _report(self):
        for e in self.cameras:
            elapsed = time.time() - e["t0"]
            if e["count"]:
                self.get_logger().info(
                    "camera %s: %d frames, %.1f fps" % (e["id"], e["count"],
                                                        e["count"] / elapsed))
            else:
                self.get_logger().warn(
                    "camera %s: nothing yet from %s%s"
                    % (e["id"], e["url"],
                       (" -- " + e["last_error"]) if e["last_error"] else ""))


def main():
    ap = argparse.ArgumentParser(
        description="republish the Go2's MJPEG cameras as ROS 2 topics")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="host:port of camera_server.py (default %s)" % DEFAULT_HOST)
    ap.add_argument("--cameras", default="2,4",
                    help="comma-separated camera numbers (default 2,4)")
    ap.add_argument("--topic-prefix", default="/camera")
    ap.add_argument("--frame-prefix", default="camera")

    # The robot's own head camera, which is a different device from the ones
    # camera_server.py serves. Off by default: it is multicast, and multicast
    # through WSL is the least reliable thing in this stack.
    ap.add_argument("--head", action="store_true",
                    help="also publish the Go2's built-in head camera, from "
                         "the multicast H.264 stream (needs gst-launch-1.0 and "
                         "the robot's video_hub service)")
    ap.add_argument("--head-address", default="230.1.1.1",
                    help="multicast group (SDK docs: 230.1.1.1)")
    ap.add_argument("--head-port", type=int, default=1720,
                    help="SDK docs: 1720")
    ap.add_argument("--head-iface", default=None,
                    help="interface to join the multicast group on; defaults "
                         "to $UNITREE_IFACE")
    ap.add_argument("--head-topic", default="/camera/head")
    ap.add_argument("--head-frame", default="camera_head")
    ap.add_argument("--head-quality", type=int, default=85,
                    help="JPEG quality for the re-encode, 1-100")
    args = ap.parse_args()

    try:
        args.cameras = [int(c) for c in args.cameras.split(",") if c.strip()]
    except ValueError:
        raise SystemExit("--cameras wants numbers, e.g. 2,4")

    if args.head and not args.head_iface:
        # setup.env.sh works this out already; no reason to make anyone repeat
        # it, and picking the wrong interface for a multicast join is a silent
        # failure rather than an error.
        args.head_iface = os.environ.get("UNITREE_IFACE")
        if not args.head_iface:
            print("note: no --head-iface and UNITREE_IFACE is unset, so the "
                  "multicast join will use the default route. Source "
                  "setup.env.sh, or pass --head-iface.")

    rclpy.init()
    node = CameraBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop = True
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
