"""
Intel RealSense depth off the Go2's Jetson, without installing librealsense.

Why this exists
---------------
The Jetson has no internet and runs Ubuntu 20.04 / ROS Foxy / Python 3.8, so
installing librealsense + ros-foxy-realsense2-camera there is an offline .deb
exercise. But a RealSense exposes its depth stream as an ordinary UVC device in
the **Z16** pixel format, and the OpenCV already on the Jetson can read that
directly -- no driver, no download.

This follows the same shape as lidar_relay.py and camera_server.py: the Jetson
does the minimum (grab frames, push bytes), and all ROS work happens on the
laptop where we have Humble and internet.

    # 1. FIRST, find out what the camera actually offers. Read-only, safe.
    python3 depth_relay.py --probe

    # 2. Then serve the depth node it identified (example: /dev/video6)
    python3 depth_relay.py --device 6 --port 8091

On the laptop, `depth_client.py` turns the stream into a ROS 2 topic.

What you get, and what you don't
--------------------------------
You get raw 16-bit depth in millimetres, which is enough to record, visualise
and run SLAM against. You do NOT get librealsense's extras: no factory
intrinsics, no depth<->colour alignment, no post-processing filters. If you need
those, install the real driver (see docs/downloads-and-storage.md, Plan B).

UNTESTED -- written 2026-09-03 without the robot present. Run --probe first.
"""
import argparse
import socket
import struct
import sys
import time

import cv2
import numpy as np

# Z16: 16 bits per pixel, depth in millimetres. This is what RealSense depth
# nodes advertise over UVC.
FOURCC_Z16 = cv2.VideoWriter_fourcc('Z', '1', '6', ' ')

MAGIC = b'GO2D'          # frame header magic, so the client can resync
HEADER = '!4sIHHQ'       # magic, payload length, width, height, timestamp_us


def probe(max_index=16):
    """Enumerate video nodes and report which one can deliver Z16 depth."""
    print("Probing /dev/video0 .. /dev/video%d\n" % max_index)
    findings = []

    for idx in range(max_index + 1):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        # Ask for raw 16-bit frames. CONVERT_RGB=0 is the critical part: without
        # it OpenCV helpfully mangles the depth data into a colour image.
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        cap.set(cv2.CAP_PROP_FOURCC, FOURCC_Z16)

        ok, frame = cap.read()
        if not ok or frame is None:
            print("  /dev/video%-2d  opens, but no frame" % idx)
            cap.release()
            continue

        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_s = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))
        is_depth = frame.dtype == np.uint16 or (frame.ndim == 2 and frame.itemsize == 2)

        note = "<-- DEPTH (Z16)" if is_depth else "colour/other"
        print("  /dev/video%-2d  fourcc=%-6s shape=%-16s dtype=%-8s %s"
              % (idx, fourcc_s.strip(), frame.shape, frame.dtype, note))

        if is_depth:
            d = frame.astype(np.uint16)
            nz = d[d > 0]
            if nz.size:
                print("               depth range %d..%d mm, %.0f%% valid pixels"
                      % (nz.min(), nz.max(), 100.0 * nz.size / d.size))
            else:
                print("               WARNING: every pixel is zero -- lens covered,"
                      " or this is not really depth")
            findings.append(idx)

        cap.release()

    print()
    if findings:
        print("Depth node(s) found: %s" % ", ".join("/dev/video%d" % i for i in findings))
        print("Serve it with:  python3 depth_relay.py --device %d" % findings[0])
    else:
        print("No Z16 depth node found.")
        print("The camera may need librealsense after all -- see")
        print("docs/downloads-and-storage.md, Plan B. Also try: v4l2-ctl --list-devices")
    return findings


def serve(device, port, width, height):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit("/dev/video%d would not open" % device)

    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, FOURCC_Z16)
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(2)
    print("serving depth from /dev/video%d on 0.0.0.0:%d  (Ctrl-C to stop)"
          % (device, port))

    frames = 0
    t0 = time.time()

    while True:
        conn, addr = srv.accept()
        print("client %s connected" % (addr,))
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            while True:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.005)
                    continue

                d = np.ascontiguousarray(frame.astype(np.uint16))
                h, w = d.shape[:2]
                payload = d.tobytes()
                head = struct.pack(HEADER, MAGIC, len(payload), w, h,
                                   int(time.time() * 1e6))
                conn.sendall(head + payload)

                frames += 1
                if frames % 300 == 0:
                    dt = time.time() - t0
                    print("  %d frames, %.1f fps, %dx%d" % (frames, frames / dt, w, h))
        except (BrokenPipeError, ConnectionResetError):
            print("client %s disconnected" % (addr,))
        finally:
            conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true",
                    help="enumerate video nodes and identify the depth one, then exit")
    ap.add_argument("--device", type=int, default=None,
                    help="/dev/videoN index to serve")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--width", type=int, default=0, help="0 = leave the driver default")
    ap.add_argument("--height", type=int, default=0)
    args = ap.parse_args()

    if args.probe or args.device is None:
        found = probe()
        if args.probe:
            return
        if not found:
            sys.exit(1)
        args.device = found[0]

    serve(args.device, args.port, args.width, args.height)


if __name__ == "__main__":
    main()
