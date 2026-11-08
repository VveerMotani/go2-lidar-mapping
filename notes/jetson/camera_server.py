"""
Live camera feed off the Go2, viewable in a browser on the laptop.

Runs on the robot's Jetson. Uses only the standard library plus the OpenCV that
is already installed there -- nothing to download, which matters because the
Jetson has no internet.

Serves MJPEG over plain TCP, so it is unaffected by the WSL multicast/firewall
problems that block DDS.

    python3 camera_server.py                # cameras 2 and 4, port 8080
    python3 camera_server.py --port 9000 --cameras 2

Then open http://192.168.123.18:8080 on the laptop.
"""
import argparse
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2

BOUNDARY = "frameboundary"


class Camera:
    """Grabs continuously so HTTP clients always get the newest frame."""

    def __init__(self, index, width, height, quality):
        self.index = index
        self.quality = quality
        self.lock = threading.Lock()
        self.jpeg = None
        self.frames = 0
        self.started = time.time()

        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"/dev/video{index} would not open")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        while True:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode(".jpg", frame, params)
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()
                    self.frames += 1

    def latest(self):
        with self.lock:
            return self.jpeg

    def fps(self):
        elapsed = time.time() - self.started
        return self.frames / elapsed if elapsed > 0 else 0.0


CAMERAS = {}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_):
        pass                                  # keep the console readable

    def do_GET(self):
        if self.path == "/":
            self._index()
        elif self.path.startswith("/stream"):
            self._stream()
        elif self.path == "/stats":
            self._stats()
        else:
            self.send_error(404)

    def _cam_arg(self):
        if "?" in self.path and "cam=" in self.path:
            try:
                return int(self.path.split("cam=")[1].split("&")[0])
            except ValueError:
                pass
        return next(iter(CAMERAS))

    def _index(self):
        tiles = "".join(
            f'<figure><img src="/stream?cam={i}" alt="camera {i}">'
            f'<figcaption>/dev/video{i}</figcaption></figure>'
            for i in CAMERAS
        )
        html = f"""<!doctype html><meta charset="utf-8">
<title>Go2 live cameras</title>
<style>
  :root {{ color-scheme: dark; }}
  body {{ margin:0; background:#111; color:#eee;
         font:14px system-ui,sans-serif; padding:16px; }}
  h1 {{ font-size:16px; font-weight:600; margin:0 0 4px; }}
  p  {{ margin:0 0 16px; color:#888; }}
  .grid {{ display:flex; flex-wrap:wrap; gap:16px; }}
  figure {{ margin:0; }}
  img {{ display:block; max-width:100%; border-radius:6px; background:#000; }}
  figcaption {{ color:#888; padding-top:6px; }}
</style>
<h1>Unitree Go2 &mdash; live cameras</h1>
<p>Streaming from the robot&rsquo;s Jetson over plain TCP.</p>
<div class="grid">{tiles}</div>
"""
        body = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stats(self):
        body = "\n".join(
            f"/dev/video{i}: {c.frames} frames, {c.fps():.1f} fps"
            for i, c in CAMERAS.items()
        ).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        cam = CAMERAS.get(self._cam_arg())
        if cam is None:
            self.send_error(404, "no such camera")
            return
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        try:
            while True:
                jpeg = cam.latest()
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                time.sleep(1 / 30)
        except (BrokenPipeError, ConnectionResetError):
            pass                              # viewer closed the tab


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--cameras", default="2,4")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--quality", type=int, default=80)
    args = ap.parse_args()

    for spec in args.cameras.split(","):
        idx = int(spec)
        try:
            CAMERAS[idx] = Camera(idx, args.width, args.height, args.quality)
            print(f"opened /dev/video{idx}")
        except RuntimeError as exc:
            print(f"skipping /dev/video{idx}: {exc}")

    if not CAMERAS:
        raise SystemExit("no cameras opened")

    print(f"serving on http://0.0.0.0:{args.port}  (Ctrl-C to stop)")
    ThreadedHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
