"""
Read a walk's depth.stream (written on the robot by jetson/depth_recorder.py).

    from depth_stream import DepthStream
    ds = DepthStream("~/go2-lidar-mapping/walks/<walk>/depth.stream")
    ds.depth_count, ds.color_count, ds.transforms, ds.depth_info
    for frame in ds.depth_frames():       # time order
        frame.t, frame.image (uint16 mm, HxW), frame.info
    for frame in ds.color_frames():
        frame.t, frame.jpeg (bytes), frame.info

Also writes the format, for tests: StreamWriter.

Packets: 4-byte big-endian header length, JSON header, 4-byte big-endian
payload length, payload. See depth_recorder.py for the kinds.
"""
import json
import struct
import zlib

import numpy as np


class Frame(object):
    __slots__ = ("kind", "t", "header", "offset", "length", "stream")

    def __init__(self, kind, t, header, offset, length, stream):
        self.kind, self.t, self.header = kind, t, header
        self.offset, self.length, self.stream = offset, length, stream

    @property
    def info(self):
        return self.header.get("info")

    def payload(self):
        with open(self.stream.path, "rb") as fh:
            fh.seek(self.offset)
            return fh.read(self.length)

    @property
    def image(self):
        """Depth in millimetres, uint16, shape (height, width)."""
        return decode_depth(self.header, self.payload())

    @property
    def jpeg(self):
        return self.payload()


def decode_depth(header, payload):
    w, h = header["width"], header["height"]
    enc = header.get("encoding", "png16")
    if enc == "png16":
        import cv2
        img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
        return img.astype(np.uint16).reshape(h, w)
    if enc == "zlib16":
        return np.frombuffer(zlib.decompress(payload), "<u2").reshape(h, w)
    if enc == "16UC1":
        return np.frombuffer(payload, "<u2").reshape(h, w)
    raise ValueError("unknown depth encoding %s" % enc)


class DepthStream(object):
    def __init__(self, path):
        self.path = path
        self.info = {}
        self.frames = []
        self.transforms = {}
        with open(path, "rb") as fh:
            total = _size(fh)
            while True:
                head = fh.read(4)
                if len(head) < 4:
                    break
                hl = struct.unpack(">I", head)[0]
                hb = fh.read(hl)
                pl_raw = fh.read(4)
                if len(hb) < hl or len(pl_raw) < 4:
                    break                                   # cut off mid-packet
                pl = struct.unpack(">I", pl_raw)[0]
                offset = fh.tell()
                if offset + pl > total:
                    break                                   # cut off mid-payload
                fh.seek(pl, 1)
                header = json.loads(hb.decode("utf-8"))
                kind = header.get("kind")
                if kind == "info":
                    self.info = header
                elif kind == "tf":
                    for tr in header.get("transforms", []):
                        self.transforms[tr["child"]] = tr
                elif kind in ("depth", "color"):
                    s = header["stamp"]
                    self.frames.append(Frame(kind, s[0] + s[1] * 1e-9, header, offset, pl, self))
        self.frames.sort(key=lambda f: f.t)

    def depth_frames(self):
        return [f for f in self.frames if f.kind == "depth"]

    def color_frames(self):
        return [f for f in self.frames if f.kind == "color"]

    @property
    def depth_count(self):
        return sum(1 for f in self.frames if f.kind == "depth")

    @property
    def color_count(self):
        return sum(1 for f in self.frames if f.kind == "color")

    @property
    def depth_info(self):
        for f in self.frames:
            if f.kind == "depth" and f.info:
                return f.info
        return None

    def optical_in_camera_link(self, optical_frame):
        """4x4 pose of an optical frame in camera_link, from the camera's own
        static transforms (Intel's factory values). None if the chain is not
        in the stream."""
        from scipy.spatial.transform import Rotation
        T = np.eye(4)
        frame = optical_frame
        for _ in range(10):
            if frame == "camera_link":
                return T
            tr = self.transforms.get(frame)
            if tr is None:
                return None
            M = np.eye(4)
            M[:3, :3] = Rotation.from_quat(tr["q"]).as_matrix()
            M[:3, 3] = tr["t"]
            T = M @ T
            frame = tr["parent"]
        return None


def _size(fh):
    here = fh.tell()
    fh.seek(0, 2)
    end = fh.tell()
    fh.seek(here)
    return end


class StreamWriter(object):
    """Writes the same format as depth_recorder.py. Used by tests."""

    def __init__(self, path):
        self.fh = open(path, "wb")
        self.packet({"kind": "info", "format": "go2-depth-stream", "version": 1,
                     "encoding": "png16"})

    def packet(self, header, payload=b""):
        h = json.dumps(header).encode("utf-8")
        self.fh.write(struct.pack(">I", len(h)) + h + struct.pack(">I", len(payload)) + payload)

    def depth(self, t, image_mm, info, frame_id="camera_depth_optical_frame"):
        import cv2
        ok, png = cv2.imencode(".png", image_mm.astype(np.uint16),
                               [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
        self.packet({"kind": "depth", "stamp": [int(t), int(round((t % 1) * 1e9)) % 1000000000],
                     "frame_id": frame_id, "width": image_mm.shape[1],
                     "height": image_mm.shape[0], "encoding": "png16", "info": info},
                    png.tobytes())

    def tf(self, transforms):
        self.packet({"kind": "tf", "transforms": transforms})

    def close(self):
        self.fh.close()


# --------------------------------------------------------------------------
# Where the camera sits on the robot
# --------------------------------------------------------------------------
MOUNT_FILE = "camera_mount.json"


def find_camera_mount(session):
    """camera_mount.json from the walk folder, else from the walks folder.

    {"xyz": [x, y, z],            metres, camera_link in base_link
     "rpy_deg": [roll, pitch, yaw],  degrees; positive pitch looks down
     "source": "...", ...}

    Unitree does not publish the camera's position, so this is measured on the
    robot and refined against the LiDAR map by depth_mount.py. Returns
    (4x4 matrix, dict, path) or (None, None, None).
    """
    import os
    from scipy.spatial.transform import Rotation
    # GO2_CONFIG_DIR last: the Docker image keeps this robot's calibrations there.
    for folder in (session, os.path.dirname(os.path.abspath(session)),
                   os.environ.get("GO2_CONFIG_DIR", "")):
        if not folder:
            continue
        path = os.path.join(folder, MOUNT_FILE)
        if os.path.isfile(path):
            with open(path) as fh:
                m = json.load(fh)
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler("xyz", m.get("rpy_deg", [0, 0, 0]),
                                            degrees=True).as_matrix()
            T[:3, 3] = m["xyz"]
            return T, m, path
    return None, None, None
