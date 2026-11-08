"""
Capture one frame from the XT16 and draw it top-down, so it can be compared
against the actual room.

Run inside the RL venv (it has matplotlib and, via --system-site-packages,
rclpy):

    source ~/unitree_ws/venv-rl/bin/activate
    python3 lidar_snapshot.py --out ~/go2-lidar-mapping/assets/lidar_topdown.png
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def xyz_from_cloud(msg):
    """Pull x, y, z out of a PointCloud2, whatever else it carries.

    Not using sensor_msgs_py.point_cloud2.read_points_numpy: in ROS 2 Humble
    it asserts that EVERY field in the cloud shares one datatype, even when you
    ask for only some of them. The Hesai driver publishes x, y, z and intensity
    as float32, ring as uint16 and timestamp as float64, so that assertion
    always fires and the helper is unusable here.

    Reading the buffer directly costs a few lines and works for any layout.
    """
    offsets = {f.name: f.offset for f in msg.fields}
    for name in ("x", "y", "z"):
        if name not in offsets:
            raise SystemExit("cloud has no '%s' field" % name)

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    raw = raw.reshape(-1, msg.point_step)

    cols = []
    for name in ("x", "y", "z"):
        off = offsets[name]
        cols.append(np.frombuffer(
            np.ascontiguousarray(raw[:, off:off + 4]).tobytes(),
            dtype=np.float32))
    pts = np.stack(cols, axis=-1)

    # Drop points with no return. The sensor reports those as exact zeros
    # rather than as NaN, and plotting them puts a false blob at the origin.
    good = np.isfinite(pts).all(axis=1) & (np.abs(pts).sum(axis=1) > 1e-6)
    return pts[good]


class Grab(Node):
    def __init__(self, topic):
        super().__init__("lidar_snapshot")
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.points = None
        self.create_subscription(PointCloud2, topic, self.cb, qos)

    def cb(self, msg):
        if self.points is None:
            self.points = xyz_from_cloud(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/lidar_points")
    ap.add_argument("--out", default="lidar_topdown.png")
    ap.add_argument("--range", type=float, default=10.0, help="plot half-width, metres")
    args = ap.parse_args()

    rclpy.init()
    node = Grab(args.topic)
    for _ in range(400):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.points is not None:
            break
    node.destroy_node()
    rclpy.shutdown()

    if node.points is None or len(node.points) == 0:
        raise SystemExit("no points received on %s" % args.topic)

    p = node.points
    finite = np.isfinite(p).all(axis=1)
    p = p[finite]
    d = np.linalg.norm(p[:, :2], axis=1)
    p = p[d > 0.05]                       # drop returns from the sensor itself
    d = d[d > 0.05]

    print("points:        %d" % len(p))
    print("range  min/med/max: %.2f / %.2f / %.2f m" % (d.min(), np.median(d), d.max()))
    print("height min/max:     %.2f / %.2f m" % (p[:, 2].min(), p[:, 2].max()))
    for pct in (50, 90, 99):
        print("  %dth pct range: %.2f m" % (pct, np.percentile(d, pct)))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6.2), facecolor="#0f1115")
    for ax in (ax1, ax2):
        ax.set_facecolor("#0f1115")
        ax.tick_params(colors="#8b93a1", labelsize=8)
        for s in ax.spines.values():
            s.set_color("#252a34")

    ax1.scatter(p[:, 0], p[:, 1], s=0.4, c=p[:, 2], cmap="viridis")
    ax1.set_xlim(-args.range, args.range)
    ax1.set_ylim(-args.range, args.range)
    ax1.set_aspect("equal")
    ax1.set_title("top-down (x/y, coloured by height)", color="#e6e8ec", fontsize=10)
    ax1.grid(color="#252a34", linewidth=0.5)
    ax1.plot(0, 0, marker="o", color="#f87171", markersize=5)

    ax2.scatter(p[:, 0], p[:, 2], s=0.4, c=p[:, 2], cmap="viridis")
    ax2.set_xlim(-args.range, args.range)
    ax2.set_ylim(-3, 3)
    ax2.set_aspect("equal")
    ax2.set_title("side view (x/z)", color="#e6e8ec", fontsize=10)
    ax2.grid(color="#252a34", linewidth=0.5)

    fig.suptitle("Hesai XT16 -- one frame, %d points" % len(p),
                 color="#e6e8ec", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110, facecolor="#0f1115")
    print("saved %s" % args.out)


if __name__ == "__main__":
    main()
