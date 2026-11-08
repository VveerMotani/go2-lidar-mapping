"""
Check the robot's odometry against a tape measure, and draw its path.

    python3 ~/go2-lidar-mapping/notes/odom_check.py

Needs odom_publisher.py running (go2-start.sh starts it). Restart this script
to zero the numbers before each test.

Prints once a second, measured from where the robot was when this started:

    straight   straight-line distance from the start      -> the 2 m walk test
    path       total distance travelled along the way
    turned     total heading change, counting past 360     -> the spin test
    heading    current heading minus starting heading, -180..180
    z          height change (should stay near 0 on a flat floor)

Publishes /odom_path (nav_msgs/Path in the odom frame) so Foxglove's 3D panel
can draw the trail. Foxglove documents no history setting for Odometry topics,
so without this the pose shows only as a single arrow.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# A new path point every 5 cm. At 300 Hz of odometry a raw trail would be
# tens of thousands of poses within minutes, for no visible difference.
PATH_SPACING = 0.05


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class OdomCheck(Node):
    def __init__(self):
        super().__init__("go2_odom_check")
        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.path_pub = self.create_publisher(Path, "/odom_path", 10)
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.start = None
        self.last = None
        self.length = 0.0
        self.turned = 0.0
        self.create_timer(1.0, self.report)
        print("waiting for /odom ...", flush=True)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        if self.start is None:
            self.start = (p.x, p.y, p.z, yaw)
            self.last = (p.x, p.y, yaw)
            self.now = (p.x, p.y, p.z, yaw)
            self.add_path_point(msg)
            print("zeroed. start x=%.2f y=%.2f heading=%.0f deg"
                  % (p.x, p.y, math.degrees(yaw)), flush=True)
            return
        lx, ly, lyaw = self.last
        self.length += math.hypot(p.x - lx, p.y - ly)
        self.turned += wrap(yaw - lyaw)
        self.last = (p.x, p.y, yaw)
        self.now = (p.x, p.y, p.z, yaw)

        tail = self.path.poses[-1].pose.position
        if math.hypot(p.x - tail.x, p.y - tail.y) >= PATH_SPACING:
            self.add_path_point(msg)

    def add_path_point(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)

    def report(self):
        if self.start is None:
            return
        sx, sy, sz, syaw = self.start
        x, y, z, yaw = self.now
        print("straight %5.2f m | path %5.2f m | turned %+6.0f deg | "
              "heading %+5.0f deg | z %+.2f m"
              % (math.hypot(x - sx, y - sy), self.length,
                 math.degrees(self.turned), math.degrees(wrap(yaw - syaw)),
                 z - sz), flush=True)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
