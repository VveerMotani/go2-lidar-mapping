"""
Start the Hesai XT16 driver WITHOUT RViz.

    ros2 launch ~/go2-lidar-mapping/notes/hesai_no_rviz.launch.py

The driver's own launch file, hesai_ros_driver/launch/start.py, starts two
nodes: the driver, and an rviz2 with its bundled config. There is no argument to
turn the second one off -- it is hardcoded. This file is that launch file with
the rviz2 node removed and nothing else changed: same package, same executable,
same namespace, so the topics come out identically.

WHY NOT RVIZ
------------
The 3D view for this project is Foxglove on Windows, for reasons recorded in
docs/decisions.md D2: rviz2 under WSLg goes through a Direct3D 12 translation
layer and died with Ogre "Vertex Buffer: Out of memory" while the launch file
restarted it in a loop -- which is what the blinking point cloud was. Foxglove
renders on the RTX 4070 through the normal Windows driver instead.

So an rviz2 nobody is looking at is not merely clutter: it is a second consumer
of the whole point cloud, competing for the same GPU memory that took it down
before.

Nothing under src/HesaiLidar_ROS_2.0 is modified. Their launch file is left
exactly as shipped, in case it is ever wanted.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace="hesai_ros_driver",
            package="hesai_ros_driver",
            executable="hesai_ros_driver_node",
            output="screen",
        ),
    ])
