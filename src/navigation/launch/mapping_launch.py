import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

config = os.path.join(get_package_share_directory("navigation"), "config", "map.yaml")


# CPU core plan: see the root README; slam_toolbox gets cores 4 and 5
# because nav2 is not running during a mapping session, so the motion and
# planning cores are free and the ceres solver can use both


def generate_launch_description():

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[config],
        prefix="taskset -c 4,5",
    )

    return LaunchDescription([slam_node])
