import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

intersection_config = os.path.join(get_package_share_directory("intersection"), "config", "config.yaml")


def generate_launch_description():
    intersection_node = Node(
        package="intersection",
        executable="intersection_node",
        output="screen",
        parameters=[{"config_path": intersection_config}],
    )

    return LaunchDescription(
        [
            intersection_node,
        ]
    )
