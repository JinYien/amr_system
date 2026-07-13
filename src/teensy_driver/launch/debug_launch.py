import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

DRIVE_COMMAND = 'Move drive motor: ros2 topic pub --once /command/drive custom_message/msg/DriveCommand "{left_velocity: 50.0, right_velocity: 50.0}"'
HANDLE_COMMAND = "Move handle motor: ros2 topic pub --once /command/handle custom_message/msg/HandleCommand \"{mode: 'pulse'}\""

config = os.path.join(get_package_share_directory("teensy_driver"), "config")


def generate_launch_description():
    teensy_node = Node(
        package="teensy_driver",
        executable="teensy_driver_node",
        output="screen",
        parameters=[{"config_path": os.path.join(config, "config.yaml")}],
    )

    return LaunchDescription(
        [
            teensy_node,
            LogInfo(msg=DRIVE_COMMAND),
            LogInfo(msg=HANDLE_COMMAND),
        ]
    )
