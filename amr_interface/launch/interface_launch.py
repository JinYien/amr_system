#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("amr_interface"), "config", "interface.yaml")
    config = DeclareLaunchArgument("config", default_value=config_path)

    return LaunchDescription(
        [
            config,
            Node(
                package="amr_interface",
                executable="interface",
                name="interface_node",
                parameters=[{"config": LaunchConfiguration("config")}],
                output="screen",
            ),
        ]
    )
