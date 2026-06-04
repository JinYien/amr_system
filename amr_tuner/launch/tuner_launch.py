#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("amr_tuner"), "config", "tuner.yaml")
    config = DeclareLaunchArgument("config", default_value=config_path)

    return LaunchDescription(
        [
            config,
            Node(
                package="amr_tuner",
                executable="bridge",
                name="bridge_node",
                parameters=[{"config": LaunchConfiguration("config")}],
                output="screen",
            ),
            Node(
                package="amr_tuner",
                executable="tuner",
                name="tuner_node",
                parameters=[{"config": LaunchConfiguration("config")}],
                output="screen",
            ),
        ]
    )
