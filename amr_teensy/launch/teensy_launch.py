#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("amr_teensy"), "config", "teensy.yaml")
    robot_path = os.path.join(get_package_share_directory("amr_robot"), "config", "robot.yaml")

    config = DeclareLaunchArgument("config", default_value=config_path)
    robot = DeclareLaunchArgument("robot", default_value=robot_path)

    return LaunchDescription(
        [
            config,
            robot,
            Node(
                package="amr_teensy",
                executable="teensy",
                name="teensy_node",
                parameters=[
                    {"config": LaunchConfiguration("config")},
                    {"robot": LaunchConfiguration("robot")},
                ],
                output="screen",
            ),
        ]
    )
