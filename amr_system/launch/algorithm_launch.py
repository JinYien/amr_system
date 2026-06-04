#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ROSBOARD_PORT = 8888


def generate_launch_description():
    command_path = os.path.join(get_package_share_directory("amr_system"), "config", "command.yaml")
    interface_path = os.path.join(get_package_share_directory("amr_interface"), "config", "interface.yaml")
    intersection_path = os.path.join(get_package_share_directory("amr_intersection"), "config", "intersection.yaml")
    robot_path = os.path.join(get_package_share_directory("amr_robot"), "config", "robot.yaml")

    command_config = DeclareLaunchArgument("command", default_value=command_path)
    interface_config = DeclareLaunchArgument("interface", default_value=interface_path)
    intersection_config = DeclareLaunchArgument("intersection", default_value=intersection_path)
    robot_config = DeclareLaunchArgument("robot", default_value=robot_path)

    return LaunchDescription(
        [
            command_config,
            interface_config,
            intersection_config,
            robot_config,
            Node(
                package="amr_system",
                executable="command",
                parameters=[
                    {"config": LaunchConfiguration("command")},
                    {"robot": LaunchConfiguration("robot")},
                ],
                output="screen",
            ),
            Node(
                package="amr_intersection",
                executable="intersection",
                name="intersection_node",
                parameters=[
                    {"config": LaunchConfiguration("intersection")},
                    {"robot": LaunchConfiguration("robot")},
                ],
                output="log",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="amr_interface",
                executable="interface",
                name="interface_node",
                parameters=[{"config": LaunchConfiguration("interface")}],
                output="log",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="rosboard",
                executable="rosboard_node",
                name="rosboard_node",
                parameters=[{"port": ROSBOARD_PORT}],
                output="log",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
