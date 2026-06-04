#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

ROBOT_STATE_PUBLISH_FREQUENCY_HZ = 30.0


def generate_launch_description():
    mid_360_path = PathJoinSubstitution([get_package_share_directory("amr_system"), "config", "mid_360.json"])
    lidar_path = PathJoinSubstitution([get_package_share_directory("amr_system"), "config", "lidar.yaml"])
    robot_path = os.path.join(get_package_share_directory("amr_robot"), "config", "robot.yaml")
    teensy_path = os.path.join(get_package_share_directory("amr_teensy"), "config", "teensy.yaml")
    urdf_path = PathJoinSubstitution([get_package_share_directory("amr_robot"), "urdf", "robot.urdf"])

    robot_config = DeclareLaunchArgument("robot", default_value=robot_path)
    teensy_config = DeclareLaunchArgument("teensy", default_value=teensy_path)
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    return LaunchDescription(
        [
            teensy_config,
            robot_config,
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="lidar_node",
                parameters=[{"user_config_path": mid_360_path}, lidar_path],
                output="log",
                prefix="taskset -c 1",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="amr_teensy",
                executable="teensy",
                name="teensy_node",
                parameters=[
                    {"config": LaunchConfiguration("teensy")},
                    {"robot": LaunchConfiguration("robot")},
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "publish_frequency": ROBOT_STATE_PUBLISH_FREQUENCY_HZ}],
                output="log",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
