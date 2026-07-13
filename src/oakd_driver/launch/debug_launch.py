import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

config = os.path.join(get_package_share_directory("oakd_driver"), "config")


def generate_launch_description():
    driver_node = Node(
        package="oakd_driver",
        executable="oakd_driver_node",
        output="screen",
        parameters=[{"config_path": os.path.join(config, "config.yaml")}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(config, "debug.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            driver_node,
            rviz_node,
        ]
    )
