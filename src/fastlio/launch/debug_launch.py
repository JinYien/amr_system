import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

fastlio_config = os.path.join(get_package_share_directory("fastlio"), "config", "config.yaml")
adapter_config = os.path.join(get_package_share_directory("fastlio_adapter"), "config", "config.yaml")
converter_config = os.path.join(get_package_share_directory("fastlio_converter"), "config", "config.yaml")
rviz_config = os.path.join(get_package_share_directory("fastlio"), "config", "debug.rviz")


def generate_launch_description():
    fastlio_node = Node(
        package="fastlio",
        executable="fastlio_node",
        output="screen",
        parameters=[{"config_path": fastlio_config}],
    )
    adapter_node = Node(
        package="fastlio_adapter",
        executable="fastlio_adapter_node",
        output="screen",
        parameters=[{"config_path": adapter_config}],
    )
    converter_node = Node(
        package="fastlio_converter",
        executable="fastlio_converter_node",
        output="screen",
        parameters=[{"config_path": converter_config}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            fastlio_node,
            adapter_node,
            converter_node,
            rviz_node,
        ]
    )
