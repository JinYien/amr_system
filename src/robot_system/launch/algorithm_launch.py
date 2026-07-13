import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

fastlio_config = os.path.join(get_package_share_directory("fastlio"), "config", "config.yaml")
adapter_config = os.path.join(get_package_share_directory("fastlio_adapter"), "config", "config.yaml")
converter_config = os.path.join(get_package_share_directory("fastlio_converter"), "config", "config.yaml")
intersection_config = os.path.join(get_package_share_directory("intersection"), "config", "config.yaml")
system_config = os.path.join(get_package_share_directory("robot_system"), "config", "config.yaml")


def generate_launch_description():
    fastlio_node = Node(
        package="fastlio",
        executable="fastlio_node",
        output="log",
        parameters=[{"config_path": fastlio_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 2",
    )

    adapter_node = Node(
        package="fastlio_adapter",
        executable="fastlio_adapter_node",
        output="log",
        parameters=[{"config_path": adapter_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 3",
    )

    converter_node = Node(
        package="fastlio_converter",
        executable="fastlio_converter_node",
        output="log",
        parameters=[{"config_path": converter_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 3",
    )

    intersection_node = Node(
        package="intersection",
        executable="intersection_node",
        output="log",
        parameters=[{"config_path": intersection_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 3",
    )

    system_node = Node(
        package="robot_system",
        executable="robot_system_node",
        output="screen",
        parameters=[{"config_path": system_config}],
        prefix="taskset -c 4",
    )

    return LaunchDescription(
        [
            fastlio_node,
            adapter_node,
            converter_node,
            # intersection_node,
            system_node,
        ]
    )
