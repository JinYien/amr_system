import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

livox_config = os.path.join(get_package_share_directory("livox_driver"), "config", "config.yaml")
oakd_config = os.path.join(get_package_share_directory("oakd_driver"), "config", "config.yaml")
teensy_config = os.path.join(get_package_share_directory("teensy_driver"), "config", "config.yaml")
sound_config = os.path.join(get_package_share_directory("sound_driver"), "config", "config.yaml")
web_config = os.path.join(get_package_share_directory("web_interface"), "config", "config.yaml")

urdf_path = PathJoinSubstitution([get_package_share_directory("robot_system"), "urdf", "robot.urdf"])
robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)


def generate_launch_description():
    livox_node = Node(
        package="livox_driver",
        executable="livox_driver_node",
        output="log",
        parameters=[{"config_path": livox_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 1",
    )

    oakd_node = Node(
        package="oakd_driver",
        executable="oakd_driver_node",
        output="log",
        parameters=[{"config_path": oakd_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 1",
    )

    teensy_node = Node(
        package="teensy_driver",
        executable="teensy_driver_node",
        output="log",
        parameters=[{"config_path": teensy_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 4",
    )

    sound_node = Node(
        package="sound_driver",
        executable="sound_driver_node",
        output="log",
        parameters=[{"config_path": sound_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 0",
    )

    state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "publish_frequency": 30.0}],
        output="log",
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 0",
    )

    web_node = Node(
        package="web_interface",
        executable="web_interface_node",
        output="log",
        parameters=[{"config_path": web_config}],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix="taskset -c 0",
    )

    return LaunchDescription(
        [
            livox_node,
            oakd_node,
            teensy_node,
            sound_node,
            state_node,
            web_node,
        ]
    )
