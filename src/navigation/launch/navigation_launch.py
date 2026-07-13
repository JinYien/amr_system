import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

navigation_config = os.path.join(get_package_share_directory("navigation"), "config", "navigation.yaml")
map_file = os.path.join(get_package_share_directory("navigation"), "map", "map.yaml")
behavior_tree_file = os.path.join(get_package_share_directory("navigation"), "config", "behavior_tree.xml")
through_poses_bt_file = os.path.join(get_package_share_directory("navigation"), "config", "navigate_through_poses_bt.xml")

LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"]


# CPU core plan: see the root README; amcl sits with the adapter and
# converter on core 3 (the localization chain that feeds it), the
# controller and smoother sit with robot_system and the teensy driver on
# core 4 (the 20 Hz motion chain), the planner, behaviors and bt run on
# core 5 where their replanning bursts cannot delay a wheel command, and
# the light map server and lifecycle managers stay on core 0


def generate_launch_description():
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[navigation_config, {"yaml_filename": map_file}],
        prefix="taskset -c 0",
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[navigation_config],
        prefix="taskset -c 3",
    )

    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[navigation_config],
        remappings=[("cmd_vel", "cmd_vel_nav")],
        prefix="taskset -c 4",
    )

    planner_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[navigation_config],
        prefix="taskset -c 5",
    )

    behavior_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[navigation_config],
        remappings=[("cmd_vel", "cmd_vel_nav")],
        prefix="taskset -c 5",
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            navigation_config,
            {
                "default_nav_to_pose_bt_xml": behavior_tree_file,
                "default_nav_through_poses_bt_xml": through_poses_bt_file,
            },
        ],
        prefix="taskset -c 5",
    )

    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[navigation_config],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/command/robot")],
        prefix="taskset -c 4",
    )

    localization_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"autostart": True, "node_names": LOCALIZATION_NODES}],
        prefix="taskset -c 0",
    )

    navigation_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"autostart": True, "node_names": NAVIGATION_NODES}],
        prefix="taskset -c 0",
    )

    return LaunchDescription(
        [
            map_server_node,
            amcl_node,
            controller_node,
            planner_node,
            behavior_node,
            bt_navigator_node,
            velocity_smoother_node,
            localization_manager_node,
            navigation_manager_node,
        ]
    )
