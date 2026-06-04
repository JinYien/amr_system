#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("amr_system"), "launch")

    sensor_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, "sensor_launch.py")))
    algorithm_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, "algorithm_launch.py")))

    return LaunchDescription([sensor_launch, algorithm_launch])
