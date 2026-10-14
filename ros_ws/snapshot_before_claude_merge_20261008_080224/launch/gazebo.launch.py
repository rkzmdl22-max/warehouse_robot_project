#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    __import__("os").path.join(
                        pkg,
                        "launch",
                        "warehouse_remodel.launch.py"
                    )
                )
            )
        )
    ])
