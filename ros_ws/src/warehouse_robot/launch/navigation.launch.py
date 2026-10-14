#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    nav2_param = os.path.join(
        pkg,
        "config",
        "nav2_params.yaml"
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg,
                "launch",
                "localization.launch.py"
            )
        )
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        output="screen",
        parameters=[nav2_param],
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[nav2_param],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "planner_server",
                    "controller_server",
                ],
            }
        ],
    )

    return LaunchDescription([
        localization,
        planner,
        controller,
        lifecycle,
    ])
