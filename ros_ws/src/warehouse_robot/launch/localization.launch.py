#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    map_yaml = os.path.join(
        pkg,
        "maps",
        "warehouse.yaml"
    )

    amcl_yaml = os.path.join(
        pkg,
        "config",
        "amcl.yaml"
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml},
            {"use_sim_time": True},
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        output="screen",
        parameters=[amcl_yaml],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
            }
        ],
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle,
    ])
