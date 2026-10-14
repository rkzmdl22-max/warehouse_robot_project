#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    amcl_params = os.path.join(pkg, "config", "amcl.yaml")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    map_yaml = os.path.join(pkg, "maps", "warehouse.yaml")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": True,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params, {"use_sim_time": True}],
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": True}],
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": True}],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": True}],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": True}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "behavior_server",
                ],
            }
        ],
    )

    return LaunchDescription([
        map_server,
        amcl,
        planner,
        controller,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
    ])
