#!/usr/bin/env python3
"""
Warehouse Remodel Launch File
=============================
PROJECT MASTER v5.3.2 — Smart Fulfillment Center Remodeled World Launch

Launches Gazebo with the new warehouse_remodel.world, spawns ur5e_station,
spawn_tron1, bottle_detector, controllers, and fleet managers.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    world = os.path.join(
        pkg,
        "worlds",
        "warehouse_remodel.world"
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "verbose": "false",
        }.items()
    )

    ur5e_station = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "ur5e_station.launch.py")
        )
    )

    tron1a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "spawn_tron1.launch.py")
        )
    )

    bottle_detector = Node(
        package="warehouse_robot",
        executable="bottle_detector",
        name="bottle_detector_node",
        output="screen",
    )

    side_grasp_verifier = Node(
        package="warehouse_robot",
        executable="side_grasp_verifier",
        name="side_grasp_verifier_node",
        output="screen",
    )

    tron1_mission_fsm = Node(
        package="warehouse_robot",
        executable="tron1_mission_fsm",
        name="tron1_mission_fsm_node",
        output="screen",
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[
            tron1a,
            bottle_detector,
            side_grasp_verifier,
            tron1_mission_fsm,
        ],
    )

    return LaunchDescription([
        gzserver,
        ur5e_station,
        delayed_spawn,
    ])
