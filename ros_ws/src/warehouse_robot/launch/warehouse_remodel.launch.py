#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    gazebo_master = SetEnvironmentVariable(
        name="GAZEBO_MASTER_URI",
        value="http://127.0.0.1:11346"
    )
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    world = os.path.join(
        pkg,
        "worlds",
        "warehouse_remodel.world"
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "gzserver.launch.py"
            )
        ),
        launch_arguments={
            "world": world,
            "verbose": "false",
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "gzclient.launch.py"
            )
        )
    )

    ur5e_station = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg,
                "launch",
                "ur5e_station.launch.py"
            )
        )
    )

    tron1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg,
                "launch",
                "spawn_tron1_classic.launch.py"
            )
        )
    )

    tb3_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg,
                "launch",
                "tb3_fleet.launch.py"
            )
        )
    )

    tiago_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg,
                "launch",
                "tiago_fleet.launch.py"
            )
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

    delayed_robots = TimerAction(
        period=5.0,
        actions=[
            tron1,
            tb3_fleet,
            tiago_fleet,
        ],
    )

    delayed_application = TimerAction(
        period=12.0,
        actions=[
            bottle_detector,
            side_grasp_verifier,
            tron1_mission_fsm,
        ],
    )

    return LaunchDescription([
        gazebo_master,
        gzserver,
        gzclient,
        ur5e_station,
        delayed_robots,
        delayed_application,
    ])
