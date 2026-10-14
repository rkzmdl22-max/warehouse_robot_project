#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "gazebo.launch.py")
        )
    )

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tb3_fleet.launch.py")
        )
    )

    # TIAGo is optional in the current Docker environment.
    # Preserve the existing launch when tiago_description is installed,
    # but do not abort the entire warehouse launch when it is unavailable.
    try:
        get_package_share_directory("tiago_description")
        tiago = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "spawn_tiago.launch.py")
            )
        )
    except Exception:
        tiago = None

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "controllers.launch.py")
        )
    )

    fleet_manager = Node(
        package="warehouse_robot",
        executable="fleet_manager",
        output="screen",
    )

    mission_manager = Node(
        package="warehouse_robot",
        executable="mission_manager",
        output="screen",
    )

    tb3_1_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tb3_1_nav2.launch.py")
        )
    )

    tb3_2_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tb3_2_nav2.launch.py")
        )
    )

    tb3_3_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tb3_3_nav2.launch.py")
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

    ur5e_station = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "ur5e_station.launch.py")
        )
    )

    tron1a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "spawn_tron1_classic.launch.py")
        )
    )

    # Gazebo ROS Factory (/spawn_entity) readiness margin.
    # Keep robot definitions unchanged; delay only the spawn orchestration.
    delayed_spawn_actions = [
        fleet,
        tron1a,
    ]

    if tiago is not None:
        delayed_spawn_actions.insert(1, tiago)

    delayed_spawn = TimerAction(
        period=8.0,
        actions=delayed_spawn_actions,
    )

    delayed_nav2 = TimerAction(
        period=15.0,
        actions=[
            tb3_1_nav2,
            tb3_2_nav2,
            tb3_3_nav2,
        ],
    )

    return LaunchDescription([
        gazebo,
        delayed_spawn,
        controllers,
        fleet_manager,
        mission_manager,
        bottle_detector,
        side_grasp_verifier,
        tron1_mission_fsm,
        ur5e_station,
        delayed_nav2,
    ])

