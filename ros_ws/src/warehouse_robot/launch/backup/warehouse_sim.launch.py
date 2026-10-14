#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    warehouse_pkg = get_package_share_directory("warehouse_robot")
    gazebo_pkg = get_package_share_directory("gazebo_ros")
    tb3_pkg = get_package_share_directory("turtlebot3_gazebo")
    tiago_pkg = get_package_share_directory("tiago_gazebo")

    world = os.path.join(
        warehouse_pkg,
        "worlds",
        "warehouse.world"
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
            "extra_gazebo_args":
                "-slibgazebo_ros_init.so "
                "-slibgazebo_ros_factory.so "
                "-slibgazebo_ros_force_system.so"
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

    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_pkg,
                "launch",
                "robot_state_publisher.launch.py"
            )
        )
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_pkg,
                "launch",
                "spawn_turtlebot3.launch.py"
            )
        ),
        launch_arguments={
            "x_pose": "0.0",
            "y_pose": "0.0"
        }.items()
    )

    tiago = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tiago_pkg,
                "launch",
                "spawn_tiago.launch.py"
            )
        )
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state,
        spawn,
        tiago
    ])
