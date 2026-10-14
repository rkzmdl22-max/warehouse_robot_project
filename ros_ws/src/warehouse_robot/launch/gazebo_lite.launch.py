#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    warehouse_pkg = get_package_share_directory("warehouse_robot")
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    use_gui = LaunchConfiguration("use_gui")

    world = os.path.join(
        warehouse_pkg,
        "worlds",
        "warehouse_lite.world",
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "verbose": "false",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(use_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Launch Gazebo GUI (set false to save RAM/GPU)",
        ),
        gzserver,
        gzclient,
    ])
