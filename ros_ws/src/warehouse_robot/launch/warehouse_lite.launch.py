#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")
    use_gui = LaunchConfiguration("use_gui")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "gazebo_lite.launch.py")
        ),
        launch_arguments={"use_gui": use_gui}.items(),
    )

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tb3_fleet.launch.py")
        ),
    )

    tiago_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tiago_fleet.launch.py")
        ),
    )

    tiago_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tiago_controllers_fleet.launch.py")
        ),
    )

    # Gazebo 기동 후 순차 스폰 (시작 시 CPU/RAM 스파이크 완화)
    delayed_fleet = TimerAction(period=5.0, actions=[fleet])
    delayed_tiago = TimerAction(period=8.0, actions=[tiago_fleet])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Gazebo GUI 사용 여부 (false면 headless)",
        ),
        gazebo,
        delayed_fleet,
        delayed_tiago,
        tiago_controllers,
    ])
