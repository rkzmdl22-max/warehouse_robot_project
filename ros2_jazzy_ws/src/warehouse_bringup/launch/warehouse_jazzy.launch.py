#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bridge_pkg = get_package_share_directory('mujoco_bridge')

    robot_id = LaunchConfiguration('robot_id')

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_pkg, 'launch', 'bridge.launch.py')
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='tiago_01',
            description='Robot ID for bridge namespace',
        ),
        bridge,
    ])
