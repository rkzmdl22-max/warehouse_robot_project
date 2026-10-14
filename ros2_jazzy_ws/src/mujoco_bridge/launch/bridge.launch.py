#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('mujoco_bridge')
    config = os.path.join(pkg, 'config', 'bridge.yaml')

    bridge = Node(
        package='mujoco_bridge',
        executable='bridge_node',
        name='mujoco_bridge_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([bridge])
