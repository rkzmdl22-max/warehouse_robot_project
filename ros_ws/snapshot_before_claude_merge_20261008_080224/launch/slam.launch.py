#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    slam_param = os.path.join(
        pkg,
        "config",
        "slam.yaml"
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_param],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription([
        slam,
        rviz,
    ])
