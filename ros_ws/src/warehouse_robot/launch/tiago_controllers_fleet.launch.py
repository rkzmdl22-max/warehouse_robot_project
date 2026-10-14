#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def create_controller_spawners(namespace, jsb_delay, diff_delay):
    """Start ros2_control spawners for one Tiago."""

    param_file = os.path.join(
        get_package_share_directory("tiago_description"),
        "ros2_control",
        "gazebo_controller_manager_cfg.yaml",
    )

    controller_manager = f"/{namespace}/controller_manager"

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager,
            "--param-file",
            param_file,
        ],
        output="screen",
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            controller_manager,
            "--param-file",
            param_file,
        ],
        output="screen",
    )

    return [
        TimerAction(period=jsb_delay, actions=[joint_state_broadcaster]),
        TimerAction(period=diff_delay, actions=[diff_drive_controller]),
    ]


def generate_launch_description():

    tiago_1_controllers = create_controller_spawners("tiago_1", 12.0, 17.0)
    tiago_2_controllers = create_controller_spawners("tiago_2", 14.0, 19.0)

    return LaunchDescription(tiago_1_controllers + tiago_2_controllers)
