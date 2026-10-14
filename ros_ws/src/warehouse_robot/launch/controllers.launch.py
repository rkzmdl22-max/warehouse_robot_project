#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")
    param_file = os.path.join(pkg, "config", "controllers.yaml")

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
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
            "/controller_manager",
            "--param-file",
            param_file,
        ],
        output="screen",
    )

    # t=8s  : joint_state_broadcaster 시작
    # t=13s : diff_drive_controller 시작 (joint_state 완료 대기 5초)
    return LaunchDescription([
        TimerAction(
            period=8.0,
            actions=[joint_state_broadcaster]
        ),
        TimerAction(
            period=13.0,
            actions=[diff_drive_controller]
        ),
    ])
