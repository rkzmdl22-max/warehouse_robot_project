#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # TIAGo xacro 파일
    xacro_file = os.path.join(
        get_package_share_directory("tiago_description"),
        "robots",
        "tiago.urdf.xacro",
    )

    # Xacro → URDF (mappings로 Gazebo Classic 플러그인 활성화)
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            "use_sim_time": "true",
            "gazebo_version": "classic",
        },
    )

    robot_description = {
        "robot_description": robot_description_config.toxml(),
        "use_sim_time": True,
    }

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Gazebo에 TIAGo 스폰
    spawn_tiago = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "tiago",
            "-topic", "robot_description",
            "-x", "2.0",
            "-y", "0.0",
            "-z", "0.10",
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_tiago,
    ])
