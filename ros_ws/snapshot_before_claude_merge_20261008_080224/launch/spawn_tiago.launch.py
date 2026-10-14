#!/usr/bin/env python3

import os

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    tiago_desc = get_package_share_directory("tiago_description")

    xacro_file = os.path.join(
        tiago_desc,
        "robots",
        "tiago.urdf.xacro"
    )

    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_sim_time": "true",
            "gazebo_version": "classic",
            # orbbec-astra-pro uses libgazebo_ros_openni_kinect.so which
            # correctly produces separate RGB + Depth streams.
            # orbbec-astra (default) uses libgazebo_ros_camera.so on a
            # type="depth" sensor → produces uniform gray RGB output.
            "camera_model": "orbbec-astra-pro",
        }
    )

    robot_description = {
        "robot_description": doc.toxml(),
        "use_sim_time": True,
    }

    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "tiago",
            "-topic", "robot_description",
            "-x", "2.0",
            "-y", "0.0",
            "-z", "0.2"
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state,
        spawn
    ])
