#!/usr/bin/env python3

import os

import xacro

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def create_tiago(namespace, entity_name, x, y, z):
    """Spawn one namespaced Tiago in Gazebo."""

    xacro_file = os.path.join(
        get_package_share_directory("tiago_description"),
        "robots",
        "tiago.urdf.xacro",
    )

    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_sim_time": "true",
            "gazebo_version": "classic",
            "namespace": namespace,
            "is_multiple": "true",
        },
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": doc.toxml(),
                "use_sim_time": True,
            }
        ],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", entity_name,
            "-topic", f"/{namespace}/robot_description",
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
        ],
    )

    return robot_state_publisher, spawn


def generate_launch_description():

    tiago_1_rsp, tiago_1_spawn = create_tiago(
        "tiago_1", "tiago_1", 4.0, -3.0, 0.10
    )


    return LaunchDescription([
        tiago_1_rsp,
        tiago_1_spawn,
    ])
