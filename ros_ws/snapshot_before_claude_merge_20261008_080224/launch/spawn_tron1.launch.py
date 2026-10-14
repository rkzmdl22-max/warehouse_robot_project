#!/usr/bin/env python3
"""
TRON1A Gazebo Entity Spawn Launch File
Spawns TRON1A into Gazebo using existing Objects_3DA_v0/PF_TRON1A/urdf/robot.urdf.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = "/ros_ws/Objects_3DA_v0/PF_TRON1A/urdf/robot.urdf"

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="tron1a_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        remappings=[
            ("/joint_states", "/tron1a/joint_states"),
        ],
    )

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_tron1a",
        output="screen",
        arguments=[
            "-world", "warehouse",
            "-name", "tron1a",
            "-file", urdf_file,
            "-x", "1.2",
            "-y", "-3.0",
            "-z", "0.82",
        ],
    )

    return LaunchDescription([
        rsp_node,
        spawn_node,
    ])
