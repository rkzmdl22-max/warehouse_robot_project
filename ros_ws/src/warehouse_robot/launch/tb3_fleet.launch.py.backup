#!/usr/bin/env python3

import os

import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def create_robot_state_publisher(namespace):
    """
    Create a robot_state_publisher for one TurtleBot3.

    The TurtleBot3 URDF uses the ${namespace} prefix,
    so namespace must end with '/'.
    """

    turtlebot3_description = get_package_share_directory(
        "turtlebot3_description"
    )

    urdf_file = os.path.join(
        turtlebot3_description,
        "urdf",
        "turtlebot3_burger.urdf"
    )

    doc = xacro.process_file(
        urdf_file,
        mappings={
            "namespace": namespace,
        }
    )

    robot_description = doc.toxml()

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace.rstrip("/"),
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )


def create_spawn_entity(entity_name, model_file, x, y, z):
    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", entity_name,
            "-file", model_file,
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
        ],
    )


def generate_launch_description():

    warehouse_pkg = get_package_share_directory("warehouse_robot")

    tb3_1_model = os.path.join(
        warehouse_pkg,
        "models",
        "tb3_1",
        "model.sdf"
    )

    tb3_2_model = os.path.join(
        warehouse_pkg,
        "models",
        "tb3_2",
        "model.sdf"
    )

    tb3_3_model = os.path.join(
        warehouse_pkg,
        "models",
        "tb3_3",
        "model.sdf"
    )

    tb3_1_rsp = create_robot_state_publisher("tb3_1/")
    tb3_2_rsp = create_robot_state_publisher("tb3_2/")
    tb3_3_rsp = create_robot_state_publisher("tb3_3/")

    tb3_1 = create_spawn_entity(
        "tb3_1",
        tb3_1_model,
        -2.0,
        -2.0,
        0.05,
    )

    tb3_2 = create_spawn_entity(
        "tb3_2",
        tb3_2_model,
        -2.0,
        2.0,
        0.05,
    )

    tb3_3 = create_spawn_entity(
        "tb3_3",
        tb3_3_model,
        2.0,
        0.0,
        0.05,
    )

    return LaunchDescription([
        tb3_1_rsp,
        tb3_2_rsp,
        tb3_3_rsp,

        tb3_1,
        tb3_2,
        tb3_3,
    ])
