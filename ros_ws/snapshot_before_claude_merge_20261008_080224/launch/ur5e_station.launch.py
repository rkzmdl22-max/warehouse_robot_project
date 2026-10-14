#!/usr/bin/env python3
"""
UR5e Station Launch File
========================
Spawns the UR5e station robot (arm + gripper + virtual RGB-D camera)
into a running Gazebo instance and loads ros2_control controllers.

Usage (Gazebo must already be running):
  ros2 launch warehouse_robot ur5e_station.launch.py

Controllers started:
  /ur5e_joint_state_broadcaster
  /ur5e_arm_controller       (JointTrajectoryController, 6-DOF)
  /ur5e_gripper_controller   (JointTrajectoryController, 2-DOF)

Camera topics (after spawn):
  /station_rgbd/rgb/image_raw
  /station_rgbd/rgb/camera_info
  /station_rgbd/depth/image_raw
  /station_rgbd/depth/camera_info
  /station_rgbd/depth/points
"""

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("warehouse_robot")

    urdf_file = os.path.join(pkg, "urdf", "ur5e_station.urdf")

    # ------------------------------------------------------------------
    # 1. robot_state_publisher — publishes /ur5e/robot_description
    #    and broadcasts TF for the arm chain.
    # ------------------------------------------------------------------
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="ur5e",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        remappings=[
            ("/joint_states", "/ur5e/joint_states"),
        ],
    )

    # ------------------------------------------------------------------
    # 2. spawn_entity — inserts the URDF into Gazebo
    # ------------------------------------------------------------------
    spawn_ur5e = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur5e_station",
        output="screen",
        arguments=[
            "-entity", "ur5e_station",
            "-file",   urdf_file,
            "-x", "1.57",
            "-y", "-3.75",
            "-z", "0.785",
        ],
    )

    # ------------------------------------------------------------------
    # 3. Controllers — loaded after spawn completes (~5s delay)
    # ------------------------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="ur5e_jsb_spawner",
        arguments=[
            "ur5e_joint_state_broadcaster",
            "--controller-manager", "/ur5e/controller_manager",
            "--param-file",
            "/ros_ws/src/warehouse_robot/config/ur5e_controllers.yaml",
        ],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="ur5e_arm_spawner",
        arguments=[
            "ur5e_arm_controller",
            "--controller-manager", "/ur5e/controller_manager",
            "--param-file",
            "/ros_ws/src/warehouse_robot/config/ur5e_controllers.yaml",
        ],
        output="screen",
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="ur5e_gripper_spawner",
        arguments=[
            "ur5e_gripper_controller",
            "--controller-manager", "/ur5e/controller_manager",
            "--param-file",
            "/ros_ws/src/warehouse_robot/config/ur5e_controllers.yaml",
        ],
        output="screen",
    )

    # Stagger controller spawners:
    # t=5s  : joint_state_broadcaster  (needs robot to be spawned)
    # t=8s  : arm_controller
    # t=11s : gripper_controller
    delayed_jsb        = TimerAction(period=5.0,  actions=[joint_state_broadcaster])
    delayed_arm        = TimerAction(period=8.0,  actions=[arm_controller])
    delayed_gripper    = TimerAction(period=11.0, actions=[gripper_controller])

    return LaunchDescription([
        robot_state_publisher,
        spawn_ur5e,
        delayed_jsb,
        delayed_arm,
        delayed_gripper,
    ])
