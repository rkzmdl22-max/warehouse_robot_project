import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("automation_cell_bringup")
    gazebo_share = get_package_share_directory(
        "turtlebot3_manipulation_gazebo"
    )

    world_path = os.path.join(
        bringup_share,
        "worlds",
        "handover_station.world",
    )

    gazebo_launch = os.path.join(
        gazebo_share,
        "launch",
        "gazebo.launch.py",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle_pi"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "world": world_path,
                    "x_pose": "0.45",
                    "y_pose": "0.0",
                    "z_pose": "0.01",
                    "yaw": "0.0",
                }.items(),
            ),

            Node(
                package="handover_fsm",
                executable="handover_fsm_node",
                name="handover_fsm",
                output="screen",
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="station_camera_mount_tf",
                arguments=[
                    "--x", "1.1",
                    "--y", "-0.75",
                    "--z", "0.75",
                    "--roll", "0.0",
                    "--pitch", "0.48",
                    "--yaw", "1.5708",
                    "--frame-id", "odom",
                    "--child-frame-id", "station_camera_link",
                ],
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="station_camera_optical_tf",
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--roll", "-1.5708",
                    "--pitch", "0.0",
                    "--yaw", "-1.5708",
                    "--frame-id", "station_camera_link",
                    "--child-frame-id", "station_camera_optical_frame",
                ],
            ),
        ]
    )