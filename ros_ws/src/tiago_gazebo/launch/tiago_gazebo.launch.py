# Copyright (c) 2024 Warehouse Robot Project. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Main Gazebo simulation launch file for TIAGo robot.

Launch sequence:
  1. Start Gazebo (gzserver + gzclient) with the specified world
  2. Spawn TIAGo into the running Gazebo instance   (spawn_tiago.launch.py)
  3. Start robot_state_publisher with the URDF       (tiago_bringup.launch.py →
                                                       robot_state_publisher.launch.py)
  4. Load ros2_control controllers                   (tiago_bringup.launch.py →
                                                       default_controllers.launch.py)

Usage:
  ros2 launch tiago_gazebo tiago_gazebo.launch.py
  ros2 launch tiago_gazebo tiago_gazebo.launch.py world:=warehouse gui:=true
  ros2 launch tiago_gazebo tiago_gazebo.launch.py \
      arm_type:=tiago-arm end_effector:=pal-hey5 laser_model:=sick-571
"""

import os
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_description.launch_arguments import TiagoArgs


# ---------------------------------------------------------------------------
# Launch argument declarations
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    # --- Robot hardware configuration ---
    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    arm_type: DeclareLaunchArgument = TiagoArgs.arm_type
    arm_motor_model: DeclareLaunchArgument = TiagoArgs.arm_motor_model
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    ft_sensor: DeclareLaunchArgument = TiagoArgs.ft_sensor
    wrist_model: DeclareLaunchArgument = TiagoArgs.wrist_model
    camera_model: DeclareLaunchArgument = TiagoArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoArgs.laser_model
    has_screen: DeclareLaunchArgument = TiagoArgs.has_screen

    # --- Simulation / runtime ---
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim
    namespace: DeclareLaunchArgument = CommonArgs.namespace
    gazebo_version: DeclareLaunchArgument = CommonArgs.gazebo_version

    # --- Gazebo-specific ---
    world: DeclareLaunchArgument = DeclareLaunchArgument(
        name="world",
        default_value="warehouse",
        description=(
            "World file name (without .world extension) located in "
            "'worlds/' at the workspace root, or an absolute path."
        ),
    )

    gui: DeclareLaunchArgument = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Set to 'false' to run Gazebo in headless (server-only) mode.",
    )

    # --- Spawn pose ---
    spawn_x: DeclareLaunchArgument = DeclareLaunchArgument(
        name="spawn_x",
        default_value="0.0",
        description="Initial X position of the robot in the world [m].",
    )
    spawn_y: DeclareLaunchArgument = DeclareLaunchArgument(
        name="spawn_y",
        default_value="0.0",
        description="Initial Y position of the robot in the world [m].",
    )
    spawn_z: DeclareLaunchArgument = DeclareLaunchArgument(
        name="spawn_z",
        default_value="0.05",
        description="Initial Z position of the robot in the world [m].",
    )
    spawn_yaw: DeclareLaunchArgument = DeclareLaunchArgument(
        name="spawn_yaw",
        default_value="0.0",
        description="Initial yaw (rotation around Z) of the robot [rad].",
    )


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    _declare_actions(ld, launch_arguments)

    return ld


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _declare_actions(
    ld: LaunchDescription,
    args: LaunchArguments,
) -> None:

    tiago_gazebo_share = get_package_share_directory("tiago_gazebo")

    # ------------------------------------------------------------------
    # 1. Gazebo model / plugin path setup
    # ------------------------------------------------------------------
    # Make sure Gazebo can find models shipped inside the workspace.
    gazebo_model_path = os.path.join(tiago_gazebo_share, "models")
    worlds_path = os.path.join(
        get_package_share_directory("tiago_gazebo"), "worlds"
    )

    ld.add_action(
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=[
                os.environ.get("GAZEBO_MODEL_PATH", ""),
                ":",
                gazebo_model_path,
            ],
        )
    )

    # ------------------------------------------------------------------
    # 2. Start Gazebo (gzserver + optional gzclient)
    # ------------------------------------------------------------------
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # gzserver — physics simulation (always started)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [worlds_path, LaunchConfiguration("world") + ".world"]
            ),
            "verbose": "false",
            "pause": "false",
        }.items(),
    )
    ld.add_action(gzserver)

    # gzclient — GUI (conditional)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    ld.add_action(gzclient)

    # ------------------------------------------------------------------
    # 3. Spawn TIAGo
    #    Delayed slightly to give gzserver time to initialise.
    # ------------------------------------------------------------------
    spawn_tiago = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tiago_gazebo"), "launch", "spawn_tiago.launch.py"]
            )
        ),
        launch_arguments={
            "base_type":     LaunchConfiguration("base_type"),
            "arm_type":      LaunchConfiguration("arm_type"),
            "arm_motor_model": LaunchConfiguration("arm_motor_model"),
            "end_effector":  LaunchConfiguration("end_effector"),
            "ft_sensor":     LaunchConfiguration("ft_sensor"),
            "wrist_model":   LaunchConfiguration("wrist_model"),
            "camera_model":  LaunchConfiguration("camera_model"),
            "laser_model":   LaunchConfiguration("laser_model"),
            "has_screen":    LaunchConfiguration("has_screen"),
            "use_sim_time":  LaunchConfiguration("use_sim_time"),
            "is_public_sim": LaunchConfiguration("is_public_sim"),
            "namespace":     LaunchConfiguration("namespace"),
            "gazebo_version": LaunchConfiguration("gazebo_version"),
            "spawn_x":       LaunchConfiguration("spawn_x"),
            "spawn_y":       LaunchConfiguration("spawn_y"),
            "spawn_z":       LaunchConfiguration("spawn_z"),
            "spawn_yaw":     LaunchConfiguration("spawn_yaw"),
        }.items(),
    )

    # 3-second delay so Gazebo physics engine is ready before spawn
    ld.add_action(TimerAction(period=3.0, actions=[spawn_tiago]))

    # ------------------------------------------------------------------
    # 4. TIAGo bringup (robot_state_publisher + controllers)
    #    Delayed further to let the robot finish spawning first.
    # ------------------------------------------------------------------
    tiago_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tiago_bringup"),
                    "launch",
                    "tiago_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "base_type":       LaunchConfiguration("base_type"),
            "arm_type":        LaunchConfiguration("arm_type"),
            "arm_motor_model": LaunchConfiguration("arm_motor_model"),
            "end_effector":    LaunchConfiguration("end_effector"),
            "ft_sensor":       LaunchConfiguration("ft_sensor"),
            "wrist_model":     LaunchConfiguration("wrist_model"),
            "camera_model":    LaunchConfiguration("camera_model"),
            "laser_model":     LaunchConfiguration("laser_model"),
            "has_screen":      LaunchConfiguration("has_screen"),
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "is_public_sim":   LaunchConfiguration("is_public_sim"),
            "namespace":       LaunchConfiguration("namespace"),
            "gazebo_version":  LaunchConfiguration("gazebo_version"),
        }.items(),
    )

    # 6-second delay: gzserver(3s) + spawn(~2s) + margin
    ld.add_action(TimerAction(period=6.0, actions=[tiago_bringup]))
