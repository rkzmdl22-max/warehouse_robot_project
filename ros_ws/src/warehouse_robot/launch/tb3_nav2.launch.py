#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):

    robot_name = LaunchConfiguration("robot_name").perform(context)
    pkg = get_package_share_directory("warehouse_robot")

    nav2_params = os.path.join(
        pkg,
        "config",
        robot_name,
        f"nav2_params_{robot_name}.yaml",
    )

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        root_key=robot_name,
        param_rewrites={},
        convert_types=True,
    )

    amcl_params = os.path.join(
        pkg,
        "config",
        robot_name,
        f"amcl_{robot_name}.yaml",
    )

    configured_amcl_params = RewrittenYaml(
        source_file=amcl_params,
        root_key=robot_name,
        param_rewrites={},
        convert_types=True,
    )


    map_yaml = os.path.join(pkg, "maps", "warehouse.yaml")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=robot_name,
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": True,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=robot_name,
        output="screen",
        parameters=[configured_amcl_params],
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=robot_name,
        output="screen",
        parameters=[configured_nav2_params],
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=robot_name,
        output="screen",
        parameters=[configured_nav2_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=robot_name,
        output="screen",
        parameters=[configured_nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=robot_name,
        output="screen",
        parameters=[configured_nav2_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=robot_name,
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "behavior_server",
                ],
            }
        ],
    )

    return [
        map_server,
        amcl,
        planner,
        controller,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value="tb3_1",
            description="TurtleBot3 namespace (tb3_1, tb3_2, or tb3_3)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
