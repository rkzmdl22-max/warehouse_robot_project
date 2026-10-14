from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf = "/ros_ws/src/warehouse_robot/urdf/claude_tron1a_classic.urdf"
    controller_config = "/ros_ws/src/pointfoot_gazebo/config/pointfoot_gazebo.yaml"

    robot_description = ParameterValue(
        Command(["cat ", urdf]),
        value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="tron1a",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_tron1a_classic",
        output="screen",
        arguments=[
            "-entity", "tron1a",
            "-file", urdf,
            "-x", "0.0",
            "-y", "-3.0",
            "-z", "0.82",
        ],
    )

    pointfoot_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="tron1_pointfoot_spawner",
        output="screen",
        arguments=[
            "pointfoot_controller",
            "--controller-manager", "/tron1a/controller_manager",
            "--param-file", controller_config,
        ],
    )

    return LaunchDescription([
        rsp,
        spawn,
        TimerAction(
            period=8.0,
            actions=[pointfoot_controller]
        ),
    ])
