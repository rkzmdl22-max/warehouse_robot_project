#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class UR5eG14Executor(Node):

    CAMERA_FRAME = 'station_camera_color_optical_frame'
    WORLD_FRAME = 'world'
    DETECTED_TOPIC = '/detected_bottle_pose'

    ARM_JOINTS = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]

    GRIPPER_JOINTS = [
        'left_finger_joint',
        'right_finger_joint',
    ]

    def __init__(self):
        super().__init__('ur5e_g14_executor')

        # ---------------------------------------------------------
        # FollowJointTrajectory ActionClients
        # ---------------------------------------------------------
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/ur5e/ur5e_arm_controller/follow_joint_trajectory'
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/ur5e/ur5e_gripper_controller/follow_joint_trajectory'
        )

        # ---------------------------------------------------------
        # TF2
        # world <- station_camera_color_optical_frame
        # ---------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # Bottle detection
        # ---------------------------------------------------------
        self.latest_camera_pose = None
        self.latest_world_pose = None

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.DETECTED_TOPIC,
            self.detected_bottle_callback,
            10
        )

        self.get_logger().info(
            'UR5e G14 executor initialized.'
        )

        self.get_logger().info(
            'TF2 configured: '
            f'{self.CAMERA_FRAME} -> {self.WORLD_FRAME}'
        )

        self.get_logger().info(
            f'Subscribed to {self.DETECTED_TOPIC}'
        )

        self.get_logger().info(
            'Executor structure check: ActionClients created.'
        )

    # =============================================================
    # Bottle pose + TF2
    # =============================================================

    def detected_bottle_callback(self, msg: PoseStamped):
        self.latest_camera_pose = msg

        source_frame = msg.header.frame_id

        if not source_frame:
            self.get_logger().warn(
                'Detected bottle pose has empty frame_id.'
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.WORLD_FRAME,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            world_pose = do_transform_pose(
                msg,
                transform
            )

            self.latest_world_pose = world_pose

            self.get_logger().info(
                'BOTTLE TF2 PASS | '
                f'{source_frame} -> {self.WORLD_FRAME} | '
                f'x={world_pose.pose.position.x:.4f} '
                f'y={world_pose.pose.position.y:.4f} '
                f'z={world_pose.pose.position.z:.4f}'
            )

        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                'BOTTLE TF2 WAIT | '
                f'{source_frame} -> {self.WORLD_FRAME} | '
                f'{exc}'
            )

    # =============================================================
    # Action goal builders
    # =============================================================

    def build_arm_goal(self, positions, duration_sec):
        if len(positions) != len(self.ARM_JOINTS):
            raise ValueError(
                f'Expected {len(self.ARM_JOINTS)} arm positions, '
                f'got {len(positions)}'
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.ARM_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]

        whole_sec = int(duration_sec)
        nanosec = int(
            (float(duration_sec) - whole_sec) * 1e9
        )

        point.time_from_start.sec = whole_sec
        point.time_from_start.nanosec = nanosec

        goal.trajectory.points = [point]

        return goal

    def build_gripper_goal(self, left, right, duration_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.GRIPPER_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = [
            float(left),
            float(right)
        ]

        whole_sec = int(duration_sec)
        nanosec = int(
            (float(duration_sec) - whole_sec) * 1e9
        )

        point.time_from_start.sec = whole_sec
        point.time_from_start.nanosec = nanosec

        goal.trajectory.points = [point]

        return goal

    # =============================================================
    # ActionServer availability
    # =============================================================

    def wait_for_servers(self, timeout_sec=2.0):
        arm_ready = self.arm_client.wait_for_server(
            timeout_sec=timeout_sec
        )

        gripper_ready = self.gripper_client.wait_for_server(
            timeout_sec=timeout_sec
        )

        self.get_logger().info(
            f'Arm ActionServer ready = {arm_ready}'
        )

        self.get_logger().info(
            f'Gripper ActionServer ready = {gripper_ready}'
        )

        return arm_ready and gripper_ready


def main(args=None):
    rclpy.init(args=args)

    node = UR5eG14Executor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
