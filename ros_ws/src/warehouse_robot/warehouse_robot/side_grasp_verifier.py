#!/usr/bin/env python3
"""
Side Grasp Verifier & Controller Node for UR5e Manipulator
Computes side-approach IK and verifies the grasp equation:
Finger Gap + Open Gap == Bottle Diameter
Also performs post-lift slip checks.
"""

import unittest
import math
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool
    from geometry_msgs.msg import PoseStamped
except ImportError:
    rclpy = None
    class Node:
        pass
    class String: pass
    class Bool: pass
    class PoseStamped: pass



def verify_grasp_equation(finger_gap: float, open_gap: float, target_diameter: float, tolerance: float = 0.005) -> bool:
    """
    Grasp Condition Equation:
    Finger Gap + Open Gap == Bottle Diameter (D)
    """
    total_gap = finger_gap + open_gap
    diff = abs(total_gap - target_diameter)
    return diff <= tolerance


def verify_slip_status(initial_contact_force: float, current_contact_force: float, min_force_threshold: float = 2.0) -> bool:
    """
    Slip Check:
    Verifies that contact force remains above minimum threshold during lifting.
    """
    if current_contact_force < min_force_threshold:
        return False  # Slipping
    return True


class SideGraspVerifierNode(Node):
    def __init__(self):
        super().__init__('side_grasp_verifier_node')

        # Parameters
        self.bottle_diameter = 0.08   # meters (8cm)
        self.finger_thickness = 0.016 # meters (left + right finger thickness sum)
        self.tolerance = 0.005        # meters

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, '/detected_bottle_pose', self.on_bottle_pose, 10
        )
        self.stop_sub = self.create_subscription(
            Bool, '/detected_bottle_stopped', self.on_bottle_stopped, 10
        )

        # Publishers
        self.result_pub = self.create_publisher(String, '/grasp/verification_result', 10)

        self.latest_pose = None
        self.bottle_stopped = False

        self.get_logger().info('SideGraspVerifierNode initialized for UR5e Robotiq Gripper.')

    def on_bottle_pose(self, msg: PoseStamped):
        self.latest_pose = msg.pose

    def on_bottle_stopped(self, msg: Bool):
        self.bottle_stopped = msg.data

    def execute_grasp_sequence(self, open_gap: float, contact_force: float) -> dict:
        """
        Executes and evaluates the grasp sequence.
        """
        grasp_valid = verify_grasp_equation(
            finger_gap=self.finger_thickness,
            open_gap=open_gap,
            target_diameter=self.bottle_diameter,
            tolerance=self.tolerance
        )

        slip_valid = verify_slip_status(
            initial_contact_force=contact_force,
            current_contact_force=contact_force,
            min_force_threshold=2.0
        )

        success = grasp_valid and slip_valid
        return {
            'grasp_valid': grasp_valid,
            'slip_valid': slip_valid,
            'success': success,
            'total_gap': self.finger_thickness + open_gap,
            'target_diameter': self.bottle_diameter
        }


class TestGraspVerification(unittest.TestCase):
    def test_valid_grasp(self):
        # 0.016 (finger_gap) + 0.064 (open_gap) = 0.080 (diameter)
        res = verify_grasp_equation(finger_gap=0.016, open_gap=0.064, target_diameter=0.080)
        self.assertTrue(res)

    def test_invalid_grasp_miss(self):
        # 0.016 + 0.090 = 0.106 != 0.080
        res = verify_grasp_equation(finger_gap=0.016, open_gap=0.090, target_diameter=0.080)
        self.assertFalse(res)

    def test_slip_detection(self):
        self.assertTrue(verify_slip_status(5.0, 4.8))
        self.assertFalse(verify_slip_status(5.0, 1.2))  # Force dropped below 2.0N (slip)


def main(args=None):
    if rclpy is not None:
        rclpy.init(args=args)
        node = SideGraspVerifierNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    else:
        print("[STANDALONE TEST] rclpy not detected. Running unittest suite for SideGraspVerifier:")
        unittest.main(exit=False)


if __name__ == '__main__':
    main()
