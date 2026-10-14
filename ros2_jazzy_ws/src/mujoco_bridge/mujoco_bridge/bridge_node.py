"""MuJoCo ↔ ROS 2 Jazzy bridge (skeleton).

FINAL architecture:
  ROS2 Command → MuJoCo Interface → mj_step() → Robot State → ROS2 Publisher

This node is a stub. Connect tiago_harness here in Phase R3.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MuJoCoBridgeNode(Node):
    """Minimal bridge stub — subscribes cmd_vel, publishes state placeholders."""

    def __init__(self):
        super().__init__('mujoco_bridge_node')

        self.declare_parameter('robot_id', 'tiago_01')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('scene_path', '')

        robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        prefix = f'/robot/{robot_id}'

        self._status_pub = self.create_publisher(String, f'{prefix}/bridge_status', 10)
        self._odom_pub = self.create_publisher(Odometry, f'{prefix}/odom', 10)
        self._cmd_sub = self.create_subscription(
            Twist,
            f'{prefix}/cmd_vel',
            self._on_cmd_vel,
            10,
        )

        self._timer = self.create_timer(0.1, self._on_timer)
        self._last_cmd = Twist()

        self.get_logger().info(
            f'MuJoCo bridge stub ready for {robot_id}. '
            'Connect tiago_harness + MuJoCo scene in Phase R3.'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        # TODO: forward to tiago_harness.send_base_command()

    def _on_timer(self) -> None:
        status = String()
        status.data = 'STUB_RUNNING'
        self._status_pub.publish(status)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
