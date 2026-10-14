#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateEcho(Node):
    def __init__(self):
        super().__init__('js_echo')
        self.sub = self.create_subscription(JointState, '/ur5e/joint_states', self.cb, 10)
        self.received = False

    def cb(self, msg):
        print('=== UR5e LIVE JOINT STATES TELEMETRY ===')
        print(f'Header Stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        for name, pos in zip(msg.name, msg.position):
            print(f'  Joint [{name:<22}]: pos = {pos:+.6f} rad')
        self.received = True

def main():
    rclpy.init()
    node = JointStateEcho()
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.received:
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
