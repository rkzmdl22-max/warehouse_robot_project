import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .machine import Event, HandoverMachine


class HandoverFsmNode(Node):
    def __init__(self) -> None:
        super().__init__("handover_fsm")
        self.machine = HandoverMachine()
        self.publisher = self.create_publisher(String, "handover/state", 10)
        self.create_subscription(String, "handover/event", self.on_event, 10)
        self.publish_state("startup")

    def on_event(self, message: String) -> None:
        try:
            event = Event[message.data.strip().upper()]
            self.machine.dispatch(event)
            self.publish_state(event.name)
        except (KeyError, ValueError) as error:
            self.get_logger().error(str(error))

    def publish_state(self, reason: str) -> None:
        payload = {"state": self.machine.state.name, "reason": reason}
        self.publisher.publish(String(data=json.dumps(payload)))
        self.get_logger().info(json.dumps(payload))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandoverFsmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

