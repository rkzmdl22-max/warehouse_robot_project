import json
import math
import time

import rclpy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ApproachController(Node):

    def __init__(self):
        super().__init__("approach_controller")

        self.declare_parameter("standoff_distance", 0.42)
        self.declare_parameter("position_tolerance", 0.03)
        self.declare_parameter("max_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.30)
        self.declare_parameter("detection_timeout", 3.0)

        self.standoff_distance = float(
            self.get_parameter("standoff_distance").value
        )
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.detection_timeout = float(
            self.get_parameter("detection_timeout").value
        )

        self.current_state = "IDLE"
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.bottle_x = None
        self.bottle_y = None
        self.last_detection_time = None

        self.completed = False
        self.was_active = False

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.event_publisher = self.create_publisher(
            String,
            "/handover/event",
            10,
        )

        self.create_subscription(
            String,
            "/handover/state",
            self.state_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/bottle_pose",
            self.bottle_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            "Approach controller started: "
            f"standoff={self.standoff_distance:.2f} m"
        )

    def state_callback(self, message):
        try:
            payload = json.loads(message.data)
            new_state = payload["state"]

            if new_state != self.current_state:
                self.get_logger().info(
                    f"FSM state: {self.current_state} -> {new_state}"
                )

            self.current_state = new_state

            if self.current_state != "APPROACH":
                self.completed = False

        except (json.JSONDecodeError, KeyError) as error:
            self.get_logger().error(f"Invalid state message: {error}")

    def bottle_callback(self, message):
        if message.header.frame_id != "odom":
            self.get_logger().warning(
                f"Expected odom frame, received {message.header.frame_id}"
            )
            return

        self.bottle_x = message.pose.position.x
        self.bottle_y = message.pose.position.y
        self.last_detection_time = time.monotonic()

    def odom_callback(self, message):
        self.robot_x = message.pose.pose.position.x
        self.robot_y = message.pose.pose.position.y

        orientation = message.pose.pose.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        self.robot_yaw = math.atan2(sin_yaw, cos_yaw)

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def control_loop(self):
        active = self.current_state == "APPROACH"

        if not active:
            if self.was_active:
                self.stop_robot()
            self.was_active = False
            return

        self.was_active = True

        if self.completed:
            self.stop_robot()
            return

        if (
            self.robot_x is None
            or self.robot_y is None
            or self.robot_yaw is None
        ):
            self.stop_robot()
            return

        if (
            self.bottle_x is None
            or self.bottle_y is None
            or self.last_detection_time is None
        ):
            self.stop_robot()
            return

        detection_age = time.monotonic() - self.last_detection_time

        if detection_age > self.detection_timeout:
            self.get_logger().warning(
                "Bottle detection timed out; stopping robot"
            )
            self.stop_robot()
            return

        # 물병의 x축 뒤쪽에 정지 지점을 생성한다.
        target_x = self.bottle_x - self.standoff_distance
        target_y = self.bottle_y

        delta_x = target_x - self.robot_x
        delta_y = target_y - self.robot_y

        distance = math.hypot(delta_x, delta_y)

        if distance <= self.position_tolerance:
            self.stop_robot()
            self.completed = True

            self.get_logger().info(
                "Approach complete: "
                f"robot=({self.robot_x:.3f}, {self.robot_y:.3f}), "
                f"target=({target_x:.3f}, {target_y:.3f})"
            )

            self.event_publisher.publish(
                String(data="APPROACH_OK")
            )
            return

        target_heading = math.atan2(delta_y, delta_x)
        heading_error = normalize_angle(
            target_heading - self.robot_yaw
        )

        command = Twist()

        command.angular.z = clamp(
            1.5 * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if abs(heading_error) < 0.20:
            command.linear.x = clamp(
                0.6 * distance,
                0.0,
                self.max_linear_speed,
            )

        self.cmd_vel_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)

    node = ApproachController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()