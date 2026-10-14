import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from warehouse_robot_interfaces.msg import MissionRequest, MissionState, AssignmentResult


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')


        self.mission_request_pub = self.create_publisher(
            MissionRequest, '/mission/request', 10
        )
        self.mission_state_pub = self.create_publisher(
            MissionState, '/mission/state', 10
        )
        # AMMR-01 is currently backed by the verified TB3-2 Nav2 stack.
        self.robot_nav_namespace = {
            'AMMR-01': '/tb3_2',
        }

        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            self.robot_nav_namespace['AMMR-01'] + '/navigate_to_pose'
        )
        self.nav_goal_sent = False
        self.nav_goal_active = False

        self.assignment_result_sub = self.create_subscription(
            AssignmentResult,
            '/fleet/assignment_result',
            self.handle_assignment_result,
            10
        )

        self.mission_id = 'BOX-001'
        self.required_capabilities = {
            'NAVIGATION',
            'TRANSPORT',
            'PICK',
            'PLACE',
            'MANIPULATION',
            'SCAN',
        }

        self.robot_capabilities = {
            'AMMR-01': {
                'NAVIGATION',
                'TRANSPORT',
                'PICK',
                'PLACE',
                'MANIPULATION',
                'SCAN',
            },
            'TRON1-01': {
                'NAVIGATION',
                'BIPEDAL_TRANSPORT',
                'TRAY_HANDOVER',
                'MANIPULATION',
            }
        }


        self.mission_request = MissionRequest()
        self.mission_request.mission_id = self.mission_id
        self.mission_request.task_id = 'TASK-001'
        self.mission_request.item_id = 'ITEM-001'
        self.mission_request.product_id = 'PRODUCT-001'
        self.mission_request.source_location = 'WAREHOUSE-A'
        self.mission_request.destination = 'STATION-01'
        self.mission_request.quantity = 1.0
        self.mission_request.required_capabilities = sorted(
            self.required_capabilities
        )
        self.mission_request.priority = 1

        self.get_logger().info(
            f'Mission created: {self.mission_id}'
        )

        self.mission_request_timer = self.create_timer(
            1.0, self.publish_mission_request
        )
        self.mission_state_timer = self.create_timer(
            1.0, self.publish_mission_state
        )

        self.mission_state = MissionState()
        self.mission_state.mission_id = self.mission_id
        self.mission_state.task_id = 'TASK-001'
        self.mission_state.state = 'REQUESTED'
        self.mission_state.assigned_robot_id = ''
        self.mission_state.error_code = ''
        self.mission_state.error_message = ''


    def publish_mission_request(self):
        self.mission_request_pub.publish(self.mission_request)

    def publish_mission_state(self):
        self.mission_state_pub.publish(self.mission_state)

    def handle_assignment_result(self, msg):
        if msg.mission_id != self.mission_id:
            return

        self.mission_state.task_id = msg.task_id

        if msg.success:
            self.mission_state.assigned_robot_id = msg.robot_id
            self.mission_state.state = 'ASSIGNED'
            self.mission_state.error_code = ''
            self.mission_state.error_message = ''

            if self.mission_request_timer is not None:
                self.mission_request_timer.cancel()

            self.get_logger().info(
                f'Assignment confirmed: {msg.mission_id} -> {msg.robot_id}'
            )

            if (
                msg.robot_id == 'AMMR-01'
                and msg.task_id == 'TASK-001'
                and not self.nav_goal_sent
            ):
                self.send_nav_goal(msg)
        else:
            if self.nav_goal_sent or self.mission_state.state in (
                'ASSIGNED',
                'NAVIGATING',
                'NAVIGATION_COMPLETE',
            ):
                self.get_logger().info(
                    f'Ignoring late assignment failure for active mission: '
                    f'{msg.mission_id} ({msg.error_code})'
                )
                return

            self.mission_state.assigned_robot_id = ''
            self.mission_state.state = 'ERROR'
            self.mission_state.error_code = msg.error_code
            self.mission_state.error_message = msg.error_message
            self.get_logger().error(
                f'Assignment failed: {msg.mission_id}'
            )

    def send_nav_goal(self, assignment):
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'Nav2 NavigateToPose action server unavailable'
            )
            self.mission_state.state = 'NAV2_UNAVAILABLE'
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rclpy.time.Time().to_msg()


        # STATION-01 -> existing warehouse delivery zone
        goal.pose.pose.position.x = 7.0
        goal.pose.pose.position.y = -8.0
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.nav_goal_sent = True
        self.nav_goal_active = True
        self.mission_state.state = 'NAVIGATING'

        self.get_logger().info(
            'Sending Nav2 goal: STATION-01 -> (7.0, -8.0)'
        )

        future = self.nav2_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        self.get_logger().info(
            f'[W405] Nav2 goal response callback: '
            f'done={future.done()} cancelled={future.cancelled()}'
        )

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.nav_goal_active = False
            self.mission_state.state = 'NAV2_GOAL_RESPONSE_ERROR'
            self.mission_state.error_code = type(exc).__name__
            self.mission_state.error_message = str(exc)
            self.get_logger().error(
                f'[W405] Nav2 goal response exception: '
                f'{type(exc).__name__}: {exc}'
            )
            return

        self.get_logger().info(
            f'[W405] Nav2 goal handle received: '
            f'accepted={goal_handle.accepted}'
        )

        if not goal_handle.accepted:
            self.nav_goal_active = False
            self.mission_state.state = 'NAV2_GOAL_REJECTED'
            self.mission_state.error_code = 'GOAL_REJECTED'
            self.mission_state.error_message = (
                'Nav2 ActionServer rejected NavigateToPose goal'
            )
            self.get_logger().error(
                '[W405] Nav2 goal rejected by ActionServer'
            )
            return

        self.get_logger().info('[W405] Nav2 goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Nav2 feedback: distance_remaining='
            f'{feedback.distance_remaining:.2f} m',
            throttle_duration_sec=2.0
        )

    def nav_result_callback(self, future):
        result = future.result()
        status = result.status

        self.nav_goal_active = False

        if status == 4:
            self.mission_state.state = 'NAVIGATION_COMPLETE'
            self.get_logger().info(
                'Nav2 navigation completed successfully'
            )
        else:
            self.mission_state.state = 'NAVIGATION_FAILED'
            self.get_logger().error(
                f'Nav2 navigation finished with status={status}'
            )

    def select_robot(self):
        for robot_id, capabilities in self.robot_capabilities.items():
            if self.required_capabilities.issubset(capabilities):
                return robot_id

        return None


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
