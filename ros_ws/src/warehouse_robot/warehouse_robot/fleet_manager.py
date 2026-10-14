import rclpy
from rclpy.node import Node
from warehouse_robot_interfaces.msg import (
    RobotState,
    AssignmentResult,
    MissionRequest,
    MissionState,
)


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')

        self.robot_state_pub = self.create_publisher(
            RobotState, '/fleet/robot_state', 10
        )
        self.assignment_result_pub = self.create_publisher(
            AssignmentResult, '/fleet/assignment_result', 10
        )
        self.mission_request_sub = self.create_subscription(
            MissionRequest,
            '/mission/request',
            self.handle_mission_request,
            10
        )

        self.mission_state_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.handle_mission_state,
            10
        )

        # Verified Nav2 mapping:
        # AMMR-01 -> TB3_2
        # AMMR-02 -> TB3_1
        # AMMR-03 -> TB3_3
        self.robot_namespaces = {
            'AMMR-01': '/tb3_2',
            'AMMR-02': '/tb3_1',
            'AMMR-03': '/tb3_3',
        }

        common_capabilities = {
            'NAVIGATION',
            'TRANSPORT',
            'PICK',
            'PLACE',
            'MANIPULATION',
            'SCAN',
        }

        self.robots = {
            'AMMR-01': set(common_capabilities),
            'AMMR-02': set(common_capabilities),
            'AMMR-03': set(common_capabilities),
            'TRON1-01': {'NAVIGATION', 'BIPEDAL_TRANSPORT', 'TRAY_HANDOVER', 'MANIPULATION'},
        }


        self.robot_states = {}

        for robot_id, capabilities in self.robots.items():
            state = RobotState()
            state.robot_id = robot_id
            state.state = 'IDLE'
            state.capabilities = sorted(capabilities)
            state.available = True
            state.current_mission_id = ''
            state.battery = 0.0
            state.pose.position.x = 0.0
            state.pose.position.y = 0.0
            state.pose.position.z = 0.0
            state.pose.orientation.w = 1.0
            state.error_code = ''
            state.error_message = ''
            self.robot_states[robot_id] = state

        self.get_logger().info('Fleet Manager ready')

        for robot_id, namespace in self.robot_namespaces.items():
            self.get_logger().info(
                f'Registered robot: {robot_id} -> {namespace}'
            )

        self.robot_state_timer = self.create_timer(
            1.0,
            self.publish_robot_states
        )

    def handle_mission_request(self, msg):
        selected, reason = self.select_robot(
            set(msg.required_capabilities)
        )

        assignment = AssignmentResult()
        assignment.mission_id = msg.mission_id
        assignment.task_id = msg.task_id
        assignment.success = selected is not None
        assignment.robot_id = selected if selected else ''

        assignment.matched_capabilities = (
            sorted(set(msg.required_capabilities))
            if selected else []
        )

        # MissionRequest currently carries source_location as a string,
        # so metric distance cannot yet be calculated.
        assignment.estimated_distance = 0.0

        assignment.status = 'ASSIGNED' if selected else reason
        assignment.error_code = '' if selected else reason
        assignment.error_message = (
            ''
            if selected
            else self.selection_error_message(reason)
        )

        self.assignment_result_pub.publish(assignment)

        if selected:
            state = self.robot_states[selected]
            state.current_mission_id = msg.mission_id
            state.state = 'ASSIGNED'
            state.available = False
            state.error_code = ''
            state.error_message = ''

            self.get_logger().info(
                f'Mission request received: '
                f'{msg.mission_id} -> {selected} '
                f'({self.robot_namespaces[selected]})'
            )
        else:
            self.get_logger().warning(
                f'Mission request received: '
                f'{msg.mission_id} -> NONE ({reason})'
            )

    def handle_mission_state(self, msg):
        terminal_states = {
            'NAVIGATION_COMPLETE',
            'NAVIGATION_FAILED',
            'ERROR',
        }

        if msg.state not in terminal_states:
            return

        robot_id = msg.assigned_robot_id
        if not robot_id or robot_id not in self.robot_states:
            return

        state = self.robot_states[robot_id]

        if state.current_mission_id != msg.mission_id:
            return

        state.current_mission_id = ''
        state.state = 'IDLE'
        state.available = True
        state.error_code = ''
        state.error_message = ''

        self.get_logger().info(
            f'Mission released: {msg.mission_id} -> {robot_id}; '
            f'robot returned to IDLE'
        )

    def select_robot(self, required_capabilities):
        for robot_id in (
            'AMMR-01',
            'AMMR-02',
            'AMMR-03',
        ):
            capabilities = self.robots[robot_id]
            state = self.robot_states[robot_id]

            if not required_capabilities.issubset(capabilities):
                continue

            if not state.available:
                continue

            if state.state not in ('IDLE', 'READY'):
                continue

            if state.current_mission_id:
                continue

            if state.error_code:
                continue

            return robot_id, ''

        return None, 'NO_AVAILABLE_ROBOT'

    def selection_error_message(self, reason):
        messages = {
            'NO_AVAILABLE_ROBOT':
                'No robot satisfies capability, state, '
                'availability, mission, and error constraints',
        }
        return messages.get(
            reason,
            'Robot selection failed'
        )

    def publish_robot_states(self):
        for state in self.robot_states.values():
            self.robot_state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
