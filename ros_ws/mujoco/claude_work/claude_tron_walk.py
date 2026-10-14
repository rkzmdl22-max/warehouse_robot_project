import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
J = ["abad_L_Joint","hip_L_Joint","knee_L_Joint","abad_R_Joint","hip_R_Joint","knee_R_Joint"]
class W(Node):
    def __init__(s):
        super().__init__("claude_tron_walk"); s.t0 = time.time()
        s.v = s.create_publisher(Twist, "/tron1a/cmd_vel", 10)
        s.j = s.create_publisher(JointTrajectory, "/tron1a/set_joint_trajectory", 10)
        s.create_timer(0.02, s.tick)
    def tick(s):
        t = time.time() - s.t0 - 6.0; go = 0 < t < 16.0
        w = math.sin(2 * math.pi * t) if go else 0.0
        m = Twist(); m.linear.x = 0.4 if go else 0.0; s.v.publish(m)
        p = JointTrajectoryPoint()
        p.positions = [0.0, 0.24*w, 0.32*max(0.0, w), 0.0, 0.24*w, 0.32*max(0.0, -w)]
        q = JointTrajectory(); q.joint_names = J; q.points = [p]; s.j.publish(q)
rclpy.init(); rclpy.spin(W())
