import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
HIP, KNEE, F, T, V = 0.24, 0.32, 1.2, 14.0, 2.5/14.0
J = ["abad_L_Joint","hip_L_Joint","knee_L_Joint","abad_R_Joint","hip_R_Joint","knee_R_Joint"]
rclpy.init(); n = Node("claude_gz_walk"); t0 = n.get_clock().now()
pj = n.create_publisher(JointTrajectory, "/tron1a/set_joint_trajectory", 10)
pv = n.create_publisher(Twist, "/tron1a/cmd_vel", 10)
def tick():
    t = (n.get_clock().now() - t0).nanoseconds * 1e-9
    walking = t < T; s = math.sin(2*math.pi*F*t) if walking else 0.0
    q = [0.0, HIP*s, KNEE*max(0.0, s), 0.0, HIP*s, KNEE*max(0.0, -s)]
    m = JointTrajectory(); m.joint_names = J
    p = JointTrajectoryPoint(); p.positions = q; p.time_from_start = Duration(sec=0, nanosec=20000000)
    m.points = [p]; pj.publish(m)
    v = Twist(); v.linear.x = V if walking else 0.0; pv.publish(v)
    if int(t*10) % 20 == 0: n.get_logger().info("t=%.1f walking=%s" % (t, walking))
n.create_timer(0.02, tick); rclpy.spin(n)
