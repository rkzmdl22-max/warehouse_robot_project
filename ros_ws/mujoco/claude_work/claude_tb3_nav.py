import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
SP = {1: (-2.0, -2.0), 2: (-2.0, 2.0), 3: (2.0, 0.0)}
WP = {1: [(4.0, -0.8), (-3.0, -0.8)], 2: [(-3.0, 0.8), (4.0, 0.8)], 3: [(6.0, 0.0), (6.0, 2.5)]}
rclpy.init()
n = Node("claude_tb3_nav")
pose, sc, k, pub = {}, {}, {1: 0, 2: 0, 3: 0}, {}
def setup(i):
    pub[i] = n.create_publisher(Twist, "/tb3_%d/cmd_vel" % i, 10)
    def on_odom(m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pose[i] = (SP[i][0] + m.pose.pose.position.x, SP[i][1] + m.pose.pose.position.y, yaw)
    def on_scan(m):
        r = [x if 0.05 < x < 50 else 9.9 for x in m.ranges]
        w = max(1, len(r) // 18)
        sc[i] = (min(r[:w] + r[-w:]), min(r[w:3 * w]), min(r[-3 * w:-w]))
    n.create_subscription(Odometry, "/tb3_%d/odom" % i, on_odom, 10)
    n.create_subscription(LaserScan, "/tb3_%d/scan" % i, on_scan, qos_profile_sensor_data)
for i in (1, 2, 3):
    setup(i)
def tick():
    for i in (1, 2, 3):
        if i not in pose or i not in sc:
            continue
        x, y, yaw = pose[i]
        wx, wy = WP[i][k[i]]
        if math.hypot(wx - x, wy - y) < 0.3:
            k[i] = (k[i] + 1) % len(WP[i])
            continue
        err = math.atan2(wy - y, wx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        f, l, rt = sc[i]
        t = Twist()
        if f < 0.55:
            t.angular.z = 0.7 if l > rt else -0.7
        elif abs(err) > 0.5:
            t.angular.z = 0.7 if err > 0 else -0.7
        else:
            t.linear.x = 0.18
            t.angular.z = max(-0.7, min(0.7, 1.2 * err))
        pub[i].publish(t)
n.create_timer(0.1, tick)
rclpy.spin(n)
