import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
rclpy.init()
n = Node("claude_tb3_drive")
front = {}
pubs = {}
def mk(i):
    pubs[i] = n.create_publisher(Twist, "/tb3_%d/cmd_vel" % i, 10)
    def cb(m):
        r = [x for x in list(m.ranges[:20]) + list(m.ranges[-20:]) if 0.05 < x < 50]
        front[i] = min(r) if r else 9.9
    n.create_subscription(LaserScan, "/tb3_%d/scan" % i, cb, qos_profile_sensor_data)
for i in (1, 2, 3):
    mk(i)
def tick():
    for i in (1, 2, 3):
        t = Twist()
        if front.get(i, 9.9) < 0.6:
            t.angular.z = 0.8 if i % 2 else -0.8
        else:
            t.linear.x = 0.15
        pubs[i].publish(t)
n.create_timer(0.1, tick)
rclpy.spin(n)
