#!/usr/bin/env python3
import sys, time, subprocess, rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
a, h, k = (float(v) for v in sys.argv[1:4])
s = -1.0 if '--mirror' in sys.argv else 1.0
names = ['abad_L_Joint','hip_L_Joint','knee_L_Joint','foot_L_Joint',
         'abad_R_Joint','hip_R_Joint','knee_R_Joint','foot_R_Joint']
pos = [a, h, k, 0.0, s*a, s*h, s*k, 0.0]
rclpy.init(); n = rclpy.create_node('pose_test')
pub = n.create_publisher(JointTrajectory, '/tron1a/set_joint_trajectory', 10)
m = JointTrajectory(); m.joint_names = names
p = JointTrajectoryPoint(); p.positions = pos; p.time_from_start = Duration(sec=1)
m.points = [p]
for _ in range(40):
    pub.publish(m); time.sleep(0.05)
n.destroy_node(); rclpy.shutdown()
time.sleep(1)
out = subprocess.run(['gz','model','-m','tron1a','-i'], capture_output=True, text=True).stdout
for l in out.splitlines():
    if '_Joint"' in l and 'tron1a::' in l or 'angle:' in l:
        print(l.strip())
