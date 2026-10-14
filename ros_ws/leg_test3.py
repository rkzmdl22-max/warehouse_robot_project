import time, rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
J = ['abad_L_Joint','hip_L_Joint','knee_L_Joint','foot_L_Joint','abad_R_Joint','hip_R_Joint','knee_R_Joint','foot_R_Joint']
STAGES = [("1 모두 0", {}),
          ("2 hip_L +0.4", {'hip_L_Joint': 0.4}), ("3 hip_L -0.4", {'hip_L_Joint': -0.4}),
          ("4 hip_R +0.4", {'hip_R_Joint': 0.4}),
          ("5 knee_L +0.6", {'knee_L_Joint': 0.6}), ("6 knee_L -0.6", {'knee_L_Joint': -0.6}),
          ("7 abad_L +0.3", {'abad_L_Joint': 0.3}), ("8 abad_L -0.3", {'abad_L_Joint': -0.3}),
          ("9 abad_R +0.3", {'abad_R_Joint': 0.3}),
          ("10 foot_L +0.3", {'foot_L_Joint': 0.3}), ("11 모두 0", {})]
rclpy.init(); n = rclpy.create_node('leg_test3')
pub = n.create_publisher(JointTrajectory, '/tron1a/set_joint_trajectory', 10)
time.sleep(3.0)
for name, d in STAGES:
    msg = JointTrajectory(); msg.joint_names = J
    pt = JointTrajectoryPoint(); pt.positions = [float(d.get(j, 0.0)) for j in J]
    pt.time_from_start = Duration(sec=0, nanosec=200000000); msg.points = [pt]
    print(name, flush=True)
    for _ in range(5):
        pub.publish(msg); time.sleep(0.1)
    time.sleep(3.0)
n.destroy_node(); rclpy.shutdown()
