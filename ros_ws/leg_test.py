import time, rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
J = ['abad_L_Joint','hip_L_Joint','knee_L_Joint','foot_L_Joint','abad_R_Joint','hip_R_Joint','knee_R_Joint','foot_R_Joint']
STAGES = [("0 모두 0 (기본 서기)", {}),
          ("1 hip_L +0.4", {'hip_L_Joint': 0.4}), ("2 hip_L -0.4", {'hip_L_Joint': -0.4}),
          ("3 knee_L +0.6", {'knee_L_Joint': 0.6}), ("4 knee_L -0.6", {'knee_L_Joint': -0.6}),
          ("5 foot_L +0.3", {'foot_L_Joint': 0.3}), ("6 abad_L +0.2", {'abad_L_Joint': 0.2}),
          ("7 모두 0 복귀", {})]
rclpy.init(); n = rclpy.create_node('leg_test')
pub = n.create_publisher(JointTrajectory, '/tron1a/set_joint_trajectory', 10)
time.sleep(1.0)
for name, d in STAGES:
    msg = JointTrajectory(); msg.joint_names = J
    pt = JointTrajectoryPoint(); pt.positions = [float(d.get(j, 0.0)) for j in J]
    pt.time_from_start = Duration(sec=0, nanosec=200000000); msg.points = [pt]
    print(name, flush=True)
    for _ in range(5):
        pub.publish(msg); time.sleep(0.1)
    time.sleep(2.0)
n.destroy_node(); rclpy.shutdown()
