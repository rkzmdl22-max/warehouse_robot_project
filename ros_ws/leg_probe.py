import re, subprocess, time, rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
J = ['abad_L_Joint','hip_L_Joint','knee_L_Joint','foot_L_Joint','abad_R_Joint','hip_R_Joint','knee_R_Joint','foot_R_Joint']
STAGES = [("zero", {}), ("hip_L +0.4", {'hip_L_Joint': 0.4}), ("hip_L -0.4", {'hip_L_Joint': -0.4}),
          ("hip_R +0.4", {'hip_R_Joint': 0.4}), ("knee_L +0.6", {'knee_L_Joint': 0.6}),
          ("knee_L -0.6", {'knee_L_Joint': -0.6}), ("abad_L +0.3", {'abad_L_Joint': 0.3}),
          ("abad_L -0.3", {'abad_L_Joint': -0.3}), ("abad_R +0.3", {'abad_R_Joint': 0.3}),
          ("foot_L +0.3", {'foot_L_Joint': 0.3})]
def links():
    out = subprocess.run(['gz', 'model', '-m', 'tron1a', '-i'], capture_output=True, text=True, timeout=10).stdout
    res = {}
    for m in re.finditer(r'link \{\s*name: "tron1a::([^"]+)"(.*?)position \{\s*x: (\S+)\s*y: (\S+)\s*z: (\S+)', out, re.S):
        if '::' in m.group(1):
            continue
        res[m.group(1)] = tuple(float(m.group(i)) for i in (3, 4, 5))
    return res, out
def pose():
    return subprocess.run(['gz', 'model', '-m', 'tron1a', '-p'], capture_output=True, text=True, timeout=5).stdout.split()[:6]
def show(name):
    L, raw = links()
    print(f'## {name} | model pose {pose()}')
    for k, v in L.items():
        if re.search(r'(foot|knee|hip)_[LR]|base', k, re.I):
            print(f'   {k}: {v[0]:.3f} {v[1]:.3f} {v[2]:.3f}')
    return L, raw
L, raw = show('initial (명령 전, 지금 자세)')
print('RAW(앞 25줄):'); print('\n'.join(raw.splitlines()[:25])); print('링크 수:', len(L), list(L)[:12])
rclpy.init(); n = rclpy.create_node('leg_probe')
pub = n.create_publisher(JointTrajectory, '/tron1a/set_joint_trajectory', 10)
time.sleep(1.5)
for name, d in STAGES:
    msg = JointTrajectory(); msg.joint_names = J
    pt = JointTrajectoryPoint(); pt.positions = [float(d.get(j, 0.0)) for j in J]
    pt.time_from_start = Duration(sec=0, nanosec=200000000); msg.points = [pt]
    for _ in range(5):
        pub.publish(msg); time.sleep(0.1)
    time.sleep(1.5)
    show(name)
n.destroy_node(); rclpy.shutdown()
