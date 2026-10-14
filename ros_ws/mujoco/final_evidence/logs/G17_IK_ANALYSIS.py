"""
G17 IK Analysis - find correct pan angle to reach place table
UR5e base: [0.6, -3.45, 0.963]
Place table target: [1.15, -3.0, 0.92]  (bottle_place_table_top)
Lifted bottle at ~z=1.30
"""
import mujoco, math
import numpy as np

XML = r"c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def xpos(n): return d.xpos[bid(n)].copy()
def qadr(n): return m.jnt_qposadr[jid(n)]

arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
           "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
           "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
mujoco.mj_forward(m, d)

base = xpos("ur5e_base")
tool0_at_Q0 = xpos("tool0")
table_top = np.array([1.15, -3.0, 0.79])   # table top surface z=0.4+0.39=0.79
bottle_place = np.array([1.15, -3.0, 0.92]) # bottle on table

print("UR5e base world:", base.round(4))
print("tool0 at Q0:    ", tool0_at_Q0.round(4))
print("bottle_place:   ", bottle_place)

# Vector from base to target (in XY plane)
vec_to_target = bottle_place[:2] - base[:2]
angle_to_target = math.atan2(vec_to_target[1], vec_to_target[0])
print("Angle to place target (from base):", round(math.degrees(angle_to_target),2), "deg")

# Current pan at Q0 = -1.0356 rad
# What pan angle needed to point toward place table?
# UR5e: shoulder_pan rotates around Z axis
# At pan=0: tool0 is at some default direction
# We need to find pan offset

# Scan pan values to find tool0 closest to place target
print("\nScanning pan angles...")
best_pan = None
best_dist = 999
# Use lift pose (shoulder_lift adjusted) to estimate
q_scan = Q0.copy()
q_scan[1] += -0.30  # lift pose approx
q_scan[2] += -0.10

for pan_test in np.linspace(-3.14, 3.14, 360):
    q_scan[0] = pan_test
    for jn,qv in zip(arm_joints,q_scan):
        d.qpos[qadr(jn)] = qv
    mujoco.mj_forward(m, d)
    t0 = xpos("tool0")
    dist_xy = float(np.linalg.norm(t0[:2] - bottle_place[:2]))
    if dist_xy < best_dist:
        best_dist = dist_xy
        best_pan = pan_test
        best_tool0 = t0.copy()

print("Best pan angle:", round(math.degrees(best_pan),2), "deg  (", round(best_pan,4), "rad)")
print("tool0 at best pan:", best_tool0.round(4))
print("dist to place target XY:", round(best_dist,4))

# Check actual direction of current Q0 pan
q_scan[0] = Q0[0]  # -1.0356 rad
for jn,qv in zip(arm_joints,q_scan):
    d.qpos[qadr(jn)] = qv
mujoco.mj_forward(m, d)
t0_q0 = xpos("tool0")
print("\ntool0 at Q0 pan (lift pose):", t0_q0.round(4))
print("pan delta needed:", round(best_pan - Q0[0], 4), "rad")
print("  = Q0[0] +", round(best_pan - Q0[0], 4))

# Also check: what bottles are at which positions
print("\nKey positions:")
print("  ur5e_base:          ", base.round(4))
print("  bottle_place_table: ", xpos("bottle_place_table").round(4))
print("  station_table:      ", xpos("station_table").round(4))
