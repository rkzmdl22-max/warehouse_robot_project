"""
G14 Handover Geometry Reconfiguration Test
Test with shifted bottle position (0.02m shift toward right finger)
"""
import mujoco
import numpy as np
import sys

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"

print("=" * 68)
print("G14 HANDOVER GEOMETRY RECONFIGURATION TEST")
print("=" * 68)

print("\nCHANGE MADE:")
print("File: tron1_station_final.xml")
print("Change: carry_bottle pos from (0, 0, 0.115) to (0.02, 0, 0.115)")
print("Purpose: Shift bottle 2cm toward right finger to balance asymmetric contact")

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

gravity_orig = m.opt.gravity.copy()
print(f"\nOriginal gravity: {gravity_orig}")

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def qadr(n): return m.jnt_qposadr[jid(n)]

arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
           "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
           "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

tron1_bi = bid("base_Link")
root_jnt  = m.body_jntadr[tron1_bi]
root_qp   = m.jnt_qposadr[root_jnt]
root_dof  = m.jnt_dofadr[root_jnt]
lq = qadr("left_finger_joint")
rq = qadr("right_finger_joint")
bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")
ai_lf = aid("act_left_finger")
ai_rf = aid("act_right_finger")
HANDOVER_QPOS7 = np.array([0.0,-3.0,0.82, 1.0,0.0,0.0,0.0])

def contact(data):
    lc=rc=False
    left_dist=right_dist=None
    for ci in range(data.ncon):
        c=data.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bgeom,lgeom}:
            lc=True
            left_dist = c.dist
        if {a,b}=={bgeom,rgeom}:
            rc=True
            right_dist = c.dist
    return lc, rc, left_dist, right_dist

def set_ctrl(model, data, q_arm, q_f):
    for jn,qv in zip(arm_joints,q_arm):
        ai=aid(act_map[jn])
        if ai>=0: data.ctrl[ai]=qv
    data.ctrl[ai_lf]=q_f; data.ctrl[ai_rf]=q_f

def fix_tron1(data):
    data.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    data.qvel[root_dof:root_dof+6] = 0.0

# Test G13 with original Q0
print("\n[G13 REGRESSION TEST - ORIGINAL Q0]")
m2 = mujoco.MjModel.from_xml_path(XML)
d2 = mujoco.MjData(m2)
m2.opt.gravity[:] = 0.0

d2.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d2.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d2.qpos[qadr(jn)] = qv
d2.qpos[lq]=0.07; d2.qpos[rq]=0.07
set_ctrl(m2, d2, Q0, 0.0)
mujoco.mj_forward(m2, d2)

for _ in range(600):
    fix_tron1(d2)
    set_ctrl(m2, d2, Q0, 0.0)
    mujoco.mj_step(m2, d2)

lc, rc, ld, rd = contact(d2)

if lc and rc:
    print(f"G13 = PASS: lc={lc}, rc={rc}, ld={ld:.6f}, rd={rd:.6f}")
    print(f"Contact strength: {abs(ld)+abs(rd):.6f}")
    print(f"Contact asymmetry: {abs(ld)-abs(rd):.6f}")
else:
    print(f"G13 = FAIL: lc={lc}, rc={rc}")
    sys.exit(1)

# Test G14 with original Q0
print("\n[G14 TEST - ORIGINAL Q0]")
m2.opt.gravity[:] = gravity_orig
mujoco.mj_forward(m2, d2)

contact_loss_step = -1
hold_duration = 2500  # 5 seconds

for step in range(hold_duration):
    fix_tron1(d2)
    set_ctrl(m2, d2, Q0, -0.015)
    mujoco.mj_step(m2, d2)
    
    lc, rc, ld, rd = contact(d2)
    
    if not (lc and rc):
        contact_loss_step = step
        break

if contact_loss_step < 0:
    print(f"*** G14 PASS: Maintained bilateral contact for 5 seconds ***")
    print(f"Final contact: lc={lc}, rc={rc}, ld={ld:.6f}, rd={rd:.6f}")
    print(f"Evidence saved to G14_PASS_HANDOVER_RECONFIG.txt")
    with open(r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_PASS_HANDOVER_RECONFIG.txt", "w") as f:
        f.write(f"G14 PASS\n")
        f.write(f"Handover geometry: bottle pos (0.02, 0, 0.115)\n")
        f.write(f"Arm pose: {Q0}\n")
        f.write(f"Left contact distance: {ld:.6f}\n")
        f.write(f"Right contact distance: {rd:.6f}\n")
        f.write(f"Contact strength: {abs(ld)+abs(rd):.6f}\n")
        f.write(f"Hold duration: 5 seconds\n")
        f.write(f"Finger control: -0.015\n")
    sys.exit(0)
else:
    loss_time = contact_loss_step * 0.002
    print(f"G14 FAIL: Contact lost at step {contact_loss_step} (t={loss_time:.3f}s)")
    print(f"Last contact: lc={lc}, rc={rc}, ld={ld:.6f}, rd={rd:.6f}")

print("\nG14 HANDOVER GEOMETRY RECONFIGURATION TEST COMPLETE")
