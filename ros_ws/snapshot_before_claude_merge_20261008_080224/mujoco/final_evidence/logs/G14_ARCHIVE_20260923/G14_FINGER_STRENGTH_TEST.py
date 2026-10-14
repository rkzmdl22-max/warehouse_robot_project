"""
G14 XML Change Verification - Finger Control Strength Test
Test with increased finger control strength
"""
import mujoco
import numpy as np
import sys

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"

print("=" * 68)
print("G14 FINGER CONTROL STRENGTH TEST")
print("=" * 68)

print("\nCHANGES MADE:")
print("1. Finger positions widened from ±0.0325 to ±0.035")
print("2. Finger actuator gains increased from 100 to 200")
print("3. Finger ctrlrange extended from -0.015 to -0.02")

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

# Test with stronger finger controls
print("\nTesting with stronger finger controls (-0.02 to -0.05 range):")

test_controls = [-0.02, -0.025, -0.03, -0.035, -0.04, -0.045, -0.05]

for ctrl in test_controls:
    print(f"\n--- Testing ctrl = {ctrl} ---")
    
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
    
    # G13 grasp
    for _ in range(600):
        fix_tron1(d2)
        set_ctrl(m2, d2, Q0, 0.0)
        mujoco.mj_step(m2, d2)
    
    lc, rc, ld, rd = contact(d2)
    
    if not (lc and rc):
        print(f"  G13 FAIL with ctrl={ctrl}")
        continue
    
    print(f"  G13 PASS: lc={lc}, rc={rc}, ld={ld:.6f}, rd={rd:.6f}")
    
    # G14 hold with gravity
    m2.opt.gravity[:] = gravity_orig
    mujoco.mj_forward(m2, d2)
    
    contact_loss_step = -1
    hold_duration = 2500  # 5 seconds
    
    for step in range(hold_duration):
        fix_tron1(d2)
        set_ctrl(m2, d2, Q0, ctrl)
        mujoco.mj_step(m2, d2)
        
        lc, rc, ld, rd = contact(d2)
        
        if not (lc and rc):
            contact_loss_step = step
            break
    
    if contact_loss_step < 0:
        print(f"  *** G14 PASS: Maintained bilateral contact for 5 seconds with ctrl={ctrl} ***")
        print(f"  Evidence saved to G14_FINGER_STRENGTH_PASS.txt")
        with open(r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_FINGER_STRENGTH_PASS.txt", "w") as f:
            f.write(f"G14 PASS\n")
            f.write(f"Finger control: {ctrl}\n")
            f.write(f"Finger gain: 200\n")
            f.write(f"Finger position offset: ±0.035\n")
            f.write(f"Arm pose: {Q0}\n")
            f.write(f"Left contact distance: {ld:.6f}\n")
            f.write(f"Right contact distance: {rd:.6f}\n")
            f.write(f"Hold duration: 5 seconds\n")
        sys.exit(0)
    else:
        loss_time = contact_loss_step * 0.002
        print(f"  G14 FAIL: Contact lost at step {contact_loss_step} (t={loss_time:.3f}s)")

print("\n[FAIL] No finger control strength achieved G14 PASS")
print("G14 FINGER CONTROL STRENGTH TEST COMPLETE")
