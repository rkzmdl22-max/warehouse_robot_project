"""
G14 Finger Control Retry Test
Test different finger controls to find one that maintains bilateral contact under gravity
Based on millisecond diagnostic showing right contact loss at step 2 with ctrl=-0.0105
"""
import mujoco
import numpy as np
import sys

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"

print("=" * 68)
print("G14 FINGER CONTROL RETRY TEST")
print("=" * 68)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

gravity_orig = m.opt.gravity.copy()
print(f"Original gravity: {gravity_orig}")

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

# Test different finger controls
test_controls = [0.0, -0.005, -0.010, -0.015, -0.020, -0.025, -0.030, -0.035, -0.040]

print("\nTesting finger controls: ", test_controls)
print("=" * 68)

results = []

for ctrl in test_controls:
    print(f"\n--- Testing ctrl = {ctrl} ---")

    # Reset simulation
    m2 = mujoco.MjModel.from_xml_path(XML)
    d2 = mujoco.MjData(m2)
    m2.opt.gravity[:] = gravity_orig

    # Setup G13 grasp
    d2.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d2.qvel[:] = 0.0
    for jn,qv in zip(arm_joints,Q0):
        d2.qpos[qadr(jn)] = qv
    d2.qpos[lq]=0.07; d2.qpos[rq]=0.07
    set_ctrl(m2, d2, Q0, 0.0)
    m2.opt.gravity[:] = 0.0
    mujoco.mj_forward(m2, d2)

    # G13 grasp
    for _ in range(600):
        fix_tron1(d2)
        set_ctrl(m2, d2, Q0, 0.0)
        mujoco.mj_step(m2, d2)

    lc_g13, rc_g13, ld_g13, rd_g13 = contact(d2)

    if not (lc_g13 and rc_g13):
        print(f"  G13 FAIL with ctrl={ctrl}")
        results.append((ctrl, "G13_FAIL", 0, 0, 0))
        continue

    print(f"  G13 PASS: lc={lc_g13}, rc={rc_g13}, ld={ld_g13:.6f}, rd={rd_g13:.6f}")

    # G14 hold with gravity
    m2.opt.gravity[:] = gravity_orig
    mujoco.mj_forward(m2, d2)

    contact_loss_step = -1
    hold_duration = 2500  # 5 seconds at 0.002 timestep

    for step in range(hold_duration):
        fix_tron1(d2)
        set_ctrl(m2, d2, Q0, ctrl)
        mujoco.mj_step(m2, d2)

        lc, rc, ld, rd = contact(d2)

        if not (lc and rc):
            contact_loss_step = step
            break

    if contact_loss_step < 0:
        print(f"  G14 PASS: Maintained bilateral contact for 5 seconds")
        results.append((ctrl, "G14_PASS", 5.0, ld, rd))
    else:
        loss_time = contact_loss_step * 0.002
        print(f"  G14 FAIL: Contact lost at step {contact_loss_step} (t={loss_time:.3f}s)")
        results.append((ctrl, "G14_FAIL", loss_time, ld, rd))

print("\n" + "=" * 68)
print("SUMMARY")
print("=" * 68)
print(f"{'Ctrl':<10} {'Result':<15} {'Duration':<10} {'Left Dist':<12} {'Right Dist':<12}")
print("-" * 68)
for ctrl, result, duration, ld, rd in results:
    if ld is not None and rd is not None:
        print(f"{ctrl:<10.4f} {result:<15} {duration:<10.3f} {ld:<12.6f} {rd:<12.6f}")
    else:
        print(f"{ctrl:<10.4f} {result:<15} {'N/A':<10} {'N/A':<12} {'N/A':<12}")

# Check if any control achieved G14 PASS
g14_pass = any(result == "G14_PASS" for _, result, _, _, _ in results)

if g14_pass:
    print("\n*** G14 PASS ACHIEVED ***")
    for ctrl, result, duration, ld, rd in results:
        if result == "G14_PASS":
            print(f"Successful control: {ctrl}")
else:
    print("\n*** NO CONTROL ACHIEVED G14 PASS ***")
    print("All controls failed to maintain bilateral contact for 5 seconds under gravity")

print("\nG14 FINGER CONTROL RETRY COMPLETE")
