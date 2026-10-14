"""
G14 ACTIVE GRAVITY COMPENSATION
Test runtime control strategy to actively compensate for gravity during hold
XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"/ros_ws/mujoco/scenes/warehouse_tron1_g14_runtime.xml"
print("G14 ACTIVE GRAVITY COMPENSATION")
print("XML MODIFIED = NO")

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

# CRITICAL: Capture original gravity BEFORE any modification
gravity_orig = m.opt.gravity.copy()
print("Original gravity:", gravity_orig)

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def xpos(n): return d.xpos[bid(n)].copy()
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

def contact():
    lc=rc=False
    for ci in range(d.ncon):
        c=d.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bgeom,lgeom}: lc=True
        if {a,b}=={bgeom,rgeom}: rc=True
    return lc,rc

def set_ctrl(q_arm, q_f):
    for jn,qv in zip(arm_joints,q_arm):
        ai=aid(act_map[jn])
        if ai>=0: d.ctrl[ai]=qv
    d.ctrl[ai_lf]=q_f; d.ctrl[ai_rf]=q_f

def fix_tron1():
    d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d.qvel[root_dof:root_dof+6] = 0.0

# ── G13: GRASP WITH ZERO GRAVITY ────────────────────────────────────────────
print("\n[G13 GRASP]")
d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.02; d.qpos[rq]=0.02
set_ctrl(Q0, 0.085)
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

got_both = False
for step in range(2000):
    fix_tron1()
    set_ctrl(Q0, 0.085)
    mujoco.mj_step(m, d)
    lc0,rc0=contact()
    if lc0 and rc0:
        got_both=True
        print("  Bilateral contact at step %d" % (step+1))
        break

if not got_both:
    print("G13 NOT REPRODUCED")
    sys.exit(1)

b_g13=xpos("carry_bottle")
print("G13 PASS  bottle=%s" % b_g13.round(4))

# ── G14: ACTIVE GRAVITY COMPENSATION HOLD ───────────────────────────────────
print("\n========================================")
print("G14: ACTIVE GRAVITY COMPENSATION HOLD")
print("========================================")
m.opt.gravity[:] = gravity_orig
b_start = xpos("carry_bottle").copy()
print("  bottle start =", b_start.round(4))

# Test different compensation strategies
strategies = [
    ("baseline", lambda step: Q0, -0.015),  # No compensation
    ("tighter_finger", lambda step: Q0, -0.020),  # Tighter finger grip
    ("shoulder_lift_down", lambda step: Q0 + np.array([0, -0.01*min(step/500, 1), 0, 0, 0, 0]), -0.015),  # Gradually lower shoulder
    ("wrist_2_adjust", lambda step: Q0 + np.array([0, 0, 0, 0, 0.01*min(step/500, 1), 0]), -0.015),  # Adjust wrist_2
]

print("\n=== TESTING COMPENSATION STRATEGIES ===")
print("strategy              | contact_loss | drift 5s | result")
print("----------------------|--------------|----------|--------")

for strategy_name, pose_func, finger_ctrl in strategies:
    m_test = mujoco.MjModel.from_xml_path(XML)
    d_test = mujoco.MjData(m_test)
    
    # Re-initialize IDs
    tron1_bi = mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_BODY, "base_Link")
    root_jnt = m_test.body_jntadr[tron1_bi]
    root_qp = m_test.jnt_qposadr[root_jnt]
    root_dof = m_test.jnt_dofadr[root_jnt]
    lq = m_test.jnt_qposadr[mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_JOINT, "left_finger_joint")]
    rq = m_test.jnt_qposadr[mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_JOINT, "right_finger_joint")]
    bgeom = mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_GEOM, "carry_bottle_collision")
    lgeom = mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_geom")
    rgeom = mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_geom")
    ai_lf = mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_left_finger")
    ai_rf = mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_right_finger")
    
    def contact():
        lc=rc=False
        for ci in range(d_test.ncon):
            c=d_test.contact[ci]; a,b=int(c.geom1),int(c.geom2)
            if {a,b}=={bgeom,lgeom}: lc=True
            if {a,b}=={bgeom,rgeom}: rc=True
        return lc,rc
    
    def set_ctrl(q_arm, q_f):
        for jn,qv in zip(arm_joints,q_arm):
            ai=mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_ACTUATOR, act_map[jn])
            if ai>=0: d_test.ctrl[ai]=qv
        d_test.ctrl[ai_lf]=q_f; d_test.ctrl[ai_rf]=q_f
    
    def fix_tron1():
        d_test.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
        d_test.qvel[root_dof:root_dof+6] = 0.0
    
    # Reset to G13 state
    d_test.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d_test.qvel[:] = 0.0
    for jn,qv in zip(arm_joints,Q0):
        d_test.qpos[m_test.jnt_qposadr[mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = qv
    d_test.qpos[lq]=0.02; d_test.qpos[rq]=0.02
    set_ctrl(Q0, 0.085)
    m_test.opt.gravity[:] = 0.0
    mujoco.mj_forward(m_test, d_test)
    
    # Achieve G13 contact
    for step in range(2000):
        fix_tron1()
        set_ctrl(Q0, 0.085)
        mujoco.mj_step(m_test, d_test)
        lc0,rc0=contact()
        if lc0 and rc0:
            break
    
    # G14 with compensation
    m_test.opt.gravity[:] = gravity_orig
    b_start = d_test.xpos[mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_BODY, "carry_bottle")].copy()
    
    contact_loss = -1
    for step in range(2500):
        fix_tron1()
        Q_current = pose_func(step)
        set_ctrl(Q_current, finger_ctrl)
        mujoco.mj_step(m_test, d_test)
        lc2,rc2=contact()
        if (not lc2 or not rc2) and contact_loss<0:
            contact_loss=step+1
            break
    
    b_end = d_test.xpos[mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_BODY, "carry_bottle")].copy()
    lc_h,rc_h=contact()
    drift=float(np.linalg.norm(b_end-b_start))
    
    result_str = "PASS" if (contact_loss<0 and lc_h and rc_h and drift<0.025) else "FAIL"
    contact_str = "LOST@%d" % contact_loss if contact_loss>=0 else "SUSTAINED"
    print(f"{strategy_name:20s} | {contact_str:12s} | {drift:.4f}   | {result_str:4s}")

print("\n=== DIAGNOSIS ===")
print("If no strategy passes, the pose Q0 is fundamentally gravity-incompatible")
print("Consider: 1) Accepting the V3 settle+measure approach")
print("          2) Modifying the handover configuration for gravity-stable pose")
print("          3) Revisiting the gate requirements for gravity compatibility")
