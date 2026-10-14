"""
G14 GRAVITY-STABLE POSE SEARCH
Systematic search for gravity-stable arm pose using inverse kinematics principles
XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
print("G14 GRAVITY-STABLE POSE SEARCH")
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

def vertical_alignment_score(m, d, Q_test):
    """Calculate how well arm aligns bottle vertically with tool0"""
    d_test = mujoco.MjData(m)
    d_test.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d_test.qvel[:] = 0.0
    for jn,qv in zip(arm_joints,Q_test):
        d_test.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = qv
    d_test.qpos[lq]=0.02; d_test.qpos[rq]=0.02
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d_test)
    
    # Get positions
    bottle_pos = d_test.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "carry_bottle")]
    tool0_pos = d_test.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tool0")]
    
    # Calculate vector from bottle to tool0
    bottle_to_tool0 = tool0_pos - bottle_pos
    if np.linalg.norm(bottle_to_tool0) > 0:
        bottle_to_tool0_norm = bottle_to_tool0 / np.linalg.norm(bottle_to_tool0)
    else:
        bottle_to_tool0_norm = np.array([0, 0, -1])
    
    # Vertical alignment (1.0 = perfect vertical alignment)
    vertical_alignment = np.dot(bottle_to_tool0_norm, np.array([0, 0, -1]))
    
    return vertical_alignment

# Systematic search around Q0 with focus on improving vertical alignment
print("\n=== GRAVITY-STABLE POSE SEARCH ===")
print("Focusing on poses that improve vertical alignment")
print("sp   | sl   | el   | w1   | w2   | w3   | vert_align | G13 | G14_5s | result")
print("------|------|------|------|------|------|------------|-----|--------|--------")

# Define search ranges around Q0 (minimal for faster execution)
search_ranges = [
    # (shoulder_pan_range, shoulder_lift_range, elbow_range, wrist_1_range, wrist_2_range, wrist_3_range)
    (0.0, 0.03, 0.0, 0.0, 0.0, 0.03),  # Focus only on shoulder_lift and wrist_3, minimal range
]

results = []
for sp_range, sl_range, el_range, w1_range, w2_range, w3_range in search_ranges:
    # Generate perturbations (minimal for faster execution)
    sp_perturbs = [0.0] if sp_range == 0 else np.linspace(-sp_range, sp_range, 2)
    sl_perturbs = np.linspace(-sl_range, sl_range, 3)  # Minimal points
    el_perturbs = [0.0] if el_range == 0 else np.linspace(-el_range, el_range, 2)
    w1_perturbs = [0.0] if w1_range == 0 else np.linspace(-w1_range, w1_range, 2)
    w2_perturbs = [0.0] if w2_range == 0 else np.linspace(-w2_range, w2_range, 2)
    w3_perturbs = np.linspace(-w3_range, w3_range, 2)  # Minimal points
    
    total_combinations = len(sp_perturbs) * len(sl_perturbs) * len(el_perturbs) * len(w1_perturbs) * len(w2_perturbs) * len(w3_perturbs)
    print(f"Testing {total_combinations} pose combinations...")
    
    tested = 0
    for sp_off in sp_perturbs:
        for sl_off in sl_perturbs:
            for el_off in el_perturbs:
                for w1_off in w1_perturbs:
                    for w2_off in w2_perturbs:
                        for w3_off in w3_perturbs:
                            tested += 1
                            if tested % 50 == 0:
                                print(f"  Progress: {tested}/{total_combinations}")
                            
                            Q_test = Q0 + np.array([sp_off, sl_off, el_off, w1_off, w2_off, w3_off])
                            
                            # Check vertical alignment first (quick filter)
                            vert_align = vertical_alignment_score(m, d, Q_test)
                            if vert_align < 0.5:  # Skip poor alignments
                                continue
                            
                            # Full test
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
                            
                            # G13: Grasp with zero gravity
                            d_test.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
                            d_test.qvel[:] = 0.0
                            for jn,qv in zip(arm_joints,Q_test):
                                d_test.qpos[m_test.jnt_qposadr[mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = qv
                            d_test.qpos[lq]=0.02; d_test.qpos[rq]=0.02
                            set_ctrl(Q_test, -0.015)
                            m_test.opt.gravity[:] = 0.0
                            mujoco.mj_forward(m_test, d_test)
                            
                            got_both = False
                            for step in range(2000):
                                fix_tron1()
                                set_ctrl(Q_test, -0.015)
                                mujoco.mj_step(m_test, d_test)
                                lc0,rc0=contact()
                                if lc0 and rc0:
                                    got_both=True
                                    break
                            
                            if not got_both:
                                continue
                            
                            # G14: Hold with real gravity
                            m_test.opt.gravity[:] = gravity_orig
                            b_start = d_test.xpos[mujoco.mj_name2id(m_test, mujoco.mjtObj.mjOBJ_BODY, "carry_bottle")].copy()
                            
                            contact_loss = -1
                            for step in range(2500):
                                fix_tron1()
                                set_ctrl(Q_test, -0.015)
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
                            
                            Q_str = f"{sp_off:4.1f}|{sl_off:4.1f}|{el_off:4.1f}|{w1_off:4.1f}|{w2_off:4.1f}|{w3_off:4.1f}"
                            print(f"{Q_str}|{vert_align:10.2f}|  YES|{contact_str:8s}|{result_str:4s}")
                            results.append((Q_test, vert_align, contact_loss, lc_h, rc_h, drift, result_str))

print("\n=== BEST POSE ===")
best_pose = None
for Q_test, vert_align, cl, lc, rc, drift, result in results:
    if result == "PASS":
        best_pose = Q_test
        print(f"CANDIDATE: Q={Q_test.round(4)}, vert_align={vert_align:.2f}, drift={drift:.4f}")

if best_pose is None:
    print("NO POSE ACHIEVED PASS")
    print("Best candidates (improved vertical alignment):")
    # Sort by vertical alignment
    results.sort(key=lambda x: x[1], reverse=True)
    for Q_test, vert_align, cl, lc, rc, drift, result in results[:5]:
        print(f"  Q={Q_test.round(4)}: vert_align={vert_align:.2f}, contact_loss={cl}, drift={drift:.4f}")
else:
    print(f"\nRECOMMENDED GRAVITY-STABLE POSE: {best_pose.round(4)}")
