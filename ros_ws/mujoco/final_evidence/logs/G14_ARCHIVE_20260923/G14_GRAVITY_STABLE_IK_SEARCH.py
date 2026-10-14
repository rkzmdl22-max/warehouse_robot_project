"""
G14 GRAVITY-STABLE POSE SEARCH USING IK SCANNING
Systematic search for gravity-stable poses using existing IK scanning approach
XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
print("G14 GRAVITY-STABLE POSE SEARCH USING IK SCANNING")
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

def vertical_alignment_score(Q_test):
    """Calculate vertical alignment score for a given pose"""
    d_test = mujoco.MjData(m)
    d_test.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d_test.qvel[:] = 0.0
    for jn,qv in zip(arm_joints,Q_test):
        d_test.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = qv
    d_test.qpos[lq]=0.02; d_test.qpos[rq]=0.02
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d_test)
    
    bottle_pos = d_test.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "carry_bottle")]
    tool0_pos = d_test.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tool0")]
    
    bottle_to_tool0 = tool0_pos - bottle_pos
    if np.linalg.norm(bottle_to_tool0) > 0:
        bottle_to_tool0_norm = bottle_to_tool0 / np.linalg.norm(bottle_to_tool0)
    else:
        bottle_to_tool0_norm = np.array([0, 0, -1])
    
    vertical_alignment = np.dot(bottle_to_tool0_norm, np.array([0, 0, -1]))
    return vertical_alignment

# ── STRATEGY: Find poses with better vertical alignment than Q0 ──────────────
print("\n=== PHASE 1: VERTICAL ALIGNMENT SCAN ===")
Q0_vert_align = vertical_alignment_score(Q0)
print(f"Q0 vertical alignment: {Q0_vert_align:.4f}")
print("Scanning for poses with better vertical alignment...")

# Focus on key joints that affect vertical alignment
# shoulder_lift and wrist_2 are most critical for vertical orientation
scan_configs = [
    # (shoulder_lift_range, wrist_2_range, description)
    (0.2, 0.2, "moderate range"),
    (0.4, 0.4, "extended range"),
]

candidates = []
for sl_range, w2_range, desc in scan_configs:
    print(f"\nScanning {desc}: sl_range={sl_range}, w2_range={w2_range}")
    
    sl_values = np.linspace(Q0[1] - sl_range, Q0[1] + sl_range, 9)
    w2_values = np.linspace(Q0[4] - w2_range, Q0[4] + w2_range, 9)
    
    for sl_val in sl_values:
        for w2_val in w2_values:
            Q_test = Q0.copy()
            Q_test[1] = sl_val  # shoulder_lift
            Q_test[4] = w2_val  # wrist_2
            
            vert_align = vertical_alignment_score(Q_test)
            
            # Only keep poses with better vertical alignment
            if vert_align > Q0_vert_align + 0.1:  # Significant improvement
                candidates.append((Q_test, vert_align))
                print(f"  Candidate: sl={sl_val:.3f}, w2={w2_val:.3f}, vert_align={vert_align:.4f}")

print(f"\nFound {len(candidates)} candidates with improved vertical alignment")

if len(candidates) == 0:
    print("No candidates found. Trying wider search...")
    # Try wider ranges
    sl_values = np.linspace(Q0[1] - 0.5, Q0[1] + 0.5, 11)
    w2_values = np.linspace(Q0[4] - 0.5, Q0[4] + 0.5, 11)
    
    for sl_val in sl_values:
        for w2_val in w2_values:
            Q_test = Q0.copy()
            Q_test[1] = sl_val
            Q_test[4] = w2_val
            
            vert_align = vertical_alignment_score(Q_test)
            
            if vert_align > Q0_vert_align + 0.05:  # Lower threshold
                candidates.append((Q_test, vert_align))
    
    print(f"After wider search: {len(candidates)} candidates")

# ── PHASE 2: TEST CANDIDATES FOR G13/G14 ─────────────────────────────────────
print("\n=== PHASE 2: TESTING CANDIDATES ===")
print("Testing top candidates for G13 bilateral contact and G14 gravity hold...")

# Sort by vertical alignment (best first)
candidates.sort(key=lambda x: x[1], reverse=True)

# Test top 10 candidates (or all if fewer)
test_count = min(10, len(candidates))
print(f"Testing top {test_count} candidates...")

for i, (Q_test, vert_align) in enumerate(candidates[:test_count]):
    print(f"\n--- Candidate {i+1}/{test_count} ---")
    print(f"Q: {Q_test.round(4)}, vert_align: {vert_align:.4f}")
    
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
        print("  G13: FAIL (no bilateral contact)")
        continue
    
    print("  G13: PASS (bilateral contact)")
    
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
    print(f"  G14: {result_str} (contact_loss={contact_str}, drift={drift:.4f})")
    
    if result_str == "PASS":
        print(f"\n*** SUCCESS: GRAVITY-STABLE POSE FOUND ***")
        print(f"Q_GRAVITY_STABLE = {Q_test.round(4)}")
        print(f"Vertical alignment: {vert_align:.4f}")
        print(f"Drift: {drift:.4f} m")
        sys.exit(0)

print("\n=== FINAL RESULT ===")
print("No gravity-stable pose found in search space")
print("Consider:")
print("1. Expanding search ranges further")
print("2. Testing additional joint combinations")
print("3. Alternative resolution approaches")
