"""
G14 GRASP GEOMETRY ANALYSIS
Analyze bottle geometry, contact points, and find gravity-stable pose
XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
print("G14 GRASP GEOMETRY ANALYSIS")
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

def contact_details():
    """Return detailed contact information"""
    left_contacts = []
    right_contacts = []
    for ci in range(d.ncon):
        c=d.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bgeom,lgeom}:
            left_contacts.append(ci)
        if {a,b}=={bgeom,rgeom}:
            right_contacts.append(ci)
    return left_contacts, right_contacts

def set_ctrl(q_arm, q_f):
    for jn,qv in zip(arm_joints,q_arm):
        ai=aid(act_map[jn])
        if ai>=0: d.ctrl[ai]=qv
    d.ctrl[ai_lf]=q_f; d.ctrl[ai_rf]=q_f

def fix_tron1():
    d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d.qvel[root_dof:root_dof+6] = 0.0

# ── GEOMETRY ANALYSIS ─────────────────────────────────────────────────────
print("\n=== BOTTLE GEOMETRY ===")
bottle_body = bid("carry_bottle")
bottle_pos = xpos("carry_bottle")
bottle_mass = m.body_mass[bottle_body]
print(f"Bottle position: {bottle_pos.round(4)}")
print(f"Bottle mass: {bottle_mass:.4f} kg")

# Get bottle geom info
bottle_geom_id = gid("carry_bottle_collision")
bottle_size = m.geom_size[bottle_geom_id]
print(f"Bottle geom size: {bottle_size}")

# Get finger geom info
left_finger_geom = gid("left_finger_geom")
right_finger_geom = gid("right_finger_geom")
left_size = m.geom_size[left_finger_geom]
right_size = m.geom_size[right_finger_geom]
print(f"Left finger size: {left_size}")
print(f"Right finger size: {right_size}")

# ── GRASP ANALYSIS IN ZERO-G ───────────────────────────────────────────────
print("\n=== GRASP ANALYSIS IN ZERO-G ===")
d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.02; d.qpos[rq]=0.02
set_ctrl(Q0, -0.015)
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

# Achieve bilateral contact
for step in range(2000):
    fix_tron1()
    set_ctrl(Q0, -0.015)
    mujoco.mj_step(m, d)
    lc0,rc0=contact()
    if lc0 and rc0:
        print(f"Bilateral contact achieved at step {step+1}")
        break

bottle_pos_g13 = xpos("carry_bottle")
left_pos = d.xpos[bid("left_finger_link")]
right_pos = d.xpos[bid("right_finger_link")]
tool0_pos = d.xpos[bid("tool0")]

print(f"Bottle position: {bottle_pos_g13.round(4)}")
print(f"Left finger position: {left_pos.round(4)}")
print(f"Right finger position: {right_pos.round(4)}")
print(f"Tool0 position: {tool0_pos.round(4)}")

# Calculate distances
left_bottle_dist = np.linalg.norm(left_pos - bottle_pos_g13)
right_bottle_dist = np.linalg.norm(right_pos - bottle_pos_g13)
print(f"Left finger to bottle distance: {left_bottle_dist:.4f} m")
print(f"Right finger to bottle distance: {right_bottle_dist:.4f} m")

# Get contact details
left_contacts, right_contacts = contact_details()
print(f"Left contact count: {len(left_contacts)}")
print(f"Right contact count: {len(right_contacts)}")

# Analyze contact points
if left_contacts:
    print(f"Left contact IDs: {left_contacts}")
if right_contacts:
    print(f"Right contact IDs: {right_contacts}")

# ── GRAVITY VECTOR ANALYSIS ─────────────────────────────────────────────────
print("\n=== GRAVITY VECTOR ANALYSIS ===")
print(f"Gravity vector: {gravity_orig}")
print(f"Bottle center of mass: {bottle_pos_g13.round(4)}")

# Calculate vector from bottle to tool0
bottle_to_tool0 = tool0_pos - bottle_pos_g13
print(f"Bottle to tool0 vector: {bottle_to_tool0.round(4)}")

# Calculate gravity effect
gravity_force = gravity_orig * bottle_mass
print(f"Gravity force on bottle: {gravity_force.round(4)} N")

# Check if bottle is centered between fingers
finger_midpoint = (left_pos + right_pos) / 2
bottle_to_midpoint = finger_midpoint - bottle_pos_g13
print(f"Bottle to finger midpoint: {bottle_to_midpoint.round(4)}")
print(f"Horizontal offset: {np.linalg.norm(bottle_to_midpoint[:2]):.4f} m")

# ── WRIST ORIENTATION ANALYSIS ─────────────────────────────────────────────
print("\n=== WRIST ORIENTATION ANALYSIS ===")
wrist_1_q = d.qpos[qadr("wrist_1_joint")]
wrist_2_q = d.qpos[qadr("wrist_2_joint")]
wrist_3_q = d.qpos[qadr("wrist_3_joint")]
print(f"Wrist 1: {wrist_1_q:.4f} rad")
print(f"Wrist 2: {wrist_2_q:.4f} rad")
print(f"Wrist 3: {wrist_3_q:.4f} rad")

# Get wrist orientations
wrist_1_body = bid("wrist_1_link")
wrist_2_body = bid("wrist_2_link")
wrist_3_body = bid("wrist_3_link")
wrist_1_xmat = d.xmat[wrist_1_body].reshape(3,3)
wrist_2_xmat = d.xmat[wrist_2_body].reshape(3,3)
wrist_3_xmat = d.xmat[wrist_3_body].reshape(3,3)

print(f"Wrist 1 orientation matrix:\n{wrist_1_xmat.round(4)}")
print(f"Wrist 2 orientation matrix:\n{wrist_2_xmat.round(4)}")
print(f"Wrist 3 orientation matrix:\n{wrist_3_xmat.round(4)}")

# Check if wrist orientation compensates for gravity
# For stable grasp, the gripper should be oriented such that
# the bottle's center of mass is directly below the tool0
vertical_alignment = np.dot(bottle_to_tool0 / np.linalg.norm(bottle_to_tool0), 
                           np.array([0, 0, -1]))
print(f"Vertical alignment (1.0 = perfect): {vertical_alignment:.4f}")

print("\n=== DIAGNOSIS ===")
if np.linalg.norm(bottle_to_midpoint[:2]) > 0.01:
    print("ISSUE: Bottle not centered between fingers horizontally")
    print(f"  Horizontal offset: {np.linalg.norm(bottle_to_midpoint[:2]):.4f} m")
if vertical_alignment < 0.9:
    print("ISSUE: Wrist orientation not optimal for gravity compensation")
    print(f"  Vertical alignment: {vertical_alignment:.4f}")
if left_bottle_dist != right_bottle_dist:
    print("ISSUE: Asymmetric finger distances to bottle")
    print(f"  Left: {left_bottle_dist:.4f} m, Right: {right_bottle_dist:.4f} m")

print("\n=== RECOMMENDATIONS ===")
print("1. Consider adjusting wrist_3 angle to center bottle between fingers")
print("2. Consider adjusting wrist_2 angle for better vertical alignment")
print("3. Test small wrist angle perturbations to find gravity-stable pose")
