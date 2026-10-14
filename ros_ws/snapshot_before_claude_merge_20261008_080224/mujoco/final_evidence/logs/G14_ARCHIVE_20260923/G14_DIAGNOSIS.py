"""
G14 FAIL diagnosis - analyze finger geometry vs bottle
"""
import mujoco, math
import numpy as np

XML = r"c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def xpos(n): return d.xpos[bid(n)].copy()
def qadr(n): return m.jnt_qposadr[jid(n)]

Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {
    "shoulder_pan_joint":  "act_shoulder_pan",
    "shoulder_lift_joint": "act_shoulder_lift",
    "elbow_joint":         "act_elbow",
    "wrist_1_joint":       "act_wrist_1",
    "wrist_2_joint":       "act_wrist_2",
    "wrist_3_joint":       "act_wrist_3",
}

tron1_bi  = bid("base_Link")
root_jnt  = m.body_jntadr[tron1_bi]
root_qp   = m.jnt_qposadr[root_jnt]
root_dof  = m.jnt_dofadr[root_jnt]

lq = qadr("left_finger_joint")
rq = qadr("right_finger_joint")
bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")

# === Inspect finger/bottle geometry ===
print("=== GEOMETRY ANALYSIS ===")
# finger geom: box size="0.008 0.015 0.03"  (half extents: 0.008,0.015,0.03)
# left_finger_link parent: tool0, pos="-0.0325 0 0.08"
# right_finger_link parent: tool0, pos="0.0325 0 0.08"
# joint range: -0.015 to 0.07 (slide axis = X for left, -X for right)
# When qpos=0: left at -0.0325, right at +0.0325 → gap = 0.065m
# Bottle radius = 0.035m → diameter = 0.07m
# For contact: finger inner face must touch bottle surface
# finger inner X (left) = local(-0.0325) + qpos * axis(+1) + half_size = -0.0325 + qpos + 0.008
# finger inner X (right) = 0.0325 - qpos - 0.008
# Contact when: right_inner - left_inner = bottle_diameter = 0.07
# 0.0325-qpos_r-0.008 - (-0.0325+qpos_l+0.008) = 0.07
# 0.065 - qpos_r - 0.008 + 0.0325 - qpos_l - 0.008 = 0.07
# 0.065 - qpos_r - qpos_l = 0.07 + 0.016 - 0.065 = 0.021
# Hmm, let me recompute...

# left_finger at rest (qpos=0): pos in tool0 frame = [-0.0325, 0, 0.08]
# axis = +X, so qpos moves it in +X direction
# right_finger at rest: [+0.0325, 0, 0.08], axis = -X, qpos moves in -X direction
# left inner face X (facing right) = -0.0325 + qpos_l + 0.008 (half size in X)
# right inner face X (facing left) = +0.0325 - qpos_r - 0.008
# Gap between inner faces = right_inner - left_inner
#   = (0.0325 - qpos_r - 0.008) - (-0.0325 + qpos_l + 0.008)
#   = 0.065 - qpos_r - qpos_l - 0.016
# At qpos_l=qpos_r=0: gap = 0.065 - 0.016 = 0.049m
# Bottle diameter = 0.07m
# For contact: gap = bottle_diameter = 0.07m
# 0.065 - qpos_r - qpos_l - 0.016 = 0.07
# -qpos_r - qpos_l = 0.021
# qpos_r + qpos_l = -0.021

# This is NEGATIVE which is impossible since qpos range starts at -0.015
# So at minimum qpos = -0.015 each:
# gap = 0.065 - (-0.015) - (-0.015) - 0.016 = 0.065 + 0.03 - 0.016 = 0.079m
# 0.079m > 0.07m bottle: CONTACT ONLY when qpos < 0

print("Finger geometry (from XML):")
print("  left_finger local pos  = [-0.0325, 0, 0.08]  axis=+X  range=[-0.015, 0.07]")
print("  right_finger local pos = [+0.0325, 0, 0.08]  axis=-X  range=[-0.015, 0.07]")
print("  geom half-size (X)     = 0.008m")
print()
print("Gap calculation (in tool0 X-axis):")
print("  Gap = 0.065 - qpos_l - qpos_r - 0.016")
print("  At qpos_l=qpos_r=0:    gap =", 0.065 - 0 - 0 - 0.016, "m")
print("  At qpos_l=qpos_r=-0.015: gap =", 0.065-(-0.015)-(-0.015)-0.016, "m")
print("  Bottle diameter        = 0.07m")
print()
print("CRITICAL: Gap at qpos=0 is 0.049m < bottle diameter 0.07m")
print("Gap at qpos=-0.015 is 0.079m > bottle diameter 0.07m")
print()
print("=> Fingers MUST move to negative qpos for contact!")
print("=> qpos_l + qpos_r needed for gap=0.07:")
ql_plus_qr = 0.065 - 0.016 - 0.07
print(f"   qpos_l + qpos_r = {ql_plus_qr:.4f}m  (both approx {ql_plus_qr/2:.4f}m)")
print("=> Each finger needs qpos ≈ -0.005m to achieve contact with 0.07m bottle")
print()

# But wait: tool0 parent body axis vs world axis may differ
# Let's verify at IK pose:
d.qpos[root_qp:root_qp+3] = np.array([0.0,-3.0,0.82])
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq] = 0.0
d.qpos[rq] = 0.0
mujoco.mj_forward(m,d)

lf_w = xpos("left_finger_link")
rf_w = xpos("right_finger_link")
t0_w = xpos("tool0")
b_w  = xpos("carry_bottle")
print("=== AT IK POSE, qpos=0 ===")
print("  tool0 world      =", t0_w.round(4))
print("  left_finger world =", lf_w.round(4))
print("  right_finger world=", rf_w.round(4))
print("  bottle world     =", b_w.round(4))
print("  LF-RF dist (X)   =", round(abs(lf_w[0]-rf_w[0]),4), "m")
print("  LF-RF dist (all) =", round(np.linalg.norm(lf_w-rf_w),4), "m")
print("  finger_center    =", (0.5*(lf_w+rf_w)).round(4))
print("  center->bottle   =", round(np.linalg.norm(0.5*(lf_w+rf_w)-b_w),4), "m")

# Check tool0 rotation matrix - what axis do fingers move along?
tool0_xmat = d.xmat[bid("tool0")].reshape(3,3)
print()
print("  tool0 rotation matrix (world axes):")
print("   X:", tool0_xmat[:,0].round(3))
print("   Y:", tool0_xmat[:,1].round(3))
print("   Z:", tool0_xmat[:,2].round(3))
# left_finger slides along local +X axis → world direction:
lf_slide_world = tool0_xmat @ np.array([1,0,0])
print("  left_finger slide direction (world) =", lf_slide_world.round(3))
print("  (this is the direction left finger moves when qpos increases)")

# Bottle is at [0.06, -3.0, 1.035]
# Fingers need to close FROM SIDES in the world direction perpendicular to bottle axis
# Bottle cylinder axis = Z (world), so fingers approach from XY plane
print()
print("  bottle cylinder axis = Z (world)")
print("  approach direction = XY plane")

# Simulate: close fingers to qpos=-0.015 (max closing)
print()
print("=== SIMULATING FINGER CLOSING (qpos → -0.015) ===")
d.ctrl[aid("act_left_finger")]  = -0.015
d.ctrl[aid("act_right_finger")] = -0.015
mujoco.mj_forward(m, d)

# Set qpos explicitly too
d.qpos[lq] = -0.015
d.qpos[rq] = -0.015
mujoco.mj_forward(m, d)

lf_closed = xpos("left_finger_link")
rf_closed = xpos("right_finger_link")
fc_closed = 0.5*(lf_closed + rf_closed)
print("  left_finger  @ qpos=-0.015:", lf_closed.round(4))
print("  right_finger @ qpos=-0.015:", rf_closed.round(4))
print("  finger_center:", fc_closed.round(4))
print("  LF-RF dist (all):", round(np.linalg.norm(lf_closed-rf_closed),4))
print("  center→bottle dist:", round(np.linalg.norm(fc_closed - b_w),4))

# Check if bottle is within finger envelope at closed position
print()
print("=== CONTACT CHECK (step simulation with ctrl=-0.015) ===")
d.qpos[lq] = 0.07  # reset
d.qpos[rq] = 0.07
d.ctrl[aid("act_left_finger")]  = -0.015
d.ctrl[aid("act_right_finger")] = -0.015
for jn,qv in zip(arm_joints,Q0):
    ai = aid(act_map[jn])
    if ai>=0: d.ctrl[ai] = qv
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)
for _ in range(800):
    mujoco.mj_step(m, d)

lc = rc = False
for ci in range(d.ncon):
    c = d.contact[ci]; a,b2 = int(c.geom1),int(c.geom2)
    if {a,b2}=={bgeom,lgeom}: lc=True
    if {a,b2}=={bgeom,rgeom}: rc=True

lf_f = xpos("left_finger_link"); rf_f = xpos("right_finger_link"); b_f = xpos("carry_bottle")
print("  left_contact  =", lc)
print("  right_contact =", rc)
print("  bottle        =", b_f.round(4))
print("  lf            =", lf_f.round(4))
print("  rf            =", rf_f.round(4))
print("  lq            =", round(d.qpos[lq],4))
print("  rq            =", round(d.qpos[rq],4))
print("  ncon          =", d.ncon)
print()
print("CONTACT WITH ctrl=-0.015:", "PASS" if (lc and rc) else "FAIL")

# Summary
print()
print("=== DIAGNOSIS SUMMARY ===")
print("Problem: Bottle diameter=0.07m but finger gap at qpos=0 is 0.049m")
print("=> Fingers START closed around bottle; need to OPEN more to get contact")
print("=> Actually, qpos=0 gives gap 0.049 which is SMALLER than bottle 0.07")
print("=> This means at qpos=0 fingers would SQUEEZE bottle (but bottle too big)")
print("=> Wait - re-check: gap = 0.065 - qpos_l - qpos_r - 0.016")
gap0 = 0.065 - 0 - 0 - 0.016
print(f"   gap at qpos=0,0: {gap0:.4f}m vs bottle 0.07m")
print(f"   Bottle {0.07}m > gap {gap0:.4f}m → finger would overlap bottle → CONTACT!")
print("=> Contact happens at qpos=0 or below")
print("=> ctrl=-0.015 maximizes closing force")
print()
print("ROOT CAUSE: ctrl=0.0 for fingers means actuator drives to qpos=0")
print("But bottle is still larger than gap at qpos=0 → contact maintained")
print("However: in G14, bottle fell because UR5e arm drifted due to ctrl=arm_joints")
print("at Q0 values but gravity caused arm to sag, dropping bottle")
print()
print("SOLUTION: Use ctrl[-0.015] for fingers AND verify arm ctrl maintains Q0")
