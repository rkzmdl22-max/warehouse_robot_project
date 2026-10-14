"""
G14 Y-axis approach analysis
Current: X-axis grasp (fingers approach from +-X)
  inner_gap = 0.0588m < bottle 0.07m -> contact impossible to sustain

Alternative: approach bottle from Y direction
  Bottle at [0.06, -3.0, 1.035]
  UR5e base at [0.6, -3.45, 0.963]
  
  If we rotate wrist so fingers approach along Y axis:
    left_finger from +Y, right_finger from -Y
    bottle diameter along Y = 0.07m
    finger positions along Y must straddle the bottle

Strategy: scan shoulder_pan to approach from a different direction
  so that finger X-axis maps to world Y-axis near the bottle.

Also: examine finger geom geometry more carefully.

XML MODIFIED = NO
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

arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

HANDOVER_QPOS7 = np.array([0.0,-3.0,0.82, 1.0,0.0,0.0,0.0])
tron1_bi = bid("base_Link")
root_jnt  = m.body_jntadr[tron1_bi]
root_qp   = m.jnt_qposadr[root_jnt]

# === GEOMETRY ANALYSIS ===
print("=== GEOMETRY AT Q0 ===")
d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[qadr("left_finger_joint")] = 0.0
d.qpos[qadr("right_finger_joint")] = 0.0
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

# Finger geom half-extents: size="0.008 0.015 0.03"
# In local frame: X=0.008, Y=0.015, Z=0.03
# Left finger: local pos=[-0.0325, 0, 0.08], axis=+X (slides in +X)
# Right finger: local pos=[+0.0325, 0, 0.08], axis=-X (slides in -X)
# At qpos=0: lf at local[-0.0325,0,0.08], rf at local[+0.0325,0,0.08]

lf = xpos("left_finger_link")
rf = xpos("right_finger_link")
t0 = xpos("tool0")
b  = xpos("carry_bottle")
print("tool0:", t0.round(4))
print("lf   :", lf.round(4))
print("rf   :", rf.round(4))
print("bottle:", b.round(4))

# Get tool0 rotation matrix
tool0_bi = bid("tool0")
tool0_xmat = d.xmat[tool0_bi].reshape(3,3)
print("tool0 xmat (col=world axis):")
print("  X:", tool0_xmat[:,0].round(3))
print("  Y:", tool0_xmat[:,1].round(3))
print("  Z:", tool0_xmat[:,2].round(3))

# Left finger slides along local +X -> world direction:
lf_slide_world = tool0_xmat @ np.array([1,0,0])
print("LF slide direction (world):", lf_slide_world.round(3))

# Bottle cylinder axis = Z world
# For optimal grasp: finger slide direction should be perpendicular to Z
# and should point through bottle center
# Current approach is ~X world

# What's the angle between finger approach and bottle-to-tool0 vector?
approach_vec = t0[:2] - b[:2]  # XY only
approach_ang = math.degrees(math.atan2(approach_vec[1], approach_vec[0]))
print("Approach angle (tool0 from bottle):", round(approach_ang,2), "deg")

# Finger inner surface at qpos=0:
# Left: inner X face = lf.x + 0.008 (facing +X direction, inner face)
# Right: inner X face = rf.x - 0.008
lf_inner_x = lf[0] + tool0_xmat[0,0] * 0.008  # inner face
rf_inner_x = rf[0] - tool0_xmat[0,0] * 0.008  # inner face
print("LF inner X:", round(lf_inner_x,4))
print("RF inner X:", round(rf_inner_x,4))
print("Inner gap:", round(abs(lf_inner_x - rf_inner_x),4), "m")
print("Bottle geom radius:", 0.035, "m  (diameter=0.07)")

# === WHAT IF WE TILT TOOL0 so fingers approach from Y? ===
print("\n=== CHECKING IF FINGER CAN REACH BOTTLE VIA Y APPROACH ===")
# Bottle at Y=-3.0, tool0 at Y=-2.92 -> tool0 is +Y relative to bottle
# If we want fingers to come from +-Y:
# finger slide axis (+X local) must map to +/-Y world
# This requires tool0 to be rotated 90deg around Z
# wrist_3 rotation (around its axis) rotates tool0 frame

for w3_test in [0.0, 0.785, 1.571, -0.785, -1.571, 2.356, -2.356]:
    q_t = Q0.copy(); q_t[5] = w3_test
    for jn,qv in zip(arm_joints,q_t):
        d.qpos[qadr(jn)] = qv
    mujoco.mj_forward(m, d)
    t0_mat = d.xmat[tool0_bi].reshape(3,3)
    lf_slide = t0_mat @ np.array([1,0,0])  # local X -> world
    # Check: does lf_slide point in Y direction?
    y_component = abs(lf_slide[1])
    t0p = xpos("tool0")
    lfp = xpos("left_finger_link"); rfp = xpos("right_finger_link")
    fc = 0.5*(lfp+rfp)
    dist = float(np.linalg.norm(fc - b))
    print("  w3=%.3f  lf_slide=%s  Y_comp=%.3f  fc_dist=%.4f  t0=%s" % (
        w3_test, lf_slide.round(3), y_component, dist, t0p.round(3)))

# === CAN bottle_target BE USED INSTEAD? ===
print("\n=== BOTTLE_TARGET ANALYSIS ===")
bt_bi = bid("bottle_target")
bt_jnt = m.body_jntadr[bt_bi]
bt_jtype = m.jnt_type[bt_jnt]
print("bottle_target body id:", bt_bi)
print("bottle_target joint type:", bt_jtype, "(3=free, 0=fixed)")
print("bottle_target xpos:", d.xpos[bt_bi].round(4))
# bottle_target is at [1.0, -3.0, 0.92] with freejoint
# It's on the station table, not the TRON1 tray
print()
print("bottle_target is at station table (NOT on TRON1 tray)")
print("carry_bottle is on TRON1 tray")
print("To grasp carry_bottle: need TRON1 to be stationary OR bottle freejoint")

# === MINIMUM XML CHANGE ANALYSIS ===
print("\n=== MINIMUM XML CHANGE ANALYSIS ===")
print("Current finger joint range: ctrlrange=[-0.015, 0.07]")
print("At qpos=-0.015: gap=0.079m > bottle 0.07m -> can achieve contact")
print("But ctrl=-0.015 drives qpos TOWARD -0.015 (more open, not more closed)")
print()
print("CLARIFICATION:")
print("  left_finger axis=+X: positive qpos moves finger in +X direction")
print("  left_finger local pos = [-0.0325, 0, 0.08]")
print("  At qpos=0: lf at X=-0.0325 from tool0")
print("  At qpos=0.07: lf at X=-0.0325+0.07 = +0.0375 from tool0")
print("  -> Positive qpos moves lf TOWARD right (+X)")
print()
print("  right_finger axis=-X: positive qpos moves finger in -X direction")
print("  right_finger local pos = [+0.0325, 0, 0.08]")
print("  At qpos=0: rf at X=+0.0325 from tool0")
print("  At qpos=0.07: rf at X=0.0325-0.07 = -0.0375 from tool0")
print("  -> Positive qpos moves rf TOWARD left (-X)")
print()
print("  ctrl=-0.015 drives TOWARD qpos=-0.015 (max OPEN)")
print("  ctrl=0.07  drives TOWARD qpos=0.07 (max CLOSE)")
print()
print("  Re-analysis: at qpos=0.07 (max close):")
print("    lf inner x from tool0: -0.0325 + 0.07 + 0.008 = 0.0455")
print("    rf inner x from tool0: +0.0325 - 0.07 - 0.008 = -0.0455")
print("    gap = 0.0455 - (-0.0455) = 0.091m")
print()
print("  Wait - recalculate at qpos=0:")
lf0_x = -0.0325 + 0.0 + 0.008
rf0_x =  0.0325 - 0.0 - 0.008
gap0 = rf0_x - lf0_x
print("    at qpos=0: lf_inner=%.4f  rf_inner=%.4f  gap=%.4f" % (lf0_x, rf0_x, gap0))
print()
lf7_x = -0.0325 + 0.07 + 0.008
rf7_x =  0.0325 - 0.07 - 0.008
gap7 = rf7_x - lf7_x
print("  at qpos=0.07 (max CLOSE): lf_inner=%.4f  rf_inner=%.4f  gap=%.4f" % (lf7_x, rf7_x, gap7))
print("  bottle diameter = 0.070m")
print()
if gap7 < 0.07:
    print("  AT MAX CLOSE: gap(%.4f) < bottle(0.07) -> OVERCLAMPING possible" % gap7)
    print("  Need qpos where gap = 0.07:")
    # rf - lf = 0.07: (0.0325-qp-0.008) - (-0.0325+qp+0.008) = 0.07
    # 0.065 - 2*qp - 0.016 = 0.07
    # -2*qp = 0.07 - 0.065 + 0.016 = 0.021
    # qp = -0.0105
    qp_contact = (0.065 - 0.016 - 0.07) / 2
    print("  Contact qpos = %.4f" % qp_contact)
    print("  -> ctrl=%.4f achieves exact bottle contact gap" % qp_contact)
else:
    print("  AT MAX CLOSE: gap > bottle -> never achieves contact from X-axis!")

print()
print("FINAL DIAGNOSIS:")
print("  ctrl=-0.015: drives to OPEN (qpos=-0.015), gap=0.079m > bottle 0.07m")
print("  Contact can only happen as bottle is already INSIDE the finger gap")
print("  This means bottle must be positioned AT the right spot before closing")
print("  Then ctrl=-0.0105 (or slightly negative) achieves contact")
print()
print("PROPOSED CTRL FIX: use ctrl=-0.0105 instead of ctrl=-0.015")
print("This drives qpos to -0.0105 which achieves gap=0.07=bottle diameter")
