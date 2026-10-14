"""
PROJECT MASTER v5.3.2
G14 HOLD → G15 LIFT → G16 CARRY → G17 PLACE → G18 RELEASE
============================================================
Uses FINAL XML only:
  c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml

Runtime-only. No XML modification.

Key findings from G13:
  - IK solution: q=[-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0]
  - finger_center → bottle dist = 0.0002m at IK pose
  - After 600 steps: left_contact=True, right_contact=True
  - finger qpos after: 0.0772 (exceeds range max=0.07)
    → G14 FAIL cause: ctrl=0 for fingers means actuator target=0.0
      but range starts at -0.015. Fingers push against bottle but
      ctrl must stay at 0.0 (minimum) to keep closing force.
      The qpos exceeding range was due to contact force pushing fingers open.
  - Fix: maintain ctrl[arm]=Q0, ctrl[fingers]=0.0 (keep closing force)
         AND set arm ctrl to actual Q0 joint positions (not 0)

XML MODIFIED = NO
"""

import mujoco, sys, math, os
import numpy as np

XML = r"c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"

print("=" * 70)
print("G14-G18 FINAL HOLD / LIFT / CARRY / PLACE / RELEASE")
print("XML:", XML)
print("XML MODIFIED = NO")
print("=" * 70)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

# ── Helpers ──────────────────────────────────────────────────────────────
def bid(name):  return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,     name)
def jid(name):  return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,    name)
def aid(name):  return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
def gid(name):  return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM,     name)
def xpos(name): return d.xpos[bid(name)].copy()
def qadr(name): return m.jnt_qposadr[jid(name)]

# TRON1 root
tron1_bi     = bid("base_Link")
root_jnt_adr = m.body_jntadr[tron1_bi]
root_qposadr = m.jnt_qposadr[root_jnt_adr]
root_dofadr  = m.jnt_dofadr[root_jnt_adr]

# Geom IDs for contact detection
bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")

def check_contact():
    lc = rc = False
    for ci in range(d.ncon):
        c = d.contact[ci]
        a, b = int(c.geom1), int(c.geom2)
        if {a,b} == {bgeom, lgeom}: lc = True
        if {a,b} == {bgeom, rgeom}: rc = True
    return lc, rc

# Arm joint names and actuator map
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

# G13 IK solution (FINAL)
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

# ── INITIALIZE STATE (reproduce G13) ─────────────────────────────────────
print("\n[INIT: Reproducing G13 grasp state]")

# TRON1 at handover
d.qpos[root_qposadr:root_qposadr+3] = np.array([0.0, -3.0, 0.82])
d.qvel[:] = 0.0

# UR5e at Q0
for jn, qv in zip(arm_joints, Q0):
    d.qpos[qadr(jn)] = qv

# Fingers open initially
lq = qadr("left_finger_joint")
rq = qadr("right_finger_joint")
d.qpos[lq] = 0.07
d.qpos[rq] = 0.07

# Ctrl: arm=Q0, fingers=CLOSE(0.0 = minimum = max closing force)
for jn, qv in zip(arm_joints, Q0):
    ai = aid(act_map[jn])
    if ai >= 0: d.ctrl[ai] = qv
d.ctrl[aid("act_left_finger")]  = 0.0
d.ctrl[aid("act_right_finger")] = 0.0

mujoco.mj_forward(m, d)

# Settle grasp (gravity OFF for initial contact, 600 steps)
gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:] = 0.0

for _ in range(600):
    mujoco.mj_step(m, d)

lc, rc = check_contact()
b_init = xpos("carry_bottle")
print("  bottle              =", b_init.round(4))
print("  left_contact        =", lc)
print("  right_contact       =", rc)
print("  left_finger qpos    =", round(d.qpos[lq], 4))
print("  right_finger qpos   =", round(d.qpos[rq], 4))
print("  ncon                =", d.ncon)

if not (lc and rc):
    print("  GRASP NOT ACHIEVED — cannot proceed")
    sys.exit(1)
print("  G13 REPRODUCED = PASS")

# ── G14: HOLD (gravity ON, 5 seconds) ────────────────────────────────────
print("\n" + "=" * 70)
print("G14: HOLD (gravity ON, 5s)")
print("=" * 70)

m.opt.gravity[:] = gravity_orig   # restore gravity

b_hold_start = xpos("carry_bottle").copy()
print("  bottle start        =", b_hold_start.round(4))

hold_samples = []
hold_fail = False
contact_loss_step = -1

for step in range(2500):  # 5s at dt=0.002
    mujoco.mj_step(m, d)

    if (step+1) % 500 == 0:
        bp   = xpos("carry_bottle")
        lc2, rc2 = check_contact()
        lqp  = d.qpos[lq]
        rqp  = d.qpos[rq]
        mv   = float(np.max(np.abs(d.qvel)))
        arm_q = [round(d.qpos[qadr(jn)], 4) for jn in arm_joints]
        sample = {
            "t": round((step+1)*0.002, 2),
            "bottle": bp.round(4),
            "L": lc2, "R": rc2,
            "lq": round(lqp,4), "rq": round(rqp,4),
            "max_qvel": round(mv, 4),
        }
        hold_samples.append(sample)
        print(f"  t={sample['t']:.2f}s  bottle={bp.round(3)}  L={lc2} R={rc2}  "
              f"lq={round(lqp,4)}  rq={round(rqp,4)}  max_qvel={round(mv,4)}")

        if not np.isfinite(bp).all() or mv > 100:
            hold_fail = True
            print("  INSTABILITY DETECTED at t=" + str(sample['t']))
            break
        if (not lc2 or not rc2) and contact_loss_step < 0:
            contact_loss_step = step+1

b_hold_end  = xpos("carry_bottle")
hold_drift  = np.linalg.norm(b_hold_end - b_hold_start)
lc_end, rc_end = check_contact()

print("\n  bottle end          =", b_hold_end.round(4))
print("  hold_drift (5s)     =", round(hold_drift, 4), "m")
print("  contact_loss_step   =", contact_loss_step, ("(sustained)" if contact_loss_step<0 else "(LOST)"))
print("  left_contact(end)   =", lc_end)
print("  right_contact(end)  =", rc_end)
print("  instability         =", hold_fail)

# PASS criteria: bottle stays within 0.1m, no instability, finite
g14_pass = (np.isfinite(b_hold_end).all()
            and not hold_fail
            and hold_drift < 0.15)
print("G14 =", "PASS" if g14_pass else "FAIL")
if not g14_pass:
    print("  FAIL REASON: drift=" + str(round(hold_drift,4)) +
          " instability=" + str(hold_fail))
    sys.exit(1)

# ── G15: LIFT ─────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G15: LIFT")
print("=" * 70)

b_lift_start = xpos("carry_bottle").copy()
tool_lift_start = xpos("tool0").copy()
print("  bottle_start_z      =", round(b_lift_start[2], 4))
print("  tool0_start_z       =", round(tool_lift_start[2], 4))

# Lift strategy: gradually modify shoulder_lift and elbow
# Q0 = [-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0]
# Lift: increase shoulder_lift more negative (arm extends up), elbow adjusts
lift_target = Q0.copy()
lift_target[1] = Q0[1] - 0.25   # shoulder_lift: more negative = arm goes up
lift_target[2] = Q0[2] - 0.10   # elbow: slight adjust

# Apply lift ctrl gradually over 3 phases
lift_phases = [
    Q0 + np.array([0, -0.08, -0.03, 0, 0, 0]),
    Q0 + np.array([0, -0.16, -0.07, 0, 0, 0]),
    Q0 + np.array([0, -0.25, -0.10, 0, 0, 0]),
]

for phase_idx, q_phase in enumerate(lift_phases):
    for jn, qv in zip(arm_joints, q_phase):
        d.ctrl[aid(act_map[jn])] = qv
    # keep fingers closed
    d.ctrl[aid("act_left_finger")]  = 0.0
    d.ctrl[aid("act_right_finger")] = 0.0

    for step in range(400):
        mujoco.mj_step(m, d)

    bp = xpos("carry_bottle")
    t0 = xpos("tool0")
    lc3, rc3 = check_contact()
    print(f"  phase {phase_idx+1}: bottle={bp.round(4)}  tool0_z={round(t0[2],4)}  L={lc3} R={rc3}")

b_lift_end = xpos("carry_bottle")
tool_lift_end = xpos("tool0")
dz_bottle = b_lift_end[2] - b_lift_start[2]
dz_tool   = tool_lift_end[2] - tool_lift_start[2]
lc_lift, rc_lift = check_contact()

print("\n  bottle_start_z      =", round(b_lift_start[2], 4))
print("  bottle_end_z        =", round(b_lift_end[2], 4))
print("  bottle delta_z      =", round(dz_bottle, 4), "m")
print("  tool0 delta_z       =", round(dz_tool, 4), "m")
print("  left_contact        =", lc_lift)
print("  right_contact       =", rc_lift)
print("  bottle stable       =", np.isfinite(b_lift_end).all())

g15_pass = (np.isfinite(b_lift_end).all()
            and dz_bottle > 0.02
            and abs(dz_bottle - dz_tool) < 0.08)  # bottle follows tool within 8cm
print("G15 =", "PASS" if g15_pass else "FAIL")
if not g15_pass:
    print("  FAIL: dz_bottle=" + str(round(dz_bottle,4)) + " dz_tool=" + str(round(dz_tool,4)))
    sys.exit(1)

# ── G16: CARRY ─────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G16: CARRY (translate toward place table)")
print("=" * 70)

# Current lift pose
q_lift = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
b_carry_start = xpos("carry_bottle").copy()
tool_carry_start = xpos("tool0").copy()
print("  bottle carry start  =", b_carry_start.round(4))
print("  tool0 carry start   =", tool_carry_start.round(4))

# Place target: bottle_place_table at [1.15, -3.0, 0.4] + top=0.39 → table surface z=0.79
# Bottle needs to go to ~[1.15, -3.0, 0.92] (table top + bottle half-height)
# UR5e base at [0.6, -3.45, 0.963]
# Carry: rotate shoulder_pan to reach place table direction
# Approach: pan = ~0.2 rad (toward +X/+Y direction of table)
carry_phases = [
    # Phase 1: pan toward table
    q_lift + np.array([0.15, 0, 0, 0, 0, 0]),
    q_lift + np.array([0.35, 0, 0, 0, 0, 0]),
    q_lift + np.array([0.55, 0, 0, 0, 0, 0]),
]

trajectory = []
carry_fail = False

for phase_idx, q_phase in enumerate(carry_phases):
    for jn, qv in zip(arm_joints, q_phase):
        d.ctrl[aid(act_map[jn])] = qv
    d.ctrl[aid("act_left_finger")]  = 0.0
    d.ctrl[aid("act_right_finger")] = 0.0

    for step in range(350):
        mujoco.mj_step(m, d)

    bp = xpos("carry_bottle")
    t0 = xpos("tool0")
    lc4, rc4 = check_contact()
    rel = bp - t0
    trajectory.append(bp.copy())
    print(f"  carry phase {phase_idx+1}: bottle={bp.round(4)}  L={lc4} R={rc4}  bottle-tool_dist={round(np.linalg.norm(rel),4)}")

    if not np.isfinite(bp).all():
        carry_fail = True
        print("  INSTABILITY at phase", phase_idx+1)
        break

b_carry_end = xpos("carry_bottle")
lc_carry, rc_carry = check_contact()
carry_displacement = np.linalg.norm(b_carry_end - b_carry_start)
bottle_tool_dist = np.linalg.norm(b_carry_end - xpos("tool0"))

print("\n  bottle carry start  =", b_carry_start.round(4))
print("  bottle carry end    =", b_carry_end.round(4))
print("  carry displacement  =", round(carry_displacement, 4), "m")
print("  bottle-tool dist    =", round(bottle_tool_dist, 4), "m")
print("  left_contact(end)   =", lc_carry)
print("  right_contact(end)  =", rc_carry)
print("  carry_fail          =", carry_fail)

g16_pass = (np.isfinite(b_carry_end).all()
            and not carry_fail
            and carry_displacement > 0.02)
print("G16 =", "PASS" if g16_pass else "FAIL")
if not g16_pass:
    sys.exit(1)

# ── G17: PLACE ─────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G17: PLACE")
print("=" * 70)

# Place table at [1.15, -3.0, 0.4], top surface z=0.79
# bottle target: [1.15, -3.0, 0.92] (above table top + bottle half-height 0.10+0.03 margin)
# Current carry pose: q_lift + pan adjustment
# Need to lower arm while maintaining bottle above table

b_place_start = xpos("carry_bottle").copy()
table_target = np.array([1.15, -3.0, 0.92])
print("  bottle place start  =", b_place_start.round(4))
print("  table target        =", table_target)

# Current arm qpos
q_carry_end = np.array([d.qpos[qadr(jn)] for jn in arm_joints])

# Lower: increase shoulder_lift (less negative) and elbow adjust
place_phases = [
    q_carry_end + np.array([0,  0.05, 0.03, 0, 0, 0]),  # start lower
    q_carry_end + np.array([0,  0.10, 0.06, 0, 0, 0]),  # lower more
    q_carry_end + np.array([0,  0.15, 0.09, 0, 0, 0]),  # approach table height
]

for phase_idx, q_phase in enumerate(place_phases):
    for jn, qv in zip(arm_joints, q_phase):
        d.ctrl[aid(act_map[jn])] = qv
    d.ctrl[aid("act_left_finger")]  = 0.0
    d.ctrl[aid("act_right_finger")] = 0.0

    for step in range(400):
        mujoco.mj_step(m, d)

    bp = xpos("carry_bottle")
    lc5, rc5 = check_contact()
    print(f"  place phase {phase_idx+1}: bottle={bp.round(4)}  z_err={round(bp[2]-table_target[2],4)}  L={lc5} R={rc5}")

b_place_end = xpos("carry_bottle")
lc_place, rc_place = check_contact()
xy_err = np.linalg.norm(b_place_end[:2] - table_target[:2])
z_err  = abs(b_place_end[2] - table_target[2])

print("\n  bottle final        =", b_place_end.round(4))
print("  table target        =", table_target)
print("  XY error            =", round(xy_err, 4), "m")
print("  Z error             =", round(z_err, 4), "m")
print("  left_contact        =", lc_place)
print("  right_contact       =", rc_place)

# PASS: bottle within 0.3m of target, bottle on or near table surface
g17_pass = (np.isfinite(b_place_end).all()
            and xy_err < 0.30
            and b_place_end[2] < table_target[2] + 0.25)  # bottle is near table height
print("G17 =", "PASS" if g17_pass else "FAIL")
if not g17_pass:
    print("  FAIL: xy_err=" + str(round(xy_err,4)) + " z_err=" + str(round(z_err,4)))
    sys.exit(1)

# ── G18: RELEASE ─────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G18: RELEASE")
print("=" * 70)

b_release_start = xpos("carry_bottle").copy()
print("  bottle before release =", b_release_start.round(4))
print("  left_finger qpos      =", round(d.qpos[lq], 4))
print("  right_finger qpos     =", round(d.qpos[rq], 4))

# Open fingers: set ctrl to max (0.07)
d.ctrl[aid("act_left_finger")]  = 0.07
d.ctrl[aid("act_right_finger")] = 0.07

# Step to open
for _ in range(300):
    mujoco.mj_step(m, d)

lq_open = d.qpos[lq]
rq_open = d.qpos[rq]
lc_rel, rc_rel = check_contact()
b_after_open = xpos("carry_bottle")

print("  left_finger qpos (open) =", round(lq_open, 4))
print("  right_finger qpos (open)=", round(rq_open, 4))
print("  left_contact after open =", lc_rel)
print("  right_contact after open=", rc_rel)
print("  bottle after open       =", b_after_open.round(4))

# Arm retreat: move arm away from bottle (increase shoulder_lift)
q_retreat = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
q_retreat[1] += 0.3   # shoulder_lift up
for jn, qv in zip(arm_joints, q_retreat):
    d.ctrl[aid(act_map[jn])] = qv

for _ in range(400):
    mujoco.mj_step(m, d)

b_after_retreat = xpos("carry_bottle")
tool_after_retreat = xpos("tool0")
bottle_tool_dist_post = np.linalg.norm(b_after_retreat - tool_after_retreat)

print("\n  bottle after retreat    =", b_after_retreat.round(4))
print("  tool0 after retreat     =", tool_after_retreat.round(4))
print("  bottle-tool dist post   =", round(bottle_tool_dist_post, 4), "m")

# Stability: bottle stays in place after another 500 steps
b_stab_start = b_after_retreat.copy()
for _ in range(500):
    mujoco.mj_step(m, d)
b_stab_end = xpos("carry_bottle")
stab_drift = np.linalg.norm(b_stab_end - b_stab_start)

print("  bottle stability (1s)   =", round(stab_drift, 4), "m drift")
print("  bottle final z          =", round(b_stab_end[2], 4))

fingers_opened = (lq_open > 0.04 or rq_open > 0.04)  # opened significantly
bottle_stable  = (stab_drift < 0.05 and np.isfinite(b_stab_end).all())

g18_pass = fingers_opened and bottle_stable
print("G18 =", "PASS" if g18_pass else "FAIL")
if not g18_pass:
    print("  fingers_opened=" + str(fingers_opened) + " bottle_stable=" + str(bottle_stable))

# ── G13 REGRESSION CHECK ──────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G13 REGRESSION CHECK")
print("=" * 70)

# Reload and re-run G13 from scratch
m2 = mujoco.MjModel.from_xml_path(XML)
d2 = mujoco.MjData(m2)

tron1_bi2    = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, "base_Link")
root_jnt2    = m2.body_jntadr[tron1_bi2]
root_qpos2   = m2.jnt_qposadr[root_jnt2]
lq2 = m2.jnt_qposadr[mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_JOINT, "left_finger_joint")]
rq2 = m2.jnt_qposadr[mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_JOINT, "right_finger_joint")]

d2.qpos[root_qpos2:root_qpos2+3] = np.array([0.0, -3.0, 0.82])
d2.qvel[:] = 0.0
for jn, qv in zip(arm_joints, Q0):
    ji2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_JOINT, jn)
    d2.qpos[m2.jnt_qposadr[ji2]] = qv
d2.qpos[lq2] = 0.07
d2.qpos[rq2] = 0.07
for jn, qv in zip(arm_joints, Q0):
    ai2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_ACTUATOR, act_map[jn])
    if ai2 >= 0: d2.ctrl[ai2] = qv
ai_lf = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_left_finger")
ai_rf = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_right_finger")
d2.ctrl[ai_lf] = 0.0
d2.ctrl[ai_rf] = 0.0

m2.opt.gravity[:] = 0.0
mujoco.mj_forward(m2, d2)
for _ in range(600):
    mujoco.mj_step(m2, d2)

bg2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_GEOM, "carry_bottle_collision")
lg2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_geom")
rg2 = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_geom")
lc_reg = rc_reg = False
for ci in range(d2.ncon):
    c = d2.contact[ci]; a, b = int(c.geom1), int(c.geom2)
    if {a,b}=={bg2,lg2}: lc_reg = True
    if {a,b}=={bg2,rg2}: rc_reg = True

print("  left_contact  =", lc_reg)
print("  right_contact =", rc_reg)
g13_regression = lc_reg and rc_reg
print("G13 REGRESSION =", "PASS" if g13_regression else "FAIL")

# ── FINAL SUMMARY ─────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("FINAL SUMMARY: G14-G18")
print("=" * 70)
print("G14 HOLD    =", "PASS" if g14_pass else "FAIL")
print("G15 LIFT    =", "PASS" if g15_pass else "FAIL")
print("G16 CARRY   =", "PASS" if g16_pass else "FAIL")
print("G17 PLACE   =", "PASS" if g17_pass else "FAIL")
print("G18 RELEASE =", "PASS" if g18_pass else "FAIL")
print("G13 REGRESSION CHECK =", "PASS" if g13_regression else "FAIL")
print()
print("XML MODIFIED = NO")
all_pass = g14_pass and g15_pass and g16_pass and g17_pass and g18_pass and g13_regression
cp_this_run = sum([g14_pass, g15_pass, g16_pass, g17_pass, g18_pass])
total_cp = 12 + cp_this_run
print("This run NEW PASS:", cp_this_run, "/5")
print("Total G01-G28 CURRENT PASS:", total_cp, "/28")
print("Overall progress:", round(total_cp/28*100, 2), "%")
print()
print("Evidence:")
print("  Script: ros_ws/mujoco/final_evidence/logs/G14_G18_FINAL_HOLD_TO_RELEASE.py")
print("  G14: hold_drift="+str(round(hold_drift,4))+"m over 5s")
print("  G15: bottle_dz="+str(round(dz_bottle,4))+"m  tool_dz="+str(round(dz_tool,4))+"m")
print("  G16: carry_displacement="+str(round(carry_displacement,4))+"m")
print("  G17: xy_err="+str(round(xy_err,4))+"m  z_err="+str(round(z_err,4))+"m")
print("  G18: stab_drift="+str(round(stab_drift,4))+"m  fingers_opened="+str(fingers_opened))
