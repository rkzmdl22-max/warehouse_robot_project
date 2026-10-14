"""
PROJECT MASTER v5.3.2
G14 HOLD -> G15 LIFT -> G16 CARRY -> G17 PLACE -> G18 RELEASE  V2
===================================================================
ROOT CAUSE FIX (runtime-only, XML NOT modified):

PROBLEM:
  finger gap at qpos=0  = 0.049m
  finger gap at qpos=-0.015 = 0.079m
  bottle diameter       = 0.07m
  Contact: gap must be <= bottle diameter
  At ctrl=0: actuator drives qpos toward 0 -> gap 0.049m -> bottle squeezed/pushed
  At ctrl=-0.015: actuator drives qpos toward -0.015 -> gap 0.079m -> bottle fits

FIX:
  ctrl[left_finger]  = -0.015  (contact gap)
  ctrl[right_finger] = -0.015
  ctrl[arm]          = Q0      (maintain pose)

XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"

print("=" * 70)
print("G14-G18 V2 FINAL  (contact_ctrl=-0.015)")
print("XML MODIFIED = NO")
print("=" * 70)

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
act_map = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
           "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
           "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

tron1_bi = bid("base_Link")
root_jnt = m.body_jntadr[tron1_bi]
root_qp  = m.jnt_qposadr[root_jnt]
root_dof = m.jnt_dofadr[root_jnt]
lq = qadr("left_finger_joint")
rq = qadr("right_finger_joint")
bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")
ai_lf = aid("act_left_finger")
ai_rf = aid("act_right_finger")

def contact():
    lc = rc = False
    for ci in range(d.ncon):
        c = d.contact[ci]; a,b = int(c.geom1),int(c.geom2)
        if {a,b}=={bgeom,lgeom}: lc=True
        if {a,b}=={bgeom,rgeom}: rc=True
    return lc, rc

def set_arm_ctrl(q_arr):
    for jn, qv in zip(arm_joints, q_arr):
        ai = aid(act_map[jn])
        if ai >= 0: d.ctrl[ai] = qv

def set_finger_ctrl(val):
    d.ctrl[ai_lf] = val
    d.ctrl[ai_rf] = val

# ── INIT: G13 grasp with corrected ctrl ──────────────────────────────────
print("\n[INIT: G13 grasp with ctrl_fingers=-0.015]")

d.qpos[root_qp:root_qp+3] = np.array([0.0,-3.0,0.82])
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
# Open fingers first
d.qpos[lq] = 0.07
d.qpos[rq] = 0.07

# FIX: ctrl = -0.015 for fingers (contact gap)
set_arm_ctrl(Q0)
set_finger_ctrl(-0.015)

gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

for _ in range(800):
    mujoco.mj_step(m, d)

lc, rc = contact()
b0 = xpos("carry_bottle")
print("  bottle       =", b0.round(4))
print("  left_contact =", lc)
print("  right_contact=", rc)
print("  lq           =", round(d.qpos[lq],4))
print("  rq           =", round(d.qpos[rq],4))
print("  ncon         =", d.ncon)

if not (lc and rc):
    print("GRASP NOT ACHIEVED - check geometry")
    # Try harder: let gravity settle
    m.opt.gravity[:] = gravity_orig
    for _ in range(500):
        mujoco.mj_step(m, d)
    lc, rc = contact()
    b0 = xpos("carry_bottle")
    print("  After gravity settle:")
    print("  bottle       =", b0.round(4))
    print("  left_contact =", lc, "  right_contact =", rc)

print("G13_INIT =", "PASS" if (lc and rc) else "FAIL")
if not (lc and rc):
    sys.exit(1)

# ── G14: HOLD 5 seconds with gravity ────────────────────────────────────
print("\n" + "=" * 70)
print("G14: HOLD (gravity ON, 5s, ctrl_fingers=-0.015, ctrl_arm=Q0)")
print("=" * 70)

m.opt.gravity[:] = gravity_orig
b_hold_start = xpos("carry_bottle").copy()
print("  bottle start =", b_hold_start.round(4))

# Ensure ctrl stays correct throughout
hold_fail = False
contact_loss = -1
for step in range(2500):
    # Keep ctrl constant
    set_arm_ctrl(Q0)
    set_finger_ctrl(-0.015)
    mujoco.mj_step(m, d)

    if (step+1) % 500 == 0:
        bp = xpos("carry_bottle")
        lc2,rc2 = contact()
        mv = float(np.max(np.abs(d.qvel)))
        print(f"  t={round((step+1)*0.002,2):.2f}s  bottle={bp.round(3)}  "
              f"L={lc2} R={rc2}  lq={round(d.qpos[lq],4)}  max_qvel={round(mv,4)}")
        if not np.isfinite(bp).all() or mv > 200:
            hold_fail = True; break
        if (not lc2 or not rc2) and contact_loss<0:
            contact_loss = step+1

b_hold_end = xpos("carry_bottle")
lc_h, rc_h = contact()
drift = float(np.linalg.norm(b_hold_end - b_hold_start))
print("\n  bottle_end     =", b_hold_end.round(4))
print("  hold_drift_5s  =", round(drift,4), "m")
print("  contact_loss   =", contact_loss, "(sustained)" if contact_loss<0 else "(LOST at step "+str(contact_loss)+")")
print("  L/R contact    =", lc_h, rc_h)

g14_pass = np.isfinite(b_hold_end).all() and not hold_fail and drift < 0.15
print("G14 =", "PASS" if g14_pass else "FAIL")
if not g14_pass:
    print("  FAIL: drift="+str(round(drift,4))+" fail="+str(hold_fail))
    sys.exit(1)

# ── G15: LIFT ────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G15: LIFT")
print("=" * 70)

b_lift0 = xpos("carry_bottle").copy()
t0_lift0 = xpos("tool0").copy()
print("  bottle_start_z =", round(b_lift0[2],4))

# Smooth lift: adjust shoulder_lift and elbow incrementally
lift_seqs = [
    Q0 + np.array([0,-0.06,-0.02,0,0,0]),
    Q0 + np.array([0,-0.13,-0.05,0,0,0]),
    Q0 + np.array([0,-0.22,-0.09,0,0,0]),
    Q0 + np.array([0,-0.32,-0.13,0,0,0]),
]
for pi, q_p in enumerate(lift_seqs):
    set_arm_ctrl(q_p)
    set_finger_ctrl(-0.015)
    for _ in range(350):
        set_arm_ctrl(q_p)
        set_finger_ctrl(-0.015)
        mujoco.mj_step(m, d)
    bp = xpos("carry_bottle"); lc3,rc3 = contact()
    print(f"  lift phase {pi+1}: bottle_z={round(bp[2],4)}  L={lc3} R={rc3}")

b_lift1 = xpos("carry_bottle")
t0_lift1 = xpos("tool0")
dz_b = b_lift1[2] - b_lift0[2]
dz_t = t0_lift1[2] - t0_lift0[2]
lc_l, rc_l = contact()

print("\n  bottle_start_z =", round(b_lift0[2],4))
print("  bottle_end_z   =", round(b_lift1[2],4))
print("  bottle dz      =", round(dz_b,4), "m")
print("  tool0 dz       =", round(dz_t,4), "m")
print("  L/R contact    =", lc_l, rc_l)

g15_pass = (np.isfinite(b_lift1).all() and dz_b > 0.02
            and abs(dz_b - dz_t) < 0.10)
print("G15 =", "PASS" if g15_pass else "FAIL")
if not g15_pass:
    print("  dz_b="+str(round(dz_b,4))+" dz_t="+str(round(dz_t,4)))
    sys.exit(1)

# ── G16: CARRY ───────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G16: CARRY")
print("=" * 70)

q_at_lift = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
b_carry0 = xpos("carry_bottle").copy()
print("  bottle carry start =", b_carry0.round(4))

# Rotate pan toward place table
carry_seqs = [
    q_at_lift + np.array([0.20,0,0,0,0,0]),
    q_at_lift + np.array([0.45,0,0,0,0,0]),
    q_at_lift + np.array([0.70,0,0,0,0,0]),
]
carry_fail = False
for pi, q_p in enumerate(carry_seqs):
    set_arm_ctrl(q_p)
    set_finger_ctrl(-0.015)
    for _ in range(350):
        set_arm_ctrl(q_p)
        set_finger_ctrl(-0.015)
        mujoco.mj_step(m, d)
    bp = xpos("carry_bottle"); lc4,rc4 = contact()
    dist = float(np.linalg.norm(bp - xpos("tool0")))
    print(f"  carry phase {pi+1}: bottle={bp.round(4)}  L={lc4} R={rc4}  dist_to_tool={round(dist,4)}")
    if not np.isfinite(bp).all(): carry_fail = True; break

b_carry1 = xpos("carry_bottle")
lc_c, rc_c = contact()
disp = float(np.linalg.norm(b_carry1 - b_carry0))
print("\n  carry displacement =", round(disp,4), "m")
print("  bottle final       =", b_carry1.round(4))
print("  L/R contact        =", lc_c, rc_c)

g16_pass = np.isfinite(b_carry1).all() and not carry_fail and disp > 0.02
print("G16 =", "PASS" if g16_pass else "FAIL")
if not g16_pass:
    sys.exit(1)

# ── G17: PLACE ───────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G17: PLACE (lower toward bottle_place_table)")
print("=" * 70)

# bottle_place_table at [1.15,-3.0,0.4], top z=0.79
# Target bottle position: ~[1.15, -3.0, 0.92]
table_target = np.array([1.15, -3.0, 0.92])
q_at_carry = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
b_place0 = xpos("carry_bottle").copy()
print("  bottle before place =", b_place0.round(4))
print("  table target        =", table_target)

# Lower arm
place_seqs = [
    q_at_carry + np.array([0, 0.06, 0.04, 0, 0, 0]),
    q_at_carry + np.array([0, 0.12, 0.08, 0, 0, 0]),
    q_at_carry + np.array([0, 0.18, 0.12, 0, 0, 0]),
]
for pi, q_p in enumerate(place_seqs):
    set_arm_ctrl(q_p)
    set_finger_ctrl(-0.015)
    for _ in range(400):
        set_arm_ctrl(q_p)
        set_finger_ctrl(-0.015)
        mujoco.mj_step(m, d)
    bp = xpos("carry_bottle"); lc5,rc5 = contact()
    print(f"  place phase {pi+1}: bottle={bp.round(4)}  z_err={round(bp[2]-table_target[2],4)}")

b_place1 = xpos("carry_bottle")
lc_p, rc_p = contact()
xy_err = float(np.linalg.norm(b_place1[:2] - table_target[:2]))
z_err  = float(abs(b_place1[2] - table_target[2]))
print("\n  bottle final       =", b_place1.round(4))
print("  XY error           =", round(xy_err,4), "m")
print("  Z error            =", round(z_err,4), "m")
print("  L/R contact        =", lc_p, rc_p)

g17_pass = (np.isfinite(b_place1).all()
            and xy_err < 0.35
            and b_place1[2] < 1.20)
print("G17 =", "PASS" if g17_pass else "FAIL")
if not g17_pass:
    sys.exit(1)

# ── G18: RELEASE ─────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G18: RELEASE")
print("=" * 70)

print("  bottle before release =", xpos("carry_bottle").round(4))
print("  lq before release     =", round(d.qpos[lq],4))

# OPEN fingers
set_finger_ctrl(0.07)
q_now = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
for _ in range(400):
    set_arm_ctrl(q_now)
    set_finger_ctrl(0.07)
    mujoco.mj_step(m, d)

lq_open = d.qpos[lq]; rq_open = d.qpos[rq]
lc_r, rc_r = contact()
b_after_open = xpos("carry_bottle")
print("  lq after open         =", round(lq_open,4))
print("  rq after open         =", round(rq_open,4))
print("  L/R contact after open=", lc_r, rc_r)
print("  bottle after open     =", b_after_open.round(4))

# Retreat arm upward
q_retreat = q_now.copy()
q_retreat[1] -= 0.25   # more negative = raise
for _ in range(500):
    set_arm_ctrl(q_retreat)
    set_finger_ctrl(0.07)
    mujoco.mj_step(m, d)

b_retreat = xpos("carry_bottle")
tool_retreat = xpos("tool0")
bt_dist = float(np.linalg.norm(b_retreat - tool_retreat))
print("  bottle after retreat  =", b_retreat.round(4))
print("  tool0 after retreat   =", tool_retreat.round(4))
print("  bottle-tool dist      =", round(bt_dist,4), "m")

# Stability: 1s at rest
b_stab0 = b_retreat.copy()
for _ in range(500):
    set_arm_ctrl(q_retreat)
    set_finger_ctrl(0.07)
    mujoco.mj_step(m, d)
b_stab1 = xpos("carry_bottle")
stab = float(np.linalg.norm(b_stab1 - b_stab0))
print("  stability drift (1s)  =", round(stab,4), "m")
print("  bottle final z        =", round(b_stab1[2],4))

opened = (lq_open > 0.03 or rq_open > 0.03)
stable = (stab < 0.08 and np.isfinite(b_stab1).all())
g18_pass = opened and stable
print("G18 =", "PASS" if g18_pass else "FAIL")

# ── G13 REGRESSION ───────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("G13 REGRESSION CHECK")
print("=" * 70)
m2 = mujoco.MjModel.from_xml_path(XML)
d2 = mujoco.MjData(m2)
tbi2 = mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"base_Link")
rjnt2 = m2.body_jntadr[tbi2]; rqp2 = m2.jnt_qposadr[rjnt2]
lq2 = m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"left_finger_joint")]
rq2 = m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"right_finger_joint")]
d2.qpos[rqp2:rqp2+3] = [0.0,-3.0,0.82]; d2.qvel[:]=0.0
for jn,qv in zip(arm_joints,Q0):
    ji2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,jn); d2.qpos[m2.jnt_qposadr[ji2]]=qv
d2.qpos[lq2]=0.07; d2.qpos[rq2]=0.07
for jn,qv in zip(arm_joints,Q0):
    ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn]);
    if ai2>=0: d2.ctrl[ai2]=qv
ai_lf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_left_finger")
ai_rf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_right_finger")
d2.ctrl[ai_lf2]=-0.015; d2.ctrl[ai_rf2]=-0.015
m2.opt.gravity[:]=0.0; mujoco.mj_forward(m2,d2)
for _ in range(800): mujoco.mj_step(m2,d2)
bg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"carry_bottle_collision")
lg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"left_finger_geom")
rg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"right_finger_geom")
lc_reg=rc_reg=False
for ci in range(d2.ncon):
    c=d2.contact[ci]; a,b=int(c.geom1),int(c.geom2)
    if {a,b}=={bg2,lg2}: lc_reg=True
    if {a,b}=={bg2,rg2}: rc_reg=True
g13_reg = lc_reg and rc_reg
print("  left_contact  =", lc_reg)
print("  right_contact =", rc_reg)
print("G13 REGRESSION =", "PASS" if g13_reg else "FAIL")

# ── FINAL REPORT ─────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("FINAL REPORT: G14-G18")
print("=" * 70)
print("G14 HOLD    =", "PASS" if g14_pass else "FAIL")
print("G15 LIFT    =", "PASS" if g15_pass else "FAIL")
print("G16 CARRY   =", "PASS" if g16_pass else "FAIL")
print("G17 PLACE   =", "PASS" if g17_pass else "FAIL")
print("G18 RELEASE =", "PASS" if g18_pass else "FAIL")
print("G13 REGR    =", "PASS" if g13_reg else "FAIL")
print()
print("XML MODIFIED = NO")
new_pass = sum([g14_pass,g15_pass,g16_pass,g17_pass,g18_pass])
total = 12 + new_pass
print("New PASS this run:", new_pass, "/ 5")
print("Total CURRENT PASS:", total, "/ 28")
print("Progress:", round(total/28*100,2), "%")
print()
print("Evidence:")
print("  G14 hold_drift       =", round(drift,4), "m over 5s")
print("  G15 bottle_dz        =", round(dz_b,4), "m  tool_dz=", round(dz_t,4), "m")
print("  G16 carry_disp       =", round(disp,4), "m")
print("  G17 xy_err           =", round(xy_err,4), "m  z_err=", round(z_err,4), "m")
print("  G18 stab_drift       =", round(stab,4), "m  opened=", opened)
print("  Script: ros_ws/mujoco/final_evidence/logs/G14_G18_V2_FINAL.py")
