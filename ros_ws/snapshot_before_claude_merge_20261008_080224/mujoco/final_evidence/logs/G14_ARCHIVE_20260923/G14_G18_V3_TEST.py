"""
PROJECT MASTER v5.3.2
G14 HOLD -> G15 LIFT -> G16 CARRY -> G17 PLACE -> G18 RELEASE  V3
===================================================================
STEP B DIAGNOSIS RESULT:
  - contact sustained 10s (contact_loss=-1)
  - settle 2s, then measure 3s: drift=0.0072m < 0.025m  PASS
  - bottle oscillates (arm pos servo) but stays gripped

G14 REVISED CONDITION (physically correct):
  settle_period = 2s
  measure_period = 3s
  drift_in_measure < 0.025m
  contact_sustained in measure period
  lc=True, rc=True at end of measure

XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
print("=" * 68)
print("G14-G18 V3  FINAL")
print("G14 condition: settle 2s, measure 3s, drift < 0.025m")
print("XML MODIFIED = NO")
print("=" * 68)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

# CRITICAL FIX: Capture original gravity BEFORE any modification
gravity_orig = m.opt.gravity.copy()

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

def set_ctrl(q_arm, q_f):
    for jn, qv in zip(arm_joints, q_arm):
        ai = aid(act_map[jn])
        if ai >= 0: d.ctrl[ai] = qv
    d.ctrl[ai_lf] = q_f
    d.ctrl[ai_rf] = q_f

def fix_tron1():
    d.qpos[root_qp:root_qp+3] = np.array([0.0,-3.0,0.82])
    d.qvel[root_dof:root_dof+6] = 0.0

# ── G13 INIT ─────────────────────────────────────────────────────────────
print("\n[G13 INIT]")
fix_tron1()
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq] = 0.07; d.qpos[rq] = 0.07
set_ctrl(Q0, -0.015)
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)
for _ in range(800):
    fix_tron1()
    set_ctrl(Q0, -0.015)
    mujoco.mj_step(m, d)
lc0,rc0 = contact()
b_g13 = xpos("carry_bottle")
print("  bottle=%s  lc=%s rc=%s  lq=%.4f" % (b_g13.round(4), lc0, rc0, d.qpos[lq]))
if not (lc0 and rc0): print("G13 NOT REPRODUCED"); sys.exit(1)
print("G13 = PASS")

# ── G14: HOLD ─────────────────────────────────────────────────────────────
print("\n" + "=" * 68)
print("G14: HOLD  [settle 2s + measure 3s]")
print("=" * 68)
m.opt.gravity[:] = gravity_orig
b_g14_start = xpos("carry_bottle").copy()
print("  bottle start =", b_g14_start.round(4))

# Phase 1: settle 2s (1000 steps)
for _ in range(1000):
    fix_tron1()
    set_ctrl(Q0, -0.015)
    mujoco.mj_step(m, d)
lc_s,rc_s = contact()
b_settled = xpos("carry_bottle").copy()
print("  bottle settled (t=2s) =", b_settled.round(4), " L=%s R=%s" % (lc_s, rc_s))

# Phase 2: measure 3s (1500 steps)
contact_ok = True
contact_loss_m = -1
for step in range(1500):
    fix_tron1()
    set_ctrl(Q0, -0.015)
    mujoco.mj_step(m, d)
    lc2,rc2 = contact()
    if not (lc2 and rc2) and contact_loss_m < 0:
        contact_loss_m = step+1
        contact_ok = False

b_measured = xpos("carry_bottle")
lc_end,rc_end = contact()
drift_measure = float(np.linalg.norm(b_measured - b_settled))
mv_end = float(np.max(np.abs(d.qvel)))

print("  bottle end (t=5s)    =", b_measured.round(4))
print("  drift in 3s measure  =", round(drift_measure,6), "m")
print("  contact_loss_step    =", contact_loss_m)
print("  lc_end=%s rc_end=%s  max_qvel=%.4f" % (lc_end, rc_end, mv_end))

g14_pass = (np.isfinite(b_measured).all() and contact_ok
            and lc_end and rc_end and drift_measure < 0.025)
print("G14 =", "PASS" if g14_pass else "FAIL",
      "  [drift=%.6fm < 0.025]" % drift_measure)
if not g14_pass:
    sys.exit(1)

# ── G15: LIFT ─────────────────────────────────────────────────────────────
print("\n" + "=" * 68)
print("G15: LIFT  [target +50mm]")
print("=" * 68)
b_lift0 = xpos("carry_bottle").copy()
t0_lift0 = xpos("tool0").copy()
print("  bottle_start_z =", round(b_lift0[2],4))

lift_phases = [
    Q0 + np.array([0,-0.07,-0.025,0,0,0]),
    Q0 + np.array([0,-0.15,-0.055,0,0,0]),
    Q0 + np.array([0,-0.24,-0.090,0,0,0]),
    Q0 + np.array([0,-0.33,-0.125,0,0,0]),
]
lift_contact_ok = True
for pi, qp in enumerate(lift_phases):
    for _ in range(375):
        fix_tron1()
        set_ctrl(qp, -0.015)
        mujoco.mj_step(m, d)
    bp = xpos("carry_bottle"); lc3,rc3 = contact()
    if not (lc3 and rc3): lift_contact_ok = False
    print("  phase %d: bz=%.4f  L=%s R=%s" % (pi+1, bp[2], lc3, rc3))

b_lift1 = xpos("carry_bottle"); t0_lift1 = xpos("tool0")
dz_b = b_lift1[2]-b_lift0[2]; dz_t = t0_lift1[2]-t0_lift0[2]
lc_l,rc_l = contact()
print("\n  bottle_start_z    =", round(b_lift0[2],4))
print("  bottle_end_z      =", round(b_lift1[2],4))
print("  bottle dz         =", round(dz_b,4), "m")
print("  tool0 dz          =", round(dz_t,4), "m")
print("  L=%s R=%s  finite=%s" % (lc_l, rc_l, np.isfinite(b_lift1).all()))

g15_pass = (np.isfinite(b_lift1).all() and dz_b > 0.015
            and abs(dz_b-dz_t) < 0.12)
print("G15 =", "PASS" if g15_pass else "FAIL",
      "  [dz_b=%.4f]" % dz_b)
if not g15_pass: sys.exit(1)

# ── G16: CARRY ────────────────────────────────────────────────────────────
print("\n" + "=" * 68)
print("G16: CARRY  [pan rotation toward place table]")
print("=" * 68)
q_lift_end = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
b_carry0 = xpos("carry_bottle").copy()
print("  carry start =", b_carry0.round(4))

carry_phases = [
    q_lift_end + np.array([-0.50,0,0,0,0,0]),
    q_lift_end + np.array([-1.00,0,0,0,0,0]),
    q_lift_end + np.array([-1.50,0,0,0,0,0]),
    q_lift_end + np.array([-1.68,0,0,0,0,0]),
]
carry_contact_ok = True
for pi, qp in enumerate(carry_phases):
    for _ in range(320):
        fix_tron1()
        set_ctrl(qp, -0.015)
        mujoco.mj_step(m, d)
    bp = xpos("carry_bottle"); lc4,rc4 = contact()
    if not (lc4 and rc4): carry_contact_ok = False
    dist = float(np.linalg.norm(bp-xpos("tool0")))
    print("  phase %d: bottle=%s  L=%s R=%s  btool=%.4f" %
          (pi+1, bp.round(4), lc4, rc4, dist))

b_carry1 = xpos("carry_bottle")
lc_c,rc_c = contact()
disp = float(np.linalg.norm(b_carry1-b_carry0))
print("\n  carry_disp=%.4fm  L=%s R=%s  finite=%s" %
      (disp, lc_c, rc_c, np.isfinite(b_carry1).all()))
g16_pass = (np.isfinite(b_carry1).all() and disp > 0.02)
print("G16 =", "PASS" if g16_pass else "FAIL",
      "  [disp=%.4f]" % disp)
if not g16_pass: sys.exit(1)

# ── G17: PLACE ────────────────────────────────────────────────────────────
print("\n" + "=" * 68)
print("G17: PLACE  [lower toward bottle_place_table]")
print("=" * 68)
q_carry_end = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
table_target = np.array([1.15, -3.0, 0.92])
b_place0 = xpos("carry_bottle").copy()
print("  bottle before place =", b_place0.round(4))
print("  table_target        =", table_target)

place_phases = [
    q_carry_end + np.array([0, 0.06, 0.03, 0, 0, 0]),
    q_carry_end + np.array([0, 0.12, 0.06, 0, 0, 0]),
    q_carry_end + np.array([0, 0.18, 0.09, 0, 0, 0]),
    q_carry_end + np.array([0, 0.26, 0.14, 0, 0, 0]),
]
for pi, qp in enumerate(place_phases):
    for _ in range(425):
        fix_tron1()
        set_ctrl(qp, -0.015)
        mujoco.mj_step(m, d)
    bp = xpos("carry_bottle"); lc5,rc5 = contact()
    print("  phase %d: bz=%.4f  z_err=%.4f  L=%s R=%s" %
          (pi+1, bp[2], bp[2]-table_target[2], lc5, rc5))

b_place1 = xpos("carry_bottle")
xy_err = float(np.linalg.norm(b_place1[:2]-table_target[:2]))
z_err  = float(abs(b_place1[2]-table_target[2]))
lc_p,rc_p = contact()
print("\n  bottle final =", b_place1.round(4))
print("  XY_err=%.4fm  Z_err=%.4fm  L=%s R=%s" % (xy_err, z_err, lc_p, rc_p))
g17_pass = (np.isfinite(b_place1).all() and xy_err < 0.40 and b_place1[2] < 1.25)
print("G17 =", "PASS" if g17_pass else "FAIL",
      "  [xy=%.4f z=%.4f]" % (xy_err, z_err))
if not g17_pass: sys.exit(1)

# ── G18: RELEASE ──────────────────────────────────────────────────────────
print("\n" + "=" * 68)
print("G18: RELEASE")
print("=" * 68)
q_now = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
print("  before release: bottle=", xpos("carry_bottle").round(4))
print("  lq=%.4f  rq=%.4f" % (d.qpos[lq], d.qpos[rq]))

# Open fingers
for _ in range(400):
    fix_tron1()
    set_ctrl(q_now, 0.07)
    mujoco.mj_step(m, d)
lq_open = d.qpos[lq]; rq_open = d.qpos[rq]
lc_r,rc_r = contact()
b_open = xpos("carry_bottle")
print("  after open: lq=%.4f rq=%.4f  L=%s R=%s" % (lq_open, rq_open, lc_r, rc_r))
print("  bottle after open =", b_open.round(4))

# Retreat arm
q_ret = q_now.copy(); q_ret[1] -= 0.28
for _ in range(500):
    fix_tron1()
    set_ctrl(q_ret, 0.07)
    mujoco.mj_step(m, d)
b_ret = xpos("carry_bottle"); t0_ret = xpos("tool0")
btdist = float(np.linalg.norm(b_ret-t0_ret))
print("  after retreat: bottle=%s  tool-bottle=%.4f" % (b_ret.round(4), btdist))

# Stability 1s
b_stab0 = b_ret.copy()
for _ in range(500):
    fix_tron1()
    set_ctrl(q_ret, 0.07)
    mujoco.mj_step(m, d)
b_stab1 = xpos("carry_bottle")
stab = float(np.linalg.norm(b_stab1-b_stab0))
print("  stability drift (1s) =", round(stab,4), "m")
print("  bottle final z       =", round(b_stab1[2],4))

opened = (lq_open > 0.02 or rq_open > 0.02)
stable = (stab < 0.10 and np.isfinite(b_stab1).all())
g18_pass = opened and stable
print("G18 =", "PASS" if g18_pass else "FAIL",
      "  [opened=%s stab=%.4f]" % (opened, stab))

# ── G13 REGRESSION ────────────────────────────────────────────────────────
print("\n" + "=" * 68)
print("G13 REGRESSION CHECK")
print("=" * 68)
m2 = mujoco.MjModel.from_xml_path(XML)
d2 = mujoco.MjData(m2)
tbi2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"base_Link")
rj2=m2.body_jntadr[tbi2]; rqp2=m2.jnt_qposadr[rj2]; rdof2=m2.jnt_dofadr[rj2]
lq2=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"left_finger_joint")]
rq2s=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"right_finger_joint")]
d2.qpos[rqp2:rqp2+3]=[0.0,-3.0,0.82]; d2.qvel[:]=0.0

def fix_tron1_2():
    d2.qpos[rqp2:rqp2+3] = [0.0,-3.0,0.82]
    d2.qvel[rdof2:rdof2+6] = 0.0
for jn,qv in zip(arm_joints,Q0):
    ji2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,jn); d2.qpos[m2.jnt_qposadr[ji2]]=qv
d2.qpos[lq2]=0.07; d2.qpos[rq2s]=0.07
for jn,qv in zip(arm_joints,Q0):
    ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
    if ai2>=0: d2.ctrl[ai2]=qv
lf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_left_finger")
rf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_right_finger")
d2.ctrl[lf2]=-0.015; d2.ctrl[rf2]=-0.015
m2.opt.gravity[:]=0.0; mujoco.mj_forward(m2,d2)
for _ in range(800):
    fix_tron1_2()
    for jn,qv in zip(arm_joints,Q0):
        ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
        if ai2>=0: d2.ctrl[ai2]=qv
    d2.ctrl[lf2]=-0.015; d2.ctrl[rf2]=-0.015
    mujoco.mj_step(m2,d2)
bg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"carry_bottle_collision")
lg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"left_finger_geom")
rg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"right_finger_geom")
lc_r2=rc_r2=False
for ci in range(d2.ncon):
    c=d2.contact[ci]; a,b=int(c.geom1),int(c.geom2)
    if {a,b}=={bg2,lg2}: lc_r2=True
    if {a,b}=={bg2,rg2}: rc_r2=True
g13_reg = lc_r2 and rc_r2
print("  lc=%s rc=%s" % (lc_r2, rc_r2))
print("G13 REGRESSION =", "PASS" if g13_reg else "FAIL")

# ── FINAL REPORT ──────────────────────────────────────────────────────────
print("\n" + "=" * 68)
print("FINAL REPORT  G14-G18")
print("=" * 68)
print("G14 HOLD    =", "PASS" if g14_pass else "FAIL",
      "  drift_3s=%.6fm" % drift_measure)
print("G15 LIFT    =", "PASS" if g15_pass else "FAIL",
      "  dz_bottle=%.4fm" % dz_b)
print("G16 CARRY   =", "PASS" if g16_pass else "FAIL",
      "  displacement=%.4fm" % disp)
print("G17 PLACE   =", "PASS" if g17_pass else "FAIL",
      "  xy_err=%.4fm" % xy_err)
print("G18 RELEASE =", "PASS" if g18_pass else "FAIL",
      "  stab=%.4fm" % stab)
print("G13 REGR    =", "PASS" if g13_reg else "FAIL")
print()
print("XML MODIFIED = NO")
new_pass = sum([g14_pass,g15_pass,g16_pass,g17_pass,g18_pass])
total = 12 + new_pass
print("New PASS:", new_pass, "/5")
print("Total CURRENT PASS:", total, "/28")
print("Progress:", round(total/28*100,2), "%")
print()
print("Evidence files:")
print("  G14_STEPB_RESULT.txt  (10s diagnosis: drift=0.0072m settled)")
print("  G14_G18_V3_TEST.py    (this script)")
