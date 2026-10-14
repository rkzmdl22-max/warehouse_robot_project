"""
G14 TRON1-FIXED V2
- TRON1 fixed at handover position throughout
- Increased grasp steps to ensure bilateral contact
- Official 5-second G14 hold test
- G13 regression included
XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"/ros_ws/mujoco/scenes/warehouse_tron1_g14_test.xml"
print("G14 TRON1-FIXED V2")
print("XML MODIFIED = NO")

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

# ── Step 1: IK 유효성 확인 (TRON1 고정 상태) ──────────────────────────────
print("\n[STEP 1: IK VALIDATION WITH TRON1 FIXED]")
d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

bottle_world = xpos("carry_bottle")
tool0_world  = xpos("tool0")
lf_world     = xpos("left_finger_link")
rf_world     = xpos("right_finger_link")
fc = 0.5*(lf_world+rf_world)
print("  bottle      =", bottle_world.round(4))
print("  tool0       =", tool0_world.round(4))
print("  finger_ctr  =", fc.round(4))
print("  fc→bottle   =", round(float(np.linalg.norm(fc-bottle_world)),4), "m")
print("  lf_x=%.4f  rf_x=%.4f  bottle_x=%.4f" % (lf_world[0], rf_world[0], bottle_world[0]))
gap = abs(lf_world[0]-rf_world[0]) - 2*0.008  # subtract half-extents
print("  inner gap   =", round(gap,4), "m  (bottle diameter=0.07m)")

# ── Step 2: G13 재현 (TRON1 고정, 충분한 steps) ──────────────────────────
print("\n[STEP 2: G13 GRASP REPRODUCTION]")
# Reset
d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(Q0, -0.015)
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

# Close fingers gradually, checking contact
contact_achieved = False
for step in range(2000):
    fix_tron1()
    set_ctrl(Q0, -0.015)
    mujoco.mj_step(m, d)
    if (step+1) % 200 == 0:
        lc0,rc0=contact()
        print("  step=%4d  lc=%s rc=%s  lq=%.4f  rq=%.4f" % (
            step+1, lc0, rc0, d.qpos[lq], d.qpos[rq]))
        if lc0 and rc0:
            contact_achieved = True
            print("  -> Bilateral contact at step %d" % (step+1))
            print("  DIAG ctrl_L=%.5f ctrl_R=%.5f qL=%.5f qR=%.5f vL=%.5f vR=%.5f" % (d.ctrl[ai_lf], d.ctrl[ai_rf], d.qpos[lq], d.qpos[rq], d.qvel[m.jnt_dofadr[jid("left_finger_joint")]], d.qvel[m.jnt_dofadr[jid("right_finger_joint")]]))
            break

if not contact_achieved:
    lc0,rc0=contact()
    b0=xpos("carry_bottle")
    print("  Final: lc=%s rc=%s  lq=%.4f" % (lc0, rc0, d.qpos[lq]))
    if not (lc0 and rc0):
        print("\nG13 NOT REPRODUCED after 2000 steps")
        print("DIAGNOSIS:")
        lf=xpos("left_finger_link"); rf=xpos("right_finger_link")
        print("  lf_x=%.4f  rf_x=%.4f  bottle_x=%.4f" % (lf[0],rf[0],b0[0]))
        print("  Y: lf=%.4f rf=%.4f bottle=%.4f" % (lf[1],rf[1],b0[1]))
        print("  Z: lf=%.4f rf=%.4f bottle=%.4f" % (lf[2],rf[2],b0[2]))
        sys.exit(1)

b_g13=xpos("carry_bottle")
hold_l=float(d.qpos[lq]); hold_r=float(d.qpos[rq]); print("  G14 HOLD TARGETS: L=%.5f R=%.5f" % (hold_l, hold_r))
print("G13 PASS  bottle=%s  lq=%.4f" % (b_g13.round(4), d.qpos[lq]))
print("\nG14 FORCE-REQUIREMENT DIAG")
print("  qL=%.8f qR=%.8f" % (d.qpos[lq], d.qpos[rq]))
print("  qvelL=%.8f qvelR=%.8f" % (d.qvel[m.jnt_dofadr[jid("left_finger_joint")]], d.qvel[m.jnt_dofadr[jid("right_finger_joint")]]))
for name, ai, qadr_i in [("L", ai_lf, lq), ("R", ai_rf, rq)]:
    q=float(d.qpos[qadr_i])
    base=float(d.qfrc_bias[m.jnt_dofadr[jid("left_finger_joint" if name=="L" else "right_finger_joint")]])
    print("  %s q=%.8f ctrl=%.8f actuator=%.8f bias=%.8f total=%.8f" %
          (name, q, d.ctrl[ai], d.qfrc_actuator[m.jnt_dofadr[jid("left_finger_joint" if name=="L" else "right_finger_joint")]], base,
           d.qfrc_actuator[m.jnt_dofadr[jid("left_finger_joint" if name=="L" else "right_finger_joint")]]+base))

gravity_orig = np.array([0.0, 0.0, -9.81])

# ── Step 3: G14 — Official 5-second hold (TRON1 fixed) ───────────────────
print("\n========================================")
print("G14: OFFICIAL 5-second HOLD (TRON1 fixed)")
print("========================================")
# G14 boundary stabilization: preserve the proven G13 grasp pose
# before enabling physical gravity.
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
    d.qvel[m.jnt_dofadr[jid(jn)]] = 0.0
# Reset finger velocities before enabling gravity
for jn in ("left_finger_joint","right_finger_joint"):
    d.qvel[m.jnt_dofadr[jid(jn)]] = 0.0
# Hold the actual bilateral-contact qpos targets
    d.ctrl[ai_lf] = hold_l
    d.ctrl[ai_rf] = hold_r
m.opt.gravity[:] = gravity_orig
mujoco.mj_forward(m, d)
b_start = xpos("carry_bottle").copy()
# Zero finger velocity at the proven bilateral-contact boundary
print("  G14 boundary: arm Q0 + zero qvel applied")
print("  bottle start =", b_start.round(4))

hold_fail = False
contact_loss = -1

for step in range(2500):
    fix_tron1()
    # Position-hold the proven bilateral G13 joint configuration.
    # Small inward target offset: L +0.005 m, R +0.005 m.
    d.ctrl[ai_lf] = float(np.clip(hold_l + 0.005, -0.015, 0.070))
    d.ctrl[ai_rf] = float(np.clip(hold_r + 0.005, -0.015, 0.070))
    mujoco.mj_step(m, d)

    if step < 20:
        lc_dbg, rc_dbg = contact()
        print("DBG step=%02d L=%s R=%s qL=%.6f qR=%.6f vL=%.6f vR=%.6f "
              "aL=%.6f aR=%.6f bx=%.6f by=%.6f bz=%.6f" %
              (step+1, lc_dbg, rc_dbg,
               d.qpos[lq], d.qpos[rq],
               d.qvel[m.jnt_dofadr[jid("left_finger_joint")]],
               d.qvel[m.jnt_dofadr[jid("right_finger_joint")]],
               d.qacc[m.jnt_dofadr[jid("left_finger_joint")]],
               d.qacc[m.jnt_dofadr[jid("right_finger_joint")]],
               xpos("carry_bottle")[0], xpos("carry_bottle")[1], xpos("carry_bottle")[2]))

    if (step+1) % 500 == 0:
        bp=xpos("carry_bottle")
        lc2,rc2=contact()
        mv=float(np.max(np.abs(d.qvel)))
        drift_now=float(np.linalg.norm(bp-b_start))
        print("  t=%.2fs  bz=%.4f  L=%s R=%s  lq=%.4f  mv=%.4f  drift=%.4f" % (
            (step+1)*0.002, bp[2], lc2, rc2, d.qpos[lq], mv, drift_now))
        if not np.isfinite(bp).all() or mv > 200:
            hold_fail=True; print("  INSTABILITY"); break
        if (not lc2 or not rc2) and contact_loss < 0:
            contact_loss = step+1
            print("  CONTACT LOSS at step=%d" % (step+1))

b_end=xpos("carry_bottle")
lc_h,rc_h=contact()
drift=float(np.linalg.norm(b_end-b_start))

print()
print("  bottle_end   =", b_end.round(4))
print("  drift (5s)   =", round(drift,6), "m")
print("  contact_loss =", contact_loss, "(sustained)" if contact_loss<0 else "(LOST)")
print("  lc=%s rc=%s  hold_fail=%s" % (lc_h,rc_h,hold_fail))

g14_pass = (np.isfinite(b_end).all() and not hold_fail
            and contact_loss < 0 and lc_h and rc_h and drift < 0.025)
print()
print("G14 =", "PASS" if g14_pass else "FAIL")
print("  finite=%s  no_fail=%s  contact_sustained=%s  lc=%s  rc=%s  drift<0.025=%s" % (
    np.isfinite(b_end).all(), not hold_fail, contact_loss<0, lc_h, rc_h, drift<0.025))

if not g14_pass:
    print("\nFAIL analysis:")
    if contact_loss >= 0:
        print("  Contact lost at step=%d t=%.2fs" % (contact_loss, contact_loss*0.002))
    if drift >= 0.025:
        print("  Drift=%.4fm exceeds 0.025m" % drift)
    print("\n  bottle_start =", b_start.round(4))
    print("  bottle_end   =", b_end.round(4))
    print("  delta        =", (b_end-b_start).round(4))

# ── G13 REGRESSION ───────────────────────────────────────────────────────
print("\n========================================")
print("G13 REGRESSION")
print("========================================")
m2=mujoco.MjModel.from_xml_path(XML)
d2=mujoco.MjData(m2)
tbi2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"base_Link")
rj2=m2.body_jntadr[tbi2]; rqp2=m2.jnt_qposadr[rj2]; rdof2=m2.jnt_dofadr[rj2]
HQPOS7=np.array([0.0,-3.0,0.82,1.0,0.0,0.0,0.0])
d2.qpos[rqp2:rqp2+7]=HQPOS7; d2.qvel[:]=0.0
lq2=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"left_finger_joint")]
rq2=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"right_finger_joint")]
for jn,qv in zip(arm_joints,Q0):
    ji2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,jn); d2.qpos[m2.jnt_qposadr[ji2]]=qv
d2.qpos[lq2]=0.07; d2.qpos[rq2]=0.07
for jn,qv in zip(arm_joints,Q0):
    ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
    if ai2>=0: d2.ctrl[ai2]=qv
lf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_left_finger")
rf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_right_finger")
d2.ctrl[lf2]=-0.015; d2.ctrl[rf2]=-0.015
m2.opt.gravity[:]=0.0; mujoco.mj_forward(m2,d2)
bg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"carry_bottle_collision")
lg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"left_finger_geom")
rg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"right_finger_geom")
contact_achieved_r=False
for step2 in range(2000):
    d2.qpos[rqp2:rqp2+7]=HQPOS7; d2.qvel[rdof2:rdof2+6]=0.0
    for jn,qv in zip(arm_joints,Q0):
        ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
        if ai2>=0: d2.ctrl[ai2]=qv
    d2.ctrl[lf2]=-0.015; d2.ctrl[rf2]=-0.015
    mujoco.mj_step(m2,d2)
    lc2=rc2=False
    for ci in range(d2.ncon):
        c=d2.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bg2,lg2}: lc2=True
        if {a,b}=={bg2,rg2}: rc2=True
    if lc2 and rc2:
        contact_achieved_r=True
        break
lc_r=rc_r=False
for ci in range(d2.ncon):
    c=d2.contact[ci]; a,b=int(c.geom1),int(c.geom2)
    if {a,b}=={bg2,lg2}: lc_r=True
    if {a,b}=={bg2,rg2}: rc_r=True
print("  lc=%s rc=%s" % (lc_r,rc_r))
print("G13 REGRESSION =","PASS" if (lc_r and rc_r) else "FAIL")

# ── FINAL REPORT ─────────────────────────────────────────────────────────
print("\n========================================")
print("FINAL REPORT")
print("========================================")
print("G13 REGRESSION =","PASS" if (lc_r and rc_r) else "FAIL")
print("G14 HOLD       =","PASS" if g14_pass else "FAIL")
print("  drift=%.6fm  contact_loss=%s  lc=%s rc=%s" % (drift,contact_loss,lc_h,rc_h))
print("XML MODIFIED   = NO")
print("Method: TRON1 root fixed every step (scenario: STOP_COMPLETE state)")
