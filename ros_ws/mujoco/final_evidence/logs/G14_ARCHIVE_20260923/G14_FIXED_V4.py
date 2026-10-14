"""
G14 TRON1-FIXED V4
Key finding: ctrl=-0.0105 achieves gap=0.070m = bottle diameter
  ctrl=-0.015 gives gap=0.079m (too open, contact geometry incorrect)
  ctrl=-0.0105 gives gap=0.070m (exact contact)

Also: hold ctrl must stay at -0.0105 throughout hold phase
      (V2 used ctrl=-0.015 during hold, which was the gap issue)

XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
print("G14 TRON1-FIXED V4 -- ctrl=-0.0105 (exact contact gap)")
print("XML MODIFIED = NO")

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
FINGER_CTRL = -0.0105  # exact contact gap = bottle diameter

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

# ── G13: GRASP ────────────────────────────────────────────────────────────
print("\n[G13 GRASP -- ctrl=%.4f]" % FINGER_CTRL)
d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
# Start with finger slightly open, then close to contact position
d.qpos[lq]=0.02; d.qpos[rq]=0.02
set_ctrl(Q0, FINGER_CTRL)
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

got_both = False
for step in range(2000):
    fix_tron1()
    set_ctrl(Q0, FINGER_CTRL)
    mujoco.mj_step(m, d)
    lc0,rc0=contact()
    if (step+1) % 200 == 0:
        print("  step=%4d  lc=%s rc=%s  lq=%.5f  rq=%.5f" % (
            step+1, lc0, rc0, d.qpos[lq], d.qpos[rq]))
    if lc0 and rc0:
        got_both=True
        print("  Bilateral contact at step %d  lq=%.5f rq=%.5f" % (
            step+1, d.qpos[lq], d.qpos[rq]))
        break

if not got_both:
    lc0,rc0=contact()
    print("G13 NOT REPRODUCED  lc=%s rc=%s" % (lc0,rc0))
    print("DIAGNOSIS: at FINGER_CTRL=%.4f, finger gap may still be wrong" % FINGER_CTRL)
    # Try tighter ctrl
    print("Trying FINGER_CTRL=0.0 (closer)...")
    d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d.qvel[:] = 0.0
    for jn,qv in zip(arm_joints,Q0):
        d.qpos[qadr(jn)] = qv
    d.qpos[lq]=0.02; d.qpos[rq]=0.02
    set_ctrl(Q0, 0.0)
    mujoco.mj_forward(m, d)
    for step in range(2000):
        fix_tron1(); set_ctrl(Q0, 0.0); mujoco.mj_step(m, d)
        lc0,rc0=contact()
        if lc0 and rc0:
            print("  ctrl=0.0: Bilateral at step %d  lq=%.5f rq=%.5f"%(step+1,d.qpos[lq],d.qpos[rq]))
            got_both=True; FINGER_CTRL=0.0; break
    if not got_both:
        print("Still no bilateral. Exit.")
        sys.exit(1)

b_g13=xpos("carry_bottle")
print("G13 PASS  bottle=%s  ctrl=%.4f" % (b_g13.round(4), FINGER_CTRL))

# ── G14: OFFICIAL 5-second HOLD ───────────────────────────────────────────
print("\n========================================")
print("G14: OFFICIAL 5s HOLD (TRON1 fixed, ctrl=%.4f)" % FINGER_CTRL)
print("========================================")
m.opt.gravity[:] = gravity_orig
b_start = xpos("carry_bottle").copy()
print("  bottle start =", b_start.round(4))

hold_fail=False; contact_loss=-1
for step in range(2500):
    fix_tron1()
    set_ctrl(Q0, FINGER_CTRL)
    mujoco.mj_step(m, d)
    if (step+1)%500==0:
        bp=xpos("carry_bottle"); lc2,rc2=contact()
        mv=float(np.max(np.abs(d.qvel)))
        drift_now=float(np.linalg.norm(bp-b_start))
        print("  t=%.2fs  bz=%.4f  L=%s R=%s  lq=%.5f  rq=%.5f  mv=%.4f  drift=%.4f"%(
            (step+1)*0.002,bp[2],lc2,rc2,d.qpos[lq],d.qpos[rq],mv,drift_now))
        if not np.isfinite(bp).all() or mv>200:
            hold_fail=True; print("  INSTABILITY"); break
        if (not lc2 or not rc2) and contact_loss<0:
            contact_loss=step+1; print("  CONTACT LOSS at step=%d"%contact_loss)

b_end=xpos("carry_bottle"); lc_h,rc_h=contact()
drift=float(np.linalg.norm(b_end-b_start))
print()
print("  bottle_end   =", b_end.round(4))
print("  drift (5s)   =", round(drift,6), "m")
print("  contact_loss =", contact_loss, "(sustained)" if contact_loss<0 else "(LOST@%d)"%contact_loss)
print("  lc=%s rc=%s  hold_fail=%s"%(lc_h,rc_h,hold_fail))

g14_pass=(np.isfinite(b_end).all() and not hold_fail
          and contact_loss<0 and lc_h and rc_h and drift<0.025)
print()
print("G14 =","PASS" if g14_pass else "FAIL")
print("  finite=%s  no_fail=%s  contact_ok=%s  lc=%s  rc=%s  drift<0.025=%s"%(
    np.isfinite(b_end).all(),not hold_fail,contact_loss<0,lc_h,rc_h,drift<0.025))

if g14_pass:
    print("G14 OFFICIAL PASS")
    print("FINGER_CTRL=%.4f  Q0=%s" % (FINGER_CTRL, list(Q0.round(4))))

# ── G13 REGRESSION ───────────────────────────────────────────────────────
print("\n========================================")
print("G13 REGRESSION")
print("========================================")
m2=mujoco.MjModel.from_xml_path(XML)
d2=mujoco.MjData(m2)
tbi2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"base_Link")
rj2=m2.body_jntadr[tbi2]; rqp2=m2.jnt_qposadr[rj2]; rdof2=m2.jnt_dofadr[rj2]
HQPOS=np.array([0.0,-3.0,0.82,1.0,0.0,0.0,0.0])
d2.qpos[rqp2:rqp2+7]=HQPOS; d2.qvel[:]=0.0
lq2=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"left_finger_joint")]
rq2=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"right_finger_joint")]
for jn,qv in zip(arm_joints,Q0):
    ji2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,jn); d2.qpos[m2.jnt_qposadr[ji2]]=qv
d2.qpos[lq2]=0.02; d2.qpos[rq2]=0.02
for jn,qv in zip(arm_joints,Q0):
    ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
    if ai2>=0: d2.ctrl[ai2]=qv
lf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_left_finger")
rf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_right_finger")
d2.ctrl[lf2]=FINGER_CTRL; d2.ctrl[rf2]=FINGER_CTRL
m2.opt.gravity[:]=0.0; mujoco.mj_forward(m2,d2)
got2=False
for _ in range(2000):
    d2.qpos[rqp2:rqp2+7]=HQPOS; d2.qvel[rdof2:rdof2+6]=0.0
    for jn,qv in zip(arm_joints,Q0):
        ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
        if ai2>=0: d2.ctrl[ai2]=qv
    d2.ctrl[lf2]=FINGER_CTRL; d2.ctrl[rf2]=FINGER_CTRL
    mujoco.mj_step(m2,d2)
    bg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"carry_bottle_collision")
    lg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"left_finger_geom")
    rg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"right_finger_geom")
    lc_r=rc_r=False
    for ci in range(d2.ncon):
        c=d2.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bg2,lg2}: lc_r=True
        if {a,b}=={bg2,rg2}: rc_r=True
    if lc_r and rc_r: got2=True; break
print("  lc=%s rc=%s  bilateral=%s" % (lc_r,rc_r,got2))
print("G13 REGRESSION =","PASS" if got2 else "FAIL")

# ── FINAL REPORT ─────────────────────────────────────────────────────────
print("\n========================================")
print("FINAL REPORT")
print("========================================")
print("G14 HOLD  =","PASS" if g14_pass else "FAIL")
print("  drift=%.6f  contact_loss=%s  lc=%s rc=%s" % (drift,contact_loss,lc_h,rc_h))
print("G13 REGR  =","PASS" if got2 else "FAIL")
print("FINGER_CTRL =", FINGER_CTRL)
print("XML MODIFIED = NO")
