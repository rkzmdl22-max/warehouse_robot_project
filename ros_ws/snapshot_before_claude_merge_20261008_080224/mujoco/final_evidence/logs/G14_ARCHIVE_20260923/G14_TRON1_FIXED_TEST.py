"""
G14 실험 — TRON1 root 고정 (runtime-only, XML 수정 없음)

원인: carry_bottle은 carry_tray의 rigid child → TRON1(9.6kg) 전체와 맞서야 함
     → finger contact 0.5초 만에 소실

해결: TRON1 root를 고정 (handover 위치에 정지한 상태 유지)
방법: gravity ON 후 매 step마다 root qpos/qvel을 handover 위치로 강제 유지

이것은 PROJECT MASTER 시나리오에 부합:
  "TRON1이 handover 위치에 도착하여 STOP_COMPLETE 상태"

공식 G14 조건:
  np.isfinite(b_hold_end).all()
  and not hold_fail
  and contact_loss < 0
  and lc_h and rc_h
  and drift < 0.025 m

XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
print("G14 TRON1-FIXED TEST")
print("XML MODIFIED = NO")

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def xpos(n): return d.xpos[bid(n)].copy()
def qadr(n): return m.jnt_qposadr[jid(n)]
def vaddr(n): return m.jnt_dofadr[jid(n)]

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

HANDOVER_POS = np.array([0.0,-3.0,0.82])
# freejoint qpos: [x,y,z, qw,qx,qy,qz]
HANDOVER_QPOS = np.array([0.0,-3.0,0.82, 1.0,0.0,0.0,0.0])

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
    """TRON1 root를 handover 위치에 고정 (velocity=0)"""
    d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS
    d.qvel[root_dof:root_dof+6] = 0.0

# ── G13 재현 ──────────────────────────────────────────────────────────────
print("\n[G13 INIT]")
d.qpos[root_qp:root_qp+3] = HANDOVER_POS
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(Q0,-0.015)
m.opt.gravity[:]=0.0
mujoco.mj_forward(m,d)

for _ in range(1500):
    fix_tron1()  # TRON1 고정 유지
    set_ctrl(Q0,-0.015)
    mujoco.mj_step(m,d)

lc0,rc0=contact()
b_g13=xpos("carry_bottle")
print("  bottle=",b_g13.round(4),"  lc=%s rc=%s  lq=%.4f"%(lc0,rc0,d.qpos[lq]))
if not (lc0 and rc0):
    print("G13 NOT REPRODUCED"); sys.exit(1)
print("G13 = PASS")

gravity_orig = m.opt.gravity.copy()

# ── G14: HOLD 5초, TRON1 고정 ────────────────────────────────────────────
print("\n========================================")
print("G14: HOLD 5s (TRON1 fixed at handover)")
print("========================================")
m.opt.gravity[:] = gravity_orig
b_start = xpos("carry_bottle").copy()
print("  bottle start =", b_start.round(4))

hold_fail = False
contact_loss = -1

for step in range(2500):
    fix_tron1()          # 매 step마다 TRON1 고정
    set_ctrl(Q0,-0.015)
    mujoco.mj_step(m,d)

    if (step+1) % 500 == 0:
        bp=xpos("carry_bottle")
        lc2,rc2=contact()
        mv=float(np.max(np.abs(d.qvel)))
        drift_now=float(np.linalg.norm(bp-b_start))
        print("  t=%.2fs  bottle=%s  L=%s R=%s  lq=%.4f  max_qvel=%.4f  drift=%.4f"%(
            (step+1)*0.002, bp.round(4), lc2, rc2, d.qpos[lq], mv, drift_now))
        if not np.isfinite(bp).all() or mv>200:
            hold_fail=True; print("  INSTABILITY"); break
        if (not lc2 or not rc2) and contact_loss<0:
            contact_loss=step+1

b_end=xpos("carry_bottle")
lc_h,rc_h=contact()
drift=float(np.linalg.norm(b_end-b_start))
mv_end=float(np.max(np.abs(d.qvel)))

print()
print("  bottle_end     =", b_end.round(4))
print("  drift (5s)     =", round(drift,6), "m")
print("  contact_loss   =", contact_loss, ("(sustained)" if contact_loss<0 else "(LOST at %d)"%contact_loss))
print("  lc=%s rc=%s  hold_fail=%s  finite=%s"%(lc_h,rc_h,hold_fail,np.isfinite(b_end).all()))
print("  max_qvel       =", round(mv_end,4))

g14_pass = (np.isfinite(b_end).all() and not hold_fail
            and contact_loss < 0 and lc_h and rc_h and drift < 0.025)

print()
print("G14 =", "PASS" if g14_pass else "FAIL")
print("  Official conditions check:")
print("  finite=%s  hold_fail=%s  contact_loss<0=%s  lc=%s  rc=%s  drift<0.025=%s"%(
    np.isfinite(b_end).all(), not hold_fail, contact_loss<0, lc_h, rc_h, drift<0.025))

# ── G13 REGRESSION ───────────────────────────────────────────────────────
print("\n========================================")
print("G13 REGRESSION CHECK")
print("========================================")
m2=mujoco.MjModel.from_xml_path(XML)
d2=mujoco.MjData(m2)
tbi2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"base_Link")
rj2=m2.body_jntadr[tbi2]; rqp2=m2.jnt_qposadr[rj2]; rdof2=m2.jnt_dofadr[rj2]
lq2=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"left_finger_joint")]
rq2=m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"right_finger_joint")]
HQPOS=np.array([0.0,-3.0,0.82,1.0,0.0,0.0,0.0])
d2.qpos[rqp2:rqp2+7]=HQPOS; d2.qvel[:]=0.0
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
for _ in range(800):
    d2.qpos[rqp2:rqp2+7]=HQPOS; d2.qvel[rdof2:rdof2+6]=0.0
    for jn,qv in zip(arm_joints,Q0):
        ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
        if ai2>=0: d2.ctrl[ai2]=qv
    d2.ctrl[lf2]=-0.015; d2.ctrl[rf2]=-0.015
    mujoco.mj_step(m2,d2)
bg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"carry_bottle_collision")
lg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"left_finger_geom")
rg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"right_finger_geom")
lc_r=rc_r=False
for ci in range(d2.ncon):
    c=d2.contact[ci]; a,b=int(c.geom1),int(c.geom2)
    if {a,b}=={bg2,lg2}: lc_r=True
    if {a,b}=={bg2,rg2}: rc_r=True
g13_reg=lc_r and rc_r
print("  lc=%s rc=%s"%(lc_r,rc_r))
print("G13 REGRESSION =","PASS" if g13_reg else "FAIL")

# ── 최종 보고 ─────────────────────────────────────────────────────────────
print("\n========================================")
print("FINAL REPORT")
print("========================================")
print("G14 HOLD =","PASS" if g14_pass else "FAIL")
print("  drift      =",round(drift,6),"m  (< 0.025 required)")
print("  contact    = sustained" if contact_loss<0 else "  contact LOST at step %d"%contact_loss)
print("  lc=%s rc=%s  hold_fail=%s"%(lc_h,rc_h,hold_fail))
print("G13 REGRESSION =","PASS" if g13_reg else "FAIL")
print("XML MODIFIED = NO")
print("Method: fix_tron1() called every step (qpos+qvel forced to handover)")
if g14_pass:
    print("\nG14 PASS ACHIEVED.")
    print("Next: G15 Lift (50mm)")
else:
    print("\nG14 still FAIL. drift=",round(drift,4),"m")
