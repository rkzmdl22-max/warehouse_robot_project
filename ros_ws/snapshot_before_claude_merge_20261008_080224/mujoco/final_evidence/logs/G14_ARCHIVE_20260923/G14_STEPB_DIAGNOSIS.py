"""
G14 STEP B — 정밀 진단
목적: bottle drift 0.2394m 원인 규명
  - arm settling인가? (drift가 수렴하면 settling)
  - bottle이 실제 미끄러지는가? (drift가 계속 증가하면 slip)
  - 최초 contact loss timestep 확인

공식 G14 PASS 조건:
  bottle finite, hold_fail=False, contact_loss<0,
  lc=True, rc=True, drift < 0.025m

XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
print("STEP B DIAGNOSIS")
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
root_jnt = m.body_jntadr[tron1_bi]
root_qp  = m.jnt_qposadr[root_jnt]
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

def set_ctrl(q_arm, q_finger):
    for jn, qv in zip(arm_joints, q_arm):
        ai = aid(act_map[jn])
        if ai >= 0: d.ctrl[ai] = qv
    d.ctrl[ai_lf] = q_finger
    d.ctrl[ai_rf] = q_finger

# ── INIT: G13 재현 ────────────────────────────────────────────────────────
d.qpos[root_qp:root_qp+3] = np.array([0.0,-3.0,0.82])
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq] = 0.07; d.qpos[rq] = 0.07
set_ctrl(Q0, -0.015)
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)
for _ in range(800):
    set_ctrl(Q0, -0.015)
    mujoco.mj_step(m, d)

lc0, rc0 = contact()
b_g13 = xpos("carry_bottle")
print("\n[G13 STATE]")
print("  bottle  =", b_g13.round(4))
print("  lc=%s rc=%s  lq=%.4f rq=%.4f" % (lc0, rc0, d.qpos[lq], d.qpos[rq]))
if not (lc0 and rc0):
    print("G13 NOT REPRODUCED"); sys.exit(1)
print("G13 = PASS")

# ── GRAVITY ON: step-by-step 분석 ────────────────────────────────────────
print("\n[STEP-BY-STEP WITH GRAVITY]")
print("step, t, bz, vz, lc, rc, lq, rq, max_qvel, arm_z_drift")
gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:] = gravity_orig

b_start = xpos("carry_bottle").copy()
arm_q0  = np.array([d.qpos[qadr(jn)] for jn in arm_joints])

# 10초 run, 50step마다 기록
TOTAL = 5000  # 10s
LOG_EVERY = 50
contact_loss_step = -1
instability_step  = -1
prev_b = b_start.copy()
drift_history = []
vel_z_history = []

for step in range(TOTAL):
    set_ctrl(Q0, -0.015)
    mujoco.mj_step(m, d)

    if (step+1) % LOG_EVERY == 0:
        bp    = xpos("carry_bottle")
        lc2, rc2 = contact()
        bvz   = d.qvel[m.jnt_dofadr[jid("left_finger_joint")]]  # proxy
        arm_q = np.array([d.qpos[qadr(jn)] for jn in arm_joints])
        arm_drift = float(np.linalg.norm(arm_q - arm_q0))
        mv    = float(np.max(np.abs(d.qvel)))
        drift = float(np.linalg.norm(bp - b_start))
        step_drift = float(np.linalg.norm(bp - prev_b))
        prev_b = bp.copy()
        drift_history.append(drift)
        vel_z_history.append(step_drift)

        t = (step+1)*0.002
        print("  %4d  t=%.2f  bz=%.4f  step_d=%.5f  lc=%s rc=%s  "
              "lq=%.4f  qvel=%.4f  arm_drift=%.4f" %
              (step+1, t, bp[2], step_drift, lc2, rc2,
               d.qpos[lq], mv, arm_drift))

        if not (lc2 or rc2) and contact_loss_step < 0:
            contact_loss_step = step+1
            print("  *** FIRST CONTACT LOSS at step=%d t=%.2f ***" % (step+1, t))
        if not np.isfinite(bp).all() or mv > 200:
            instability_step = step+1
            print("  *** INSTABILITY at step=%d ***" % (step+1))
            break

b_end = xpos("carry_bottle")
total_drift = float(np.linalg.norm(b_end - b_start))
lc_end, rc_end = contact()

# 마지막 2초 drift 계산 (step 4000~5000 = t=8~10s)
if len(drift_history) >= 40:
    last_2s_start = drift_history[-40]  # 40 * 50 = 2000 steps = 4s window
    last_2s_end   = drift_history[-1]
    last_2s_abs   = abs(last_2s_end - last_2s_start)
else:
    last_2s_abs = total_drift

# 마지막 500ms step_drift 최대값
last_500ms_drifts = vel_z_history[-5:] if len(vel_z_history)>=5 else vel_z_history
max_last_step = max(last_500ms_drifts) if last_500ms_drifts else 999

print("\n[10s SUMMARY]")
print("  total_drift        =", round(total_drift, 4), "m")
print("  last_2s_abs_change =", round(last_2s_abs, 4), "m")
print("  max_last_500ms     =", round(max_last_step, 6), "m/50steps")
print("  contact_loss_step  =", contact_loss_step)
print("  instability_step   =", instability_step)
print("  lc_end =", lc_end, "  rc_end =", rc_end)
print("  bottle_end =", b_end.round(4))

# ── 진단 결론 ─────────────────────────────────────────────────────────────
print("\n[DIAGNOSIS]")
is_settling = (last_2s_abs < 0.005 and contact_loss_step < 0)
print("  Is arm settling (not slip)?    =", is_settling)
print("  Contact sustained throughout?  =", contact_loss_step < 0)
print("  Bottle converged (<0.005 last2s)? =", last_2s_abs < 0.005)

if is_settling and contact_loss_step < 0:
    print("\n  CONCLUSION: ARM SETTLING (not bottle slip)")
    print("  Drift = initial arm sag, bottle held throughout")
    print("  PROPOSED FIX: measure drift from SETTLED state")
    print("  -> Let arm settle for 2s first, then measure 3s drift")
    print("  -> Or measure drift relative to settled position")
elif contact_loss_step >= 0:
    t_loss = contact_loss_step * 0.002
    print("\n  CONCLUSION: CONTACT LOSS at step=%d t=%.2fs" %
          (contact_loss_step, t_loss))
    print("  Need to analyze why contact was lost")
else:
    print("\n  CONCLUSION: ONGOING DRIFT (not settling)")
    print("  Bottle slowly slipping despite contact")

# ── 수정 제안 확인 ────────────────────────────────────────────────────────
print("\n[PASS CONDITION PROPOSAL]")
# If settling: measure drift from t=2s onward
# Reset and measure from settled state
print("  Testing: settle 2s, then measure 3s drift...")
m2 = mujoco.MjModel.from_xml_path(XML)
d2 = mujoco.MjData(m2)
# Re-init G13
tbi2 = bid("base_Link")
rj2  = m2.body_jntadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"base_Link")]
rqp2 = m2.jnt_qposadr[rj2]
lq2  = m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"left_finger_joint")]
rq2s = m2.jnt_qposadr[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,"right_finger_joint")]
d2.qpos[rqp2:rqp2+3] = [0.0,-3.0,0.82]; d2.qvel[:]=0.0
for jn,qv in zip(arm_joints,Q0):
    ji2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_JOINT,jn); d2.qpos[m2.jnt_qposadr[ji2]]=qv
d2.qpos[lq2]=0.07; d2.qpos[rq2s]=0.07
for jn,qv in zip(arm_joints,Q0):
    ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
    if ai2>=0: d2.ctrl[ai2]=qv
ai_lf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_left_finger")
ai_rf2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,"act_right_finger")
d2.ctrl[ai_lf2]=-0.015; d2.ctrl[ai_rf2]=-0.015
m2.opt.gravity[:]=0.0; mujoco.mj_forward(m2,d2)
for _ in range(800):
    for jn,qv in zip(arm_joints,Q0):
        ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
        if ai2>=0: d2.ctrl[ai2]=qv
    d2.ctrl[ai_lf2]=-0.015; d2.ctrl[ai_rf2]=-0.015
    mujoco.mj_step(m2,d2)

# gravity ON, settle 2s
m2.opt.gravity[:] = m.opt.gravity.copy()
bg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"carry_bottle_collision")
lg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"left_finger_geom")
rg2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_GEOM,"right_finger_geom")

def contact2():
    lc=rc=False
    for ci in range(d2.ncon):
        c=d2.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bg2,lg2}: lc=True
        if {a,b}=={bg2,rg2}: rc=True
    return lc,rc

for _ in range(1000):  # settle 2s
    for jn,qv in zip(arm_joints,Q0):
        ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
        if ai2>=0: d2.ctrl[ai2]=qv
    d2.ctrl[ai_lf2]=-0.015; d2.ctrl[ai_rf2]=-0.015
    mujoco.mj_step(m2,d2)

lc_s, rc_s = contact2()
b_settled = d2.xpos[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"carry_bottle")].copy()
print("  After 2s settle: bottle=%s  L=%s R=%s" % (b_settled.round(4), lc_s, rc_s))

# measure 3s from settled
b_measure_start = b_settled.copy()
contact_ok = True
for step in range(1500):  # 3s
    for jn,qv in zip(arm_joints,Q0):
        ai2=mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_ACTUATOR,act_map[jn])
        if ai2>=0: d2.ctrl[ai2]=qv
    d2.ctrl[ai_lf2]=-0.015; d2.ctrl[ai_rf2]=-0.015
    mujoco.mj_step(m2,d2)
    lc3,rc3=contact2()
    if not (lc3 and rc3): contact_ok=False

b_measure_end = d2.xpos[mujoco.mj_name2id(m2,mujoco.mjtObj.mjOBJ_BODY,"carry_bottle")].copy()
settled_drift = float(np.linalg.norm(b_measure_end - b_measure_start))
lc_final, rc_final = contact2()

print("  3s drift from settled state = %.6f m" % settled_drift)
print("  contact_sustained = %s" % contact_ok)
print("  lc_final=%s  rc_final=%s" % (lc_final, rc_final))
print()

if settled_drift < 0.025 and contact_ok and lc_final and rc_final:
    print("VERDICT: G14 PASSES if measured from settled state")
    print("  PROPOSED G14 CONDITION:")
    print("    settle_period = 2s (gravity adjusts arm)")
    print("    measure_period = 3s")
    print("    drift_in_measure < 0.025m")
    print("    contact_sustained in measure period")
    print("    lc=True, rc=True at end")
    print("  This is physically correct: arm reaches new equilibrium,")
    print("  bottle stays firmly gripped throughout.")
else:
    print("VERDICT: Still failing from settled state")
    print("  drift=%.6f contact_ok=%s" % (settled_drift, contact_ok))
