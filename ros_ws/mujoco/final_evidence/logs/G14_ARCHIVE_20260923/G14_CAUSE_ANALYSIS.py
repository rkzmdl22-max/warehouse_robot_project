"""
G14 원인분석 — arm settling vs bottle slip 분리
공식 G14 조건: 5초 runtime, contact_loss<0, drift<0.025m

목표:
  bottle drift 0.2394m 중 얼마가 arm settling이고
  얼마가 실제 bottle 미끄러짐인지 분리한다.

핵심 질문:
  Q1. arm ctrl=Q0 인데 왜 arm이 처지는가?
      -> position servo 특성 vs gravity 불균형?
  Q2. bottle이 finger에서 미끄러지는가?
      -> finger-bottle 상대 위치 변화 측정
  Q3. 초기 0~0.5초 구간에서 무슨 일이 일어나는가?
      -> 가장 큰 변화가 이 구간에 집중

XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
print("G14 CAUSE ANALYSIS")
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

def set_ctrl(q_arm, q_f):
    for jn,qv in zip(arm_joints,q_arm):
        ai = aid(act_map[jn])
        if ai>=0: d.ctrl[ai]=qv
    d.ctrl[ai_lf]=q_f; d.ctrl[ai_rf]=q_f

# ── G13 재현 ──────────────────────────────────────────────────────────────
d.qpos[root_qp:root_qp+3] = [0.0,-3.0,0.82]
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(Q0,-0.015)
m.opt.gravity[:]=0.0
mujoco.mj_forward(m,d)
for _ in range(800):
    set_ctrl(Q0,-0.015)
    mujoco.mj_step(m,d)
lc0,rc0=contact()
b_g13=xpos("carry_bottle")
print("\n[G13 state]")
print("  bottle=",b_g13.round(4)," lc=%s rc=%s"%(lc0,rc0))
assert lc0 and rc0,"G13 NOT REPRODUCED"
print("G13 PASS")

# ── 분석 1: arm actuator force vs gravity ────────────────────────────────
print("\n=== ANALYSIS 1: ARM ACTUATOR FORCE vs GRAVITY (gravity ON, step-by-step) ===")
gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:]=gravity_orig

# t=0 직후 첫 20 steps 세밀하게 기록
arm_dof_addrs = [vaddr(jn) for jn in arm_joints]
print("step, t, shoulder_lift_q, shoulder_lift_qd, actuator_force, gravity_force, bottle_z, lc, rc")
b_start = xpos("carry_bottle").copy()

for step in range(100):  # 0.2s
    set_ctrl(Q0,-0.015)

    # 기록 (step 전)
    if step < 20 or step % 10 == 9:
        sl_q  = d.qpos[qadr("shoulder_lift_joint")]
        sl_qd = d.qvel[vaddr("shoulder_lift_joint")]
        # actuator force for shoulder_lift (id=1)
        af = d.qfrc_actuator[vaddr("shoulder_lift_joint")]
        gf = d.qfrc_bias[vaddr("shoulder_lift_joint")]   # gravity+coriolis
        cf = d.qfrc_constraint[vaddr("shoulder_lift_joint")]
        bp = xpos("carry_bottle")
        lc2,rc2=contact()
        print("  %3d  t=%.3f  sl_q=%.4f  sl_qd=%.5f  af=%.4f  gf=%.4f  cf=%.4f  bz=%.4f  L=%s R=%s"%(
            step, step*0.002, sl_q, sl_qd, af, gf, cf, bp[2], lc2, rc2))

    mujoco.mj_step(m,d)

b_01s = xpos("carry_bottle")
print("\n  bottle after 0.2s:",b_01s.round(4))
print("  drift 0.2s:",round(float(np.linalg.norm(b_01s-b_start)),4),"m")

# ── 분석 2: bottle vs tool0 상대 위치 ────────────────────────────────────
print("\n=== ANALYSIS 2: BOTTLE-TOOL0 RELATIVE POSITION (finger slip check) ===")

# Reset
d.qpos[root_qp:root_qp+3]=[0.0,-3.0,0.82]; d.qvel[:]=0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)]=qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(Q0,-0.015)
m.opt.gravity[:]=0.0; mujoco.mj_forward(m,d)
for _ in range(800):
    set_ctrl(Q0,-0.015); mujoco.mj_step(m,d)
m.opt.gravity[:]=gravity_orig

b_ref=xpos("carry_bottle"); t0_ref=xpos("tool0")
rel_ref=b_ref-t0_ref   # bottle relative to tool0 in world frame
print("  Initial bottle-tool0 relative:",rel_ref.round(4))

print("\n  step, t, bottle_z, rel_change, arm_drift, lc, rc")
b_s2=xpos("carry_bottle").copy()
arm_q0_s2=np.array([d.qpos[qadr(jn)] for jn in arm_joints])

for step in range(2500):  # 5s
    set_ctrl(Q0,-0.015)
    mujoco.mj_step(m,d)
    if (step+1)%250==0:
        bp=xpos("carry_bottle"); t0p=xpos("tool0")
        rel_cur=bp-t0p
        rel_change=float(np.linalg.norm(rel_cur-rel_ref))
        arm_qc=np.array([d.qpos[qadr(jn)] for jn in arm_joints])
        arm_drift=float(np.linalg.norm(arm_qc-arm_q0_s2))
        total_drift=float(np.linalg.norm(bp-b_s2))
        lc3,rc3=contact()
        print("  %4d  t=%.2f  bz=%.4f  rel_chg=%.5f  arm_drft=%.4f  tot_drft=%.4f  L=%s R=%s"%(
            step+1,round((step+1)*0.002,2),bp[2],rel_change,arm_drift,total_drift,lc3,rc3))

b_end2=xpos("carry_bottle")
print("\n  total_drift 5s:",round(float(np.linalg.norm(b_end2-b_s2)),4),"m")

# ── 분석 3: Q0 target vs actual qpos 차이 ────────────────────────────────
print("\n=== ANALYSIS 3: Q0 TARGET vs ACTUAL QPOS (why arm sags) ===")

d.qpos[root_qp:root_qp+3]=[0.0,-3.0,0.82]; d.qvel[:]=0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)]=qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(Q0,-0.015)
m.opt.gravity[:]=0.0; mujoco.mj_forward(m,d)
for _ in range(800):
    set_ctrl(Q0,-0.015); mujoco.mj_step(m,d)
m.opt.gravity[:]=gravity_orig

print("  Q0 target vs actual after G13+gravity steps:")
print("  joint                | Q0 target | actual   | error")
for i,jn in enumerate(arm_joints):
    q_actual=d.qpos[qadr(jn)]
    err=q_actual-Q0[i]
    print("  %-20s | %9.4f | %8.4f | %+.4f"%(jn,Q0[i],q_actual,err))

# Run 5s and check final
for _ in range(2500):
    set_ctrl(Q0,-0.015); mujoco.mj_step(m,d)

print("\n  After 5s gravity ON:")
print("  joint                | Q0 target | actual   | error   | kp*err (actuator force approx)")
for i,jn in enumerate(arm_joints):
    q_actual=d.qpos[qadr(jn)]
    err=q_actual-Q0[i]
    # kp from defaults: size3=2000, size1=500
    kp = 2000.0 if i<3 else 500.0
    af_approx = kp*err
    af_real = d.qfrc_actuator[vaddr(jn)]
    gf_real = d.qfrc_bias[vaddr(jn)]
    print("  %-20s | %9.4f | %8.4f | %+.4f  | kp*err=%8.2f  af=%8.2f  gf=%8.2f"%(
        jn,Q0[i],q_actual,err,af_approx,af_real,gf_real))

# ── 분석 4: 실제 ctrl vs Q0 ──────────────────────────────────────────────
print("\n=== ANALYSIS 4: CTRL VALUES BEING SET ===")
print("  Confirming ctrl is actually Q0 during step:")
d.qpos[root_qp:root_qp+3]=[0.0,-3.0,0.82]; d.qvel[:]=0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)]=qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(Q0,-0.015)
m.opt.gravity[:]=0.0; mujoco.mj_forward(m,d)
for _ in range(800):
    set_ctrl(Q0,-0.015); mujoco.mj_step(m,d)
m.opt.gravity[:]=gravity_orig
mujoco.mj_forward(m,d)

print("  ctrl values at start of gravity phase:")
for i,jn in enumerate(arm_joints):
    ai=aid(act_map[jn])
    print("  [%d] %-20s ctrl=%.4f  Q0=%.4f  match=%s"%(
        i,jn,d.ctrl[ai],Q0[i],abs(d.ctrl[ai]-Q0[i])<0.001))
print("  left_finger ctrl=%.4f  right_finger ctrl=%.4f"%(d.ctrl[ai_lf],d.ctrl[ai_rf]))

# ── 진단 결론 ─────────────────────────────────────────────────────────────
print("\n=== DIAGNOSIS CONCLUSION ===")
print("""
물리 시스템:
  position servo: force = kp*(target-qpos) - kd*qvel
  kp(arm) = 2000 (size3) or 500 (size1)
  kd(arm) = 400 (size3) or 100 (size1)  (from biasprm[2])

만약 arm이 처진다면:
  - kp*(Q0-qpos) < gravity_torque 인 경우
  - 즉 Q0로의 restoring force가 gravity를 이기지 못하는 경우

해결 방향:
  A. ctrl을 Q0보다 높게 설정 (over-drive) → arm이 gravity 대비 더 강하게 유지
  B. G13 완료 후 current qpos를 ctrl로 사용 (gravity 포함 평형점 타겟팅)
  C. arm damping 증가 (진동 감소, settling 속도 증가)
  D. kp 증가는 XML 수정 필요 → 먼저 A/B/C runtime 방법 시도

이 스크립트 결과에서:
  - rel_change가 작으면 → arm만 처지고 bottle은 arm과 함께 이동 (slip 아님)
  - rel_change가 크면 → bottle이 finger에서 미끄러짐 (slip)
""")
