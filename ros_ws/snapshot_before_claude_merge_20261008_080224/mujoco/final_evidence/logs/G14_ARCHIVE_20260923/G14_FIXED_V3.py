"""
G14 TRON1-FIXED V3
Progress: drift=0.0m ACHIEVED. Contact loss at t=4s (rc=False).
Issue: rq near range max (0.07), right finger pushed out by bottle.
Fix: scan wrist angles to find better approach geometry where
     both fingers wrap around bottle center symmetrically.

Strategy:
  Keep Q0 pan/lift/elbow, scan wrist_1/wrist_2/wrist_3
  to find pose where right finger stays in contact.
  Also try slight adjustment of shoulder_pan to change approach angle.

XML MODIFIED = NO
"""
import mujoco, math, sys
import numpy as np

XML = r"c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
print("G14 TRON1-FIXED V3 -- wrist scan for stable bilateral contact")
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

def try_grasp_and_hold(q_test, label, n_grasp=1500, n_hold=2500):
    """Returns (g13_ok, contact_loss_step, drift_5s, lc_end, rc_end)"""
    # Reset
    d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d.qvel[:] = 0.0
    for jn,qv in zip(arm_joints,q_test):
        d.qpos[qadr(jn)] = qv
    d.qpos[lq]=0.07; d.qpos[rq]=0.07
    set_ctrl(q_test, -0.015)
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d)

    # Grasp
    got_both = False
    for _ in range(n_grasp):
        fix_tron1(); set_ctrl(q_test,-0.015); mujoco.mj_step(m,d)
        lc0,rc0=contact()
        if lc0 and rc0: got_both=True; break

    if not got_both:
        return False, -2, 999, False, False

    # Hold 5s
    gravity_orig2 = m.opt.gravity.copy()
    m.opt.gravity[:] = np.array([0,0,-9.81])
    b_s = xpos("carry_bottle").copy()
    cl=-1; hf=False
    for step in range(n_hold):
        fix_tron1(); set_ctrl(q_test,-0.015); mujoco.mj_step(m,d)
        lc2,rc2=contact()
        bp=xpos("carry_bottle")
        if not np.isfinite(bp).all() or float(np.max(np.abs(d.qvel)))>200:
            hf=True; break
        if (not lc2 or not rc2) and cl<0:
            cl=step+1
    b_e=xpos("carry_bottle"); lce,rce=contact()
    drift=float(np.linalg.norm(b_e-b_s))
    m.opt.gravity[:] = gravity_orig2
    return True, cl, drift, lce, rce

# ── SCAN: wrist_3 variation (rotation of tool frame around approach axis) ──
print("\n=== WRIST_3 SCAN ===")
print("  wrist_3 | G13 | contact_loss | drift   | lc  | rc")
best_q = None
best_score = -1
for w3 in np.linspace(-0.5, 0.5, 11):
    q_test = Q0.copy(); q_test[5] = w3
    g13_ok, cl, drift, lce, rce = try_grasp_and_hold(q_test, "w3=%.2f"%w3, 1500, 2500)
    sustained = (cl < 0)
    score = (1 if g13_ok else 0) + (2 if sustained else 0) + (1 if drift<0.025 else 0)
    print("  w3=%.2f | %s | %s | %.4f | %s | %s  score=%d" % (
        w3, "OK" if g13_ok else "NO", "OK" if cl<0 else "LOST@%d"%cl,
        drift, lce, rce, score))
    if score > best_score:
        best_score=score; best_q=q_test.copy()

print("\nBest score=%d  best_w3=%.4f" % (best_score, best_q[5]))

# ── If wrist scan insufficient, try wrist_2 ───────────────────────────────
if best_score < 4:
    print("\n=== WRIST_2 SCAN ===")
    for w2 in np.linspace(-1.2, -0.7, 6):
        q_test = Q0.copy(); q_test[4] = w2; q_test[5] = best_q[5]
        g13_ok, cl, drift, lce, rce = try_grasp_and_hold(q_test, "w2=%.2f"%w2, 1500, 2500)
        score = (1 if g13_ok else 0) + (2 if cl<0 else 0) + (1 if drift<0.025 else 0)
        print("  w2=%.2f | %s | %s | %.4f | %s | %s  score=%d" % (
            w2, "OK" if g13_ok else "NO", "OK" if cl<0 else "LOST@%d"%cl,
            drift, lce, rce, score))
        if score > best_score:
            best_score=score; best_q=q_test.copy()

# ── Run official G14 with best config ────────────────────────────────────
print("\n=== OFFICIAL G14 WITH BEST CONFIG ===")
print("Best Q =", best_q.round(4))

d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,best_q):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(best_q,-0.015)
m.opt.gravity[:]=0.0; mujoco.mj_forward(m,d)

got_both=False
for step in range(1500):
    fix_tron1(); set_ctrl(best_q,-0.015); mujoco.mj_step(m,d)
    lc0,rc0=contact()
    if lc0 and rc0: got_both=True; print("  G13 bilateral at step %d"%step); break

if not got_both:
    print("G13 NOT ACHIEVED with best config"); sys.exit(1)

b_g13=xpos("carry_bottle")
print("G13 PASS  bottle=%s  lq=%.4f  rq=%.4f"%(b_g13.round(4),d.qpos[lq],d.qpos[rq]))

m.opt.gravity[:] = np.array([0,0,-9.81])
b_start=xpos("carry_bottle").copy()
cl=-1; hf=False

for step in range(2500):
    fix_tron1(); set_ctrl(best_q,-0.015); mujoco.mj_step(m,d)
    lc2,rc2=contact(); bp=xpos("carry_bottle")
    if not np.isfinite(bp).all() or float(np.max(np.abs(d.qvel)))>200:
        hf=True; break
    if (not lc2 or not rc2) and cl<0: cl=step+1
    if (step+1)%500==0:
        drift_now=float(np.linalg.norm(bp-b_start))
        print("  t=%.2fs  L=%s R=%s  lq=%.4f  rq=%.4f  drift=%.4f"%(
            (step+1)*0.002,lc2,rc2,d.qpos[lq],d.qpos[rq],drift_now))

b_end=xpos("carry_bottle"); lch,rch=contact()
drift=float(np.linalg.norm(b_end-b_start))
g14_pass=(np.isfinite(b_end).all() and not hf and cl<0 and lch and rch and drift<0.025)

print()
print("G14 =","PASS" if g14_pass else "FAIL")
print("  drift=%.6f  contact_loss=%s  lc=%s rc=%s  hf=%s"%(drift,cl,lch,rch,hf))
print("XML MODIFIED = NO")
if g14_pass:
    print("G14 OFFICIAL PASS ACHIEVED")
    # Save best Q
    print("Best IK Q =", list(best_q.round(6)))
