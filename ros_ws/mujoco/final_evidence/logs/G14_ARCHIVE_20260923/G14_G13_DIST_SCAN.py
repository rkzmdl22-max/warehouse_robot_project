"""
G13 = kiro (finger ctrl=0.0, 600 zero-G). Then measure right penetration vs pan.
Only run official 5s if both contacts exist AND right dist < -0.002 (deeper than hairline).
Hold: finger 0.07 + arm FF.
XML MODIFIED = NO
"""
import sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_G13_DIST_SCAN_RESULT.txt"
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
ARM = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
       "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
ACT = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
       "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
       "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}

m0 = mujoco.MjModel.from_xml_path(XML)  # unused warmup

def g13_dists(q_arm):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    tron1 = bid("base_Link")
    root_qp = m.jnt_qposadr[m.body_jntadr[tron1]]
    root_dof = m.jnt_dofadr[m.body_jntadr[tron1]]
    lq = m.jnt_qposadr[jid("left_finger_joint")]
    rq = m.jnt_qposadr[jid("right_finger_joint")]
    bgeom = gid("carry_bottle_collision")
    lgeom = gid("left_finger_geom"); rgeom = gid("right_finger_geom")
    ai_lf = aid("act_left_finger"); ai_rf = aid("act_right_finger")
    arm_act = [aid(ACT[j]) for j in ARM]
    arm_q = [m.jnt_qposadr[jid(j)] for j in ARM]
    d.qpos[root_qp:root_qp+7] = HANDOVER
    d.qvel[:] = 0
    for adr, qv in zip(arm_q, q_arm):
        d.qpos[adr] = qv
    d.qpos[lq] = 0.07; d.qpos[rq] = 0.07
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d)
    for _ in range(600):
        d.qpos[root_qp:root_qp+7] = HANDOVER
        d.qvel[root_dof:root_dof+6] = 0.0
        for ai, qv in zip(arm_act, q_arm):
            d.ctrl[ai] = qv
        d.ctrl[ai_lf] = 0.0; d.ctrl[ai_rf] = 0.0
        mujoco.mj_step(m, d)
    lc=rc=False; dl=dr=None
    for ci in range(d.ncon):
        c=d.contact[ci]; pair={int(c.geom1),int(c.geom2)}
        if pair=={bgeom,lgeom}:
            lc=True; dl=float(c.dist)
        if pair=={bgeom,rgeom}:
            rc=True; dr=float(c.dist)
    return lc, rc, dl, dr, float(d.qpos[lq]), float(d.qpos[rq])


def official_hold(q_arm, tag):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    g0 = m.opt.gravity.copy()
    def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    tron1 = bid("base_Link")
    root_qp = m.jnt_qposadr[m.body_jntadr[tron1]]
    root_dof = m.jnt_dofadr[m.body_jntadr[tron1]]
    lq = m.jnt_qposadr[jid("left_finger_joint")]
    rq = m.jnt_qposadr[jid("right_finger_joint")]
    bb = bid("carry_bottle")
    bgeom = gid("carry_bottle_collision")
    lgeom = gid("left_finger_geom"); rgeom = gid("right_finger_geom")
    ai_lf = aid("act_left_finger"); ai_rf = aid("act_right_finger")
    arm_act = [aid(ACT[j]) for j in ARM]
    arm_q = [m.jnt_qposadr[jid(j)] for j in ARM]
    arm_dof = [m.jnt_dofadr[jid(j)] for j in ARM]
    kp_arm = np.array([float(m.actuator_gainprm[ai][0]) for ai in arm_act])
    def contact():
        lc=rc=False; dl=dr=None
        for ci in range(d.ncon):
            c=d.contact[ci]; pair={int(c.geom1),int(c.geom2)}
            if pair=={bgeom,lgeom}:
                lc=True; dl=float(c.dist)
            if pair=={bgeom,rgeom}:
                rc=True; dr=float(c.dist)
        return lc,rc,dl,dr
    def fix():
        d.qpos[root_qp:root_qp+7]=HANDOVER
        d.qvel[root_dof:root_dof+6]=0.0
    def setc(ff, lf, rf):
        ctrl=np.asarray(q_arm,float)
        if ff:
            mujoco.mj_forward(m,d)
            extra=np.array([float(d.qfrc_bias[dof])/(kp if kp else 1.0) for dof,kp in zip(arm_dof,kp_arm)])
            ctrl=ctrl+extra
        for ai,qv in zip(arm_act,ctrl):
            d.ctrl[ai]=qv
        d.ctrl[ai_lf]=lf; d.ctrl[ai_rf]=rf
    d.qpos[root_qp:root_qp+7]=HANDOVER; d.qvel[:]=0
    for adr,qv in zip(arm_q,q_arm):
        d.qpos[adr]=qv
    d.qpos[lq]=0.07; d.qpos[rq]=0.07
    m.opt.gravity[:]=0.0
    for _ in range(600):
        fix(); setc(False, 0.0, 0.0); mujoco.mj_step(m,d)
    lc,rc,dl,dr=contact()
    if not (lc and rc):
        print("  skip hold, G13 fail")
        return {"tag":tag,"g13":False,"pass":False}
    m.opt.gravity[:]=g0
    b0=d.xpos[bb].copy()
    loss=-1; fail=False
    for step in range(2500):
        fix(); setc(True, 0.07, 0.07); mujoco.mj_step(m,d)
        lc2,rc2,dl2,dr2=contact()
        if (not np.isfinite(d.xpos[bb]).all()) or float(np.max(np.abs(d.qvel)))>200:
            fail=True
            if loss<0: loss=step+1
            break
        if (not lc2 or not rc2) and loss<0:
            loss=step+1
            print("  LOSS t=%.4fs L=%s R=%s dL=%s dR=%s"%((step+1)*0.002,lc2,rc2,dl2,dr2))
        if (step+1)%500==0:
            print("  t=%.2fs L=%s R=%s loss=%s dL=%s dR=%s"%((step+1)*0.002,lc2,rc2,loss,dl2,dr2))
    lc_h,rc_h,_,_=contact()
    drift=float(np.linalg.norm(d.xpos[bb]-b0))
    g14=np.isfinite(d.xpos[bb]).all() and not fail and loss<0 and lc_h and rc_h and drift<0.025
    print("  G14=%s loss=%s drift=%.6f"%("PASS" if g14 else "FAIL", loss, drift))
    return {"tag":tag,"g13":True,"pass":bool(g14),"contact_loss":int(loss),"drift":drift}

print("G13 DIST SCAN then conditional official G14")
print("XML MODIFIED = NO")
print("joint_offset  L  R  distL  distR  lq rq")
cands=[]
# pan, wrist2, wrist3, wrist1 around Q0
for j, name, vals in [
    (0, "pan", np.linspace(-0.08, 0.08, 17)),
    (4, "w2", np.linspace(-0.15, 0.15, 13)),
    (5, "w3", np.linspace(-0.3, 0.3, 13)),
    (3, "w1", np.linspace(-0.15, 0.15, 13)),
]:
    for dv in vals:
        q=Q0.copy(); q[j]+=float(dv)
        lc,rc,dl,dr,lq,rq=g13_dists(q)
        print("%s%+.3f  L=%s R=%s  dL=%s dR=%s  lq=%.4f rq=%.4f"%(
            name, dv, lc, rc,
            None if dl is None else round(dl,5),
            None if dr is None else round(dr,5), lq, rq))
        if lc and rc and dr is not None and dr < -0.002:
            cands.append((q, "%s%+.3f"%(name,dv), dl, dr))

print("\nCandidates with distR < -0.002:", len(cands))
results=[]
winner=None
for q, tag, dl, dr in cands:
    print("\n==== OFFICIAL 5s %s distR=%.5f ===="%(tag, dr))
    r=official_hold(q, tag)
    results.append(r)
    if r.get("pass"):
        winner=r
        break

lines=["G14 G13 DIST SCAN","XML MODIFIED = NO","candidates=%d"%len(cands)]
for r in results:
    lines.append("%s G14=%s loss=%s"%(r.get("tag"), r.get("pass"), r.get("contact_loss")))
lines.append("G14 = %s"%("PASS" if winner else "FAIL"))
open(OUT,"w",encoding="utf-8").write("\n".join(lines)+"\n")
print("Wrote", OUT)
print("G14", "PASS" if winner else "FAIL")
sys.exit(0 if winner else 1)
