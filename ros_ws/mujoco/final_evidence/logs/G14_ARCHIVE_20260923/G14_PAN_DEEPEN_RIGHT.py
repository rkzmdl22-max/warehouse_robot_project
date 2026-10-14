"""
After kiro G13, RIGHT contact is hairline (dist~-0.00026) and is lost at t=6ms.
LEFT is deep (dist~-0.021) and holds 5s with arm FF.

One-variable: small shoulder_pan offset during G13+hold to deepen RIGHT overlap.
Finger ctrl=0.07, arm gravity FF, official 5s.
XML MODIFIED = NO
"""
import sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_PAN_DEEPEN_RIGHT_RESULT.txt"
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
ARM = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
       "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
ACT = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
       "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
       "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}


def official(q_arm, tag):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    g0 = m.opt.gravity.copy()
    def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    def qadr(n): return m.jnt_qposadr[jid(n)]
    tron1 = bid("base_Link")
    root_qp = m.jnt_qposadr[m.body_jntadr[tron1]]
    root_dof = m.jnt_dofadr[m.body_jntadr[tron1]]
    lq = qadr("left_finger_joint"); rq = qadr("right_finger_joint")
    bb = bid("carry_bottle")
    bgeom = gid("carry_bottle_collision")
    lgeom = gid("left_finger_geom"); rgeom = gid("right_finger_geom")
    ai_lf = aid("act_left_finger"); ai_rf = aid("act_right_finger")
    arm_act = [aid(ACT[j]) for j in ARM]
    arm_q = [qadr(j) for j in ARM]
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
    def fix_tron1():
        d.qpos[root_qp:root_qp+7]=HANDOVER
        d.qvel[root_dof:root_dof+6]=0.0
    def set_ctrl(ff_on):
        ctrl=np.asarray(q_arm,float)
        if ff_on:
            mujoco.mj_forward(m,d)
            extra=np.array([float(d.qfrc_bias[dof])/(kp if kp else 1.0) for dof,kp in zip(arm_dof,kp_arm)])
            ctrl=ctrl+extra
        for ai,qv in zip(arm_act,ctrl):
            d.ctrl[ai]=qv
        d.ctrl[ai_lf]=0.07; d.ctrl[ai_rf]=0.07

    d.qpos[root_qp:root_qp+7]=HANDOVER; d.qvel[:]=0
    for adr,qv in zip(arm_q,q_arm):
        d.qpos[adr]=qv
    d.qpos[lq]=0.07; d.qpos[rq]=0.07
    m.opt.gravity[:]=0.0
    mujoco.mj_forward(m,d)
    for _ in range(600):
        fix_tron1(); set_ctrl(False); mujoco.mj_step(m,d)
    lc,rc,dl,dr=contact()
    print("  G13 L=%s R=%s distL=%s distR=%s lq=%.5f rq=%.5f"%(lc,rc,dl,dr,d.qpos[lq],d.qpos[rq]))
    if not (lc and rc):
        return {"tag":tag,"g13":False,"pass":False,"distL":dl,"distR":dr}

    m.opt.gravity[:]=g0
    b_start=d.xpos[bb].copy()
    contact_loss=-1; hold_fail=False
    for step in range(2500):
        fix_tron1(); set_ctrl(True); mujoco.mj_step(m,d)
        lc2,rc2,dl2,dr2=contact()
        bp=d.xpos[bb]
        if (not np.isfinite(bp).all()) or float(np.max(np.abs(d.qvel)))>200:
            hold_fail=True
            if contact_loss<0: contact_loss=step+1
            break
        if (not lc2 or not rc2) and contact_loss<0:
            contact_loss=step+1
            print("  LOSS t=%.4fs L=%s R=%s dL=%s dR=%s"%( (step+1)*0.002,lc2,rc2,dl2,dr2))
        if (step+1)%500==0:
            print("  t=%.2fs L=%s R=%s loss=%s dL=%s dR=%s"%(
                (step+1)*0.002,lc2,rc2,contact_loss,dl2,dr2))
    b_end=d.xpos[bb].copy(); lc_h,rc_h,dl_h,dr_h=contact()
    drift=float(np.linalg.norm(b_end-b_start))
    g14=np.isfinite(b_end).all() and not hold_fail and contact_loss<0 and lc_h and rc_h and drift<0.025
    print("  G14=%s loss=%s drift=%.6f"%( "PASS" if g14 else "FAIL", contact_loss, drift))
    return {"tag":tag,"g13":True,"pass":bool(g14),"contact_loss":int(contact_loss),
            "drift":drift,"lc":bool(lc_h),"rc":bool(rc_h),"distL":dl,"distR":dr,
            "q": q_arm.tolist()}

print("G14 PAN OFFSET TO DEEPEN RIGHT")
print("XML MODIFIED = NO")
results=[]
winner=None
# also wrist_3 as separate one-joint scans (still one joint family at a time)
for dp in [0.0, -0.01, 0.01, -0.02, 0.02, -0.03, 0.03, -0.04, 0.04, -0.05, 0.05]:
    q=Q0.copy(); q[0]+=dp
    tag="pan%+.3f"%dp
    print("\n==== %s ===="%tag)
    r=official(q, tag)
    results.append(r)
    if r.get("pass"):
        winner=r
        break

if winner is None:
    for dw in [-0.05, 0.05, -0.10, 0.10, -0.15, 0.15]:
        q=Q0.copy(); q[5]+=dw
        tag="w3%+.2f"%dw
        print("\n==== %s ===="%tag)
        r=official(q, tag)
        results.append(r)
        if r.get("pass"):
            winner=r
            break

if winner is None:
    for dw2 in [-0.05, 0.05, -0.10, 0.10]:
        q=Q0.copy(); q[4]+=dw2
        tag="w2%+.2f"%dw2
        print("\n==== %s ===="%tag)
        r=official(q, tag)
        results.append(r)
        if r.get("pass"):
            winner=r
            break

print("\n==== SUMMARY ====")
lines=["G14 PAN DEEPEN RIGHT","XML MODIFIED = NO"]
for r in results:
    msg="%s G13=%s G14=%s loss=%s distL=%s distR=%s"%(
        r["tag"], r.get("g13"), r.get("pass"), r.get("contact_loss"), r.get("distL"), r.get("distR"))
    print(msg); lines.append(msg)
lines.append("G14 = %s"%("PASS" if winner else "FAIL"))
if winner:
    lines.append("WINNER %s Q=%s"% (winner["tag"], winner["q"]))
open(OUT,"w",encoding="utf-8").write("\n".join(lines)+"\n")
print("Wrote", OUT)
sys.exit(0 if winner else 1)
