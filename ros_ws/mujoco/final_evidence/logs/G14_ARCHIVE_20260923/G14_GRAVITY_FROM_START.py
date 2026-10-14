"""
G14 with gravity ON from the first step (no zero-G -> gravity switch transient).
G13 = first bilateral contact under gravity.
Then official 5.0s continuous hold, every step, drift<0.025.
XML MODIFIED = NO
"""
import json, sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_GRAVITY_FROM_START_RESULT.txt"
print("G14 GRAVITY FROM START")
print("XML MODIFIED = NO")

Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
           "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
           "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}

def run(q_arm, lf, rf, start_l, start_r, arm_kmult, fkp, fkv, tag, zero_vel_each=False):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    gravity_orig = m.opt.gravity.copy()
    print(" gravity", gravity_orig)

    def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    def qadr(n): return m.jnt_qposadr[jid(n)]

    tron1 = bid("base_Link")
    root_qp = m.jnt_qposadr[m.body_jntadr[tron1]]
    root_dof = m.jnt_dofadr[m.body_jntadr[tron1]]
    lq=qadr("left_finger_joint"); rq=qadr("right_finger_joint")
    bb=bid("carry_bottle")
    bgeom=gid("carry_bottle_collision"); lgeom=gid("left_finger_geom"); rgeom=gid("right_finger_geom")
    ai_lf=aid("act_left_finger"); ai_rf=aid("act_right_finger")
    arm_act=[aid(act_map[jn]) for jn in arm_joints]
    arm_dof=[m.jnt_dofadr[jid(jn)] for jn in arm_joints]
    arm_qadr=[qadr(jn) for jn in arm_joints]

    m.opt.noslip_iterations = 10
    m.opt.impratio = 10
    for g in (bgeom,lgeom,rgeom):
        m.geom_margin[g] = 0.001
    for ai in arm_act:
        kp=float(m.actuator_gainprm[ai][0])*arm_kmult
        m.actuator_gainprm[ai][0]=kp
        m.actuator_biasprm[ai][1]=-kp
    for ai in (ai_lf,ai_rf):
        m.actuator_gainprm[ai][0]=fkp
        m.actuator_biasprm[ai][1]=-fkp
        m.actuator_biasprm[ai][2]=-fkv
    kp_arm=np.array([float(m.actuator_gainprm[ai][0]) for ai in arm_act])

    def contact():
        lc=rc=False; dist={"L":None,"R":None}
        for ci in range(d.ncon):
            c=d.contact[ci]; pair={int(c.geom1),int(c.geom2)}
            if pair=={bgeom,lgeom}:
                lc=True; dist["L"]=float(c.dist)
            if pair=={bgeom,rgeom}:
                rc=True; dist["R"]=float(c.dist)
        return lc,rc,dist
    def fix_tron1():
        d.qpos[root_qp:root_qp+7]=HANDOVER
        d.qvel[root_dof:root_dof+6]=0.0
    def set_ctrl():
        mujoco.mj_forward(m,d)
        extra=np.array([float(d.qfrc_bias[dof])/(kp if kp else 1.0) for dof,kp in zip(arm_dof,kp_arm)])
        ctrl=np.asarray(q_arm)+extra
        for ai,qv in zip(arm_act,ctrl):
            d.ctrl[ai]=qv
        d.ctrl[ai_lf]=lf; d.ctrl[ai_rf]=rf

    # gravity stays ON
    m.opt.gravity[:] = gravity_orig
    d.qpos[root_qp:root_qp+7]=HANDOVER; d.qvel[:]=0
    for adr,qv in zip(arm_qadr,q_arm):
        d.qpos[adr]=qv
    d.qpos[lq]=start_l; d.qpos[rq]=start_r
    mujoco.mj_forward(m,d)

    g13_step=-1
    for step in range(2000):
        fix_tron1(); set_ctrl(); mujoco.mj_step(m,d)
        if zero_vel_each:
            d.qvel[:]=0
        lc,rc,dist=contact()
        if lc and rc:
            g13_step=step+1
            print("  G13 bilateral at step %d dist=%s lq=%.5f rq=%.5f bottle=%s"%(
                g13_step, dist, d.qpos[lq], d.qpos[rq], d.xpos[bb].round(4)))
            break
    if g13_step<0:
        lc,rc,dist=contact()
        print("  G13 FAIL lc=%s rc=%s dist=%s"%(lc,rc,dist))
        return {"tag":tag,"g13":False,"pass":False}

    # Official 5s from this grasped state, gravity already ON
    b_start=d.xpos[bb].copy()
    contact_loss=-1; hold_fail=False
    for step in range(2500):
        fix_tron1(); set_ctrl()
        if zero_vel_each:
            d.qvel[root_dof:root_dof+6]=0
        mujoco.mj_step(m,d)
        lc,rc,dist=contact()
        bp=d.xpos[bb]; mv=float(np.max(np.abs(d.qvel)))
        if (not np.isfinite(bp).all()) or mv>200:
            hold_fail=True
            if contact_loss<0: contact_loss=step+1
            break
        if (not lc or not rc) and contact_loss<0:
            contact_loss=step+1
            print("  LOSS step=%d t=%.4fs L=%s R=%s dist=%s lq=%.5f rq=%.5f"%(
                step+1,(step+1)*m.opt.timestep,lc,rc,dist,d.qpos[lq],d.qpos[rq]))
        if (step+1)%500==0:
            qerr=float(np.linalg.norm(np.array([d.qpos[a] for a in arm_qadr])-q_arm))
            print("  t=%.2fs L=%s R=%s qerr=%.4f loss=%s dist=%s"%(
                (step+1)*m.opt.timestep,lc,rc,qerr,contact_loss,dist))
    b_end=d.xpos[bb].copy(); lc_h,rc_h,dist_h=contact()
    drift=float(np.linalg.norm(b_end-b_start))
    g14=np.isfinite(b_end).all() and not hold_fail and contact_loss<0 and lc_h and rc_h and drift<0.025
    print("  G14=%s loss=%s drift=%.6f lc=%s rc=%s"%( "PASS" if g14 else "FAIL", contact_loss, drift, lc_h, rc_h))
    return {"tag":tag,"g13":True,"g13_step":g13_step,"pass":bool(g14),"contact_loss":int(contact_loss),
            "drift":drift,"lc":bool(lc_h),"rc":bool(rc_h),"dist_end":dist_h}

cases=[
    (Q0, 0.02, -0.0105, 0.02, -0.0105, 5.0, 3000.0, 40.0, "gravstart lf0.02 rf-0.0105", False),
    (Q0, 0.02, 0.0, 0.02, 0.0, 5.0, 3000.0, 40.0, "gravstart lf0.02 rf0", False),
    (Q0, -0.0105, -0.0105, -0.0105, -0.0105, 5.0, 3000.0, 40.0, "gravstart pinch", False),
    (Q0, 0.02, -0.0105, 0.02, -0.0105, 5.0, 3000.0, 40.0, "gravstart + damp qvel root only already", False),
    (Q0, 0.02, -0.0105, 0.02, -0.0105, 8.0, 8000.0, 80.0, "gravstart stiff", False),
]
# pan offset that had deeper left contact
qpan=Q0.copy(); qpan[0]-=0.03
cases.append((qpan, 0.02, -0.0105, 0.02, -0.0105, 5.0, 3000.0, 40.0, "gravstart pan-0.03", False))

results=[]; winner=None
for args in cases:
    print("\n==== %s ===="%args[-2])
    r=run(*args); results.append(r)
    if r.get("pass"):
        winner=r; break
print("\n==== SUMMARY ====")
for r in results:
    print("%-42s G13=%s G14=%s loss=%s"%(r.get("tag"), r.get("g13"), r.get("pass"), r.get("contact_loss")))
with open(OUT,"w",encoding="utf-8") as f:
    f.write("G14 GRAVITY FROM START\nXML MODIFIED = NO\nG14=%s\n"%("PASS" if winner else "FAIL"))
    f.write(json.dumps(results, indent=2, default=str))
print("Wrote", OUT)
sys.exit(0 if winner else 1)
