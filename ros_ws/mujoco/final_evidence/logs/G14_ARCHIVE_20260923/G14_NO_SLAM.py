"""
G14: initialize at a settled pinch (no finger slam), arm gravity FF, kp_arm default.
Official 5s, gravity ON from start. XML MODIFIED = NO.
"""
import json, sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_NO_SLAM_RESULT.txt"
print("G14 NO-SLAM SETTLED INIT")
print("XML MODIFIED = NO")

Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
           "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
           "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}

def run(lf, rf, sl, sr, fkp, fkv, tag, noslip=True, margin=0.0):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    g0 = m.opt.gravity.copy()

    def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    def qadr(n): return m.jnt_qposadr[jid(n)]

    tron1=bid("base_Link")
    root_qp=m.jnt_qposadr[m.body_jntadr[tron1]]
    root_dof=m.jnt_dofadr[m.body_jntadr[tron1]]
    lq=qadr("left_finger_joint"); rq=qadr("right_finger_joint")
    bb=bid("carry_bottle")
    bgeom=gid("carry_bottle_collision"); lgeom=gid("left_finger_geom"); rgeom=gid("right_finger_geom")
    ai_lf=aid("act_left_finger"); ai_rf=aid("act_right_finger")
    arm_act=[aid(act_map[jn]) for jn in arm_joints]
    arm_dof=[m.jnt_dofadr[jid(jn)] for jn in arm_joints]
    arm_qadr=[qadr(jn) for jn in arm_joints]
    kp_arm=np.array([float(m.actuator_gainprm[ai][0]) for ai in arm_act])

    if noslip:
        m.opt.noslip_iterations=10
        m.opt.impratio=10
    if margin:
        for g in (bgeom,lgeom,rgeom):
            m.geom_margin[g]=margin
    for ai in (ai_lf,ai_rf):
        m.actuator_gainprm[ai][0]=fkp
        m.actuator_biasprm[ai][1]=-fkp
        m.actuator_biasprm[ai][2]=-fkv

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
        ctrl=Q0+extra
        for ai,qv in zip(arm_act,ctrl):
            d.ctrl[ai]=qv
        d.ctrl[ai_lf]=lf; d.ctrl[ai_rf]=rf

    m.opt.gravity[:]=g0
    d.qpos[root_qp:root_qp+7]=HANDOVER; d.qvel[:]=0
    for adr,qv in zip(arm_qadr,Q0):
        d.qpos[adr]=qv
    d.qpos[lq]=sl; d.qpos[rq]=sr
    mujoco.mj_forward(m,d)
    lc0,rc0,dist0=contact()
    print("  init contact L=%s R=%s dist=%s"%(lc0,rc0,dist0))

    g13=-1
    for step in range(500):
        fix_tron1(); set_ctrl(); mujoco.mj_step(m,d)
        lc,rc,dist=contact()
        if lc and rc:
            g13=step+1
            print("  G13 at step %d dist=%s lq=%.5f rq=%.5f"%(g13,dist,d.qpos[lq],d.qpos[rq]))
            break
    if g13<0:
        print("  G13 FAIL")
        return {"tag":tag,"g13":False,"pass":False}

    b_start=d.xpos[bb].copy()
    contact_loss=-1; hold_fail=False
    for step in range(2500):
        fix_tron1(); set_ctrl(); mujoco.mj_step(m,d)
        lc,rc,dist=contact()
        bp=d.xpos[bb]; mv=float(np.max(np.abs(d.qvel)))
        if (not np.isfinite(bp).all()) or mv>200:
            hold_fail=True
            if contact_loss<0: contact_loss=step+1
            break
        if (not lc or not rc) and contact_loss<0:
            contact_loss=step+1
            print("  LOSS step=%d t=%.4fs L=%s R=%s dist=%s lq=%.5f rq=%.5f"%(
                step+1,(step+1)*0.002,lc,rc,dist,d.qpos[lq],d.qpos[rq]))
        if (step+1)%500==0:
            qerr=float(np.linalg.norm(np.array([d.qpos[a] for a in arm_qadr])-Q0))
            print("  t=%.2fs L=%s R=%s qerr=%.4f loss=%s dist=%s lq=%.5f rq=%.5f"%(
                (step+1)*0.002,lc,rc,qerr,contact_loss,dist,d.qpos[lq],d.qpos[rq]))
    b_end=d.xpos[bb].copy(); lc_h,rc_h,dist_h=contact()
    drift=float(np.linalg.norm(b_end-b_start))
    g14=np.isfinite(b_end).all() and not hold_fail and contact_loss<0 and lc_h and rc_h and drift<0.025
    print("  G14=%s loss=%s drift=%.6f"%( "PASS" if g14 else "FAIL", contact_loss, drift))
    return {"tag":tag,"g13":True,"g13_step":g13,"pass":bool(g14),"contact_loss":int(contact_loss),
            "drift":drift,"lc":bool(lc_h),"rc":bool(rc_h),"dist_end":dist_h,
            "finger_end":[float(d.qpos[lq]),float(d.qpos[rq])]}

grid=[]
# start == command, light/medium pinch, no slam
for qf in (-0.0105, -0.008, -0.006, -0.004, -0.002, 0.0, 0.002, 0.005):
    grid.append((qf,qf,qf,qf, 400.0, 80.0, "sym qf=%.4f kp400 kv80"%qf, True, 0.0))
# asymmetric: left more closed
for lf in (0.0, 0.002, 0.005, 0.008):
    grid.append((lf, -0.0105, lf, -0.0105, 400.0, 80.0, "L=%.3f R=-0.0105"%lf, True, 0.0))
# with 1mm margin
grid.append((0.0, -0.0105, 0.0, -0.0105, 800.0, 120.0, "L0 R-0.0105 m1mm", True, 0.001))
grid.append((-0.006, -0.006, -0.006, -0.006, 800.0, 120.0, "sym-0.006 m1mm", True, 0.001))

results=[]; winner=None
for args in grid:
    print("\n==== %s ===="%args[6])
    r=run(*args); results.append(r)
    if r.get("pass"):
        winner=r; break
print("\n==== SUMMARY ====")
for r in results:
    print("%-36s G13=%s G14=%s loss=%s"%(r.get("tag"), r.get("g13"), r.get("pass"), r.get("contact_loss")))
with open(OUT,"w",encoding="utf-8") as f:
    f.write("G14 NO SLAM\nXML MODIFIED = NO\nG14=%s\n"%("PASS" if winner else "FAIL"))
    if winner: f.write("WINNER %s\n"%winner["tag"])
    f.write(json.dumps(results, indent=2, default=str))
print("Wrote", OUT)
sys.exit(0 if winner else 1)
