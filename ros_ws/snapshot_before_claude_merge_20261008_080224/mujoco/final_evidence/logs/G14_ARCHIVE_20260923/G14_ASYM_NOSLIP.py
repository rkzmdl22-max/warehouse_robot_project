"""
G14: asymmetric pinch + arm PD boost + noslip, official 5s.
XML file not modified. Runtime actuator/opt only.
"""
import json, sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_ASYM_NOSLIP_RESULT.txt"
print("G14 ASYM PINCH + NOSLIP + ARM PD")
print("XML MODIFIED = NO")

Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
           "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
           "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}


def run(q_arm, lf, rf, start_l, start_r, arm_kmult, finger_kp, finger_kv, noslip, impratio, margin, tag):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    gravity_orig = m.opt.gravity.copy()

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
    arm_act = [aid(act_map[jn]) for jn in arm_joints]
    arm_dof = [m.jnt_dofadr[jid(jn)] for jn in arm_joints]
    arm_qadr = [qadr(jn) for jn in arm_joints]

    if noslip:
        m.opt.noslip_iterations = 10
    if impratio is not None:
        m.opt.impratio = impratio
    if margin is not None:
        for g in (bgeom, lgeom, rgeom):
            m.geom_margin[g] = margin

    for ai in arm_act:
        kp = float(m.actuator_gainprm[ai][0]) * arm_kmult
        kv = -m.actuator_biasprm[ai][2] * arm_kmult
        m.actuator_gainprm[ai][0] = kp
        m.actuator_biasprm[ai][1] = -kp
        # keep existing damping scale
        if m.actuator_biasprm[ai][2] != 0:
            m.actuator_biasprm[ai][2] = -abs(m.actuator_biasprm[ai][2]) * arm_kmult
    for ai in (ai_lf, ai_rf):
        m.actuator_gainprm[ai][0] = finger_kp
        m.actuator_biasprm[ai][0] = 0.0
        m.actuator_biasprm[ai][1] = -finger_kp
        m.actuator_biasprm[ai][2] = -finger_kv

    kp_arm = np.array([float(m.actuator_gainprm[ai][0]) for ai in arm_act])

    def contact():
        lc=rc=False; dist={"L":None,"R":None}
        for ci in range(d.ncon):
            c=d.contact[ci]; pair={int(c.geom1),int(c.geom2)}
            if pair=={bgeom,lgeom}:
                lc=True; dist["L"]=float(c.dist)
            if pair=={bgeom,rgeom}:
                rc=True; dist["R"]=float(c.dist)
        return lc, rc, dist

    def fix_tron1():
        d.qpos[root_qp:root_qp+7] = HANDOVER
        d.qvel[root_dof:root_dof+6] = 0.0

    def set_ctrl(use_ff):
        ctrl = np.asarray(q_arm, float)
        if use_ff:
            mujoco.mj_forward(m, d)
            extra = np.array([float(d.qfrc_bias[dof])/(kp if kp else 1.0) for dof,kp in zip(arm_dof,kp_arm)])
            ctrl = ctrl + extra
        for ai,qv in zip(arm_act, ctrl):
            d.ctrl[ai]=qv
        d.ctrl[ai_lf]=lf; d.ctrl[ai_rf]=rf

    d.qpos[root_qp:root_qp+7]=HANDOVER; d.qvel[:]=0.0
    for adr,qv in zip(arm_qadr,q_arm):
        d.qpos[adr]=qv
    d.qpos[lq]=start_l; d.qpos[rq]=start_r
    m.opt.gravity[:]=0.0
    mujoco.mj_forward(m,d)

    streak=0
    for step in range(1000):
        fix_tron1(); set_ctrl(True); mujoco.mj_step(m,d)
        lc,rc,dist=contact()
        streak = streak+1 if (lc and rc) else 0
    lc,rc,dist=contact()
    print("  G13 lc=%s rc=%s streak=%d lq=%.5f rq=%.5f dist=%s"%(lc,rc,streak,d.qpos[lq],d.qpos[rq],dist))
    if not (lc and rc):
        return {"tag":tag,"g13":False,"pass":False}

    m.opt.gravity[:]=gravity_orig
    b_start=d.xpos[bb].copy()
    contact_loss=-1; hold_fail=False
    for step in range(2500):
        fix_tron1(); set_ctrl(True); mujoco.mj_step(m,d)
        lc,rc,dist=contact()
        bp=d.xpos[bb]; mv=float(np.max(np.abs(d.qvel)))
        if (not np.isfinite(bp).all()) or mv>200:
            hold_fail=True
            if contact_loss<0: contact_loss=step+1
            break
        if (not lc or not rc) and contact_loss<0:
            contact_loss=step+1
            print("  LOSS step=%d L=%s R=%s lq=%.5f rq=%.5f dist=%s"%(step+1,lc,rc,d.qpos[lq],d.qpos[rq],dist))
        if (step+1)%500==0:
            qerr=float(np.linalg.norm(np.array([d.qpos[a] for a in arm_qadr])-q_arm))
            print("  t=%.2fs L=%s R=%s qerr=%.4f loss=%s dist=%s lq=%.5f rq=%.5f"%(
                (step+1)*m.opt.timestep,lc,rc,qerr,contact_loss,dist,d.qpos[lq],d.qpos[rq]))
    b_end=d.xpos[bb].copy(); lc_h,rc_h,dist_h=contact()
    drift=float(np.linalg.norm(b_end-b_start))
    g14=np.isfinite(b_end).all() and not hold_fail and contact_loss<0 and lc_h and rc_h and drift<0.025
    print("  G14=%s loss=%s drift=%.6f" % ("PASS" if g14 else "FAIL", contact_loss, drift))
    return {"tag":tag,"g13":True,"pass":bool(g14),"contact_loss":int(contact_loss),
            "drift":drift,"lc":bool(lc_h),"rc":bool(rc_h),"dist_end":dist_h,
            "finger_end":[float(d.qpos[lq]),float(d.qpos[rq])]}

# cases: left more closed (positive) than right
cases=[]
base = dict(arm_kmult=5.0, finger_kp=3000.0, finger_kv=40.0, noslip=True, impratio=10.0, margin=0.001)
finger_pairs = [
    (0.02, -0.0105, 0.02, -0.0105),
    (0.01, 0.0, 0.01, 0.0),
    (0.02, 0.0, 0.02, 0.0),
    (0.03, 0.0, 0.03, 0.0),
    (0.0, 0.0, 0.0, 0.0),
    (0.01, -0.0105, 0.01, -0.0105),
    (0.005, 0.005, 0.005, 0.005),
]
for lf,rf,sl,sr in finger_pairs:
    tag="lf=%.3f rf=%.3f k5 noslip m1mm"% (lf,rf)
    cases.append((Q0,lf,rf,sl,sr, base["arm_kmult"], base["finger_kp"], base["finger_kv"], True, 10.0, 0.001, tag))

# small pan offsets with best-looking pair 0.02/-0.0105
for dp in (-0.03,-0.02,-0.01,0.01,0.02,0.03):
    q=Q0.copy(); q[0]+=dp
    tag="pan%+.3f lf0.02 rf-0.0105" % dp
    cases.append((q, 0.02, -0.0105, 0.02, -0.0105, 5.0, 3000.0, 40.0, True, 10.0, 0.001, tag))
# wrist_3 roll
for dw in (-0.2,-0.1,0.1,0.2):
    q=Q0.copy(); q[5]+=dw
    tag="w3%+.2f lf0.02 rf-0.0105" % dw
    cases.append((q, 0.02, -0.0105, 0.02, -0.0105, 5.0, 3000.0, 40.0, True, 10.0, 0.001, tag))
# margin 2mm
cases.append((Q0, 0.02, 0.0, 0.02, 0.0, 8.0, 8000.0, 80.0, True, 20.0, 0.002, "deep+m2mm"))

results=[]; winner=None
for args in cases:
    print("\n==== %s ====" % args[-1])
    r=run(*args); results.append(r)
    if r.get("pass"):
        winner=r; break

print("\n==== SUMMARY ====")
for r in results:
    print("%-42s G13=%s G14=%s loss=%s"%(r["tag"], r.get("g13"), r.get("pass"), r.get("contact_loss")))
with open(OUT,"w",encoding="utf-8") as f:
    f.write("G14 ASYM NOSLIP\nXML MODIFIED = NO\nG14 = %s\n"%("PASS" if winner else "FAIL"))
    if winner: f.write("WINNER %s\n"%winner["tag"])
    f.write(json.dumps(results, indent=2, default=str))
print("Wrote", OUT)
sys.exit(0 if winner else 1)
