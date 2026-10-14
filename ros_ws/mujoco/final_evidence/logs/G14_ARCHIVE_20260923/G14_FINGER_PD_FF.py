"""
G14: runtime finger PD (not XML) + arm gravity feedforward + official 5s.

Evidence: pinch at qf=-0.0105 is only ~0.4mm; finger kp=100 loses the left
finger to contact repulsion. Official G13 kiro also ended at q~0.07 (deep
overlap) after 600 steps. This test only changes compiled actuator PD at
runtime (same pattern as G14_STIFFNESS_DIAG). XML file is not written.
"""
import json
import sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_FINGER_PD_FF_RESULT.txt"

print("G14 FINGER PD + ARM GRAVITY FF")
print("XML MODIFIED = NO")

Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
arm_joints = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
act_map = {
    "shoulder_pan_joint": "act_shoulder_pan",
    "shoulder_lift_joint": "act_shoulder_lift",
    "elbow_joint": "act_elbow",
    "wrist_1_joint": "act_wrist_1",
    "wrist_2_joint": "act_wrist_2",
    "wrist_3_joint": "act_wrist_3",
}


def run(finger_kp, finger_kv, lf, rf, start_q, arm_ff, tag):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    gravity_orig = m.opt.gravity.copy()

    def jid(n):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)

    def aid(n):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)

    def gid(n):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)

    def bid(n):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)

    def qadr(n):
        return m.jnt_qposadr[jid(n)]

    tron1 = bid("base_Link")
    root_qp = m.jnt_qposadr[m.body_jntadr[tron1]]
    root_dof = m.jnt_dofadr[m.body_jntadr[tron1]]
    lq = qadr("left_finger_joint")
    rq = qadr("right_finger_joint")
    bb = bid("carry_bottle")
    bgeom = gid("carry_bottle_collision")
    lgeom = gid("left_finger_geom")
    rgeom = gid("right_finger_geom")
    ai_lf = aid("act_left_finger")
    ai_rf = aid("act_right_finger")
    arm_act = [aid(act_map[jn]) for jn in arm_joints]
    arm_dof = [m.jnt_dofadr[jid(jn)] for jn in arm_joints]
    arm_qadr = [qadr(jn) for jn in arm_joints]
    kp_arm = np.array([float(m.actuator_gainprm[ai][0]) for ai in arm_act])

    # Runtime finger PD (compiled model only)
    for ai in (ai_lf, ai_rf):
        m.actuator_gainprm[ai][0] = finger_kp
        m.actuator_biasprm[ai][0] = 0.0
        m.actuator_biasprm[ai][1] = -finger_kp
        m.actuator_biasprm[ai][2] = -finger_kv

    def contact():
        lc = rc = False
        dist = {"L": None, "R": None}
        extra = []
        for ci in range(d.ncon):
            c = d.contact[ci]
            a, b = int(c.geom1), int(c.geom2)
            pair = {a, b}
            if pair == {bgeom, lgeom}:
                lc = True
                dist["L"] = float(c.dist)
            elif pair == {bgeom, rgeom}:
                rc = True
                dist["R"] = float(c.dist)
            elif lgeom in pair or rgeom in pair:
                na = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, a)
                nb = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, b)
                extra.append((na, nb, float(c.dist)))
        return lc, rc, dist, extra

    def fix_tron1():
        d.qpos[root_qp:root_qp + 7] = HANDOVER
        d.qvel[root_dof:root_dof + 6] = 0.0

    def set_ctrl(q_arm, use_ff):
        ctrl_arm = np.asarray(q_arm, float)
        extra = np.zeros(6)
        if use_ff:
            mujoco.mj_forward(m, d)
            extra = np.array([float(d.qfrc_bias[dof]) / (kp if kp else 1.0)
                              for dof, kp in zip(arm_dof, kp_arm)])
            ctrl_arm = ctrl_arm + extra
        for ai, qv in zip(arm_act, ctrl_arm):
            d.ctrl[ai] = qv
        d.ctrl[ai_lf] = lf
        d.ctrl[ai_rf] = rf
        return extra

    d.qpos[root_qp:root_qp + 7] = HANDOVER
    d.qvel[:] = 0.0
    for adr, qv in zip(arm_qadr, Q0):
        d.qpos[adr] = qv
    d.qpos[lq] = start_q
    d.qpos[rq] = start_q
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d)

    g13_both_end = False
    for step in range(800):
        fix_tron1()
        set_ctrl(Q0, False)
        mujoco.mj_step(m, d)
        lc, rc, dist, extra = contact()
        if step == 799:
            g13_both_end = bool(lc and rc)
            print("  G13 end grav=%s lc=%s rc=%s lq=%.5f rq=%.5f dist=%s extra=%s" % (
                m.opt.gravity, lc, rc, d.qpos[lq], d.qpos[rq], dist, extra[:4]))

    if not g13_both_end:
        print("  %s G13 FAIL" % tag)
        return {"tag": tag, "g13": False, "pass": False}

    m.opt.gravity[:] = gravity_orig
    b_start = d.xpos[bb].copy()
    contact_loss = -1
    hold_fail = False
    loss_detail = None
    for step in range(2500):
        fix_tron1()
        set_ctrl(Q0, arm_ff)
        mujoco.mj_step(m, d)
        lc, rc, dist, extra = contact()
        bp = d.xpos[bb]
        mv = float(np.max(np.abs(d.qvel)))
        if not np.isfinite(bp).all() or mv > 200:
            hold_fail = True
            if contact_loss < 0:
                contact_loss = step + 1
            break
        if (not lc or not rc) and contact_loss < 0:
            contact_loss = step + 1
            loss_detail = {
                "step": step + 1, "lc": lc, "rc": rc,
                "lq": float(d.qpos[lq]), "rq": float(d.qpos[rq]),
                "dist": dist, "extra": extra[:8],
                "arm_q": [float(d.qpos[a]) for a in arm_qadr],
            }
            print("  LOSS step=%d L=%s R=%s lq=%.5f rq=%.5f dist=%s extra=%s" % (
                step + 1, lc, rc, d.qpos[lq], d.qpos[rq], dist, extra[:6]))
        if (step + 1) % 500 == 0:
            qerr = float(np.linalg.norm(np.array([d.qpos[a] for a in arm_qadr]) - Q0))
            print("  t=%.2fs L=%s R=%s lq=%.5f rq=%.5f qerr=%.4f loss=%s dist=%s" % (
                (step + 1) * m.opt.timestep, lc, rc, d.qpos[lq], d.qpos[rq],
                qerr, contact_loss, dist))

    b_end = d.xpos[bb].copy()
    lc_h, rc_h, dist_h, extra_h = contact()
    drift = float(np.linalg.norm(b_end - b_start))
    g14 = (np.isfinite(b_end).all() and not hold_fail and contact_loss < 0
           and lc_h and rc_h and drift < 0.025)
    print("  G14=%s loss=%s drift=%.6f lc=%s rc=%s" % (
        "PASS" if g14 else "FAIL", contact_loss, drift, lc_h, rc_h))
    return {
        "tag": tag, "g13": True, "pass": bool(g14),
        "contact_loss": int(contact_loss), "drift": drift,
        "lc": bool(lc_h), "rc": bool(rc_h), "hold_fail": bool(hold_fail),
        "finger_end": [float(d.qpos[lq]), float(d.qpos[rq])],
        "dist_end": dist_h, "loss_detail": loss_detail,
        "finger_kp": finger_kp, "finger_kv": finger_kv, "lf": lf, "rf": rf,
        "arm_ff": arm_ff,
    }


grid = []
for kp in (100, 500, 2000, 10000, 50000):
    for kv in (0.0, 20.0, 100.0):
        for qf in (-0.0105, 0.0):
            grid.append((kp, kv, qf, qf, qf, True, "kp=%d kv=%.0f qf=%.4f FF" % (kp, kv, qf)))

results = []
winner = None
for args in grid:
    print("\n==== %s ====" % args[-1])
    r = run(*args)
    results.append(r)
    if r.get("pass"):
        winner = r
        break

print("\n==== SUMMARY ====")
for r in results:
    print("%-40s G13=%s G14=%s loss=%s" % (r["tag"], r.get("g13"), r.get("pass"), r.get("contact_loss")))

with open(OUT, "w", encoding="utf-8") as f:
    f.write("G14 FINGER PD + ARM GRAVITY FF\nXML MODIFIED = NO\n")
    f.write("G14 = %s\n" % ("PASS" if winner else "FAIL"))
    if winner:
        f.write("WINNER %s\n" % winner["tag"])
        f.write("finger_kp=%s kv=%s ctrl=%s %s arm_ff=%s\n" % (
            winner["finger_kp"], winner["finger_kv"], winner["lf"], winner["rf"], winner["arm_ff"]))
    f.write(json.dumps([{k: v for k, v in r.items() if k != "loss_detail"} | {"loss_detail": r.get("loss_detail")}
                        for r in results], indent=2, default=str))
print("Wrote", OUT)
sys.exit(0 if winner else 1)
