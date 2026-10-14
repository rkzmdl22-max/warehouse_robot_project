"""
G14 official 5s after verified cause.

CAUSE (measured, Protocol A vs B identical at t=0.032s):
  Left loss is finger OPENING, not gravity.
  V4: start q=0.02, ctrl=-0.0105, lqv~-1.18, lq crosses below pinch, lc drops.
  Zero-G hold loses left at the same step.

MINIMAL one-variable correction:
  Use kiro G13 (qpos=0.07, ctrl=0.0, 600 zero-G steps) which already PASSes G13.
  During G14 hold, set finger ctrl to 0.07 (hold closed at joint limit)
  instead of 0.0 (which pulls fingers toward open/uncross).
  Optional second isolated test: same + arm gravity FF.

XML MODIFIED = NO. No weld. Full 5s. G13 first.
"""
import os, sys, csv
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
LOGDIR = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs"
OUT = os.path.join(LOGDIR, "G14_HOLD_CLOSED_RESULT.txt")
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
ARM = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
       "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
ACT = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
       "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
       "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}


def run(hold_lf, hold_rf, use_ff, tag):
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
        return lc, rc, dl, dr

    def fix_tron1():
        d.qpos[root_qp:root_qp+7] = HANDOVER
        d.qvel[root_dof:root_dof+6] = 0.0

    def set_arm_fingers(q_arm, lf, rf, ff):
        ctrl = np.asarray(q_arm, float)
        if ff:
            mujoco.mj_forward(m, d)
            extra = np.array([float(d.qfrc_bias[dof])/(kp if kp else 1.0)
                              for dof, kp in zip(arm_dof, kp_arm)])
            ctrl = ctrl + extra
        for ai, qv in zip(arm_act, ctrl):
            d.ctrl[ai] = qv
        d.ctrl[ai_lf] = lf
        d.ctrl[ai_rf] = rf

    # G13 kiro path, TRON1 fixed (G14 constraint)
    d.qpos[root_qp:root_qp+7] = HANDOVER
    d.qvel[:] = 0.0
    for adr, qv in zip(arm_q, Q0):
        d.qpos[adr] = qv
    d.qpos[lq] = 0.07
    d.qpos[rq] = 0.07
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d)
    for _ in range(600):
        fix_tron1()
        set_arm_fingers(Q0, 0.0, 0.0, False)
        mujoco.mj_step(m, d)
    lc, rc, dl, dr = contact()
    print("  G13 L=%s R=%s lq=%.5f rq=%.5f distL=%s distR=%s" % (lc, rc, d.qpos[lq], d.qpos[rq], dl, dr))
    if not (lc and rc):
        return {"tag": tag, "g13": False, "pass": False}

    # If hold targets are None, use measured qpos (hold-in-place)
    lf = float(d.qpos[lq]) if hold_lf is None else hold_lf
    rf = float(d.qpos[rq]) if hold_rf is None else hold_rf
    print("  hold finger ctrl L=%.5f R=%.5f ff=%s" % (lf, rf, use_ff))

    m.opt.gravity[:] = g0
    b_start = d.xpos[bb].copy()
    contact_loss = -1
    hold_fail = False
    loss_note = ""
    for step in range(2500):
        fix_tron1()
        set_arm_fingers(Q0, lf, rf, use_ff)
        mujoco.mj_step(m, d)
        lc2, rc2, dl2, dr2 = contact()
        bp = d.xpos[bb]
        if (not np.isfinite(bp).all()) or float(np.max(np.abs(d.qvel))) > 200:
            hold_fail = True
            if contact_loss < 0:
                contact_loss = step+1
            break
        if (not lc2 or not rc2) and contact_loss < 0:
            contact_loss = step+1
            loss_note = "t=%.4fs L=%s R=%s lq=%.5f rq=%.5f dL=%s dR=%s" % (
                (step+1)*m.opt.timestep, lc2, rc2, d.qpos[lq], d.qpos[rq], dl2, dr2)
            print("  LOSS", loss_note)
        if (step+1) % 500 == 0:
            print("  t=%.2fs L=%s R=%s lq=%.5f rq=%.5f loss=%s dL=%s dR=%s" % (
                (step+1)*m.opt.timestep, lc2, rc2, d.qpos[lq], d.qpos[rq],
                contact_loss, dl2, dr2))
    b_end = d.xpos[bb].copy()
    lc_h, rc_h, dl_h, dr_h = contact()
    drift = float(np.linalg.norm(b_end-b_start))
    g14 = np.isfinite(b_end).all() and not hold_fail and contact_loss < 0 and lc_h and rc_h and drift < 0.025
    print("  G14=%s loss=%s drift=%.6f lc=%s rc=%s" % ("PASS" if g14 else "FAIL", contact_loss, drift, lc_h, rc_h))
    return {"tag": tag, "g13": True, "pass": bool(g14), "contact_loss": int(contact_loss),
            "drift": drift, "lc": bool(lc_h), "rc": bool(rc_h), "hold_fail": bool(hold_fail),
            "loss_note": loss_note, "lf": lf, "rf": rf, "ff": use_ff,
            "end_q": [float(d.qpos[lq]), float(d.qpos[rq])]}


print("G14 HOLD CLOSED AFTER KIRO G13")
print("XML MODIFIED = NO")

cases = [
    # isolated: finger hold-closed only
    (0.07, 0.07, False, "hold_ctrl=0.07 noFF"),
    # isolated: measured qpos hold only
    (None, None, False, "hold_measured_q noFF"),
    # then add FF only if needed / as second variable
    (0.07, 0.07, True, "hold_ctrl=0.07 FF"),
    (None, None, True, "hold_measured_q FF"),
    # G13-style ctrl=0.0 with FF (baseline already failed; keep for contrast)
    (0.0, 0.0, True, "hold_ctrl=0.0 FF baseline"),
]

results = []
winner = None
for args in cases:
    print("\n==== %s ====" % args[-1])
    r = run(*args)
    results.append(r)
    if r.get("pass") and winner is None:
        winner = r
        break

# G13 regression with kiro path (fresh model, zero-G, 600 steps)
print("\n==== G13 REGRESSION (kiro path) ====")
reg = run(0.07, 0.07, False, "regression_only_check_g13")
print("G13 regression", "PASS" if reg.get("g13") else "FAIL")

print("\n==== SUMMARY ====")
lines = ["G14 HOLD CLOSED", "XML MODIFIED = NO"]
for r in results:
    msg = "%s G13=%s G14=%s loss=%s drift=%s" % (
        r["tag"], r.get("g13"), r.get("pass"), r.get("contact_loss"), r.get("drift"))
    print(msg)
    lines.append(msg)
    if r.get("loss_note"):
        lines.append("  " + r["loss_note"])
lines.append("G14 = %s" % ("PASS" if winner else "FAIL"))
if winner:
    lines.append("WINNER %s lf=%s rf=%s ff=%s" % (winner["tag"], winner["lf"], winner["rf"], winner["ff"]))
with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines)+"\n")
print("Wrote", OUT)
sys.exit(0 if winner else 1)
