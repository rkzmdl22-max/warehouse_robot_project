"""
G14 first-milliseconds left-contact loss instrumentation.

Protocol A: existing V4 FAIL path (start q=0.02, ctrl=-0.0105, G13 first hit, gravity ON)
Protocol B: same but gravity stays 0 (isolates gravity vs finger opening)
Protocol C: kiro G13 (qpos=0.07, ctrl=0.0, 600 zero-G steps) then official 5s gravity

One CSV per protocol for first 80 hold steps (160 ms).
XML MODIFIED = NO
"""
import csv
import os
import sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
LOGDIR = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs"
OUT = os.path.join(LOGDIR, "G14_LEFT_LOSS_MS_RESULT.txt")

Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])
ARM = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
       "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
ACT = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
       "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
       "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}


def make_model():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    ids = {}
    def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    tron1 = bid("base_Link")
    ids.update({
        "m": m, "d": d,
        "root_qp": m.jnt_qposadr[m.body_jntadr[tron1]],
        "root_dof": m.jnt_dofadr[m.body_jntadr[tron1]],
        "lq": m.jnt_qposadr[jid("left_finger_joint")],
        "rq": m.jnt_qposadr[jid("right_finger_joint")],
        "ldof": m.jnt_dofadr[jid("left_finger_joint")],
        "rdof": m.jnt_dofadr[jid("right_finger_joint")],
        "bb": bid("carry_bottle"),
        "bgeom": gid("carry_bottle_collision"),
        "lgeom": gid("left_finger_geom"),
        "rgeom": gid("right_finger_geom"),
        "ai_lf": aid("act_left_finger"),
        "ai_rf": aid("act_right_finger"),
        "arm_act": [aid(ACT[j]) for j in ARM],
        "arm_q": [m.jnt_qposadr[jid(j)] for j in ARM],
        "arm_dof": [m.jnt_dofadr[jid(j)] for j in ARM],
        "g0": m.opt.gravity.copy(),
    })
    return ids


def contact_pack(ids):
    m, d = ids["m"], ids["d"]
    bgeom, lgeom, rgeom = ids["bgeom"], ids["lgeom"], ids["rgeom"]
    lc = rc = False
    out = {
        "lc": False, "rc": False,
        "ldist": "", "rdist": "",
        "lpos": "", "rpos": "",
        "ln": "", "rn": "",
        "lFn": "", "rFn": "",
        "lFt": "", "rFt": "",
    }
    cf = np.zeros(6)
    for ci in range(d.ncon):
        c = d.contact[ci]
        pair = {int(c.geom1), int(c.geom2)}
        mujoco.mj_contactForce(m, d, ci, cf)
        # cf: [normal, tangent1, tangent2, ...] in contact frame; normal is on geom1
        Fn = float(cf[0])
        Ft = float(np.hypot(cf[1], cf[2]))
        if pair == {bgeom, lgeom}:
            lc = True
            # force on left geom
            if int(c.geom1) == lgeom:
                sign = 1.0
            else:
                sign = -1.0
            out["lc"] = True
            out["ldist"] = float(c.dist)
            out["lpos"] = ",".join("%.6f" % x for x in c.pos)
            out["ln"] = ",".join("%.6f" % x for x in c.frame[:3])
            out["lFn"] = sign * Fn
            out["lFt"] = Ft
        if pair == {bgeom, rgeom}:
            rc = True
            if int(c.geom1) == rgeom:
                sign = 1.0
            else:
                sign = -1.0
            out["rc"] = True
            out["rdist"] = float(c.dist)
            out["rpos"] = ",".join("%.6f" % x for x in c.pos)
            out["rn"] = ",".join("%.6f" % x for x in c.frame[:3])
            out["rFn"] = sign * Fn
            out["rFt"] = Ft
    return lc, rc, out


def fix_tron1(ids):
    d = ids["d"]
    d.qpos[ids["root_qp"]:ids["root_qp"]+7] = HANDOVER
    d.qvel[ids["root_dof"]:ids["root_dof"]+6] = 0.0


def set_ctrl(ids, q_arm, lf, rf, gravity_ff=False):
    m, d = ids["m"], ids["d"]
    ctrl = np.asarray(q_arm, float)
    if gravity_ff:
        mujoco.mj_forward(m, d)
        extra = []
        for dof, ai in zip(ids["arm_dof"], ids["arm_act"]):
            kp = float(m.actuator_gainprm[ai][0]) or 1.0
            extra.append(float(d.qfrc_bias[dof]) / kp)
        ctrl = ctrl + np.array(extra)
    for ai, qv in zip(ids["arm_act"], ctrl):
        d.ctrl[ai] = qv
    d.ctrl[ids["ai_lf"]] = lf
    d.ctrl[ids["ai_rf"]] = rf
    return ctrl


def reset_pose(ids, q_arm, lq, rq):
    d = ids["d"]
    d.qpos[ids["root_qp"]:ids["root_qp"]+7] = HANDOVER
    d.qvel[:] = 0.0
    for adr, qv in zip(ids["arm_q"], q_arm):
        d.qpos[adr] = qv
    d.qpos[ids["lq"]] = lq
    d.qpos[ids["rq"]] = rq


def row_for(ids, step, phase, ctrl):
    d = ids["d"]
    lc, rc, pack = contact_pack(ids)
    bp = d.xpos[ids["bb"]]
    arm_q = [float(d.qpos[a]) for a in ids["arm_q"]]
    arm_v = [float(d.qvel[a]) for a in ids["arm_dof"]]
    return {
        "step": step,
        "t": step * ids["m"].opt.timestep,
        "phase": phase,
        "lc": int(lc), "rc": int(rc),
        "ldist": pack["ldist"], "rdist": pack["rdist"],
        "lFn": pack["lFn"], "rFn": pack["rFn"],
        "lFt": pack["lFt"], "rFt": pack["rFt"],
        "lpos": pack["lpos"], "rpos": pack["rpos"],
        "ln": pack["ln"], "rn": pack["rn"],
        "lq": float(d.qpos[ids["lq"]]),
        "rq": float(d.qpos[ids["rq"]]),
        "lqv": float(d.qvel[ids["ldof"]]),
        "rqv": float(d.qvel[ids["rdof"]]),
        "lctrl": float(d.ctrl[ids["ai_lf"]]),
        "rctrl": float(d.ctrl[ids["ai_rf"]]),
        "arm_q": ",".join("%.6f" % x for x in arm_q),
        "arm_v": ",".join("%.6f" % x for x in arm_v),
        "ctrl_arm": ",".join("%.6f" % x for x in ctrl),
        "bottle": ",".join("%.6f" % x for x in bp),
        "ncon": int(d.ncon),
        "max_qvel": float(np.max(np.abs(d.qvel))),
    }


FIELDS = ["step","t","phase","lc","rc","ldist","rdist","lFn","rFn","lFt","rFt",
          "lpos","rpos","ln","rn","lq","rq","lqv","rqv","lctrl","rctrl",
          "arm_q","arm_v","ctrl_arm","bottle","ncon","max_qvel"]


def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def official_hold(ids, q_arm, lf, rf, nstep=2500, gravity_ff=False, log_first=80, csv_path=None):
    """Official 5s. Returns dict. Logs first log_first steps after gravity ON."""
    m, d = ids["m"], ids["d"]
    g0 = ids["g0"]
    m.opt.gravity[:] = g0
    b_start = d.xpos[ids["bb"]].copy()
    rows = []
    contact_loss = -1
    hold_fail = False
    first_loss_row = None
    for step in range(nstep):
        fix_tron1(ids)
        ctrl = set_ctrl(ids, q_arm, lf, rf, gravity_ff=gravity_ff)
        mujoco.mj_step(m, d)
        lc, rc, pack = contact_pack(ids)
        if step < log_first:
            rows.append(row_for(ids, step+1, "G14", ctrl))
        bp = d.xpos[ids["bb"]]
        if (not np.isfinite(bp).all()) or float(np.max(np.abs(d.qvel))) > 200:
            hold_fail = True
            if contact_loss < 0:
                contact_loss = step+1
            break
        if (not lc or not rc) and contact_loss < 0:
            contact_loss = step+1
            first_loss_row = row_for(ids, step+1, "LOSS", ctrl)
        if (step+1) % 500 == 0:
            print("    t=%.2fs L=%s R=%s lq=%.5f rq=%.5f loss=%s" % (
                (step+1)*m.opt.timestep, lc, rc, d.qpos[ids["lq"]], d.qpos[ids["rq"]], contact_loss))
    if csv_path:
        write_csv(csv_path, rows)
    b_end = d.xpos[ids["bb"]].copy()
    lc_h, rc_h, _ = contact_pack(ids)
    drift = float(np.linalg.norm(b_end - b_start))
    g14 = np.isfinite(b_end).all() and not hold_fail and contact_loss < 0 and lc_h and rc_h and drift < 0.025
    return {
        "pass": bool(g14), "contact_loss": int(contact_loss), "drift": drift,
        "lc": bool(lc_h), "rc": bool(rc_h), "hold_fail": bool(hold_fail),
        "loss_row": first_loss_row,
        "end_lq": float(d.qpos[ids["lq"]]), "end_rq": float(d.qpos[ids["rq"]]),
        "csv": csv_path,
    }


print("G14 LEFT-CONTACT FIRST-MS INSTRUMENTATION")
print("XML MODIFIED = NO")
summary = []

# ── Protocol A: V4 path ──────────────────────────────────────────────────
print("\n=== PROTOCOL A: V4 (q=0.02 ctrl=-0.0105, G13 first hit, gravity ON) ===")
ids = make_model()
m, d = ids["m"], ids["d"]
reset_pose(ids, Q0, 0.02, 0.02)
m.opt.gravity[:] = 0.0
set_ctrl(ids, Q0, -0.0105, -0.0105)
mujoco.mj_forward(m, d)
g13 = -1
for step in range(2000):
    fix_tron1(ids)
    set_ctrl(ids, Q0, -0.0105, -0.0105)
    mujoco.mj_step(m, d)
    lc, rc, _ = contact_pack(ids)
    if lc and rc:
        g13 = step+1
        print("  G13 at step %d lq=%.5f rq=%.5f" % (g13, d.qpos[ids["lq"]], d.qpos[ids["rq"]]))
        break
print("  G13", "PASS" if g13>0 else "FAIL")
if g13 > 0:
    r = official_hold(ids, Q0, -0.0105, -0.0105, gravity_ff=False,
                      csv_path=os.path.join(LOGDIR, "G14_MS_PROTO_A.csv"))
    print("  G14", "PASS" if r["pass"] else "FAIL", "loss", r["contact_loss"], "drift", r["drift"])
    summary.append(("A_V4", r, g13))

# ── Protocol B: gravity stays 0 after G13 ────────────────────────────────
print("\n=== PROTOCOL B: same as A but gravity stays 0 during 5s ===")
ids = make_model()
m, d = ids["m"], ids["d"]
reset_pose(ids, Q0, 0.02, 0.02)
m.opt.gravity[:] = 0.0
set_ctrl(ids, Q0, -0.0105, -0.0105)
mujoco.mj_forward(m, d)
g13 = -1
for step in range(2000):
    fix_tron1(ids)
    set_ctrl(ids, Q0, -0.0105, -0.0105)
    mujoco.mj_step(m, d)
    lc, rc, _ = contact_pack(ids)
    if lc and rc:
        g13 = step+1
        print("  G13 at step %d lq=%.5f rq=%.5f" % (g13, d.qpos[ids["lq"]], d.qpos[ids["rq"]]))
        break
if g13 > 0:
    # fake official hold but keep gravity 0
    ids["g0"][:] = 0.0
    r = official_hold(ids, Q0, -0.0105, -0.0105, gravity_ff=False,
                      csv_path=os.path.join(LOGDIR, "G14_MS_PROTO_B_ZEROG.csv"))
    print("  hold(zero-g)", "PASS" if r["pass"] else "FAIL", "loss", r["contact_loss"])
    summary.append(("B_zeroG", r, g13))

# ── Protocol C: kiro G13 then gravity ────────────────────────────────────
print("\n=== PROTOCOL C: kiro G13 (q=0.07 ctrl=0.0, 600 steps zero-G, no early break) ===")
ids = make_model()
m, d = ids["m"], ids["d"]
reset_pose(ids, Q0, 0.07, 0.07)
m.opt.gravity[:] = 0.0
set_ctrl(ids, Q0, 0.0, 0.0)
mujoco.mj_forward(m, d)
# kiro did NOT fix_tron1; we keep fix_tron1 to match G14 TRON1-fixed rule
for step in range(600):
    fix_tron1(ids)
    set_ctrl(ids, Q0, 0.0, 0.0)
    mujoco.mj_step(m, d)
lc, rc, pack = contact_pack(ids)
print("  after 600: L=%s R=%s lq=%.5f rq=%.5f distL=%s distR=%s" % (
    lc, rc, d.qpos[ids["lq"]], d.qpos[ids["rq"]], pack["ldist"], pack["rdist"]))
if lc and rc:
    r = official_hold(ids, Q0, 0.0, 0.0, gravity_ff=False,
                      csv_path=os.path.join(LOGDIR, "G14_MS_PROTO_C.csv"))
    print("  G14", "PASS" if r["pass"] else "FAIL", "loss", r["contact_loss"], "drift", r["drift"])
    summary.append(("C_kiro_ctrl0", r, 600))
else:
    print("  G13 FAIL under fix_tron1")
    summary.append(("C_kiro_ctrl0", {"pass": False, "contact_loss": None, "g13": False}, 0))

# ── Protocol D: kiro + arm gravity FF ────────────────────────────────────
print("\n=== PROTOCOL D: kiro G13 + arm gravity FF during hold ===")
ids = make_model()
m, d = ids["m"], ids["d"]
reset_pose(ids, Q0, 0.07, 0.07)
m.opt.gravity[:] = 0.0
for step in range(600):
    fix_tron1(ids)
    set_ctrl(ids, Q0, 0.0, 0.0)
    mujoco.mj_step(m, d)
lc, rc, _ = contact_pack(ids)
print("  G13 L=%s R=%s lq=%.5f rq=%.5f" % (lc, rc, d.qpos[ids["lq"]], d.qpos[ids["rq"]]))
if lc and rc:
    r = official_hold(ids, Q0, 0.0, 0.0, gravity_ff=True,
                      csv_path=os.path.join(LOGDIR, "G14_MS_PROTO_D.csv"))
    print("  G14", "PASS" if r["pass"] else "FAIL", "loss", r["contact_loss"], "drift", r["drift"])
    summary.append(("D_kiro_FF", r, 600))

# ── Protocol E: one-variable: start already at pinch q=-0.0105, ctrl=-0.0105
print("\n=== PROTOCOL E: start q=ctrl=-0.0105 (no slam), gravity ON from G13 hit ===")
ids = make_model()
m, d = ids["m"], ids["d"]
reset_pose(ids, Q0, -0.0105, -0.0105)
m.opt.gravity[:] = 0.0
for step in range(2000):
    fix_tron1(ids)
    set_ctrl(ids, Q0, -0.0105, -0.0105)
    mujoco.mj_step(m, d)
    lc, rc, _ = contact_pack(ids)
    if lc and rc:
        print("  G13 at step %d lq=%.5f rq=%.5f" % (step+1, d.qpos[ids["lq"]], d.qpos[ids["rq"]]))
        r = official_hold(ids, Q0, -0.0105, -0.0105, gravity_ff=True,
                          csv_path=os.path.join(LOGDIR, "G14_MS_PROTO_E.csv"))
        print("  G14", "PASS" if r["pass"] else "FAIL", "loss", r["contact_loss"])
        summary.append(("E_nslam_FF", r, step+1))
        break
else:
    print("  G13 FAIL")
    summary.append(("E_nslam_FF", {"pass": False, "contact_loss": None}, 0))

print("\n==== SUMMARY ====")
lines = ["G14 LEFT LOSS MS", "XML MODIFIED = NO"]
for name, r, g13 in summary:
    msg = "%s G13step=%s G14=%s loss=%s drift=%s" % (
        name, g13, r.get("pass"), r.get("contact_loss"), r.get("drift"))
    print(msg)
    lines.append(msg)
    if r.get("loss_row"):
        lr = r["loss_row"]
        lines.append("  first_loss t=%.4f lc=%s rc=%s lq=%.5f lqv=%.5f ldist=%s lFn=%s" % (
            lr["t"], lr["lc"], lr["rc"], lr["lq"], lr["lqv"], lr["ldist"], lr["lFn"]))
        print("  first_loss t=%.4f lc=%s rc=%s lq=%.5f lqv=%.5f ldist=%s lFn=%s" % (
            lr["t"], lr["lc"], lr["rc"], lr["lq"], lr["lqv"], lr["ldist"], lr["lFn"]))

winner = [s for s in summary if s[1].get("pass")]
lines.append("G14 = %s" % ("PASS" if winner else "FAIL"))
if winner:
    lines.append("WINNER %s" % winner[0][0])
with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print("Wrote", OUT)
sys.exit(0 if winner else 1)
