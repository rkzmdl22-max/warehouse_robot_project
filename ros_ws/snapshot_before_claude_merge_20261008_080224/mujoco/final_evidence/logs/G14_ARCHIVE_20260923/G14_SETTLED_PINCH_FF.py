"""
G14 official 5s hold after a SETTLED G13 pinch.

Cause of prior FAIL: G13 broke at first-step penetration (qpos~0.02) while
finger ctrl=-0.0105 drives fingers OPEN toward the true pinch. Contact is
lost around step 16 during that opening, even if the arm is gravity-compensated.

This test:
  - settle G13 in zero-G until finger qpos reaches the pinch command
  - then official 5.0s gravity ON, every step checked, no settle window
  - arm gravity feedforward on existing position actuators
  - XML MODIFIED = NO, no weld, no teleport of bottle
"""
import json
import sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_SETTLED_PINCH_FF_RESULT.txt"

print("G14 SETTLED PINCH + GRAVITY FF")
print("XML MODIFIED = NO")

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
gravity_orig = m.opt.gravity.copy()
print("gravity_orig", gravity_orig)


def bid(n):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)


def jid(n):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)


def aid(n):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)


def gid(n):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)


def qadr(n):
    return m.jnt_qposadr[jid(n)]


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
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
HANDOVER = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])

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


def contact():
    lc = rc = False
    dists = {"L": None, "R": None}
    for ci in range(d.ncon):
        c = d.contact[ci]
        pair = {int(c.geom1), int(c.geom2)}
        if pair == {bgeom, lgeom}:
            lc = True
            dists["L"] = float(c.dist)
        if pair == {bgeom, rgeom}:
            rc = True
            dists["R"] = float(c.dist)
    return lc, rc, dists


def fix_tron1():
    d.qpos[root_qp:root_qp + 7] = HANDOVER
    d.qvel[root_dof:root_dof + 6] = 0.0


def set_ctrl(q_arm, lf, rf):
    for ai, qv in zip(arm_act, q_arm):
        d.ctrl[ai] = qv
    d.ctrl[ai_lf] = lf
    d.ctrl[ai_rf] = rf


def gravity_ff(q_arm):
    mujoco.mj_forward(m, d)
    extra = np.array([float(d.qfrc_bias[dof]) / (kp if kp else 1.0)
                      for dof, kp in zip(arm_dof, kp_arm)])
    return np.asarray(q_arm) + extra, extra


def run_case(q_arm, lf, rf, start_finger, settle_steps, use_ff, tag):
    print("\n==== %s ====" % tag)
    print("lf=%s rf=%s start_finger=%s settle=%d ff=%s" % (lf, rf, start_finger, settle_steps, use_ff))
    d.qpos[root_qp:root_qp + 7] = HANDOVER
    d.qvel[:] = 0.0
    for adr, qv in zip(arm_qadr, q_arm):
        d.qpos[adr] = qv
    d.qpos[lq] = start_finger
    d.qpos[rq] = start_finger
    set_ctrl(q_arm, lf, rf)
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d)

    both_seen = False
    last_both = -1
    for step in range(settle_steps):
        fix_tron1()
        set_ctrl(q_arm, lf, rf)
        mujoco.mj_step(m, d)
        lc, rc, dist = contact()
        if lc and rc:
            both_seen = True
            last_both = step + 1

    lc, rc, dist = contact()
    print("G13 settle: both_seen=%s last_both=%s lc=%s rc=%s lq=%.5f rq=%.5f dist=%s bottle=%s" % (
        both_seen, last_both, lc, rc, d.qpos[lq], d.qpos[rq], dist, d.xpos[bb].round(4)))
    if not (lc and rc):
        print("G13 FAIL (no bilateral at end of settle)")
        return {"g13": False, "pass": False, "tag": tag}

    # Official G14: 5.000s = 2500 steps at dt=0.002
    m.opt.gravity[:] = gravity_orig
    b_start = d.xpos[bb].copy()
    hold_fail = False
    contact_loss = -1
    nstep = 2500
    logs = []
    for step in range(nstep):
        fix_tron1()
        ctrl_arm = q_arm
        extra = np.zeros(6)
        if use_ff:
            ctrl_arm, extra = gravity_ff(q_arm)
        set_ctrl(ctrl_arm, lf, rf)
        mujoco.mj_step(m, d)
        lc2, rc2, dist2 = contact()
        bp = d.xpos[bb].copy()
        mv = float(np.max(np.abs(d.qvel)))
        if not np.isfinite(bp).all() or mv > 200:
            hold_fail = True
            if contact_loss < 0:
                contact_loss = step + 1
            break
        if (not lc2 or not rc2) and contact_loss < 0:
            contact_loss = step + 1
            print("  CONTACT LOSS at step=%d t=%.4fs L=%s R=%s lq=%.5f rq=%.5f dist=%s" % (
                step + 1, (step + 1) * m.opt.timestep, lc2, rc2, d.qpos[lq], d.qpos[rq], dist2))
        if (step + 1) % 500 == 0:
            qerr = float(np.linalg.norm(np.array([d.qpos[a] for a in arm_qadr]) - q_arm))
            print("  t=%.2fs L=%s R=%s drift=%.6f lq=%.5f rq=%.5f qerr=%.4f loss=%s dist=%s" % (
                (step + 1) * m.opt.timestep, lc2, rc2, float(np.linalg.norm(bp - b_start)),
                d.qpos[lq], d.qpos[rq], qerr, contact_loss, dist2))
            logs.append({
                "t": (step + 1) * m.opt.timestep, "lc": lc2, "rc": rc2,
                "lq": float(d.qpos[lq]), "rq": float(d.qpos[rq]),
                "dist": dist2, "qerr": qerr,
            })

    b_end = d.xpos[bb].copy()
    lc_h, rc_h, dist_h = contact()
    drift = float(np.linalg.norm(b_end - b_start))
    g14 = (np.isfinite(b_end).all() and not hold_fail and contact_loss < 0
           and lc_h and rc_h and drift < 0.025)
    print("G14=%s finite=%s no_fail=%s loss=%s lc=%s rc=%s drift=%.6f" % (
        "PASS" if g14 else "FAIL", np.isfinite(b_end).all(), not hold_fail,
        contact_loss, lc_h, rc_h, drift))
    return {
        "tag": tag, "g13": True, "pass": bool(g14),
        "contact_loss": int(contact_loss), "lc": bool(lc_h), "rc": bool(rc_h),
        "drift": drift, "hold_fail": bool(hold_fail),
        "finger_end": [float(d.qpos[lq]), float(d.qpos[rq])],
        "dist_end": dist_h, "logs": logs,
        "q_arm": np.asarray(q_arm).tolist(), "lf": lf, "rf": rf,
        "use_ff": use_ff, "start_finger": start_finger,
    }


cases = []
# Primary: start already at pinch, settle, gravity FF
for lf in (-0.0105, 0.0, -0.008, -0.005, -0.012, -0.015):
    cases.append((Q0, lf, lf, lf, 1500, True, "start=ctrl=%.4f FF" % lf))
# start from open-ish 0.02 then settle to pinch
cases.append((Q0, -0.0105, -0.0105, 0.02, 2000, True, "start0.02 ctrl-0.0105 FF"))
cases.append((Q0, 0.0, 0.0, 0.02, 2000, True, "start0.02 ctrl0.0 FF"))
# no FF control
cases.append((Q0, -0.0105, -0.0105, -0.0105, 1500, False, "start=ctrl=-0.0105 noFF"))

results = []
winner = None
for args in cases:
    r = run_case(*args)
    results.append(r)
    if r.get("pass") and winner is None:
        winner = r
        break

print("\n==== SUMMARY ====")
for r in results:
    print("%-36s G13=%s G14=%s loss=%s drift=%s" % (
        r.get("tag"), r.get("g13"), r.get("pass"), r.get("contact_loss"), r.get("drift")))

with open(OUT, "w", encoding="utf-8") as f:
    f.write("G14 SETTLED PINCH + GRAVITY FF\nXML MODIFIED = NO\n")
    f.write("G14 = %s\n" % ("PASS" if winner else "FAIL"))
    if winner:
        f.write("WINNER %s\n" % winner["tag"])
        f.write("Q0 %s fingers %s %s\n" % (winner["q_arm"], winner["lf"], winner["rf"]))
    f.write(json.dumps(results, indent=2, default=str))
    f.write("\n")
print("Wrote", OUT)
sys.exit(0 if winner else 1)
