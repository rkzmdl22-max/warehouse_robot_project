"""
G14 gravity feedforward + local Q0 neighborhood search.

Hypothesis (from XML inspect, NOT a protected edit):
  carry_bottle is a child of carry_tray with NO freejoint, so bottle world
  motion is TRON1 motion. With fix_tron1(), official drift is ~0.
  G14 FAIL is arm sag under gravity losing finger contact with the tray bottle.

This script:
  1) Confirms bottle DOF / actuator limits / gravity bias at Q0
  2) Reproduces G13 at known Q0
  3) Official 5s G14 with gravity feedforward on existing position actuators
     (ctrl = Q_hold + qfrc_bias/kp). Not weld, not XML edit.
  4) If needed, local neighborhood around Q0 that still gets G13, then G14.

XML MODIFIED = NO
Official G14: full 5s, no settle-window, no weld.
"""
import json
import os
import sys
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
OUT = r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_GRAVITY_FF_AND_LOCAL_SEARCH_RESULT.txt"

print("G14 GRAVITY FEEDFORWARD + LOCAL Q0 SEARCH")
print("XML MODIFIED = NO")
print("XML:", XML)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
gravity_orig = m.opt.gravity.copy()
print("Original gravity:", gravity_orig)
print("timestep:", m.opt.timestep, "nq:", m.nq, "nv:", m.nv, "nu:", m.nu)


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
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
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
HANDOVER_QPOS7 = np.array([0.0, -3.0, 0.82, 1.0, 0.0, 0.0, 0.0])

tron1_bi = bid("base_Link")
root_jnt = m.body_jntadr[tron1_bi]
root_qp = m.jnt_qposadr[root_jnt]
root_dof = m.jnt_dofadr[root_jnt]
lq = qadr("left_finger_joint")
rq = qadr("right_finger_joint")
bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")
ai_lf = aid("act_left_finger")
ai_rf = aid("act_right_finger")
arm_act = [aid(act_map[jn]) for jn in arm_joints]
arm_dof = [m.jnt_dofadr[jid(jn)] for jn in arm_joints]
arm_qadr = [qadr(jn) for jn in arm_joints]
kp_arm = np.array([float(m.actuator_gainprm[ai][0]) for ai in arm_act])
fr_arm = np.array([m.actuator_forcerange[ai].copy() for ai in arm_act])

print("\n=== MODEL FACTS ===")
bb = bid("carry_bottle")
print("carry_bottle parent:", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[bb]))
print("carry_bottle njoints (body_jntnum):", int(m.body_jntnum[bb]))
print("carry_bottle mass:", float(m.body_mass[bb]))
print("left/right finger range:", m.jnt_range[jid("left_finger_joint")], m.jnt_range[jid("right_finger_joint")])
print("finger kp:", m.actuator_gainprm[ai_lf][0], m.actuator_gainprm[ai_rf][0])
print("arm actuator names / kp / forcerange:")
for jn, ai, kp, fr in zip(arm_joints, arm_act, kp_arm, fr_arm):
    print(f"  {jn:22s} ai={ai} kp={kp:.1f} forcerange={fr} gaintype={int(m.actuator_gaintype[ai])} biastype={int(m.actuator_biastype[ai])}")
print("actuator_biasprm (first 3) per arm act:")
for jn, ai in zip(arm_joints, arm_act):
    print(f"  {jn:22s} {m.actuator_biasprm[ai][:3]}")


def contact(data=None):
    data = d if data is None else data
    lc = rc = False
    details = []
    for ci in range(data.ncon):
        c = data.contact[ci]
        a, b = int(c.geom1), int(c.geom2)
        pair = {a, b}
        if pair == {bgeom, lgeom}:
            lc = True
            details.append(("L", ci, c.dist, c.pos.copy(), c.frame.copy()[:3]))
        if pair == {bgeom, rgeom}:
            rc = True
            details.append(("R", ci, c.dist, c.pos.copy(), c.frame.copy()[:3]))
    return lc, rc, details


def force_on_geom(geom_id, data=None):
    data = d if data is None else data
    cforce = np.zeros(6)
    f = np.zeros(6)
    for ci in range(data.ncon):
        c = data.contact[ci]
        if int(c.geom1) == geom_id or int(c.geom2) == geom_id:
            mujoco.mj_contactForce(m, data, ci, cforce)
            # force is on geom1; flip if needed
            if int(c.geom1) == geom_id:
                f += cforce
            else:
                f -= cforce
    return f


def fix_tron1(data=None):
    data = d if data is None else data
    data.qpos[root_qp:root_qp + 7] = HANDOVER_QPOS7
    data.qvel[root_dof:root_dof + 6] = 0.0


def set_arm_finger(q_arm, q_lf, q_rf, data=None):
    data = d if data is None else data
    for ai, qv in zip(arm_act, q_arm):
        if ai >= 0:
            data.ctrl[ai] = qv
    data.ctrl[ai_lf] = q_lf
    data.ctrl[ai_rf] = q_rf


def reset_g13_pose(q_arm, qf_init=0.02):
    d.qpos[root_qp:root_qp + 7] = HANDOVER_QPOS7
    d.qvel[:] = 0.0
    for adr, qv in zip(arm_qadr, q_arm):
        d.qpos[adr] = qv
    d.qpos[lq] = qf_init
    d.qpos[rq] = qf_init
    set_arm_finger(q_arm, q_arm_fingers[0] if False else -0.0105, -0.0105)
    # placeholder overwritten by caller


def grasp_zero_g(q_arm, lf, rf, max_steps=2000):
    d.qpos[root_qp:root_qp + 7] = HANDOVER_QPOS7
    d.qvel[:] = 0.0
    for adr, qv in zip(arm_qadr, q_arm):
        d.qpos[adr] = qv
    d.qpos[lq] = 0.02
    d.qpos[rq] = 0.02
    set_arm_finger(q_arm, lf, rf)
    m.opt.gravity[:] = 0.0
    mujoco.mj_forward(m, d)
    got = False
    step_got = -1
    for step in range(max_steps):
        fix_tron1()
        set_arm_finger(q_arm, lf, rf)
        mujoco.mj_step(m, d)
        lc, rc, _ = contact()
        if lc and rc:
            got = True
            step_got = step + 1
            break
    return got, step_got


def gravity_ff_ctrl(q_arm):
    """Position-actuator gravity compensation: ctrl = q_des + qfrc_bias/kp."""
    mujoco.mj_forward(m, d)
    extra = np.zeros(6)
    for i, dof in enumerate(arm_dof):
        bias = float(d.qfrc_bias[dof])
        kp = kp_arm[i] if kp_arm[i] != 0 else 1.0
        extra[i] = bias / kp
    return np.asarray(q_arm) + extra, extra


def official_g14(q_arm, lf, rf, use_ff=True, nstep=2500, log_every=500):
    m.opt.gravity[:] = gravity_orig
    b_start = d.xpos[bb].copy()
    hold_fail = False
    contact_loss = -1
    snapshots = []
    for step in range(nstep):
        fix_tron1()
        if use_ff:
            ctrl_arm, extra = gravity_ff_ctrl(q_arm)
        else:
            ctrl_arm, extra = np.asarray(q_arm), np.zeros(6)
        set_arm_finger(ctrl_arm, lf, rf)
        mujoco.mj_step(m, d)
        lc, rc, det = contact()
        bp = d.xpos[bb].copy()
        if not np.isfinite(bp).all() or float(np.max(np.abs(d.qvel))) > 200:
            hold_fail = True
            contact_loss = step + 1 if contact_loss < 0 else contact_loss
            break
        if (not lc or not rc) and contact_loss < 0:
            contact_loss = step + 1
        if (step + 1) % log_every == 0 or step == 0:
            arm_q = np.array([d.qpos[a] for a in arm_qadr])
            arm_v = np.array([d.qvel[dof] for dof in arm_dof])
            snapshots.append({
                "step": step + 1,
                "t": (step + 1) * m.opt.timestep,
                "lc": bool(lc),
                "rc": bool(rc),
                "bottle": bp.tolist(),
                "drift": float(np.linalg.norm(bp - b_start)),
                "lq": float(d.qpos[lq]),
                "rq": float(d.qpos[rq]),
                "arm_q": arm_q.tolist(),
                "arm_v": arm_v.tolist(),
                "finger_qvel": [float(d.qvel[m.jnt_dofadr[jid("left_finger_joint")]]),
                                float(d.qvel[m.jnt_dofadr[jid("right_finger_joint")]])],
                "ctrl_arm": ctrl_arm.tolist(),
                "ff_extra": extra.tolist(),
                "ncon_details": [(x[0], float(x[2]), x[3].tolist(), x[4].tolist()) for x in det],
                "f_left": force_on_geom(lgeom).tolist(),
                "f_right": force_on_geom(rgeom).tolist(),
            })
            print("  t=%.2fs L=%s R=%s drift=%.4f lq=%.5f rq=%.5f qerr=%.4f loss=%s" % (
                (step + 1) * m.opt.timestep, lc, rc, float(np.linalg.norm(bp - b_start)),
                d.qpos[lq], d.qpos[rq], float(np.linalg.norm(arm_q - q_arm)),
                contact_loss))
    b_end = d.xpos[bb].copy()
    lc_h, rc_h, det_end = contact()
    drift = float(np.linalg.norm(b_end - b_start))
    g14 = (np.isfinite(b_end).all() and not hold_fail and contact_loss < 0
           and lc_h and rc_h and drift < 0.025)
    return {
        "pass": bool(g14),
        "hold_fail": bool(hold_fail),
        "contact_loss": int(contact_loss),
        "lc": bool(lc_h),
        "rc": bool(rc_h),
        "drift": drift,
        "b_start": b_start.tolist(),
        "b_end": b_end.tolist(),
        "finger_q": [float(d.qpos[lq]), float(d.qpos[rq])],
        "arm_q": [float(d.qpos[a]) for a in arm_qadr],
        "arm_qerr": float(np.linalg.norm(np.array([d.qpos[a] for a in arm_qadr]) - q_arm)),
        "snapshots": snapshots,
        "end_contacts": [(x[0], float(x[2]), x[3].tolist(), x[4].tolist()) for x in det_end],
    }


# ── 1. Gravity bias at Q0 ──────────────────────────────────────────────────
print("\n=== GRAVITY BIAS AT Q0 ===")
d.qpos[root_qp:root_qp + 7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for adr, qv in zip(arm_qadr, Q0):
    d.qpos[adr] = qv
d.qpos[lq] = 0.02
d.qpos[rq] = 0.02
m.opt.gravity[:] = gravity_orig
mujoco.mj_forward(m, d)
print("joint | q | qfrc_bias | kp | bias/kp | forcerange")
for jn, adr, dof, kp, fr in zip(arm_joints, arm_qadr, arm_dof, kp_arm, fr_arm):
    b = float(d.qfrc_bias[dof])
    print(f"  {jn:22s} q={d.qpos[adr]:+.4f} bias={b:+8.3f} kp={kp:.1f} dctrl={b/kp:+.5f} fr={fr}")

# ── 2. G13 at Q0 ───────────────────────────────────────────────────────────
print("\n=== G13 AT Q0 ===")
finger_sets = [
    (-0.0105, -0.0105),
    (-0.015, -0.015),
    (-0.015, -0.0105),
    (-0.0105, -0.015),
    (0.0, 0.0),
    (-0.015, 0.0),
    (0.0, -0.015),
]
g13_ok = None
for lf, rf in finger_sets:
    got, st = grasp_zero_g(Q0, lf, rf)
    lc, rc, det = contact()
    print("  fingers L=%.4f R=%.4f  G13=%s step=%s  lc=%s rc=%s  lq=%.5f rq=%.5f nconL/R=%d" % (
        lf, rf, got, st, lc, rc, d.qpos[lq], d.qpos[rq], len(det)))
    if got and g13_ok is None:
        g13_ok = (lf, rf, st)
        # keep this state for first G14 attempt
        break

if g13_ok is None:
    print("G13 NOT REPRODUCED — cannot run G14")
    sys.exit(1)

lf0, rf0, st0 = g13_ok
print("Using G13 fingers L=%.4f R=%.4f at step %d" % (lf0, rf0, st0))
print("bottle G13:", d.xpos[bb].round(4))
lc, rc, det = contact()
for item in det:
    print("  contact", item[0], "dist", item[2], "pos", item[3], "n", item[4])

# ── 3. Official G14 baseline (no FF) vs FF ─────────────────────────────────
results = {}

# Need fresh G13 state for each hold. Helper:
def g13_then_hold(q_arm, lf, rf, use_ff, tag):
    got, st = grasp_zero_g(q_arm, lf, rf)
    print(f"\n[{tag}] G13={'PASS' if got else 'FAIL'} step={st}")
    if not got:
        return {"pass": False, "g13": False}
    r = official_g14(q_arm, lf, rf, use_ff=use_ff)
    r["g13"] = True
    r["q_arm"] = np.asarray(q_arm).tolist()
    r["fingers"] = [lf, rf]
    r["use_ff"] = use_ff
    print(f"[{tag}] G14={'PASS' if r['pass'] else 'FAIL'} loss={r['contact_loss']} drift={r['drift']:.6f} lc={r['lc']} rc={r['rc']} qerr={r['arm_qerr']:.4f}")
    return r


results["Q0_noff"] = g13_then_hold(Q0, lf0, rf0, False, "Q0 no-FF")
results["Q0_ff"] = g13_then_hold(Q0, lf0, rf0, True, "Q0 gravity-FF")

# extra finger combos with FF if first FF failed
if not results["Q0_ff"].get("pass"):
    for lf, rf in [(-0.015, -0.015), (-0.015, -0.0105), (-0.0105, -0.015), (0.0, -0.015), (-0.015, 0.0)]:
        tag = "Q0 FF fingers %s/%s" % (lf, rf)
        results[tag] = g13_then_hold(Q0, lf, rf, True, tag)
        if results[tag].get("pass"):
            break

# ── 4. Local neighborhood around Q0 (only if still FAIL) ───────────────────
best = None
for k, v in results.items():
    if v.get("pass"):
        best = (k, v)
        break

if best is None:
    print("\n=== LOCAL Q0 NEIGHBORHOOD (G13-preserving) ===")
    # wrist_3, wrist_2, wrist_1, shoulder_lift small deltas
    deltas = []
    for j in range(6):
        for dv in (-0.08, -0.04, -0.02, 0.02, 0.04, 0.08):
            dq = np.zeros(6)
            dq[j] = dv
            deltas.append(dq)
    # paired lift/elbow to keep EE near bottle
    for a, b in [(1, 2), (2, 3), (3, 4), (4, 5)]:
        for dv in (-0.05, 0.05):
            dq = np.zeros(6)
            dq[a] = dv
            dq[b] = -dv
            deltas.append(dq)
    # wrist_3 denser (tool-axis roll)
    for dv in np.linspace(-0.4, 0.4, 9):
        if abs(dv) < 1e-9:
            continue
        dq = np.zeros(6)
        dq[5] = float(dv)
        deltas.append(dq)

    tested = 0
    g13_hits = 0
    for dq in deltas:
        q = Q0 + dq
        got, st = grasp_zero_g(q, lf0, rf0)
        tested += 1
        if not got:
            continue
        g13_hits += 1
        print("  G13 hit dq=%s step=%d — running official G14 FF" % (np.round(dq, 4), st))
        r = official_g14(q, lf0, rf0, use_ff=True)
        r["g13"] = True
        r["q_arm"] = q.tolist()
        r["dq"] = dq.tolist()
        tag = "local " + str(np.round(dq, 4).tolist())
        results[tag] = r
        print("    G14=%s loss=%s drift=%.4f" % ("PASS" if r["pass"] else "FAIL", r["contact_loss"], r["drift"]))
        if r["pass"]:
            best = (tag, r)
            break
    print("Neighborhood tested=%d G13_hits=%d" % (tested, g13_hits))

# ── 5. Summary ─────────────────────────────────────────────────────────────
print("\n=== SUMMARY ===")
g14_pass = False
winner = None
for k, v in results.items():
    ok = bool(v.get("pass"))
    print("  %-40s G13=%s G14=%s loss=%s drift=%s" % (
        k[:40], v.get("g13"), ok, v.get("contact_loss"), v.get("drift")))
    if ok and not g14_pass:
        g14_pass = True
        winner = k

lines = []
lines.append("G14 GRAVITY FEEDFORWARD + LOCAL SEARCH RESULT")
lines.append("XML MODIFIED = NO")
lines.append("G13 PASS fingers = %s" % (g13_ok,))
lines.append("G14 = %s" % ("PASS" if g14_pass else "FAIL"))
if winner:
    lines.append("WINNER = %s" % winner)
    lines.append("Q = %s" % results[winner].get("q_arm"))
    lines.append("fingers = %s" % results[winner].get("fingers"))
lines.append("carry_bottle body_jntnum = %d (0 means welded to parent)" % int(m.body_jntnum[bb]))
# compact JSON without huge snapshots
compact = {}
for k, v in results.items():
    compact[k] = {ik: iv for ik, iv in v.items() if ik != "snapshots"}
    compact[k]["n_snapshots"] = len(v.get("snapshots") or [])
lines.append(json.dumps(compact, indent=2, default=str))
text = "\n".join(lines) + "\n"
with open(OUT, "w", encoding="utf-8") as f:
    f.write(text)
print("Wrote", OUT)
print("G14", "PASS" if g14_pass else "FAIL")
sys.exit(0 if g14_pass else 1)
