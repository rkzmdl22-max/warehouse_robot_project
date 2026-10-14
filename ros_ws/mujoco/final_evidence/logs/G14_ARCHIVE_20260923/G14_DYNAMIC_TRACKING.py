import mujoco
import numpy as np
import sys

XML = "/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def xpos(n): return d.xpos[bid(n)].copy()

arm_joints = [
    "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
    "wrist_1_joint","wrist_2_joint","wrist_3_joint"
]
act_names = [
    "act_shoulder_pan","act_shoulder_lift","act_elbow",
    "act_wrist_1","act_wrist_2","act_wrist_3"
]
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

tron1 = bid("base_Link")
root_jnt = m.body_jntadr[tron1]
root_qp = m.jnt_qposadr[root_jnt]

tool0 = bid("tool0")
bottle = bid("carry_bottle")

bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")

lq = m.jnt_qposadr[jid("left_finger_joint")]
rq = m.jnt_qposadr[jid("right_finger_joint")]

arm_dofs = np.array([
    m.jnt_dofadr[jid(jn)] for jn in arm_joints
], dtype=int)

arm_act = np.array([aid(a) for a in act_names], dtype=int)
lf_act = aid("act_left_finger")
rf_act = aid("act_right_finger")

ctrl_lo = m.actuator_ctrlrange[arm_act, 0].copy()
ctrl_hi = m.actuator_ctrlrange[arm_act, 1].copy()

def set_arm_ctrl(q):
    d.ctrl[arm_act] = np.clip(q, ctrl_lo, ctrl_hi)

def contacts():
    lc = rc = False
    for ci in range(d.ncon):
        a = int(d.contact[ci].geom1)
        b = int(d.contact[ci].geom2)
        if {a,b} == {bgeom,lgeom}: lc = True
        if {a,b} == {bgeom,rgeom}: rc = True
    return lc, rc

# ============================================================
# G13 proven initial grasp state
# ============================================================
d.qvel[:] = 0.0
d.qpos[root_qp:root_qp+3] = [0.0, -3.0, 0.82]

for jn, qv in zip(arm_joints, Q0):
    d.qpos[m.jnt_qposadr[jid(jn)]] = qv

d.qpos[lq] = 0.07
d.qpos[rq] = 0.07

set_arm_ctrl(Q0)
d.ctrl[lf_act] = 0.0
d.ctrl[rf_act] = 0.0

gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

# Establish the same proven G13 bilateral contact.
for _ in range(600):
    mujoco.mj_step(m, d)

lc0, rc0 = contacts()
if not (lc0 and rc0):
    print("G13_INIT = FAIL")
    print("left_contact =", lc0, "right_contact =", rc0)
    sys.exit(1)

# Record the actual contact-time relative geometry.
b_start = xpos("carry_bottle").copy()
tool_start = xpos("tool0").copy()
relative_start = b_start - tool_start

# ============================================================
# G14 DYNAMIC TRACKING
# ============================================================
m.opt.gravity[:] = gravity_orig
mujoco.mj_forward(m, d)

print("=" * 72)
print("G14 DYNAMIC TRACKING")
print("XML MODIFIED = NO")
print("=" * 72)
print("start_bottle =", b_start)
print("start_tool0  =", tool_start)
print("relative     =", relative_start)
print("gravity      =", m.opt.gravity)

# Resolved-rate controller:
# target tool velocity =
#   Kp * position error + Kv * bottle velocity
#
# The position target preserves the proven G13 bottle/tool relationship.
# This is live tracking, not a root/body lock.
Kp = 8.0
Kv = 0.75
dt = float(m.opt.timestep)

q_target = np.array([
    d.qpos[m.jnt_qposadr[jid(jn)]] for jn in arm_joints
], dtype=float)

max_drift = 0.0
max_track_error = 0.0
min_bilateral = True
contact_loss_step = -1
finite_ok = True

samples = []

steps = int(round(5.0 / dt))

for step in range(steps):
    bp = xpos("carry_bottle")
    tp = xpos("tool0")

    bv = d.cvel[bottle, 3:6].copy()
    tv = d.cvel[tool0, 3:6].copy()

    # Desired tool position follows live bottle position
    # while preserving the empirically observed G13 relative geometry.
    target_tool = bp - relative_start
    pos_err = target_tool - tp

    target_vel = Kp * pos_err + Kv * bv

    Jp = np.zeros((3, m.nv))
    Jr = np.zeros((3, m.nv))
    mujoco.mj_jacBody(m, d, Jp, Jr, tool0)
    J = Jp[:, arm_dofs]

    dq = np.linalg.pinv(J, rcond=1e-4) @ target_vel

    # Velocity limit prevents numerical spikes.
    dq_raw = dq.copy()
    dq = np.clip(dq, -1.5, 1.5)

    if (step + 1) % 500 == 0:
        print("  joint_dq_raw =", np.round(dq_raw,4))
        print("  joint_dq_cmd =", np.round(dq,4))
        print("  q_target     =", np.round(q_target,4))

    q_target = np.array([
        d.qpos[m.jnt_qposadr[jid(jn)]] for jn in arm_joints
    ], dtype=float) + dq * dt

    # Keep targets inside the actual actuator ranges.
    q_target = np.clip(q_target, ctrl_lo, ctrl_hi)

    set_arm_ctrl(q_target)

    # Preserve the proven bilateral-contact finger command.
    d.ctrl[lf_act] = 0.0
    d.ctrl[rf_act] = 0.0

    mujoco.mj_step(m, d)

    bp2 = xpos("carry_bottle")
    tp2 = xpos("tool0")

    drift = float(np.linalg.norm(bp2 - b_start))
    track_error = float(np.linalg.norm((bp2 - relative_start) - tp2))

    lc, rc = contacts()
    finite = np.isfinite(bp2).all() and np.isfinite(tp2).all() and np.isfinite(d.qpos).all()

    max_drift = max(max_drift, drift)
    max_track_error = max(max_track_error, track_error)

    if not (lc and rc):
        min_bilateral = False
        if contact_loss_step < 0:
            contact_loss_step = step + 1

    if not finite:
        finite_ok = False
        break

    if (step + 1) % 500 == 0:
        samples.append((
            (step + 1) * dt,
            bp2.copy(),
            drift,
            track_error,
            lc,
            rc,
            float(np.max(np.abs(d.qvel)))
        ))

# ============================================================
# Official G14 result
# ============================================================
b_end = xpos("carry_bottle").copy()
final_drift = float(np.linalg.norm(b_end - b_start))
lc_f, rc_f = contacts()

print("\n[5s RUNTIME SAMPLES]")
for t, bp, dr, te, lc, rc, mv in samples:
    print(
        f"t={t:.2f}s bottle={np.round(bp,4)} "
        f"drift={dr:.5f} track_err={te:.5f} "
        f"L={lc} R={rc} max_qvel={mv:.4f}"
    )

print("\n[OFFICIAL G14]")
print("bottle_start       =", b_start)
print("bottle_end         =", b_end)
print("final_drift        =", final_drift)
print("max_drift          =", max_drift)
print("max_track_error    =", max_track_error)
print("contact_loss_step  =", contact_loss_step)
print("final_L_contact    =", lc_f)
print("final_R_contact    =", rc_f)
print("bilateral_5s       =", min_bilateral)
print("finite_ok          =", finite_ok)

# Official criterion: 5 seconds, continuous bilateral contact,
# no contact loss, drift < 0.025 m, finite/stable state.
g14_pass = (
    finite_ok
    and min_bilateral
    and contact_loss_step < 0
    and final_drift < 0.025
    and max_drift < 0.025
)

print("\nG14 =", "PASS" if g14_pass else "FAIL")

# Restore original gravity before exit.
m.opt.gravity[:] = gravity_orig

if not g14_pass:
    sys.exit(1)
