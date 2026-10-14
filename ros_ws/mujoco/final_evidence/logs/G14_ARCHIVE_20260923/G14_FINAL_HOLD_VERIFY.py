import mujoco
import numpy as np

XML = "/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

def bid(name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)

def gid(name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)

def xpos(name):
    return d.xpos[bid(name)].copy()

BOTTLE = "carry_bottle"
LEFT_FINGER = "left_finger_geom"
RIGHT_FINGER = "right_finger_geom"

bottle_bid = bid(BOTTLE)
left_gid = gid(LEFT_FINGER)
right_gid = gid(RIGHT_FINGER)

print("=" * 80)
print("G14 FINAL HOLD VERIFY")
print("=" * 80)
print("XML =", XML)
print("neq =", m.neq)
print("bottle body =", bottle_bid)
print("left finger geom =", left_gid)
print("right finger geom =", right_gid)

# ------------------------------------------------------------
# Initial forward
# ------------------------------------------------------------
mujoco.mj_forward(m, d)

print()
print("INITIAL")
print("bottle =", xpos(BOTTLE))
print("left finger qpos =", d.qpos[47])
print("right finger qpos =", d.qpos[48])

# ------------------------------------------------------------
# Reproduce G13 grasp pose from current Final XML
# ------------------------------------------------------------
target_q = np.array([
     0.52942822,
    -5.26687124,
     2.84989839,
     5.44033518,
    -3.16557839,
    -3.72042067
])

# UR5e arm qpos = 41:47
d.qpos[41:47] = target_q

# Finger close
d.qpos[47] = 0.07
d.qpos[48] = 0.07

# Start with zero velocity
d.qvel[:] = 0.0

mujoco.mj_forward(m, d)

print()
print("GRASP INITIALIZED")
print("bottle =", xpos(BOTTLE))
print("left finger qpos =", d.qpos[47])
print("right finger qpos =", d.qpos[48])

# ------------------------------------------------------------
# Gravity OFF ONLY for grasp settling
# ------------------------------------------------------------
gravity_original = m.opt.gravity.copy()
m.opt.gravity[:] = 0.0

for _ in range(300):
    mujoco.mj_step(m, d)

mujoco.mj_forward(m, d)

bottle_start = xpos(BOTTLE)

print()
print("AFTER GRASP SETTLE")
print("bottle =", bottle_start)
print("left finger qpos =", d.qpos[47])
print("right finger qpos =", d.qpos[48])
print("ncon =", d.ncon)

# ------------------------------------------------------------
# Check finger contacts with bottle geom 117
# ------------------------------------------------------------
bottle_gid = gid("carry_bottle_collision")

left_contact = False
right_contact = False

for i in range(d.ncon):
    c = d.contact[i]
    a, b = c.geom1, c.geom2

    if bottle_gid in (a, b):
        other = b if a == bottle_gid else a
        if other == left_gid:
            left_contact = True
        if other == right_gid:
            right_contact = True

print("left_contact =", left_contact)
print("right_contact =", right_contact)

# ------------------------------------------------------------
# G14 HOLD TEST
# Gravity restored.
# No arm trajectory.
# Fingers remain closed.
# ------------------------------------------------------------
m.opt.gravity[:] = gravity_original

hold_steps = 1000

max_disp = 0.0
max_qvel = 0.0
max_qerr = 0.0
min_left_q = 999.0
min_right_q = 999.0
contact_loss_step = None

for step in range(hold_steps):
    d.ctrl[6] = 0.0
    d.ctrl[7] = 0.0

    mujoco.mj_step(m, d)

    bp = xpos(BOTTLE)
    disp = np.linalg.norm(bp - bottle_start)
    qvel = np.max(np.abs(d.qvel))
    qerr = np.max(np.abs(d.qpos[41:47] - target_q))

    max_disp = max(max_disp, disp)
    max_qvel = max(max_qvel, qvel)
    max_qerr = max(max_qerr, qerr)

    min_left_q = min(min_left_q, d.qpos[47])
    min_right_q = min(min_right_q, d.qpos[48])

    lc = False
    rc = False

    for i in range(d.ncon):
        c = d.contact[i]
        a, b = c.geom1, c.geom2

        if bottle_gid in (a, b):
            other = b if a == bottle_gid else a
            if other == left_gid:
                lc = True
            if other == right_gid:
                rc = True

    if not (lc and rc) and contact_loss_step is None:
        contact_loss_step = step

# ------------------------------------------------------------
# Final result
# ------------------------------------------------------------
bottle_end = xpos(BOTTLE)
final_disp = np.linalg.norm(bottle_end - bottle_start)

mujoco.mj_forward(m, d)

final_left = False
final_right = False

for i in range(d.ncon):
    c = d.contact[i]
    a, b = c.geom1, c.geom2

    if bottle_gid in (a, b):
        other = b if a == bottle_gid else a
        if other == left_gid:
            final_left = True
        if other == right_gid:
            final_right = True

print()
print("=" * 80)
print("G14 RESULT")
print("=" * 80)
print("hold_steps =", hold_steps)
print("bottle_start =", bottle_start)
print("bottle_end   =", bottle_end)
print("final_disp_m =", final_disp)
print("max_disp_m   =", max_disp)
print("max_qvel     =", max_qvel)
print("max_qerr_rad =", max_qerr)
print("min_left_q   =", min_left_q)
print("min_right_q  =", min_right_q)
print("contact_loss_step =", contact_loss_step)
print("final_left_contact  =", final_left)
print("final_right_contact =", final_right)

hold_ok = (
    np.isfinite(bottle_end).all()
    and final_left
    and final_right
    and contact_loss_step is None
    and final_disp < 0.02
)

print()
print("G14_HOLD =", "PASS" if hold_ok else "FAIL")
print("=" * 80)
