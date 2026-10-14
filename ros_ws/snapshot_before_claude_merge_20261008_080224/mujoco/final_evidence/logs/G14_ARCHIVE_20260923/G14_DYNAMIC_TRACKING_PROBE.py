import mujoco
import numpy as np

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

bottle = bid("carry_bottle")
tool0 = bid("tool0")
lf = bid("left_finger_link")
rf = bid("right_finger_link")

bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")

lq = m.jnt_qposadr[jid("left_finger_joint")]
rq = m.jnt_qposadr[jid("right_finger_joint")]

# Proven G13 initialization
d.qvel[:] = 0.0
d.qpos[root_qp:root_qp+3] = [0.0, -3.0, 0.82]

for jn, qv in zip(arm_joints, Q0):
    d.qpos[m.jnt_qposadr[jid(jn)]] = qv

d.qpos[lq] = 0.07
d.qpos[rq] = 0.07

for an, qv in zip(act_names, Q0):
    d.ctrl[aid(an)] = qv

d.ctrl[aid("act_left_finger")] = 0.0
d.ctrl[aid("act_right_finger")] = 0.0

mujoco.mj_forward(m, d)

gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:] = 0.0

for _ in range(600):
    mujoco.mj_step(m, d)

m.opt.gravity[:] = gravity_orig
mujoco.mj_forward(m, d)

# Actual post-contact geometry
bp = xpos("carry_bottle")
tp = xpos("tool0")
fp = 0.5 * (xpos("left_finger_link") + xpos("right_finger_link"))

# Actual bottle velocity and tool velocity
bv = d.cvel[bottle, 3:6].copy()
tv = d.cvel[tool0, 3:6].copy()

# Translational Jacobian of tool0
Jp = np.zeros((3, m.nv))
Jr = np.zeros((3, m.nv))
mujoco.mj_jacBody(m, d, Jp, Jr, tool0)
arm_dofs = np.array([
    m.jnt_dofadr[jid(jn)] for jn in arm_joints
], dtype=int)
J = Jp[:, arm_dofs]

sv = np.linalg.svd(J, compute_uv=False)
rank = int(np.linalg.matrix_rank(J))
cond = float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else float("inf")
dq_track = np.linalg.pinv(J, rcond=1e-4) @ (bv - tv)

# Contact state
lc = rc = False
for ci in range(d.ncon):
    a = int(d.contact[ci].geom1)
    b = int(d.contact[ci].geom2)
    if {a,b} == {bgeom,lgeom}: lc = True
    if {a,b} == {bgeom,rgeom}: rc = True

print("=" * 72)
print("G14 DYNAMIC TRACKING PROBE")
print("XML MODIFIED = NO")
print("=" * 72)
print("bottle_pos       =", bp)
print("tool0_pos        =", tp)
print("finger_center    =", fp)
print("bottle-tool      =", bp - tp)
print("bottle velocity  =", bv)
print("tool velocity    =", tv)
print("J_position       =")
print(J)
print("singular_values  =", sv)
print("rank             =", rank)
print("condition        =", cond)
print("dq_tracking      =", dq_track)
print("dq_finite        =", np.isfinite(dq_track).all())
print("left_contact     =", lc)
print("right_contact    =", rc)
print("ncon             =", d.ncon)

ok = (
    np.isfinite(J).all()
    and rank == 3
    and np.isfinite(cond)
    and np.isfinite(dq_track).all()
    and (lc and rc)
)

print("DYNAMIC_TRACKING_PROBE =", "PASS" if ok else "FAIL")
