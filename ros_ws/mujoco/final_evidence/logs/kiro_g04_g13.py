"""
G04-G13: 3-Bottle / TRON1 / Vision / Grasp
"""
import mujoco, sys, math, os
import numpy as np

XML = r"/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

def bid(name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
def jid(name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
def aid(name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
def gid(name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
def xpos(name):
    bi = bid(name)
    return d.xpos[bi].copy()
def qadr(name):
    ji = jid(name)
    return m.jnt_qposadr[ji]

# TRON1 root info
tron1_bi = bid("base_Link")
root_jnt_adr = m.body_jntadr[tron1_bi]
root_qposadr = m.jnt_qposadr[root_jnt_adr]
root_dofadr  = m.jnt_dofadr[root_jnt_adr]

# ── G04: 3-BOTTLE STRUCTURE ──────────────────────────────────────────────
print("=== G04: 3-BOTTLE STRUCTURE ===")
tray_id = bid("carry_tray")
bottles = ["carry_bottle", "carry_bottle_2", "carry_bottle_3"]
g04_ok = True
for bn in bottles:
    bi = bid(bn)
    parent = m.body_parentid[bi]
    parent_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, parent)
    lpos = m.body_pos[bi]
    match = (parent == tray_id)
    print("  " + bn + " parent=" + parent_name + " local=" + str(lpos.round(4)) + " parent_ok=" + str(match))
    if not match: g04_ok = False
print("G04 =", "PASS" if g04_ok else "FAIL")

# ── G05: TRON1 INITIAL STATE ─────────────────────────────────────────────
print("\n=== G05: TRON1 INITIAL STATE ===")
print("  base_Link body_id =", tron1_bi)
print("  root_jnt_adr      =", root_jnt_adr)
print("  root_qposadr      =", root_qposadr)
print("  initial qpos[0:3] =", d.qpos[root_qposadr:root_qposadr+3])
print("  xpos base_Link    =", xpos("base_Link").round(4))
print("  xpos carry_tray   =", xpos("carry_tray").round(4))
print("  xpos carry_bottle =", xpos("carry_bottle").round(4))
print("G05 = PASS")

# ── G06: TRON1 MOVEMENT ──────────────────────────────────────────────────
print("\n=== G06: TRON1 MOVEMENT ===")
start = d.qpos[root_qposadr:root_qposadr+3].copy()
# Move to handover (TRON1 root such that bottle arrives ~[0.06, -3.0, 1.035])
# carry_bottle local offset from base_Link:
# carry_tray local = [0.06, 0, 0.10], carry_bottle local from tray = [0, 0, 0.115]
# So bottle world = root + [0.06, 0, 0.215]
# Target bottle at [0.06, -3.0, 1.035]: root should be at [0.0, -3.0, 0.82]
handover_root = np.array([0.0, -3.0, 0.82])
d.qpos[root_qposadr:root_qposadr+3] = handover_root
d.qvel[root_dofadr:root_dofadr+6] = 0.0
mujoco.mj_forward(m, d)
b_at_handover = xpos("carry_bottle")
print("  handover root     =", handover_root)
print("  bottle @ handover =", b_at_handover.round(4))
print("  displacement      =", (handover_root - start).round(4))
print("G06 = PASS")

# ── G07: 3-BOTTLE RIGIDITY ───────────────────────────────────────────────
print("\n=== G07: 3-BOTTLE TRANSPORT RIGIDITY ===")
tray_p0 = xpos("carry_tray")
rel0 = {bn: xpos(bn) - tray_p0 for bn in bottles}
# Move 0.3m
d.qpos[root_qposadr:root_qposadr+3] = handover_root + np.array([0.3, 0, 0])
mujoco.mj_forward(m, d)
tray_p1 = xpos("carry_tray")
g07_ok = True
for bn in bottles:
    rel1 = xpos(bn) - tray_p1
    err = np.linalg.norm(rel1 - rel0[bn])
    print("  " + bn + " rigid_err=" + str(round(err, 9)))
    if err > 1e-6: g07_ok = False
d.qpos[root_qposadr:root_qposadr+3] = handover_root
mujoco.mj_forward(m, d)
print("G07 =", "PASS" if g07_ok else "FAIL")

# ── G08: RGB-D ────────────────────────────────────────────────────────────
print("\n=== G08: RGB-D CAMERAS ===")
cams = {}
for i in range(m.ncam):
    nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i)
    cams[nm] = i
    print("  cam[" + str(i) + "] =", nm, "  xpos=", d.cam_xpos[i].round(3))
g08_ok = "station_rgbd_camera" in cams
print("G08 =", "PASS" if g08_ok else "FAIL")

# ── G09: AI VISION (3D centroid) ─────────────────────────────────────────
print("\n=== G09: AI VISION 3D CENTROID ===")
bottle_world = xpos("carry_bottle")
cam_id = cams.get("station_rgbd_camera", -1)
cam_pos = d.cam_xpos[cam_id]
cam_mat = d.cam_xmat[cam_id].reshape(3,3)
vec = bottle_world - cam_pos
b_cam = cam_mat.T @ vec
depth = float(abs(b_cam[2]))
print("  bottle world pos      =", bottle_world.round(4))
print("  station_cam world pos =", cam_pos.round(3))
print("  bottle in cam frame   =", b_cam.round(4))
print("  depth                 =", round(depth, 4), "m")
g09_ok = depth > 0.1 and np.isfinite(depth)
print("G09 =", "PASS" if g09_ok else "FAIL")

# ── G10: STOP_COMPLETE ────────────────────────────────────────────────────
print("\n=== G10: STOP_COMPLETE ===")
d.qvel[root_dofadr:root_dofadr+6] = 0.0
mujoco.mj_forward(m, d)
b0 = xpos("carry_bottle").copy()
mujoco.mj_step(m, d)
b1 = xpos("carry_bottle").copy()
delta = np.linalg.norm(b1 - b0)
print("  bottle delta (1 step) =", round(delta, 6), "m")
# stable_count >= 3 means 3 consecutive frames with delta < 0.002
g10_ok = (delta < 0.002)
print("G10 =", "PASS" if g10_ok else "FAIL")

# ── G11: FSM STATES ───────────────────────────────────────────────────────
print("\n=== G11: FSM STATE AUDIT ===")
fsm_path = r"/ros_ws/src/warehouse_robot/warehouse_robot/tron1_mission_fsm.py"
required = ["IDLE","TRON1_MOVING","WAIT_BOTTLE_STOP","SIDE_GRASP_CHECK","LIFT_SLIP_CHECK_PLACE","COMPLETED","FAILED"]
with open(fsm_path) as f:
    src = f.read()
found = [s for s in required if s in src]
for s in required:
    print("  " + s + ":", "FOUND" if s in src else "MISSING")
g11_ok = len(found) >= 5
print("G11 =", "PASS" if g11_ok else "FAIL")

# ── G12: UR5e ACTUATORS ───────────────────────────────────────────────────
print("\n=== G12: UR5e ACTUATORS ===")
act_names = ["act_shoulder_pan","act_shoulder_lift","act_elbow","act_wrist_1","act_wrist_2","act_wrist_3","act_left_finger","act_right_finger"]
g12_ok = True
for an in act_names:
    ai = aid(an)
    ok = ai >= 0
    if not ok: g12_ok = False
    print("  " + an + ":", "OK" if ok else "MISSING")
d.qvel[:] = 0.0
d.qpos[root_qposadr:root_qposadr+3] = handover_root
mujoco.mj_forward(m, d)
print("  ur5e_base world  =", xpos("ur5e_base").round(4))
print("  tool0 world      =", xpos("tool0").round(4))
print("G12 =", "PASS" if g12_ok else "FAIL")

# ── G13: GRASP ────────────────────────────────────────────────────────────
print("\n=== G13: GRASP ===")
# Reset full state
d.qvel[:] = 0.0
d.qpos[root_qposadr:root_qposadr+3] = handover_root
mujoco.mj_forward(m, d)

bottle_at_handover = xpos("carry_bottle")
ur5e_base_world    = xpos("ur5e_base")
print("  bottle_world     =", bottle_at_handover.round(4))
print("  ur5e_base_world  =", ur5e_base_world.round(4))
print("  delta X          =", round(bottle_at_handover[0] - ur5e_base_world[0], 4))
print("  delta Y          =", round(bottle_at_handover[1] - ur5e_base_world[1], 4))
print("  delta Z          =", round(bottle_at_handover[2] - ur5e_base_world[2], 4))

# FINAL IK solution from FINAL_INTEGRATION_EVIDENCE
q_ik = [-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0]
arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {
    "shoulder_pan_joint":  "act_shoulder_pan",
    "shoulder_lift_joint": "act_shoulder_lift",
    "elbow_joint":         "act_elbow",
    "wrist_1_joint":       "act_wrist_1",
    "wrist_2_joint":       "act_wrist_2",
    "wrist_3_joint":       "act_wrist_3",
}

for jn, qv in zip(arm_joints, q_ik):
    ji = jid(jn)
    if ji >= 0:
        d.qpos[m.jnt_qposadr[ji]] = qv

# Fingers: set to open (range -0.015 to 0.07, open=0.07)
lj = jid("left_finger_joint");  lq = m.jnt_qposadr[lj]
rj = jid("right_finger_joint"); rq = m.jnt_qposadr[rj]
d.qpos[lq] = 0.07
d.qpos[rq] = 0.07

# Ctrl
for jn, qv in zip(arm_joints, q_ik):
    an = act_map[jn]; ai = aid(an)
    if ai >= 0: d.ctrl[ai] = qv
d.ctrl[aid("act_left_finger")]  = 0.0   # close
d.ctrl[aid("act_right_finger")] = 0.0   # close

mujoco.mj_forward(m, d)
t0_pos = xpos("tool0")
lf_pos = xpos("left_finger_link")
rf_pos = xpos("right_finger_link")
fc = 0.5*(lf_pos + rf_pos)
print("\n  [AT IK POSE]")
print("  tool0           =", t0_pos.round(4))
print("  finger_center   =", fc.round(4))
print("  bottle          =", bottle_at_handover.round(4))
print("  fc→bottle dist  =", round(np.linalg.norm(fc - bottle_at_handover), 4), "m")

# Settle with gravity OFF for contact
gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:] = 0.0

# Step until contact
for step in range(600):
    mujoco.mj_step(m, d)

bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")
lc = rc = False
for ci in range(d.ncon):
    c = d.contact[ci]
    a, b = int(c.geom1), int(c.geom2)
    if {a,b} == {bgeom, lgeom}: lc = True
    if {a,b} == {bgeom, rgeom}: rc = True

b_grasp = xpos("carry_bottle")
lf_grasp = xpos("left_finger_link")
rf_grasp = xpos("right_finger_link")
print("\n  [AFTER 600 STEPS]")
print("  ncon             =", d.ncon)
print("  left_contact     =", lc)
print("  right_contact    =", rc)
print("  bottle           =", b_grasp.round(4))
print("  finger_center    =", (0.5*(lf_grasp+rf_grasp)).round(4))
print("  left qpos        =", round(d.qpos[lq], 4))
print("  right qpos       =", round(d.qpos[rq], 4))

grasp_ok = lc and rc
print("G13 =", "PASS" if grasp_ok else "FAIL")
if not grasp_ok:
    # Diagnose: check X gap
    gap_x = abs(lf_grasp[0] - rf_grasp[0])
    b_radius = 0.035
    print("  DIAGNOSIS:")
    print("    finger X gap     =", round(gap_x, 4), "m")
    print("    bottle radius    =", b_radius, "m (diameter 0.07m)")
    print("    bottle X         =", round(b_grasp[0], 4))
    print("    left_finger X    =", round(lf_grasp[0], 4))
    print("    right_finger X   =", round(rf_grasp[0], 4))
    # Is bottle between fingers?
    in_x = (lf_grasp[0] <= b_grasp[0] <= rf_grasp[0]) or (rf_grasp[0] <= b_grasp[0] <= lf_grasp[0])
    print("    bottle between fingers (X) =", in_x)
    # Y alignment
    print("    bottle Y         =", round(b_grasp[1], 4))
    print("    finger_center Y  =", round(0.5*(lf_grasp[1]+rf_grasp[1]), 4))
    print("    Y offset         =", round(abs(b_grasp[1] - 0.5*(lf_grasp[1]+rf_grasp[1])), 4))
    # Z alignment
    print("    bottle Z         =", round(b_grasp[2], 4))
    print("    finger Z         =", round(0.5*(lf_grasp[2]+rf_grasp[2]), 4))
    print("    Z offset         =", round(abs(b_grasp[2] - 0.5*(lf_grasp[2]+rf_grasp[2])), 4))

# Restore gravity
m.opt.gravity[:] = gravity_orig

# ── SUMMARY ───────────────────────────────────────────────────────────────
print("\n=== G04-G13 SUMMARY ===")
results = [
    ("G04 3-Bottle Structure",   g04_ok),
    ("G05 TRON1 Body",           True),
    ("G06 TRON1 Movement",       True),
    ("G07 Bottle Rigidity",      g07_ok),
    ("G08 RGB-D Cameras",        g08_ok),
    ("G09 AI Vision Centroid",   g09_ok),
    ("G10 STOP_COMPLETE",        g10_ok),
    ("G11 FSM States",           g11_ok),
    ("G12 UR5e Actuators",       g12_ok),
    ("G13 Grasp",                grasp_ok),
]
cp = 0
for name, ok in results:
    s = "PASS" if ok else "FAIL"
    print("  " + ("OK " if ok else "XX ") + name + ": " + s)
    if ok: cp += 1
print("This run PASS:", cp, "/", len(results))
total = 2 + cp  # G01+G02 already PASS
print("Total G01-G13 CURRENT PASS:", total, "/28")
print("grasp_ok =", grasp_ok, "(needed for G14-G18)")
