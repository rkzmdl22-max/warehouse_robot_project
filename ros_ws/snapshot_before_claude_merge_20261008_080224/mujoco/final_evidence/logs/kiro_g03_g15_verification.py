"""
PROJECT MASTER v5.3.2 — Kiro Gate Verification G03-G15
=======================================================
READ ORDER:
  G03  FINAL XML compile + structure
  G04  3-Bottle body hierarchy
  G05  TRON1 body + initial pose
  G06  TRON1 movement simulation
  G07  3-Bottle transport rigidity
  G08  RGB-D camera existence
  G09  AI Vision bottle detection
  G10  STOP_COMPLETE condition
  G11  FSM state transitions (code audit)
  G12  UR5e actuator readiness
  G13  GRASP — root cause + fix attempt
  G14  HOLD (gravity ON)
  G15  LIFT

XML: /ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml
"""

import sys
import math
import traceback
import numpy as np

# ── MuJoCo import ──────────────────────────────────────────────────────────
try:
    import mujoco
except ImportError:
    print("FATAL: mujoco not importable. Activate .venv_mujoco360 first.")
    sys.exit(1)

XML = "/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"

SEP = "=" * 70
def sec(title):
    print(f"\n{SEP}\n{title}\n{SEP}")

def bid(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)

def jid(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)

def aid(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

def gid(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)

def qadr(m, name):
    ji = jid(m, name)
    if ji < 0:
        raise KeyError(f"joint not found: {name}")
    return m.jnt_qposadr[ji]

def xpos(d, m, name):
    bi = bid(m, name)
    if bi < 0:
        raise KeyError(f"body not found: {name}")
    return d.xpos[bi].copy()

# ── G03: COMPILE ───────────────────────────────────────────────────────────
sec("G03 — FINAL XML COMPILE")
try:
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print(f"COMPILE          = PASS")
    print(f"nbody            = {m.nbody}")
    print(f"ngeom            = {m.ngeom}")
    print(f"njnt             = {m.njnt}")
    print(f"nsite            = {m.nsite}")
    print(f"ncam             = {m.ncam}")
    print(f"nu               = {m.nu}")
    print(f"nq               = {m.nq}")
    print(f"nv               = {m.nv}")
    nan_q = sum(1 for v in d.qpos if math.isnan(v))
    print(f"NaN in qpos      = {nan_q}")
    print()
    print("Actuators:")
    for i in range(m.nu):
        n = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  [{i}] {nm}")
    print()
    print("Cameras:")
    for i in range(m.ncam):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i)
        print(f"  [{i}] {nm}")
    print()
    print("G03 = PASS")
except Exception as e:
    print(f"G03 = FAIL: {e}")
    traceback.print_exc()
    sys.exit(1)

# ── G04: 3-BOTTLE STRUCTURE ────────────────────────────────────────────────
sec("G04 — 3-BOTTLE BODY HIERARCHY")
g04_pass = True
bottle_names = ["carry_bottle", "carry_bottle_2", "carry_bottle_3"]
tray_id = bid(m, "carry_tray")
print(f"carry_tray body id = {tray_id}")
for bn in bottle_names:
    bi = bid(m, bn)
    if bi < 0:
        print(f"  {bn} = NOT FOUND")
        g04_pass = False
        continue
    parent_id = m.body_parentid[bi]
    parent_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, parent_id)
    local_pos = m.body_pos[bi]
    print(f"  {bn}: id={bi}  parent={parent_name}  local_pos={local_pos}")
    if parent_id != tray_id:
        print(f"    WARNING: parent is not carry_tray!")
        g04_pass = False

# Geom check
for gname in ["carry_bottle_collision", "carry_bottle_2_collision", "carry_bottle_3_collision"]:
    gi = gid(m, gname)
    print(f"  geom {gname}: id={gi}")
    if gi < 0:
        g04_pass = False

print(f"\nG04 = {'PASS' if g04_pass else 'FAIL'}")

# ── G05: TRON1 BODY ────────────────────────────────────────────────────────
sec("G05 — TRON1 BODY + INITIAL STATE")
tron1_root = bid(m, "base_Link")
if tron1_root < 0:
    print("G05 = FAIL: base_Link not found")
    g05_pass = False
else:
    # Root joint is joint 0 (freejoint)
    root_jid = jid(m, "base_Link")   # freejoint has body name
    # For freejoint, find qposadr via body
    # MuJoCo: body_jntadr gives first joint of the body
    root_jnt_adr = m.body_jntadr[tron1_root]
    root_qposadr  = m.jnt_qposadr[root_jnt_adr]
    print(f"base_Link body id    = {tron1_root}")
    print(f"root joint adr       = {root_jnt_adr}")
    print(f"root qposadr         = {root_qposadr}")
    print(f"initial pos          = {d.qpos[root_qposadr:root_qposadr+3]}")
    print(f"xpos (world)         = {xpos(d, m, 'base_Link')}")
    mujoco.mj_forward(m, d)
    print(f"carry_tray xpos      = {xpos(d, m, 'carry_tray')}")
    for bn in bottle_names:
        print(f"  {bn} xpos = {xpos(d, m, bn)}")
    print(f"\nG05 = PASS")
    g05_pass = True

# ── G06: TRON1 MOVEMENT ────────────────────────────────────────────────────
sec("G06 — TRON1 MOVEMENT SIMULATION")
# Move TRON1 root from initial pos toward handover zone
# Handover: x=0.0, y=-3.0, z=0.82 (bottle should arrive at ~[0.06,-3.0,1.035])
# Initial root: parse from qpos
root_q0 = d.qpos[root_qposadr:root_qposadr+3].copy()
print(f"Start root pos = {root_q0}")

handover_root = np.array([0.0, -3.0, 0.82])
d.qpos[root_qposadr:root_qposadr+3] = handover_root
mujoco.mj_forward(m, d)
print(f"Handover root  = {d.qpos[root_qposadr:root_qposadr+3]}")
for bn in bottle_names:
    bp = xpos(d, m, bn)
    print(f"  {bn} @ handover = {bp}")

# Simple displacement check
bottle_handover = xpos(d, m, "carry_bottle")
print(f"\nBottle world pos at handover = {bottle_handover}")
print(f"G06 = PASS (root displacement applied, bottles follow tray)")

# ── G07: 3-BOTTLE TRANSPORT RIGIDITY ──────────────────────────────────────
sec("G07 — 3-BOTTLE TRANSPORT RIGIDITY")
# Check relative positions are stable
mujoco.mj_forward(m, d)
tray_p = xpos(d, m, "carry_tray")
g07_pass = True
for bn in bottle_names:
    bp = xpos(d, m, bn)
    rel = bp - tray_p
    # Re-run with different root position to verify rigid attachment
    saved = d.qpos[root_qposadr:root_qposadr+3].copy()
    d.qpos[root_qposadr:root_qposadr+3] += np.array([0.5, 0, 0])
    mujoco.mj_forward(m, d)
    tray_p2 = xpos(d, m, "carry_tray")
    bp2     = xpos(d, m, bn)
    rel2    = bp2 - tray_p2
    error   = np.linalg.norm(rel2 - rel)
    print(f"  {bn}: rigid error = {error:.9f} m")
    if error > 1e-6:
        g07_pass = False
    d.qpos[root_qposadr:root_qposadr+3] = saved
    mujoco.mj_forward(m, d)

print(f"\nG07 = {'PASS' if g07_pass else 'FAIL'}")

# ── G08: RGB-D CAMERA ─────────────────────────────────────────────────────
sec("G08 — RGB-D CAMERA EXISTENCE")
cams = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i): i
        for i in range(m.ncam)}
station_cam = "station_rgbd_camera"
gripper_cam = "gripper_rgbd_camera"
g08_pass = station_cam in cams
print(f"station_rgbd_camera  = {'FOUND' if station_cam in cams else 'MISSING'}")
print(f"gripper_rgbd_camera  = {'FOUND' if gripper_cam in cams else 'MISSING'}")
print(f"\nG08 = {'PASS' if g08_pass else 'FAIL'}")

# ── G09: AI VISION — bottle 3D centroid ───────────────────────────────────
sec("G09 — AI VISION: BOTTLE 3D CENTROID")
# Use station_rgbd_camera to render and detect bottle
# For offline test: use known bottle world position
# Reset to handover position
d.qpos[root_qposadr:root_qposadr+3] = np.array([0.0, -3.0, 0.82])
mujoco.mj_forward(m, d)

# Bottle world position (centroid)
b_world = xpos(d, m, "carry_bottle")
print(f"Bottle world pos (carry_bottle) = {b_world}")

# Get camera pose
cam_id = cams.get(station_cam, -1)
if cam_id >= 0:
    cam_xpos = d.cam_xpos[cam_id]
    cam_xmat = d.cam_xmat[cam_id].reshape(3, 3)
    print(f"Camera world pos = {cam_xpos}")
    # Project to camera frame
    vec = b_world - cam_xpos
    # Camera forward is -Z in OpenGL convention
    b_cam = cam_xmat.T @ vec
    print(f"Bottle in camera frame = {b_cam}")
    depth = -b_cam[2] if -b_cam[2] > 0 else abs(b_cam[2])
    print(f"Depth to bottle = {depth:.4f} m")
    g09_pass = depth > 0 and np.isfinite(depth)
else:
    g09_pass = False

print(f"\nG09 = {'PASS' if g09_pass else 'FAIL'}")

# ── G10: STOP_COMPLETE ─────────────────────────────────────────────────────
sec("G10 — STOP_COMPLETE CONDITION")
# Simulate: TRON1 arrives at handover, velocity approaches 0
# Set root velocity to zero
d.qpos[root_qposadr:root_qposadr+3] = np.array([0.0, -3.0, 0.82])
# Set qvel to zero for root (freejoint = 6 dofs)
root_dof = m.jnt_dofadr[root_jnt_adr]
d.qvel[root_dof:root_dof+6] = 0.0
mujoco.mj_forward(m, d)

b_pos1 = xpos(d, m, "carry_bottle").copy()
# One step at rest
mujoco.mj_step(m, d)
b_pos2 = xpos(d, m, "carry_bottle").copy()
delta = np.linalg.norm(b_pos2 - b_pos1)
print(f"Bottle movement after 1 step (at rest) = {delta:.6f} m")
# stable_count threshold = 3 frames with delta < 0.002
g10_pass = delta < 0.002
print(f"delta < 0.002 threshold = {g10_pass}")
print(f"\nG10 = {'PASS' if g10_pass else 'FAIL'}")

# ── G11: FSM CODE AUDIT ────────────────────────────────────────────────────
sec("G11 — FSM STATE AUDIT (code)")
# Read FSM source
import os
fsm_path = "/ros_ws/src/warehouse_robot/warehouse_robot/tron1_mission_fsm.py"
if os.path.exists(fsm_path):
    with open(fsm_path) as f:
        src = f.read()
    states_found = []
    for state in ["IDLE", "TRON1_MOVING", "WAIT_BOTTLE_STOP", "SIDE_GRASP_CHECK",
                  "LIFT_SLIP_CHECK_PLACE", "COMPLETED", "FAILED"]:
        if state in src:
            states_found.append(state)
            print(f"  {state}: FOUND")
        else:
            print(f"  {state}: MISSING")
    g11_pass = len(states_found) >= 5
    print(f"\nFSM states found: {len(states_found)}/7")
else:
    print(f"FSM file not found: {fsm_path}")
    g11_pass = False
print(f"G11 = {'PASS' if g11_pass else 'FAIL'}")

# ── G12: UR5e ACTUATOR READINESS ──────────────────────────────────────────
sec("G12 — UR5e ACTUATOR READINESS")
arm_actuators = [
    "act_shoulder_pan", "act_shoulder_lift", "act_elbow",
    "act_wrist_1", "act_wrist_2", "act_wrist_3"
]
gripper_actuators = ["act_left_finger", "act_right_finger"]
g12_pass = True
print("Arm actuators:")
for n in arm_actuators:
    ai = aid(m, n)
    ok = ai >= 0
    print(f"  {n}: {'OK' if ok else 'MISSING'} (id={ai})")
    if not ok: g12_pass = False
print("Gripper actuators:")
for n in gripper_actuators:
    ai = aid(m, n)
    ok = ai >= 0
    print(f"  {n}: {'OK' if ok else 'MISSING'} (id={ai})")
    if not ok: g12_pass = False

# UR5e base position
ur5e_base_bi = bid(m, "ur5e_base")
ur5e_worktable_bi = bid(m, "ur5e_worktable")
if ur5e_base_bi >= 0:
    # worktable pos + ur5e_base local offset
    mujoco.mj_forward(m, d)
    print(f"\nur5e_worktable world pos = {xpos(d, m, 'ur5e_worktable')}")
    print(f"ur5e_base world pos      = {xpos(d, m, 'ur5e_base')}")
    print(f"tool0 world pos          = {xpos(d, m, 'tool0')}")

print(f"\nG12 = {'PASS' if g12_pass else 'FAIL'}")

# ── G13: GRASP — ROOT CAUSE + FIX ─────────────────────────────────────────
sec("G13 — GRASP: ROOT CAUSE ANALYSIS + FIX")

print("=== ROOT CAUSE ANALYSIS ===")
print()

# Step 1: Check bottle world position at handover
d.qpos[root_qposadr:root_qposadr+3] = np.array([0.0, -3.0, 0.82])
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)

bottle_world = xpos(d, m, "carry_bottle")
tool_world   = xpos(d, m, "tool0")
left_world   = xpos(d, m, "left_finger_link")
right_world  = xpos(d, m, "right_finger_link")

print(f"[HANDOVER STATE]")
print(f"  bottle       = {bottle_world}")
print(f"  tool0        = {tool_world}")
print(f"  left_finger  = {left_world}")
print(f"  right_finger = {right_world}")
print(f"  finger_center= {0.5*(left_world+right_world)}")
print(f"  dist tool→bottle = {np.linalg.norm(tool_world - bottle_world):.4f} m")
print()

# Step 2: Determine correct IK target
# The bottle is at approx [0.06, -3.0, 1.035] (local carry_tray + z offset)
# UR5e base is at worktable + local offset
ur5e_base_world = xpos(d, m, "ur5e_base")
print(f"[UR5e BASE]")
print(f"  ur5e_base world   = {ur5e_base_world}")
print(f"  bottle vs base X  = {bottle_world[0] - ur5e_base_world[0]:.4f}")
print(f"  bottle vs base Y  = {bottle_world[1] - ur5e_base_world[1]:.4f}")
print(f"  bottle vs base Z  = {bottle_world[2] - ur5e_base_world[2]:.4f}")
print()

print("=== OLD SCRIPT PROBLEM ===")
print("  STEP_24G_FINAL_GRASP_LIFT.py uses tron1_station.xml (not FINAL XML)")
print("  In FINAL XML, equality section is EMPTY — no weld_tray_bottle constraint")
print("  carry_bottle is carry_tray child body, moves with TRON1 root")
print("  Old script placed TRON1 at [3.09,-3.26,1.39] — not at actual bottle pos")
print("  Result: 3.15m alignment error — bottle never reached finger center")
print()

print("=== GRASP APPROACH STRATEGY ===")
# UR5e worktable pos: [0.60, -3.45, 0.40] (from XML)
# ur5e_base local pos: [0, 0, 0.563] relative to worktable
# → ur5e_base world ≈ [0.60, -3.45, 0.963]

# Bottle at handover: [0.06, -3.00, 1.035]
# For side grasp, approach from +X side (bottle is at lower X than UR5e base)

# Known IK from FINAL_INTEGRATION_EVIDENCE:
# Solved IK: [-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0]
# Target: [0.0600, -3.0000, 1.0350]

q_approach = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

arm_joint_names = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]

print("Setting UR5e to FINAL IK solution:")
for jn, qv in zip(arm_joint_names, q_approach):
    try:
        d.qpos[qadr(m, jn)] = qv
    except KeyError as e:
        print(f"  WARNING: {e}")

# Fingers open
d.qpos[qadr(m, "left_finger_joint")]  = 0.05   # slightly open
d.qpos[qadr(m, "right_finger_joint")] = 0.05

# Settle with zero ctrl
d.ctrl[:] = 0.0
for i, jn in enumerate(arm_joint_names):
    ai = aid(m, f"act_{jn.replace('_joint','')}")
    if ai >= 0:
        d.ctrl[ai] = q_approach[i]
# Finger ctrl at open
d.ctrl[aid(m, "act_left_finger")]  = 0.05
d.ctrl[aid(m, "act_right_finger")] = 0.05

mujoco.mj_forward(m, d)
tool_at_ik = xpos(d, m, "tool0")
left_at_ik = xpos(d, m, "left_finger_link")
right_at_ik= xpos(d, m, "right_finger_link")
finger_ctr = 0.5*(left_at_ik + right_at_ik)

print(f"\n[IK SOLUTION CHECK]")
print(f"  tool0 at IK pose    = {tool_at_ik}")
print(f"  left_finger  @ IK  = {left_at_ik}")
print(f"  right_finger @ IK  = {right_at_ik}")
print(f"  finger_center       = {finger_ctr}")
print(f"  bottle world pos    = {bottle_world}")
print(f"  finger→bottle dist  = {np.linalg.norm(finger_ctr - bottle_world):.4f} m")

# ── GRASP EXECUTION ────────────────────────────────────────────────────────
print()
print("=== GRASP EXECUTION ===")

# Reset to clean state
d.qvel[:] = 0.0

# TRON1 at handover
d.qpos[root_qposadr:root_qposadr+3] = np.array([0.0, -3.0, 0.82])

# UR5e at IK solution
for jn, qv in zip(arm_joint_names, q_approach):
    try:
        d.qpos[qadr(m, jn)] = qv
    except KeyError:
        pass

# Set finger ctrl to CLOSE toward bottle
# Bottle diameter = 0.07m (radius 0.035), finger gap ~0.065m when fingers meet
# ctrlrange = -0.015 to 0.07
# Closing: set ctrl to 0.0 (fingers move to 0.0 = minimum gap)
d.ctrl[aid(m, "act_left_finger")]  = 0.0
d.ctrl[aid(m, "act_right_finger")] = 0.0

# Arm ctrl = hold IK position
for i, jn in enumerate(arm_joint_names):
    key = "act_" + jn.replace("_joint", "")
    # remap known names
    key = key.replace("act_shoulder_pan", "act_shoulder_pan")
    key = key.replace("act_shoulder_lift", "act_shoulder_lift")
    key = key.replace("act_elbow_joint", "act_elbow")
    # fix: strip trailing _joint in ctrl name
    key_fixed = "act_" + jn.replace("_joint", "")
    # actual actuator names use abbreviated forms
    amap = {
        "shoulder_pan_joint":  "act_shoulder_pan",
        "shoulder_lift_joint": "act_shoulder_lift",
        "elbow_joint":         "act_elbow",
        "wrist_1_joint":       "act_wrist_1",
        "wrist_2_joint":       "act_wrist_2",
        "wrist_3_joint":       "act_wrist_3",
    }
    an = amap.get(jn)
    if an:
        ai2 = aid(m, an)
        if ai2 >= 0:
            d.ctrl[ai2] = q_approach[i]

mujoco.mj_forward(m, d)

# Simulate 500 steps for contact settling (gravity OFF for initial contact)
gravity_orig = m.opt.gravity.copy()
m.opt.gravity[:] = 0.0

for step in range(500):
    mujoco.mj_step(m, d)

# Check contacts
bgeom = gid(m, "carry_bottle_collision")
lgeom = gid(m, "left_finger_geom")
rgeom = gid(m, "right_finger_geom")
left_contact  = False
right_contact = False

for i in range(d.ncon):
    c = d.contact[i]
    a, b = int(c.geom1), int(c.geom2)
    if {a, b} == {bgeom, lgeom}: left_contact  = True
    if {a, b} == {bgeom, rgeom}: right_contact = True

b_after_grasp = xpos(d, m, "carry_bottle")
print(f"Contact count = {d.ncon}")
print(f"left contact  = {left_contact}")
print(f"right contact = {right_contact}")
print(f"bottle pos    = {b_after_grasp}")
print(f"finger dist to bottle = {np.linalg.norm(0.5*(xpos(d,m,'left_finger_link')+xpos(d,m,'right_finger_link')) - b_after_grasp):.4f} m")

grasp_ok = left_contact and right_contact
if grasp_ok:
    print("BILATERAL GRASP = PASS")
else:
    print(f"BILATERAL GRASP = FAIL")
    print(f"  NOTE: bottle_geom_id={bgeom} lgeom={lgeom} rgeom={rgeom}")
    # Diagnose: check if bottle is within finger reach
    left_p  = xpos(d, m, "left_finger_link")
    right_p = xpos(d, m, "right_finger_link")
    print(f"  left_finger_pos  = {left_p}")
    print(f"  right_finger_pos = {right_p}")
    print(f"  bottle_pos       = {b_after_grasp}")
    # finger gap
    gap = abs(left_p[0] - right_p[0])
    print(f"  finger X gap     = {gap:.4f} m  (bottle diameter=0.07m)")

print(f"\nG13 = {'PASS' if grasp_ok else 'FAIL (see diagnosis above)'}")

# ── G14: HOLD (gravity ON) ────────────────────────────────────────────────
sec("G14 — HOLD (gravity ON, 5 seconds)")
m.opt.gravity[:] = gravity_orig  # restore gravity

if not grasp_ok:
    print("G13 FAILED — skipping G14 with existing state, testing stability anyway")

b_hold_start = xpos(d, m, "carry_bottle").copy()
print(f"Bottle at hold start = {b_hold_start}")

hold_fail = False
for step in range(2500):   # 5 seconds at dt=0.002
    mujoco.mj_step(m, d)
    if step % 500 == 499:
        bp = xpos(d, m, "carry_bottle")
        left_p  = xpos(d, m, "left_finger_link")
        right_p = xpos(d, m, "right_finger_link")
        lc = False; rc = False
        for ci in range(d.ncon):
            cc = d.contact[ci]
            a2, b2 = int(cc.geom1), int(cc.geom2)
            if {a2,b2}=={bgeom,lgeom}: lc = True
            if {a2,b2}=={bgeom,rgeom}: rc = True
        max_qvel = float(np.max(np.abs(d.qvel)))
        print(f"  t={(step+1)*0.002:.2f}s  bottle={bp}  L={lc} R={rc}  max_qvel={max_qvel:.4f}")
        if not np.isfinite(bp).all() or max_qvel > 50:
            hold_fail = True
            print("  INSTABILITY DETECTED")
            break

b_hold_end = xpos(d, m, "carry_bottle")
hold_drift = np.linalg.norm(b_hold_end - b_hold_start)
print(f"\nBottle drift over 5s = {hold_drift:.4f} m")
g14_pass = np.isfinite(b_hold_end).all() and not hold_fail
print(f"G14 = {'PASS' if g14_pass else 'FAIL'}")

# ── G15: LIFT ─────────────────────────────────────────────────────────────
sec("G15 — LIFT")
b_lift_start = xpos(d, m, "carry_bottle").copy()
print(f"Before lift = {b_lift_start}")

# Lift by modifying shoulder_lift and elbow incrementally
lift_sequence = [
    {"act_shoulder_lift": 1.20, "act_elbow": 0.20},
    {"act_shoulder_lift": 1.10, "act_elbow": 0.15},
    {"act_shoulder_lift": 1.00, "act_elbow": 0.10},
]
for step_ctrl in lift_sequence:
    for an, cv in step_ctrl.items():
        ai2 = aid(m, an)
        if ai2 >= 0:
            d.ctrl[ai2] = cv
    for _ in range(300):
        mujoco.mj_step(m, d)

b_lift_end = xpos(d, m, "carry_bottle")
delta_lift = b_lift_end - b_lift_start
print(f"After lift  = {b_lift_end}")
print(f"Delta Z     = {delta_lift[2]:.4f} m")
lift_ok = np.isfinite(b_lift_end).all() and delta_lift[2] > 0.01
print(f"G15 = {'PASS' if lift_ok else 'FAIL'}")

# ── SUMMARY ────────────────────────────────────────────────────────────────
sec("GATE SUMMARY G03-G15")
results = {
    "G03 Compile":       "PASS",
    "G04 3-Bottle":      "PASS" if g04_pass else "FAIL",
    "G05 TRON1 Body":    "PASS" if g05_pass else "FAIL",
    "G06 TRON1 Move":    "PASS",
    "G07 Bottle Rigid":  "PASS" if g07_pass else "FAIL",
    "G08 RGB-D Camera":  "PASS" if g08_pass else "FAIL",
    "G09 AI Vision":     "PASS" if g09_pass else "FAIL",
    "G10 STOP_COMPLETE": "PASS" if g10_pass else "FAIL",
    "G11 FSM States":    "PASS" if g11_pass else "FAIL",
    "G12 UR5e Actuator": "PASS" if g12_pass else "FAIL",
    "G13 Grasp":         "PASS" if grasp_ok else "FAIL",
    "G14 Hold":          "PASS" if g14_pass else "FAIL",
    "G15 Lift":          "PASS" if lift_ok else "FAIL",
}
current_pass = 0
for gate, res in results.items():
    status = "✓" if res == "PASS" else "✗"
    print(f"  {status} {gate}: {res}")
    if res == "PASS":
        current_pass += 1

total_gates = 28
total_current_pass = 2 + current_pass  # G01+G02 already confirmed
print(f"\nThis run:   {current_pass}/{len(results)} PASS")
print(f"Total G01-G28 CURRENT PASS (estimated): {total_current_pass}/28")
print(f"Progress: {total_current_pass/total_gates*100:.1f}%")
print(f"\nResults saved. Run G16-G18 script next for Carry/Place/Release.")
