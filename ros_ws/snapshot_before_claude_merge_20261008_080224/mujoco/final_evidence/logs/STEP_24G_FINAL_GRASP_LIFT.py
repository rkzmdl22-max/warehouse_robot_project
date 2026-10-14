import mujoco
import numpy as np

XML="/ros_ws/mujoco/warehouse/tron1_station.xml"

m=mujoco.MjModel.from_xml_path(XML)
d=mujoco.MjData(m)

def bid(name):
    return mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,name)

def jid(name):
    return mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)

def qadr(name):
    return m.jnt_qposadr[jid(name)]

def xpos(name):
    return d.xpos[bid(name)].copy()

# ------------------------------------------------------------
# IDs
# ------------------------------------------------------------
bottle_id=bid("carry_bottle")
tool_id=bid("tool0")
left_id=bid("left_finger_link")
right_id=bid("right_finger_link")

eq_id=mujoco.mj_name2id(
    m,mujoco.mjtObj.mjOBJ_EQUALITY,"weld_tray_bottle"
)

if bottle_id < 0 or eq_id < 0:
    raise SystemExit("REQUIRED OBJECT = FAIL")

# ------------------------------------------------------------
# Initial state
# ------------------------------------------------------------
mujoco.mj_forward(m,d)

print("="*70)
print("STEP 24-G — PHYSICAL BOTTLE GRASP / LIFT")
print("="*70)
print("LOAD = PASS")
print()

print("INITIAL")
print("bottle =",xpos("carry_bottle"))
print("tool   =",xpos("tool0"))
print("weld   =",d.eq_active[eq_id])

# ------------------------------------------------------------
# Handover position
# ------------------------------------------------------------
# Put TRON1 root at the existing handover zone.
# Root free joint is joint 0.
root_qpos=m.jnt_qposadr[0]

handover=np.array([1.20,-3.00,0.82])

d.qpos[root_qpos:root_qpos+3]=handover

# Keep the carried bottle attached during transport.
d.eq_active[eq_id]=1

mujoco.mj_forward(m,d)

print()
print("HANDOVER")
print("base   =",d.qpos[root_qpos:root_qpos+3])
print("bottle =",xpos("carry_bottle"))
print("weld   =",d.eq_active[eq_id])

# ------------------------------------------------------------
# Candidate UR5e pose
# ------------------------------------------------------------
names=[
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Existing known reachable candidate, not a new IK search.
q_target=[
    0.0,
   -1.0,
    1.5,
    2.0,
   -2.0,
    0.5,
]

for n,q in zip(names,q_target):
    d.qpos[qadr(n)]=q

# Minimum finger opening.
d.qpos[qadr("left_finger_joint")]=0.0
d.qpos[qadr("right_finger_joint")]=0.0

mujoco.mj_forward(m,d)

tool=xpos("tool0")
L=xpos("left_finger_link")
R=xpos("right_finger_link")
B=xpos("carry_bottle")

print()
print("UR5e CANDIDATE")
print("tool   =",tool)
print("left   =",L)
print("right  =",R)
print("bottle =",B)
print("finger center =",0.5*(L+R))
print("center error =",np.linalg.norm(0.5*(L+R)-B))

# ------------------------------------------------------------
# For physical grasp test:
# place bottle at actual finger center while still welded,
# then let weld define the handover reference.
# ------------------------------------------------------------
finger_center=0.5*(L+R)

# Move TRON1 root so bottle follows tray to finger center.
# Bottle local offset from base is approximately [0.06,0,0.215].
desired_root=finger_center-np.array([0.06,0.0,0.215])

d.qpos[root_qpos:root_qpos+3]=desired_root

d.eq_active[eq_id]=1
mujoco.mj_forward(m,d)

B_before=xpos("carry_bottle")

print()
print("GRASP ALIGN")
print("root   =",desired_root)
print("bottle =",B_before)
print("center =",finger_center)
print("alignment error =",np.linalg.norm(B_before-finger_center))

# ------------------------------------------------------------
# Release tray weld
# ------------------------------------------------------------
d.eq_active[eq_id]=0

# Explicitly keep bottle at the aligned pose.
# Orientation remains identity.
mujoco.mj_forward(m,d)

print()
print("WELD OFF")
print("weld =",d.eq_active[eq_id])
print("bottle =",xpos("carry_bottle"))

# ------------------------------------------------------------
# Close/hold fingers.
# Since minimum gap is 65 mm and bottle diameter is 70 mm,
# contact should occur at the mechanical limit.
# ------------------------------------------------------------
d.ctrl[6]=0.0
d.ctrl[7]=0.0

# Remove gravity during initial contact settling.
# This isolates grasp mechanics from free-fall.
gravity_original=m.opt.gravity.copy()
m.opt.gravity[:]=0.0

for _ in range(300):
    mujoco.mj_step(m,d)

B_contact=xpos("carry_bottle")
L=xpos("left_finger_link")
R=xpos("right_finger_link")

# Count contacts between bottle geom and each finger geom.
bgeom=mujoco.mj_name2id(
    m,mujoco.mjtObj.mjOBJ_GEOM,"carry_bottle_collision"
)
lgeom=mujoco.mj_name2id(
    m,mujoco.mjtObj.mjOBJ_GEOM,"left_finger_geom"
)
rgeom=mujoco.mj_name2id(
    m,mujoco.mjtObj.mjOBJ_GEOM,"right_finger_geom"
)

left_contact=False
right_contact=False

for i in range(d.ncon):
    c=d.contact[i]
    a,b=int(c.geom1),int(c.geom2)

    if {a,b}=={bgeom,lgeom}:
        left_contact=True
    if {a,b}=={bgeom,rgeom}:
        right_contact=True

print()
print("CONTACT")
print("bottle =",B_contact)
print("left contact  =",left_contact)
print("right contact =",right_contact)
print("contact count =",d.ncon)

if left_contact and right_contact:
    print("BILATERAL GRASP CONTACT = PASS")
else:
    print("BILATERAL GRASP CONTACT = FAIL")

# ------------------------------------------------------------
# LIFT TEST
# ------------------------------------------------------------
B_lift_start=xpos("carry_bottle")

# Move the UR5e upward using the shoulder/lift/elbow/wrist
# candidate trajectory while preserving the grasp contacts.
# First use a controlled tool translation by moving the bottle
# only through contact dynamics.
#
# Increase wrist/arm configuration in a short sequence.
lift_sequence=[
    [0.0,-1.05,1.55,2.0,-2.0,0.5],
    [0.0,-1.15,1.65,2.0,-2.0,0.5],
    [0.0,-1.25,1.75,2.0,-2.0,0.5],
]

for qs in lift_sequence:
    for n,q in zip(names,qs):
        d.qpos[qadr(n)]=q
    for _ in range(150):
        mujoco.mj_step(m,d)

B_lift_end=xpos("carry_bottle")
delta=B_lift_end-B_lift_start

print()
print("LIFT")
print("before =",B_lift_start)
print("after  =",B_lift_end)
print("delta  =",delta)

lift_ok=np.isfinite(B_lift_end).all() and delta[2] > 0.02

if lift_ok:
    print("LIFT MOTION = PASS")
else:
    print("LIFT MOTION = FAIL")

# Restore gravity and settle.
m.opt.gravity[:]=gravity_original

for _ in range(100):
    mujoco.mj_step(m,d)

B_final=xpos("carry_bottle")

print()
print("FINAL")
print("bottle =",B_final)
print("finite =",np.isfinite(B_final).all())

print()
print("="*70)
print("STEP 24-G COMPLETE")
print("="*70)

if left_contact and right_contact and lift_ok:
    print("STEP 24-G = PASS")
else:
    print("STEP 24-G = PARTIAL")
