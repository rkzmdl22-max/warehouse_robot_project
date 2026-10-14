"""
G14 Millisecond-Level Diagnostic
Detailed step-by-step analysis of first contact loss mechanism
"""
import mujoco
import numpy as np
import sys

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"

print("=" * 68)
print("G14 MILLISECOND-LEVEL DIAGNOSTIC")
print("=" * 68)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

# CRITICAL: Capture original gravity BEFORE any modification
gravity_orig = m.opt.gravity.copy()
print(f"Original gravity: {gravity_orig}")

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def xpos(n): return d.xpos[bid(n)].copy()
def qadr(n): return m.jnt_qposadr[jid(n)]

arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
act_map = {"shoulder_pan_joint":"act_shoulder_pan","shoulder_lift_joint":"act_shoulder_lift",
           "elbow_joint":"act_elbow","wrist_1_joint":"act_wrist_1",
           "wrist_2_joint":"act_wrist_2","wrist_3_joint":"act_wrist_3"}
Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])

tron1_bi = bid("base_Link")
root_jnt  = m.body_jntadr[tron1_bi]
root_qp   = m.jnt_qposadr[root_jnt]
root_dof  = m.jnt_dofadr[root_jnt]
lq = qadr("left_finger_joint")
rq = qadr("right_finger_joint")
bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom")
rgeom = gid("right_finger_geom")
ai_lf = aid("act_left_finger")
ai_rf = aid("act_right_finger")
HANDOVER_QPOS7 = np.array([0.0,-3.0,0.82, 1.0,0.0,0.0,0.0])

def contact():
    lc=rc=False
    left_dist=right_dist=None
    for ci in range(d.ncon):
        c=d.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bgeom,lgeom}:
            lc=True
            left_dist = c.dist
        if {a,b}=={bgeom,rgeom}:
            rc=True
            right_dist = c.dist
    return lc, rc, left_dist, right_dist

def set_ctrl(q_arm, q_f):
    for jn,qv in zip(arm_joints,q_arm):
        ai=aid(act_map[jn])
        if ai>=0: d.ctrl[ai]=qv
    d.ctrl[ai_lf]=q_f; d.ctrl[ai_rf]=q_f

def fix_tron1():
    d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    d.qvel[root_dof:root_dof+6] = 0.0

# === G13 GRASP (ZERO-G) ===
print("\n[G13 GRASP - ZERO-G]")
d.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
d.qvel[:] = 0.0
for jn,qv in zip(arm_joints,Q0):
    d.qpos[qadr(jn)] = qv
d.qpos[lq]=0.07; d.qpos[rq]=0.07
set_ctrl(Q0, 0.0)
m.opt.gravity[:] = 0.0
mujoco.mj_forward(m, d)

# Run G13 grasp
for _ in range(600):
    fix_tron1()
    set_ctrl(Q0, 0.0)
    mujoco.mj_step(m, d)

lc0, rc0, ld0, rd0 = contact()
b_g13 = xpos("carry_bottle")
print(f"Bottle: {b_g13}")
print(f"Left contact: {lc0}, Right contact: {rc0}")
print(f"Left dist: {ld0}, Right dist: {rd0}")
print(f"Left qpos: {d.qpos[lq]}, Right qpos: {d.qpos[rq]}")

if not (lc0 and rc0):
    print("G13 NOT REPRODUCED")
    sys.exit(1)
print("G13 = PASS")

# === G14 HOLD WITH DETAILED MILLISECOND ANALYSIS ===
print("\n" + "=" * 68)
print("G14 MILLISECOND-LEVEL CONTACT LOSS ANALYSIS")
print("=" * 68)

# Restore gravity
m.opt.gravity[:] = gravity_orig
mujoco.mj_forward(m, d)

print(f"Gravity restored to: {m.opt.gravity.copy()}")

# Initialize data collection
data = []
first_loss_step = -1
loss_cause = "none"

# Run 100 steps (0.2s) with detailed logging
for step in range(100):
    fix_tron1()
    set_ctrl(Q0, -0.0105)
    mujoco.mj_step(m, d)

    lc, rc, ld, rd = contact()
    b_pos = xpos("carry_bottle")
    b_vel = d.qvel[root_dof:root_dof+3]
    l_qpos = d.qpos[lq]
    r_qpos = d.qpos[rq]
    l_qvel = d.qvel[m.jnt_dofadr[jid("left_finger_joint")]]
    r_qvel = d.qpos[m.jnt_dofadr[jid("right_finger_joint")]]
    arm_qpos = [d.qpos[qadr(jn)] for jn in arm_joints]
    arm_qvel = [d.qvel[m.jnt_dofadr[jid(jn)]] for jn in arm_joints]

    data.append({
        'step': step,
        'time': step * 0.002,
        'lc': lc,
        'rc': rc,
        'ld': ld,
        'rd': rd,
        'b_pos': b_pos.copy(),
        'b_vel': b_vel.copy(),
        'l_qpos': l_qpos,
        'r_qpos': r_qpos,
        'l_qvel': l_qvel,
        'r_qvel': r_qvel,
        'arm_qpos': arm_qpos.copy(),
        'arm_qvel': arm_qvel.copy(),
        'ctrl': -0.0105
    })

    # Check for first contact loss
    if first_loss_step < 0 and (not lc or not rc):
        first_loss_step = step
        loss_cause = "left" if not lc else "right"
        print(f"\n*** FIRST CONTACT LOSS DETECTED AT STEP {step} (t={step*0.002:.3f}s) ***")
        print(f"Loss cause: {loss_cause} contact lost")
        print(f"Left contact: {lc}, Right contact: {rc}")
        print(f"Left dist: {ld}, Right dist: {rd}")
        print(f"Left qpos: {l_qpos:.6f}, Right qpos: {r_qpos:.6f}")
        print(f"Left qvel: {l_qvel:.6f}, Right qvel: {r_qvel:.6f}")
        print(f"Bottle pos: {b_pos}")
        print(f"Bottle vel: {b_vel}")
        print(f"Arm qpos: {arm_qpos}")
        print(f"Arm qvel: {arm_qvel}")
        print(f"Finger ctrl: -0.0105")
        break

# Save detailed data
with open(r"C:\work\warehouse_robot_project\ros_ws\mujoco\final_evidence\logs\G14_MILLISECOND_DATA.csv", "w") as f:
    f.write("step,time,lc,rc,ld,rd,bx,by,bz,bvx,bvy,bvz,lqpos,rqpos,lqvel,rqvel,")
    f.write("sp,sl,el,w1,w2,w3,sv,sl,el,wv1,wv2,wv3,ctrl\n")
    for d in data:
        f.write(f"{d['step']},{d['time']:.4f},{d['lc']},{d['rc']},{d['ld']},{d['rd']},")
        f.write(f"{d['b_pos'][0]:.6f},{d['b_pos'][1]:.6f},{d['b_pos'][2]:.6f},")
        f.write(f"{d['b_vel'][0]:.6f},{d['b_vel'][1]:.6f},{d['b_vel'][2]:.6f},")
        f.write(f"{d['l_qpos']:.6f},{d['r_qpos']:.6f},{d['l_qvel']:.6f},{d['r_qvel']:.6f},")
        f.write(f"{d['arm_qpos'][0]:.6f},{d['arm_qpos'][1]:.6f},{d['arm_qpos'][2]:.6f},")
        f.write(f"{d['arm_qpos'][3]:.6f},{d['arm_qpos'][4]:.6f},{d['arm_qpos'][5]:.6f},")
        f.write(f"{d['arm_qvel'][0]:.6f},{d['arm_qvel'][1]:.6f},{d['arm_qvel'][2]:.6f},")
        f.write(f"{d['arm_qvel'][3]:.6f},{d['arm_qvel'][4]:.6f},{d['arm_qvel'][5]:.6f},")
        f.write(f"{d['ctrl']}\n")

print(f"\nData saved to G14_MILLISECOND_DATA.csv")
print(f"Total steps analyzed: {len(data)}")
print(f"First contact loss at step: {first_loss_step}")

# Analysis
print("\n" + "=" * 68)
print("CONTACT LOSS ANALYSIS")
print("=" * 68)

if first_loss_step >= 0:
    loss_data = data[first_loss_step]
    prev_data = data[first_loss_step - 1] if first_loss_step > 0 else data[0]

    print(f"\nLoss detected at step {first_loss_step} (t={first_loss_step*0.002:.3f}s)")
    print(f"\nComparison with previous step:")
    print(f"Left qpos change: {prev_data['l_qpos']:.6f} → {loss_data['l_qpos']:.6f} (Δ={loss_data['l_qpos']-prev_data['l_qpos']:.6f})")
    print(f"Right qpos change: {prev_data['r_qpos']:.6f} → {loss_data['r_qpos']:.6f} (Δ={loss_data['r_qpos']-prev_data['r_qpos']:.6f})")
    print(f"Left qvel: {loss_data['l_qvel']:.6f}")
    print(f"Right qvel: {loss_data['r_qvel']:.6f}")
    print(f"\nFinger control: -0.0105 (closing)")
    print(f"Initial finger positions: 0.07")
    print(f"\nHypothesis: Finger control -0.0105 is too weak to maintain contact")
    print(f"when starting from qpos=0.07 under gravity")
else:
    print("No contact loss detected in first 100 steps (0.2s)")
    print("This is unexpected based on previous diagnostics")

print("\nG14 DIAGNOSTIC COMPLETE")
