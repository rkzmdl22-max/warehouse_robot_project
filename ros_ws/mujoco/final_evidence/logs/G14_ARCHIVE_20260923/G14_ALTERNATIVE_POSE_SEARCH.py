"""
G14 Alternative Pose Search - Try different arm poses around Q0
Goal: Find a pose that achieves bilateral contact with better contact strength
"""
import mujoco
import numpy as np
import sys

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"

print("=" * 68)
print("G14 ALTERNATIVE POSE SEARCH")
print("=" * 68)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

gravity_orig = m.opt.gravity.copy()
print(f"Original gravity: {gravity_orig}")

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
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

def contact(data):
    lc=rc=False
    left_dist=right_dist=None
    for ci in range(data.ncon):
        c=data.contact[ci]; a,b=int(c.geom1),int(c.geom2)
        if {a,b}=={bgeom,lgeom}:
            lc=True
            left_dist = c.dist
        if {a,b}=={bgeom,rgeom}:
            rc=True
            right_dist = c.dist
    return lc, rc, left_dist, right_dist

def set_ctrl(model, data, q_arm, q_f):
    for jn,qv in zip(arm_joints,q_arm):
        ai=aid(act_map[jn])
        if ai>=0: data.ctrl[ai]=qv
    data.ctrl[ai_lf]=q_f; data.ctrl[ai_rf]=q_f

def fix_tron1(data):
    data.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
    data.qvel[root_dof:root_dof+6] = 0.0

print("\nSearching poses around Q0 with better bilateral contact strength...")
print("Target: Both contacts deeper than -0.005 m (5 mm)")

# Search around Q0 with small variations
variations = {
    'shoulder_pan_joint': [-0.02, -0.01, 0.0, 0.01, 0.02],
    'shoulder_lift_joint': [-0.02, -0.01, 0.0, 0.01, 0.02],
    'elbow_joint': [-0.02, -0.01, 0.0, 0.01, 0.02],
    'wrist_1_joint': [-0.02, -0.01, 0.0, 0.01, 0.02],
    'wrist_2_joint': [-0.02, -0.01, 0.0, 0.01, 0.02],
    'wrist_3_joint': [-0.02, -0.01, 0.0, 0.01, 0.02],
}

best_candidates = []

# Try variations
for i, (joint, vars) in enumerate(variations.items()):
    joint_idx = arm_joints.index(joint)
    
    for var in vars:
        q_test = Q0.copy()
        q_test[joint_idx] += var
        
        # Test this pose
        m2 = mujoco.MjModel.from_xml_path(XML)
        d2 = mujoco.MjData(m2)
        m2.opt.gravity[:] = 0.0
        
        d2.qpos[root_qp:root_qp+7] = HANDOVER_QPOS7
        d2.qvel[:] = 0.0
        for jn,qv in zip(arm_joints,q_test):
            d2.qpos[qadr(jn)] = qv
        d2.qpos[lq]=0.07; d2.qpos[rq]=0.07
        set_ctrl(m2, d2, q_test, 0.0)
        mujoco.mj_forward(m2, d2)
        
        # Grasp
        for _ in range(600):
            fix_tron1(d2)
            set_ctrl(m2, d2, q_test, 0.0)
            mujoco.mj_step(m2, d2)
        
        lc, rc, ld, rd = contact(d2)
        
        if lc and rc:
            # Check contact strength
            contact_strength = abs(ld) + abs(rd)
            print(f"Pose {joint}+{var:.3f}: lc={lc}, rc={rc}, ld={ld:.6f}, rd={rd:.6f}, strength={contact_strength:.6f}")
            
            if contact_strength > 0.010:  # Both contacts deeper than 5mm combined
                best_candidates.append((q_test.copy(), ld, rd, contact_strength))
                
                # Test G14 hold with gravity
                m2.opt.gravity[:] = gravity_orig
                mujoco.mj_forward(m2, d2)
                
                contact_loss_step = -1
                for step in range(2500):
                    fix_tron1(d2)
                    set_ctrl(m2, d2, q_test, -0.015)
                    mujoco.mj_step(m2, d2)
                    
                    lc, rc, ld, rd = contact(d2)
                    if not (lc and rc):
                        contact_loss_step = step
                        break
                
                if contact_loss_step < 0:
                    print(f"  *** G14 PASS with {joint}+{var:.3f}! Maintained contact for 5 seconds ***")
                    best_candidates.append((q_test.copy(), ld, rd, contact_strength, "G14_PASS"))
                    break
                else:
                    print(f"  G14 FAIL: Contact lost at step {contact_loss_step} (t={contact_loss_step*0.002:.3f}s)")

print(f"\nTotal candidates found: {len(best_candidates)}")

if best_candidates:
    print("\nBest candidates:")
    for i, (q, ld, rd, strength, *extra) in enumerate(best_candidates):
        if extra and extra[0] == "G14_PASS":
            print(f"  Candidate {i}: STRENGTH={strength:.6f}, G14 PASS")
        else:
            print(f"  Candidate {i}: STRENGTH={strength:.6f}")
else:
    print("\nNo better poses found in local search")

print("\nG14 ALTERNATIVE POSE SEARCH COMPLETE")
