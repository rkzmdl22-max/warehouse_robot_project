"""Finger close-direction diagnostic. XML MODIFIED = NO."""
import numpy as np
import mujoco

XML = r"C:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)

def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def jid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def qadr(n): return m.jnt_qposadr[jid(n)]

Q0 = np.array([-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0])
arm_joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
              "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
HANDOVER = np.array([0.0,-3.0,0.82, 1.0,0.0,0.0,0.0])
tron1 = bid("base_Link")
root_qp = m.jnt_qposadr[m.body_jntadr[tron1]]
lq = qadr("left_finger_joint"); rq = qadr("right_finger_joint")
bgeom = gid("carry_bottle_collision")
lgeom = gid("left_finger_geom"); rgeom = gid("right_finger_geom")

d.qpos[root_qp:root_qp+7] = HANDOVER
d.qvel[:] = 0
for jn,qv in zip(arm_joints, Q0):
    d.qpos[qadr(jn)] = qv

print("q  | left_geom_x y z | right_geom | bottle | gap | lc rc distL distR")
for qf in [-0.015, -0.0105, 0.0, 0.02, 0.035, 0.05, 0.07]:
    d.qpos[lq] = qf
    d.qpos[rq] = qf
    mujoco.mj_forward(m, d)
    lp = d.geom_xpos[lgeom]; rp = d.geom_xpos[rgeom]; bp = d.geom_xpos[bgeom]
    gap = float(np.linalg.norm(rp-lp))
    lc=rc=False; dl=dr=None
    for ci in range(d.ncon):
        c=d.contact[ci]; pair={int(c.geom1),int(c.geom2)}
        if pair=={bgeom,lgeom}:
            lc=True; dl=c.dist
        if pair=={bgeom,rgeom}:
            rc=True; dr=c.dist
    print("qf=%+.4f  L=%s  R=%s  B=%s  gap=%.4f  lc=%s rc=%s  dL=%s dR=%s" % (
        qf, np.round(lp,4), np.round(rp,4), np.round(bp,4), gap, lc, rc, dl, dr))
