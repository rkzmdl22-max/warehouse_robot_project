import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path(r"mujoco\scenes\warehouse.xml")
d = mujoco.MjData(m)

names = [
    "abad_L_Joint",
    "hip_L_Joint",
    "knee_L_Joint",
    "abad_R_Joint",
    "hip_R_Joint",
    "knee_R_Joint",
]

adr = [m.joint(n).qposadr for n in names]

foot_l = m.geom("foot_L_collision").id
foot_r = m.geom("foot_R_collision").id
base = m.body("base_Link").id

mujoco.mj_resetData(m, d)

d.qpos[0:3] = [1.2, -3.0, 0.85]
d.qpos[3:7] = [1, 0, 0, 0]

vals = [0.0, -0.2, 0.0, 0.0, -0.2, 0.0]

for a, v in zip(adr, vals):
    d.qpos[a] = v

mujoco.mj_forward(m, d)

print("===== TRON1 CANDIDATE VERIFY =====")
print("INITIAL base_z =", d.xpos[base][2])
print("INITIAL foot_L_z =", d.geom_xpos[foot_l][2])
print("INITIAL foot_R_z =", d.geom_xpos[foot_r][2])
print("INITIAL ncon =", d.ncon)

mujoco.mj_step(m, d, nstep=1000)

print("===== AFTER 1000 STEP =====")
print("base_pos =", d.xpos[base].tolist())
print("foot_L_pos =", d.geom_xpos[foot_l].tolist())
print("foot_R_pos =", d.geom_xpos[foot_r].tolist())
print("ncon =", d.ncon)
print("qpos_finite =", np.all(np.isfinite(d.qpos)))
print("qvel_finite =", np.all(np.isfinite(d.qvel)))