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

candidates = []

for hip in np.linspace(-0.8, 0.8, 17):
    for knee in np.linspace(-0.8, 0.8, 17):
        for root_z in np.linspace(0.65, 1.0, 8):

            mujoco.mj_resetData(m, d)

            d.qpos[0:3] = [1.2, -3.0, root_z]
            d.qpos[3:7] = [1, 0, 0, 0]

            vals = [0, hip, knee, 0, hip, knee]

            for a, v in zip(adr, vals):
                d.qpos[a] = v

            mujoco.mj_forward(m, d)
            mujoco.mj_step(m, d, nstep=50)

            bz = d.xpos[base][2]
            zl = d.geom_xpos[foot_l][2]
            zr = d.geom_xpos[foot_r][2]

            finite = (
                np.all(np.isfinite(d.qpos))
                and np.all(np.isfinite(d.qvel))
            )

            if (
                finite
                and bz > 0.55
                and 0.015 < zl < 0.08
                and 0.015 < zr < 0.08
            ):
                score = (
                    abs(bz - 0.82)
                    + abs(zl - 0.032)
                    + abs(zr - 0.032)
                    + abs(zl - zr)
                )

                candidates.append(
                    (score, hip, knee, root_z, bz, zl, zr)
                )

print("===== TRON1 DYNAMIC STANDING SEARCH =====")
print("VALID_CANDIDATES =", len(candidates))

if candidates:
    candidates.sort()

    print("TOP 10")

    for candidate in candidates[:10]:
        print(candidate)

else:
    print("NO_VALID_CANDIDATE")
