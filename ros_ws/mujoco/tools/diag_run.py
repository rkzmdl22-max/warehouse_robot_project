import runpy
import numpy as np
import mujoco

_orig = mujoco.mj_step
_s = {"n": 0, "b": None, "L": None, "R": None, "bg": None}
f6 = np.zeros(6)

def _patched(m, d, *a, **k):
    _orig(m, d, *a, **k)
    if _s["b"] is None:
        _s["b"] = next(i for i in range(m.nbody) if abs(m.body_mass[i] - 0.35) < 1e-6)
        _s["L"] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_geom")
        _s["R"] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_geom")
        _s["bg"] = {g for g in range(m.ngeom) if m.geom_bodyid[g] == _s["b"]}
    _s["n"] += 1
    if _s["n"] % 25 == 0 and 22.3 < d.time < 23.1:
        fl = fr = 0.0
        nl = nr = 0
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {c.geom1, c.geom2}
            if pair & _s["bg"]:
                if _s["L"] in pair:
                    mujoco.mj_contactForce(m, d, i, f6); fl += abs(f6[0]); nl += 1
                if _s["R"] in pair:
                    mujoco.mj_contactForce(m, d, i, f6); fr += abs(f6[0]); nr += 1
        b = _s["b"]
        print("C t=%.2f ctrl=%s finger_q=%s" % (d.time, d.ctrl[-2:].round(3), [round(float(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]]), 4) for n in ("left_finger_joint", "right_finger_joint")]))
        up = d.xmat[b].reshape(3, 3)[2, 2]
        print("T t=%.1f bottle=(%.3f,%.3f,%.3f) upright=%.3f L:n=%d F=%.1fN R:n=%d F=%.1fN"
              % (d.time, d.xpos[b][0], d.xpos[b][1], d.xpos[b][2], up, nl, fl, nr, fr))

mujoco.mj_step = _patched
runpy.run_path("ur5e_grasp_test_v2.py", run_name="__main__")
