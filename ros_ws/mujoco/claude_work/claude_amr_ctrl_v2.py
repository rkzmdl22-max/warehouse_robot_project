import mujoco, numpy as np
V = [0.5, 0.6, 0.7]; SG = [-1, -1, 1]; L = 9.0; S = {}
def amr_update(m, d):
    if not S:
        S["a"] = [[m.actuator("act_amr%d_%s" % (i, c)).id for c in "xy"] for i in (1, 2, 3)]
        S["b"] = [m.body("claude_amr%d" % i).id for i in (1, 2, 3)]
        S["off"] = [0.0] * 3; S["hx"] = [None] * 3; S["dir"] = [1] * 3; S["n"] = [0] * 3
    for k in range(3):
        p = d.xpos[S["b"][k]]; s = (V[k] * max(0.0, d.time - 1.5 * k)) % (2 * L)
        if k == 0: s = min(V[k] * d.time, 1.6)
        u = s if s < L else 2 * L - s; dr = 1 if s < L else -1
        if dr != S["dir"][k]: S["dir"][k] = dr; S["hx"][k] = None; S["off"][k] = 0.0; S["n"][k] += 1; print("AMR%d 구간 %d 완료: t=%.1fs" % (k + 1, S["n"][k], d.time))
        if S["hx"][k] is None:
            dist = mujoco.mj_ray(m, d, p + np.array([0, 0, 0.05]), np.array([float(dr), 0, 0]), None, 1, S["b"][k], np.zeros(1, dtype=np.int32))
            if 0 < dist < 1.0: S["hx"][k] = p[0] + dr * dist; S["off"][k] = SG[k] * 0.6; print("AMR%d 장애물 감지 x=%.2f 우회" % (k + 1, S["hx"][k]))
        elif (p[0] - S["hx"][k]) * dr > 0.8: S["off"][k] = 0.0
        d.ctrl[S["a"][k][0]] = u
        d.ctrl[S["a"][k][1]] = S["off"][k]
