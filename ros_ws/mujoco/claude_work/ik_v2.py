exec(open("/ros_ws/mujoco/claude_work/ik_probe.py").read())
home = m.qpos0[qa].copy()
for j in fj: d.qpos[m.jnt_qposadr[j]] = -0.015
def seg_bad(q1, q2, n=40):
    bad = set()
    for s in np.linspace(0, 1, n):
        q = q1 + (q2-q1)*s
        for k,a in enumerate(qa): d.qpos[a] = q[k]
        mujoco.mj_forward(m, d)
        bad.update(collisions())
    return sorted(bad)
qpre = np.array(sol["pregrasp"]); qg = np.array(sol["grasp"])
print("pre→grasp path (fingers open):", seg_bad(qpre, qg))
found = None
for off in [(0.2,0,0.22),(0.25,0,0.3),(0.3,0,0.35),(0.25,-0.15,0.3),(0.35,0.1,0.3)]:
    qab, ep, er = ik(bottle+np.array(off), Rt, qpre)
    b1 = seg_bad(home, qab); b2 = seg_bad(qab, qpre)
    print(off, "pos err %.4f" % ep, "home→above:", b1, "above→pre:", b2)
    if ep < 2e-3 and not b1 and not b2:
        found = qab; break
if found is None:
    print("NO CLEAN PATH"); raise SystemExit
sol["above"] = found.tolist()
json.dump(sol, open("/ros_ws/mujoco/claude_work/ik_solution.json","w"), indent=1)
print("saved above waypoint")
