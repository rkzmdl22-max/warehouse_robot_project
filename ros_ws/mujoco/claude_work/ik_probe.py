import mujoco, numpy as np, json
SCENE = "/ros_ws/mujoco/scenes/claude_handover_scene.xml"
m = mujoco.MjModel.from_xml_path(SCENE); d = mujoco.MjData(m)
B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
Jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
names = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
jids = [Jid(n) for n in names]
qa = [m.jnt_qposadr[j] for j in jids]; va = [m.jnt_dofadr[j] for j in jids]
lo = m.jnt_range[jids,0]; hi = m.jnt_range[jids,1]
fj = [Jid("left_finger_joint"), Jid("right_finger_joint")]
tool, lf, rf = B("tool0"), B("left_finger_link"), B("right_finger_link")
mujoco.mj_forward(m, d)
bottle = d.xpos[B("carry_bottle_3")].copy()
print("bottle center:", np.round(bottle,3))

# --- finger diagnostics (kinematic) ---
print("== finger geoms ==")
for g in range(m.ngeom):
    if m.geom_bodyid[g] in (lf, rf):
        print(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g), "type", m.geom_type[g], "size", np.round(m.geom_size[g],3),
              "pos", np.round(m.geom_pos[g],3), "contype", m.geom_contype[g], "fric", m.geom_friction[g])
for q in (-0.015, 0.0, 0.035, 0.07):
    for j in fj: d.qpos[m.jnt_qposadr[j]] = q
    mujoco.mj_forward(m, d)
    Rt = d.xmat[tool].reshape(3,3)
    print("finger q=%.3f  gap(finger origins)=%.4f  left/right in tool x: %.3f %.3f" %
          (q, np.linalg.norm(d.xpos[lf]-d.xpos[rf]),
           Rt[:,0] @ (d.xpos[lf]-d.xpos[tool]), Rt[:,0] @ (d.xpos[rf]-d.xpos[tool])))
for j in fj: d.qpos[m.jnt_qposadr[j]] = 0.0

# --- arm-moving body set ---
base_link = B("ur5e_base_link")
def under(b):
    while b > 0:
        if b == base_link: return True
        b = m.body_parentid[b]
    return False
moving = {b for b in range(m.nbody) if under(b)}
ignore = {B("ur5e_base"), B("ur5e_worktable")}

def collisions():
    out = set()
    for i in range(d.ncon):
        b1 = m.geom_bodyid[d.contact[i].geom1]; b2 = m.geom_bodyid[d.contact[i].geom2]
        if (b1 in moving) != (b2 in moving):
            o = b2 if b1 in moving else b1
            if o not in ignore:
                out.add((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b1), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b2)))
    return sorted(out)

def grasp_pt():
    return 0.5*(d.xpos[lf]+d.xpos[rf])

def ik(p_t, R_t, q0, iters=400):
    q = q0.copy()
    for _ in range(iters):
        for k,a in enumerate(qa): d.qpos[a] = q[k]
        mujoco.mj_forward(m, d)
        p = grasp_pt(); R = d.xmat[tool].reshape(3,3)
        ep = p_t - p
        er = 0.5*sum(np.cross(R[:,i], R_t[:,i]) for i in range(3))
        if np.linalg.norm(ep) < 5e-4 and np.linalg.norm(er) < 5e-3: break
        jp = np.zeros((3,m.nv)); jr = np.zeros((3,m.nv))
        mujoco.mj_jac(m, d, jp, jr, p, tool)
        J = np.vstack([jp[:,va], jr[:,va]]); e = np.concatenate([ep, er])
        dq = J.T @ np.linalg.solve(J@J.T + 1e-3*np.eye(6), e)
        q = np.clip(q + np.clip(dq, -0.2, 0.2), lo, hi)
    for k,a in enumerate(qa): d.qpos[a] = q[k]
    mujoco.mj_forward(m, d)
    return q, np.linalg.norm(p_t-grasp_pt()), np.linalg.norm(0.5*sum(np.cross(d.xmat[tool].reshape(3,3)[:,i], R_t[:,i]) for i in range(3)))

Rs = {"x=+y": np.array([[0,1,0],[0,0,-1],[-1,0,0]]).T.T,
      "x=-y": np.array([[0,-1,0],[0,0,1],[-1,0,0]]).T.T}
# columns are tool x,y,z axes in world
Rs = {"x=+y": np.column_stack([(0,1,0),(0,0,-1),(-1,0,0)]),
      "x=-y": np.column_stack([(0,-1,0),(0,0,1),(-1,0,0)])}
ready = np.array([0,-1.57,1.57,-1.57,-1.57,0])
rng = np.random.default_rng(0)
best = None
for name, Rt in Rs.items():
    for _ in range(120):
        q0 = np.clip(ready + rng.uniform(-1.5,1.5,6), lo, hi)
        q, ep, er = ik(bottle, Rt, q0)
        if ep < 2e-3 and er < 2e-2:
            col = collisions()
            score = np.abs(q-ready).sum() + 5*len(col)
            if best is None or score < best[0]:
                best = (score, name, q.copy(), ep, er, col)
if best is None:
    print("NO IK SOLUTION"); raise SystemExit
_, name, qg, ep, er, col = best
Rt = Rs[name]
print("orientation:", name, "| grasp q:", np.round(qg,3), "| pos err %.4f rot err %.4f" % (ep, er))
print("collisions at grasp (finger open not set):", col)
sol = {"orientation": name, "grasp": qg.tolist()}
for label, off in (("pregrasp", np.array([0.12,0,0])), ("lift", np.array([0,0,0.12]))):
    q, ep, er = ik(bottle+off, Rt, qg)
    print(label, np.round(q,3), "pos err %.4f rot err %.4f" % (ep, er), "collisions:", collisions())
    sol[label] = q.tolist()
json.dump(sol, open("/ros_ws/mujoco/claude_work/ik_solution.json","w"), indent=1)
print("saved ik_solution.json")
