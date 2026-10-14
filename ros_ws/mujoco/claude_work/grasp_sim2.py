import mujoco, numpy as np, json
m = mujoco.MjModel.from_xml_path("/ros_ws/mujoco/scenes/claude_handover_scene.xml"); d = mujoco.MjData(m)
B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
Jn = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
sol = json.load(open("/ros_ws/mujoco/claude_work/ik_solution.json"))
Q = {k: np.array(sol[k]) for k in ("above","pregrasp","grasp","lift")}
OPEN, GRIP = -0.015, 0.03
tool, lf, rf, bt = B("tool0"), B("left_finger_link"), B("right_finger_link"), B("carry_bottle_3")
base_link = B("ur5e_base_link")
def under(b):
    while b > 0:
        if b == base_link: return True
        b = m.body_parentid[b]
    return False
moving = {b for b in range(m.nbody) if under(b)}
for b in moving: m.body_gravcomp[b] = 1.0
ignore = {B("ur5e_base"), B("ur5e_worktable")}
KP = [1000,1000,600,200,120,80]; KV = [100,100,60,20,12,8]
for i in range(6):
    m.actuator_gainprm[i,0] = KP[i]; m.actuator_biasprm[i,1] = -KP[i]; m.actuator_biasprm[i,2] = -KV[i]
def ext_collisions():
    out = set()
    for i in range(d.ncon):
        b1 = m.geom_bodyid[d.contact[i].geom1]; b2 = m.geom_bodyid[d.contact[i].geom2]
        if (b1 in moving) != (b2 in moving):
            o = b2 if b1 in moving else b1
            if o not in ignore and o != bt:
                out.add(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, o))
    return out
def finger_contacts():
    L = R = False
    for i in range(d.ncon):
        bs = {m.geom_bodyid[d.contact[i].geom1], m.geom_bodyid[d.contact[i].geom2]}
        if bt in bs and lf in bs: L = True
        if bt in bs and rf in bs: R = True
    return L, R
arm = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
mujoco.mj_forward(m, d)
d.ctrl[:6] = [d.qpos[m.jnt_qposadr[Jn(n)]] for n in arm]
d.ctrl[6:8] = OPEN
touched = set()
def move(label, q_arm, f, T):
    c0 = d.ctrl.copy(); c1 = c0.copy(); c1[:6] = q_arm; c1[6:8] = f
    n = int(T / m.opt.timestep)
    for i in range(n):
        s = (i+1)/n; s = s*s*(3-2*s)
        d.ctrl[:] = c0 + (c1-c0)*s
        mujoco.mj_step(m, d)
        if i % 25 == 0: touched.update(ext_collisions())
    err = max(abs(d.qpos[m.jnt_qposadr[Jn(n)]]-q_arm[i]) for i,n in enumerate(arm))
    print("   joint tracking err max %.4f rad" % err)
    gp = 0.5*(d.xpos[lf]+d.xpos[rf])
    print("%-9s mid-finger→bottle %.4f | bottle z %.3f | finger contacts L/R %s | ext collisions so far %s" %
          (label, np.linalg.norm(gp-d.xpos[bt]), d.xpos[bt][2], finger_contacts(), sorted(touched)))
z0 = d.xpos[bt].copy()
move("above", Q["above"], OPEN, 3.0)
move("pregrasp", Q["pregrasp"], OPEN, 2.0)
move("approach", Q["grasp"], OPEN, 2.0)
move("close", Q["grasp"], GRIP, 1.0)
for _ in range(int(0.5/m.opt.timestep)): mujoco.mj_step(m, d)
rel0 = d.xmat[tool].reshape(3,3).T @ (d.xpos[bt]-d.xpos[tool])
print("grip settled | finger q:", np.round([d.qpos[m.jnt_qposadr[Jn(n)]] for n in ("left_finger_joint","right_finger_joint")],4))
move("lift", Q["lift"], GRIP, 2.0)
for t in range(6):
    for _ in range(int(0.5/m.opt.timestep)): mujoco.mj_step(m, d)
    rel = d.xmat[tool].reshape(3,3).T @ (d.xpos[bt]-d.xpos[tool])
    print("hold %.1fs | bottle z %.3f | slip vs gripper %.4f m | contacts L/R %s" % ((t+1)*0.5, d.xpos[bt][2], np.linalg.norm(rel-rel0), finger_contacts()))
print("lifted by %.3f m (bottle z %.3f → %.3f)" % (d.xpos[bt][2]-z0[2], z0[2], d.xpos[bt][2]))
