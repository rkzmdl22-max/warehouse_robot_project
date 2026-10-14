import sys, time, json
import mujoco, numpy as np
HEADLESS = "--headless" in sys.argv
m = mujoco.MjModel.from_xml_path("/ros_ws/mujoco/scenes/claude_handover_scene.xml"); d = mujoco.MjData(m)
B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
Jn = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
sol = json.load(open("/ros_ws/mujoco/claude_work/ik_solution.json"))
Q = {k: np.array(sol[k]) for k in ("above","pregrasp","grasp","lift")}
OPEN, GRIP = -0.015, 0.07
OPENR = -0.06  # 놓기/후퇴 때 손가락을 확실히 벌리는 목표(관절 한계에서 멈춤)
print('actuator ctrlrange(arm):', m.actuator_ctrlrange[:6].round(2).tolist())
PLACE_Z = 0.902          # 병 중심 z (테이블 윗면 0.80 + 반높이 0.10 + 2mm)
tool, lf, rf, bt = B("tool0"), B("left_finger_link"), B("right_finger_link"), B("carry_bottle_3")
bt_tgt = B("bottle_target"); base_b = B("ur5e_base")
for g in range(m.ngeom):
    if m.geom_bodyid[g] in (lf, rf, bt): m.geom_friction[g,0] = 2.0
m.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC; m.opt.impratio = 10
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
arm = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
VADR = [m.jnt_dofadr[Jn(n)] for n in arm]; QADR = [m.jnt_qposadr[Jn(n)] for n in arm]
LIM = [m.jnt_range[Jn(n)] if m.jnt_limited[Jn(n)] else (-6.28, 6.28) for n in arm]
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
def grip_force():
    f = np.zeros(6); out = {lf: 0.0, rf: 0.0}
    for i in range(d.ncon):
        c = d.contact[i]; b1 = m.geom_bodyid[c.geom1]; b2 = m.geom_bodyid[c.geom2]
        if bt in (b1, b2):
            o = b2 if b1 == bt else b1
            if o in out:
                mujoco.mj_contactForce(m, d, i, f); out[o] += abs(f[0])
    return round(out[lf], 2), round(out[rf], 2)
# ---- viewer / pacing ----
vw = None; ST = [0]; T0 = [0.0, 0.0]
RE = max(1, int(1.0/(60*m.opt.timestep)))
def step():
    mujoco.mj_step(m, d)
    if vw is not None:
        ST[0] += 1
        if ST[0] % RE == 0:
            if not vw.is_running(): sys.exit(0)
            vw.sync()
            ahead = (d.time - T0[1]) - (time.time() - T0[0])
            if ahead > 0: time.sleep(ahead)
def wait(T):
    for _ in range(int(T/m.opt.timestep)): step()
mujoco.mj_forward(m, d)
d.ctrl[:6] = [d.qpos[QADR[i]] for i in range(6)]; d.ctrl[6:8] = OPEN
if not HEADLESS:
    import mujoco.viewer
    vw = mujoco.viewer.launch_passive(m, d)
    vw.cam.azimuth = 120; vw.cam.elevation = -25; vw.cam.distance = 2.2; vw.cam.lookat[:] = [0.5, -3.2, 1.0]
    t = time.time()
    print("5초 대기: 지금 녹화 시작 (Win+Alt+R), 마우스로 시점 조정")
    while time.time() - t < 5.0:
        vw.sync(); time.sleep(0.03)
    T0[0] = time.time(); T0[1] = d.time
CMD = d.ctrl.copy(); LASTFF = [np.zeros(6)]; DD = mujoco.MjData(m)
def ff(q):
    DD.qpos[:] = d.qpos; DD.qvel[:] = 0; DD.qacc[:] = 0
    for i, x in enumerate(QADR): DD.qpos[x] = q[i]
    mujoco.mj_inverse(m, DD)
    return DD.qfrc_inverse[VADR] / np.array(KP)
touched = set()
def move(label, q_arm, f, T):
    c0 = CMD.copy(); c1 = c0.copy(); c1[:6] = q_arm; c1[6:8] = f
    FF0 = LASTFF[0].copy(); FF1 = ff(q_arm); LASTFF[0] = FF1
    n = int(T / m.opt.timestep)
    for i in range(n):
        s = (i+1)/n; s = s*s*(3-2*s)
        CMD[:] = c0 + (c1-c0)*s
        d.ctrl[:] = CMD
        d.ctrl[:6] += FF0 + (FF1-FF0)*s
        step()
        if i % 25 == 0: touched.update(ext_collisions())
    err = max(abs(d.qpos[QADR[i]]-q_arm[i]) for i in range(6))
    gp = 0.5*(d.xpos[lf]+d.xpos[rf])
    print("%-9s trk err %.4f rad | mid-finger→bottle %.4f | bottle z %.3f | contacts L/R %s | ext coll %s" %
          (label, err, np.linalg.norm(gp-d.xpos[bt]), d.xpos[bt][2], finger_contacts(), sorted(touched)))
# ---- 지름 검증 ----
gl = next((g for g in range(m.ngeom) if m.geom_bodyid[g]==lf and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX), None)
gr = next((g for g in range(m.ngeom) if m.geom_bodyid[g]==rf and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX), None)
gb = next((g for g in range(m.ngeom) if m.geom_bodyid[g]==bt and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_CYLINDER), None)
def finger_gap():
    if gl is None or gr is None: return float("nan")
    xax = d.xmat[tool].reshape(3,3)[:,0]
    return abs((d.geom_xpos[gr]-d.geom_xpos[gl]) @ xax) - m.geom_size[gl][0] - m.geom_size[gr][0]
# ---- IK (감쇠 최소제곱) ----
def ik(pos, R, q0):
    q = np.array(q0, float)
    for it in range(800):
        DD.qpos[:] = d.qpos
        for i, x in enumerate(QADR): DD.qpos[x] = q[i]
        mujoco.mj_forward(m, DD)
        ep = pos - DD.xpos[tool]
        Re = R @ DD.xmat[tool].reshape(3,3).T
        qu = np.zeros(4); mujoco.mju_mat2Quat(qu, Re.flatten()); er = np.zeros(3); mujoco.mju_quat2Vel(er, qu, 1.0)
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(er) < 1e-3: break
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, DD, jp, jr, tool)
        J = np.vstack([jp[:, VADR], jr[:, VADR]]); e = np.concatenate([ep, er])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4*np.eye(6), e)
        nn = np.linalg.norm(dq)
        if nn > 0.2: dq *= 0.2/nn
        q = np.array([np.clip(q[i]+dq[i], LIM[i][0], LIM[i][1]) for i in range(6)])
    return q, np.linalg.norm(ep), np.linalg.norm(er)
# ================= 실행 =================
z0 = d.xpos[bt].copy()
move("above", Q["above"], OPEN, 3.0)
move("pregrasp", Q["pregrasp"], OPEN, 2.0)
move("approach", Q["grasp"], OPEN, 2.0)
open_gap = finger_gap()
move("close", Q["grasp"], GRIP, 1.0)
wait(0.5)
rel0 = d.xmat[tool].reshape(3,3).T @ (d.xpos[bt]-d.xpos[tool])
dia_true = 2*m.geom_size[gb][0] if gb is not None else float("nan")
cl_gap = finger_gap()
print("bottle in tool frame:", np.round(rel0,3))
print("지름 검증: 벌림(열림) 간격 %.4f m | 닫힌 손가락 안쪽 간격 %.4f m | 병 지름(모델) %.4f m | 차이 %.1f mm" %
      (open_gap, cl_gap, dia_true, abs(cl_gap-dia_true)*1000))
print("close 후 접촉 법선력(N) 좌/우:", grip_force())
move("lift", Q["lift"], GRIP, 2.0)
wait(1.5)
rel1 = d.xmat[tool].reshape(3,3).T @ (d.xpos[bt]-d.xpos[tool])
print("lift 후 접촉 법선력(N) 좌/우:", grip_force())
print("lift hold 1.5s | bottle z %.3f | slip %.4f m | contacts %s | lifted %.3f m" %
      (d.xpos[bt][2], np.linalg.norm(rel1-rel0), finger_contacts(), d.xpos[bt][2]-z0[2]))
# ---- 놓기: 후보 탐색 ----
Rt = d.xmat[tool].reshape(3,3).copy(); q_now = np.array([d.qpos[x] for x in QADR]); base_xy = d.xpos[base_b][:2]
def plan(px, py):
    a = np.array([px, py]) - base_xy; a0 = Rt[:, 2][:2]
    th = np.arctan2(a[1], a[0]) - np.arctan2(a0[1], a0[0]); th = (th + np.pi) % (2*np.pi) - np.pi
    c, s = np.cos(th), np.sin(th); Rg = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ Rt
    tp = lambda z: np.array([px, py, z]) - Rg @ rel0
    P = {}; seed = q_now; worst = 0
    for name, pos in (("hi", tp(1.20)), ("above", tp(PLACE_Z+0.06)), ("place", tp(PLACE_Z+0.010)), ("back", tp(PLACE_Z+0.010) - 0.12*Rg[:, 2])):
        q, ep, er = ik(pos, Rg, seed)
        qu = q + 2*np.pi*np.round((seed - q)/(2*np.pi))
        if all(LIM[i][0] <= qu[i] <= LIM[i][1] for i in range(6)): q = qu
        worst = max(worst, ep/0.003, er/0.02)
        P[name] = q; seed = q
    jump = max(np.max(np.abs(P[n]-q_now)) for n in P)
    return P, worst, jump
cands = [(x,y) for x in (0.85,0.82,0.88,0.90) for y in (-3.10,-3.07,-3.04,-3.13)]
chosen = None
for (px, py) in cands:
    P, worst, jump = plan(px, py)
    print("place 후보 (%.2f,%.2f): IK 오차비율 %.2f | 현재자세 대비 최대 관절변화 %.2f rad" % (px, py, worst, jump))
    if worst <= 2.0 and jump <= 1.7 and (chosen is None or jump < chosen[3]): chosen = (px, py, P, jump)
if chosen is None:
    print("놓기 IK 후보 전부 실패 → 이 출력을 그대로 알려주세요"); 
    if vw: wait(3.0)
    sys.exit(1)
px, py, P, _w = chosen
print("선택된 놓기 위치: (%.2f, %.2f), 병 중심 z %.3f" % (px, py, PLACE_Z))
move("transit", P["hi"], GRIP, 3.0)
move("above_pl", P["above"], GRIP, 1.5)
move("lower", P["place"], GRIP, 1.5)
move("release", P["place"], OPENR, 1.0)
for _k in range(6):
    wait(0.2); _z = d.xmat[bt].reshape(3,3)[:, 2]
    print("   release+%.1fs: bottle z %.4f | tilt %.1f deg | contacts %s | force %s" % (0.2*(_k+1), d.xpos[bt][2], np.degrees(np.arccos(np.clip(_z[2], -1, 1))), finger_contacts(), grip_force()))
print("release 후 손가락 안쪽 간격 %.4f m (병 지름 0.070)" % finger_gap())
print("release 후 손가락-병 접촉:", finger_contacts())
move("ret_back", P["back"], OPENR, 2.0)
wait(2.0)
# ---- 결과 판정 (측정값 기준) ----
bz = d.xmat[bt].reshape(3,3)[:, 2]; tilt = np.degrees(np.arccos(np.clip(bz[2], -1, 1)))
bj = m.body_jntadr[bt]; da = m.jnt_dofadr[bj]; spd = np.linalg.norm(d.qvel[da:da+3])
dist_t = np.linalg.norm(d.xpos[bt][:2]-d.xpos[bt_tgt][:2]) if bt_tgt >= 0 else float("nan")
print("최종 병 위치", np.round(d.xpos[bt],3), "| 목표 z %.3f | 기울기 %.1f deg | 속도 %.4f m/s | bottle_target과 거리 %.3f m" % (PLACE_Z, tilt, spd, dist_t))
print("외부 충돌 기록:", sorted(touched))
ok = abs(d.xpos[bt][2]-PLACE_Z) < 0.006 and tilt < 5 and spd < 0.01
print("놓기 안정 판정(측정 기준):", "PASS" if ok else "FAIL")
if vw: wait(3.0)
