import os, sys, time, json, zlib, struct
from claude_amr_ctrl_v2 import amr_update
from claude_vis2 import ann, pump, ann_side, cam_info
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco, numpy as np
HEADLESS = "--headless" in sys.argv; USE_SAVED = "--use-saved" in sys.argv
WD = "/ros_ws/mujoco/claude_work/"
m = mujoco.MjModel.from_xml_path("/ros_ws/mujoco/scenes/claude_amr_scene_v3.xml"); d = mujoco.MjData(m); DD = mujoco.MjData(m)
B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
Jn = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
sol = json.load(open(WD + "ik_solution.json"))
Q = {k: np.array(sol[k]) for k in ("above", "pregrasp", "grasp", "lift")}
OPEN, GRIP, OPENR = -0.015, 0.07, -0.06
PLACE_Z = 0.902
NOM = np.array([0.15, -3.0, 1.037])   # IK 해를 구할 때 쓴 공칭 병 중심(설계 상수)
HAND = np.array([0.06, -3.0, 1.0])    # 스테이션 카메라 고정 조준점(핸드오버 구역 중심)
tool, lf, rf, bt = B("tool0"), B("left_finger_link"), B("right_finger_link"), B("carry_bottle_3")
base_b = B("ur5e_base")
for g in range(m.ngeom):
    if m.geom_bodyid[g] in (lf, rf, bt): m.geom_friction[g, 0] = 2.0
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
KP = [1000, 1000, 600, 200, 120, 80]; KV = [100, 100, 60, 20, 12, 8]
for i in range(6):
    m.actuator_gainprm[i, 0] = KP[i]; m.actuator_biasprm[i, 1] = -KP[i]; m.actuator_biasprm[i, 2] = -KV[i]
m.vis.map.znear = 0.0003; m.vis.map.zfar = 3.0
arm = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
VADR = [m.jnt_dofadr[Jn(n)] for n in arm]; QADR = [m.jnt_qposadr[Jn(n)] for n in arm]
LIM = [m.jnt_range[Jn(n)] if m.jnt_limited[Jn(n)] else (-6.28, 6.28) for n in arm]
H, W = 480, 640
sys.path.insert(0, WD + "v1014")
from claude_hri import make_hri
HRI = None if os.environ.get("HRI_OFF") else make_hri(m, d, VADR)
R = None if USE_SAVED else mujoco.Renderer(m, H, W)
if USE_SAVED:  # 영상용: 렌더링만 변경(충돌/물리는 그대로)
    _hide = ("bottle_target", "bottle_place_table")
    _hide_geom = ("station_table_top",)
    _color = {"station_table_base": (0.22, 0.24, 0.28, 1),
              "ur5e_worktable_top": (0.35, 0.37, 0.40, 1), "ur5e_worktable_base": (0.25, 0.27, 0.30, 1)}
    for g in range(m.ngeom):
        bn, gn = m.body(m.geom_bodyid[g]).name, (m.geom(g).name or "")
        if bn in _hide or gn in _hide_geom:
            m.geom_matid[g] = -1; m.geom_rgba[g] = (0, 0, 0, 0)
        elif gn in _color:
            m.geom_matid[g] = -1; m.geom_rgba[g] = _color[gn]
def ext_collisions():
    out = set()
    for i in range(d.ncon):
        b1 = m.geom_bodyid[d.contact[i].geom1]; b2 = m.geom_bodyid[d.contact[i].geom2]
        if (b1 in moving) != (b2 in moving):
            o = b2 if b1 in moving else b1
            if o not in ignore and o != bt: out.add(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, o))
    return out
def finger_contacts():
    L = Rr = False
    for i in range(d.ncon):
        bs = {m.geom_bodyid[d.contact[i].geom1], m.geom_bodyid[d.contact[i].geom2]}
        if bt in bs and lf in bs: L = True
        if bt in bs and rf in bs: Rr = True
    return L, Rr
def grip_force():
    f = np.zeros(6); out = {lf: 0.0, rf: 0.0}
    for i in range(d.ncon):
        c = d.contact[i]; b1 = m.geom_bodyid[c.geom1]; b2 = m.geom_bodyid[c.geom2]
        if bt in (b1, b2):
            o = b2 if b1 == bt else b1
            if o in out: mujoco.mj_contactForce(m, d, i, f); out[o] += abs(f[0])
    return round(float(out[lf]), 2), round(float(out[rf]), 2)
vw = None; ST = [0]; T0 = [0.0, 0.0]
RE = max(1, int(1.0 / (60 * m.opt.timestep)))
ESTOP = [False]; ESTOP_REQ = [False]
ESTOP_AT = float(__import__("os").environ.get("ESTOP_AT", "0") or 0)
G_BTN = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "claude_estop_btn")
def estop_halt():
    ESTOP[0] = True
    m.geom_rgba[G_BTN] = (1.0, 0.1, 0.1, 1); m.geom_pos[G_BTN, 2] -= 0.015
    jx = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "tron1_slide_x")]
    print("!! E-STOP 작동: t=%.2fs | 베이스 x %.3f | 병 위치 %s | 이후 모든 구동 정지(동결)" % (d.time, d.qpos[jx], d.xpos[bt].round(3)))
    mujoco.mj_forward(m, d)
    if vw is not None:
        while vw.is_running(): vw.sync(); time.sleep(0.03)
    sys.exit(0)
def step():
    if ESTOP_REQ[0] or (ESTOP_AT > 0 and d.time >= ESTOP_AT): estop_halt()
    amr_update(m, d); pump()
    if HRI: HRI.pre()
    _q0, _v0 = d.qpos.copy(), d.qvel.copy()
    mujoco.mj_step(m, d)
    if HRI:
        HRI.post(_q0, _v0)
        if HRI.estop_pressed: ESTOP_REQ[0] = True
    if vw is not None:
        ST[0] += 1
        if ST[0] % RE == 0:
            if not vw.is_running(): sys.exit(0)
            vw.sync()
            ahead = (d.time - T0[1]) - (time.time() - T0[0])
            if ahead > 0: time.sleep(ahead)
def wait(T):
    for _ in range(int(T / m.opt.timestep)): step()
def ff(q):
    DD.qpos[:] = d.qpos; DD.qvel[:] = 0; DD.qacc[:] = 0
    for i, x in enumerate(QADR): DD.qpos[x] = q[i]
    mujoco.mj_inverse(m, DD)
    return DD.qfrc_inverse[VADR] / np.array(KP)
touched = set(); LASTFF = [np.zeros(6)]
def move(label, q_arm, f, T):
    c0 = CMD.copy(); c1 = c0.copy(); c1[:6] = q_arm; c1[6:8] = f
    FF0 = LASTFF[0].copy(); FF1 = ff(q_arm); LASTFF[0] = FF1
    n = int(T / m.opt.timestep); p = 0.0; i = 0
    while p < n:
        s = min(1.0, (p + 1) / n); s = s * s * (3 - 2 * s)
        CMD[:] = c0 + (c1 - c0) * s
        d.ctrl[:] = CMD; d.ctrl[:6] += FF0 + (FF1 - FF0) * s
        step()
        p += HRI.scale if HRI else 1.0; i += 1
        if i % 25 == 0: touched.update(ext_collisions())
    err = max(abs(d.qpos[QADR[i]] - q_arm[i]) for i in range(6))
    gp = 0.5 * (d.xpos[lf] + d.xpos[rf])
    print("%-9s trk err %.4f rad | mid-finger→bottle %.4f | bottle z %.3f | contacts L/R %s | ext coll %s" %
          (label, err, np.linalg.norm(gp - d.xpos[bt]), d.xpos[bt][2], finger_contacts(), sorted(touched)) + (" | HRI " + HRI.tag() if HRI else ""))
gl = next((g for g in range(m.ngeom) if m.geom_bodyid[g] == lf and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX), None)
gr = next((g for g in range(m.ngeom) if m.geom_bodyid[g] == rf and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX), None)
gb = next((g for g in range(m.ngeom) if m.geom_bodyid[g] == bt and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER), None)
def finger_gap():
    xax = d.xmat[tool].reshape(3, 3)[:, 0]
    return abs((d.geom_xpos[gr] - d.geom_xpos[gl]) @ xax) - m.geom_size[gl][0] - m.geom_size[gr][0]
def ik(pos, Rg, q0):
    q = np.array(q0, float)
    for it in range(800):
        DD.qpos[:] = d.qpos
        for i, x in enumerate(QADR): DD.qpos[x] = q[i]
        mujoco.mj_forward(m, DD)
        ep = pos - DD.xpos[tool]
        Re = Rg @ DD.xmat[tool].reshape(3, 3).T
        qu = np.zeros(4); mujoco.mju_mat2Quat(qu, Re.flatten()); er = np.zeros(3); mujoco.mju_quat2Vel(er, qu, 1.0)
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(er) < 1e-3: break
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, DD, jp, jr, tool)
        J = np.vstack([jp[:, VADR], jr[:, VADR]]); e = np.concatenate([ep, er])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        nn = np.linalg.norm(dq)
        if nn > 0.2: dq *= 0.2 / nn
        q = np.array([np.clip(q[i] + dq[i], LIM[i][0], LIM[i][1]) for i in range(6)])
    return q, np.linalg.norm(ep), np.linalg.norm(er)
def fk(q):
    DD.qpos[:] = d.qpos; DD.qvel[:] = 0; DD.qacc[:] = 0
    for i, x in enumerate(QADR): DD.qpos[x] = q[i]
    mujoco.mj_forward(m, DD)
    return DD.xpos[tool].copy(), DD.xmat[tool].reshape(3, 3).copy()
FKS = {k: fk(Q[k]) for k in Q}
OFF = {k: FKS[k][0] - NOM for k in Q}; RT = {k: FKS[k][1] for k in Q}
def waypoints(c, keys):
    out = {}
    for k in keys:
        q, ep, er = ik(c + OFF[k], RT[k], Q[k]); out[k] = q
        print("   IK %-8s 목표 위치오차 %.2f mm | 자세오차 %.4f | 저장해 대비 최대 관절차 %.3f rad" % (k, ep * 1000, er, np.max(np.abs(q - Q[k]))))
    return out
# ---- 카메라 ----
def cam_id(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, n)
def mount_look_at(cam, target):
    cid = cam_id(cam); mujoco.mj_forward(m, d)
    f = target - d.cam_xpos[cid]; f = f / np.linalg.norm(f); z = -f
    x = np.cross([0, 0, 1.0], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    Rl = d.xmat[m.cam_bodyid[cid]].reshape(3, 3).T @ np.stack([x, y, z], 1)
    q = np.zeros(4); mujoco.mju_mat2Quat(q, Rl.flatten()); m.cam_quat[cid] = q; mujoco.mj_forward(m, d)
def mount_along_tool(cam):
    cid = cam_id(cam); mujoco.mj_forward(m, d)
    Rw = d.xmat[tool].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
    Rl = d.xmat[m.cam_bodyid[cid]].reshape(3, 3).T @ Rw
    q = np.zeros(4); mujoco.mju_mat2Quat(q, Rl.flatten()); m.cam_quat[cid] = q; mujoco.mj_forward(m, d)
def save_png(path, img):
    img = np.ascontiguousarray(img.astype(np.uint8)); h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))
    def ch(t, dat): return struct.pack(">I", len(dat)) + t + dat + struct.pack(">I", zlib.crc32(t + dat) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + ch(b"IDAT", zlib.compress(raw, 6)) + ch(b"IEND", b""))
def detect(cam, radius=None):
    mujoco.mj_forward(m, d); cid = cam_id(cam)
    R.update_scene(d, camera=cam); rgb = R.render().copy()
    R.update_scene(d, camera=cam); R.enable_depth_rendering(); depth = R.render().copy(); R.disable_depth_rendering()
    R.update_scene(d, camera=cam); R.enable_segmentation_rendering(); seg = R.render().copy(); R.disable_segmentation_rendering()
    ids, typ = seg[:, :, 0], seg[:, :, 1]; valid = typ == int(mujoco.mjtObj.mjOBJ_GEOM)
    mask = np.zeros((H, W), bool); mask[valid] = m.geom_bodyid[ids[valid]] == bt
    v, u = np.nonzero(mask); out = {"npix": len(u), "rgb": rgb, "mask": mask}
    if len(u) < 30: return out
    z = depth[v, u]; ok = np.isfinite(z) & (z > 0); u, v, z = u[ok], v[ok], z[ok]
    f = (H / 2) / np.tan(np.radians(m.cam_fovy[cid]) / 2)
    pw = d.cam_xpos[cid] + (d.cam_xmat[cid].reshape(3, 3) @ np.stack([(u - W / 2) / f * z, -(v - H / 2) / f * z, -z], 1).T).T
    ray = pw.mean(0) - d.cam_xpos[cid]; h = ray[:2] / np.linalg.norm(ray[:2]); p = np.array([-h[1], h[0]])
    diam = float(np.ptp(pw[:, :2] @ p)); r = radius if radius else diam / 2
    front = pw[z <= z.min() + 0.004]; cxy = front[:, :2].mean(0) + r * h
    zr = (pw[:, 2].min(), pw[:, 2].max())
    out.update(center=np.array([cxy[0], cxy[1], 0.5 * (zr[0] + zr[1])]), diam=diam)
    return out
def overlay(r):
    img = r["rgb"].copy(); mk = r["mask"]; img[mk] = (0.5 * img[mk] + 0.5 * np.array([255, 0, 0])).astype(np.uint8); return img
def err_mm(c): return np.linalg.norm(c - d.xpos[bt]) * 1000
# ---- 실행 준비 ----
mujoco.mj_forward(m, d)
d.ctrl[:6] = [d.qpos[x] for x in QADR]; d.ctrl[6:8] = OPEN
Q_HOME = np.array([d.qpos[x] for x in QADR])   # 시작(홈) 자세 저장
if not USE_SAVED:
    mount_look_at("station_rgbd_camera", HAND); mount_along_tool("gripper_rgbd_camera")
if not HEADLESS:
    import mujoco.viewer
    vw = mujoco.viewer.launch_passive(m, d, key_callback=lambda k: ESTOP_REQ.__setitem__(0, True) if k in (ord('X'), ord('x')) else None)
    vw.cam.azimuth = 90; vw.cam.elevation = -30; vw.cam.distance = 8; vw.cam.lookat[:] = [-0.5, -0.2, 0.4]
    t = time.time(); print("5초 대기: 지금 녹화 시작 (Win+Alt+R), 마우스로 시점 조정")
    while time.time() - t < 5.0: vw.sync(); time.sleep(0.03)
    T0[0] = time.time(); T0[1] = d.time
CMD = d.ctrl.copy()
# ---- 보행 구간 (zip 스크립트 보행 파형 이식, 스크립트 보행) ----
import math
import os
START_X, WALK_T = -2.5, 14.0
HIP, KNEE = float(os.environ.get('GAIT_A','0.15')), float(os.environ.get('GAIT_K','0.40'))
GF = float(os.environ.get('GAIT_F','0.6')); SG = float(os.environ.get('GAIT_SIGN','1'))
AI = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
LEGS = ["abad_L","hip_L","knee_L","abad_R","hip_R","knee_R"]
A_X = AI("act_tron1_x"); A_L = {n: AI("act_leg_"+n) for n in LEGS}
J_X = Jn("tron1_slide_x"); tray = B("carry_tray")
d.qpos[m.jnt_qposadr[J_X]] = START_X
d.qpos[m.jnt_qposadr[Jn("carry_bottle_3_free")]] += START_X   # 초기 배치만(시작 위치)
d.ctrl[A_X] = START_X
mujoco.mj_forward(m, d); wait(1.0)
rel0 = d.xpos[bt][:2] - d.xpos[tray][:2]
smooth = lambda u: (lambda v: v*v*(3-2*v))(max(0.0, min(1.0, u)))
print("AMR1 입고 대기 지점(x=-3.4) 도착 대기")
while d.xpos[B("claude_amr1")][0] < -3.5: wait(1/60.0)
print("== -1) TRON1 보행(스크립트 보행): x %.1f -> 0.0, %.0fs" % (START_X, WALK_T))
t0 = d.time
while d.time - t0 < WALK_T:
    t = d.time - t0; u = t / WALK_T
    d.ctrl[A_X] = START_X * (1 - smooth(u))
    bl = min(1.0, t / 1.0) * max(0.0, min(1.0, (1.0 - u) / 0.15))
    ph = 2 * math.pi * GF * t
    d.ctrl[A_L["abad_L"]] = 0.0; d.ctrl[A_L["abad_R"]] = 0.0
    d.ctrl[A_L["hip_L"]] = SG * HIP * math.sin(ph) * bl; d.ctrl[A_L["hip_R"]] = -SG * HIP * math.sin(ph) * bl
    d.ctrl[A_L["knee_L"]] = KNEE * max(0.0, SG * math.cos(ph)) * bl; d.ctrl[A_L["knee_R"]] = KNEE * max(0.0, -SG * math.cos(ph)) * bl
    wait(1/60.0)
d.ctrl[A_X] = 0.0
for n in LEGS: d.ctrl[A_L[n]] = 0.0
wait(2.0)
shift = np.linalg.norm((d.xpos[bt][:2] - d.xpos[tray][:2]) - rel0) * 1000
ok = d.xpos[bt][2] > 0.95 and shift < 30
print("보행 종료: 베이스 x %.3f | 병 z %.3f | 트레이 대비 병 이동 %.1f mm | 병 유지 판정(측정 기준): %s" % (d.qpos[m.jnt_qposadr[J_X]], d.xpos[bt][2], shift, "PASS" if ok else "FAIL"))
if not ok: print("병이 트레이에서 이탈 → 중단"); sys.exit(1)
print("== 0) 자체 검증: 공칭 중심(%.2f,%.2f,%.3f)으로 IK를 풀면 저장해와 같아야 함" % tuple(NOM)); waypoints(NOM, ("above", "pregrasp", "grasp", "lift"))
SAVED = json.load(open(WD + "detections.json")) if USE_SAVED else {}
# ---- 1) 스테이션 RGB-D: 감지 + 정지 대기 ----
print("== 1) 스테이션 카메라 감지, 병 정지 대기(3프레임 연속 2mm 이내)")
if USE_SAVED:
    c_st = np.array(SAVED["station"]); diam_st = SAVED["diam"]; print("   (저장된 감지값 사용)", np.round(c_st, 4))
else:
    prev = None; stable = 0; res = None
    for k in range(40):
        res = detect("station_rgbd_camera")
        if res["npix"] < 30: print("   frame %d: 병 안 보임" % k); wait(0.25); continue
        c = res["center"]
        stable = stable + 1 if prev is not None and np.linalg.norm(c[:2] - prev[:2]) < 0.002 else 0
        print("   frame %d t=%.2fs: 중심 (%.4f, %.4f, %.4f) 픽셀 %d | 직전 프레임 대비 변화 %s | 안정 %d/3" %
              (k, d.time, c[0], c[1], c[2], res["npix"], "-" if prev is None else "%.2f mm" % (np.linalg.norm(c[:2] - prev[:2]) * 1000), stable))
        prev = c
        if stable >= 3: break
        wait(0.25)
    if res is None or res["npix"] < 30 or stable < 3: print("정지 판정 실패 → 중단"); sys.exit(1)
    c_st = res["center"]; diam_st = res["diam"]; save_png(WD + "det_station.png", ann_side("STATION RGB-D", res, ["STATIONARY: change < 2mm for %d/3 frames" % stable] + cam_info(d, cam_id("station_rgbd_camera"), res, d.xpos[bt]), 1380, 40))
    print("   정지 확인. 스테이션 추정 중심 (%.4f, %.4f, %.4f) | 지름 %.1f mm | (평가용) 정답 대비 3D 오차 %.1f mm" % (c_st[0], c_st[1], c_st[2], diam_st * 1000, err_mm(c_st)))
# ---- 2) 감지 → 접근 경유점 ----
print("== 2) 감지 중심에서 above/pregrasp IK"); W1 = waypoints(c_st, ("above", "pregrasp"))
z0 = d.xpos[bt].copy()
if HRI and os.environ.get("HRI_SCEN") != "full": HRI.start()   # full: 병 옮기기가 먼저, 협업은 그 뒤
move("above", W1["above"], OPEN, 3.0)
move("pregrasp", W1["pregrasp"], OPEN, 2.0)
wait(0.4)
# ---- 3) 그리퍼 RGB-D로 정밀화 ----
print("== 3) 그리퍼 카메라(pregrasp)로 x,y 정밀화")
c_fin = c_st.copy()
if USE_SAVED:
    gxy = SAVED.get("gripper_xy"); 
    if gxy: c_fin[:2] = gxy; print("   (저장된 감지값 사용)", np.round(gxy, 4))
else:
    gres = detect("gripper_rgbd_camera", radius=diam_st / 2)
    if gres["npix"] >= 30:
        c_fin[:2] = gres["center"][:2]; save_png(WD + "det_gripper.png", ann_side("GRIPPER RGB-D (pregrasp)", gres, ["pregrasp view"] + cam_info(d, cam_id("gripper_rgbd_camera"), gres, d.xpos[bt]), 1380, 420))
        print("   그리퍼 추정 xy (%.4f, %.4f) | 스테이션 xy와 차이 %.1f mm | (평가용) 정답 xy 오차 %.1f mm | 픽셀 %d" %
              (c_fin[0], c_fin[1], np.linalg.norm(c_fin[:2] - c_st[:2]) * 1000, np.linalg.norm(c_fin[:2] - d.xpos[bt][:2]) * 1000, gres["npix"]))
    else: print("   그리퍼 카메라에서 병 안 보임 → 스테이션 값 사용")
    json.dump({"station": c_st.tolist(), "diam": diam_st, "gripper_xy": c_fin[:2].tolist()}, open(WD + "detections.json", "w"))
W2 = waypoints(c_fin, ("grasp", "lift"))
# ---- 4) 파지, 들어올림 ----
move("approach", W2["grasp"], OPEN, 2.0)
open_gap = finger_gap()
move("close", W2["grasp"], GRIP, 1.0); wait(0.5)
rel0 = d.xmat[tool].reshape(3, 3).T @ (d.xpos[bt] - d.xpos[tool])
dia_true = 2 * m.geom_size[gb][0]; cl_gap = finger_gap()
print("bottle in tool frame:", np.round(rel0, 3))
print("지름 검증: 벌림(열림) 간격 %.4f m | 닫힌 손가락 안쪽 간격 %.4f m | 병 지름 %.4f m | 차이 %.1f mm" % (open_gap, cl_gap, dia_true, abs(cl_gap - dia_true) * 1000))
print("close 후 접촉 법선력(N) 좌/우:", grip_force())
move("lift", W2["lift"], GRIP, 2.0); wait(1.5)
rel1 = d.xmat[tool].reshape(3, 3).T @ (d.xpos[bt] - d.xpos[tool]); slip = np.linalg.norm(rel1 - rel0)
print("lift 후 접촉 법선력(N) 좌/우:", grip_force())
print("lift hold 1.5s | bottle z %.3f | slip %.4f m | contacts %s | lifted %.3f m" % (d.xpos[bt][2], slip, finger_contacts(), d.xpos[bt][2] - z0[2]))
if not USE_SAVED:
    gs = detect("station_rgbd_camera", radius=diam_st / 2)
    L = ["open gap %.1f mm | closed inner gap %.1f mm | bottle dia(model) %.1f mm | diff %.1f mm" % (open_gap * 1000, cl_gap * 1000, dia_true * 1000, abs(cl_gap - dia_true) * 1000),
         "held & lifted %.0f mm | slip %.1f mm | grip force L/R (N): %s" % ((d.xpos[bt][2] - z0[2]) * 1000, slip * 1000, grip_force())]
    save_png(WD + "det_lift.png", ann_side("STATION RGB-D (bottle held, lifted)", gs, L + cam_info(d, cam_id("station_rgbd_camera"), gs, d.xpos[bt]), 1380, 800))
grasp_ok = finger_contacts() == (True, True) and d.xpos[bt][2] - z0[2] > 0.08 and slip < 0.005
print("감지 기반 파지·들어올림 판정(측정 기준):", "PASS" if grasp_ok else "FAIL")
if not grasp_ok:
    print("파지 실패 → 놓기 생략"); 
    if vw: wait(3.0)
    sys.exit(1)
# ---- 5) 놓기 ----
Rt = d.xmat[tool].reshape(3, 3).copy(); q_now = np.array([d.qpos[x] for x in QADR]); base_xy = d.xpos[base_b][:2]
def plan(px, py):
    a = np.array([px, py]) - base_xy; a0 = Rt[:, 2][:2]
    th = np.arctan2(a[1], a[0]) - np.arctan2(a0[1], a0[0]); th = (th + np.pi) % (2 * np.pi) - np.pi
    c, s = np.cos(th), np.sin(th); Rg = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ Rt
    tp = lambda z: np.array([px, py, z]) - Rg @ rel0
    P = {}; seed = q_now; worst = 0
    for name, pos in (("hi", tp(1.20)), ("above", tp(PLACE_Z + 0.06)), ("place", tp(PLACE_Z + 0.010)), ("back", tp(PLACE_Z + 0.010) - 0.12 * Rg[:, 2])):
        q, ep, er = ik(pos, Rg, seed)
        qu = q + 2 * np.pi * np.round((seed - q) / (2 * np.pi))
        if all(LIM[i][0] <= qu[i] <= LIM[i][1] for i in range(6)): q = qu
        worst = max(worst, ep / 0.003, er / 0.02); P[name] = q; seed = q
    return P, worst, max(np.max(np.abs(P[n] - q_now)) for n in P)
chosen = None
for (px, py) in [(x, y) for x in (0.85, 0.82, 0.88, 0.90) for y in (-3.10, -3.07, -3.04, -3.13)]:
    P, worst, jump = plan(px, py)
    if worst <= 2.0 and jump <= 1.7 and (chosen is None or jump < chosen[3]): chosen = (px, py, P, jump)
if chosen is None:
    print("놓기 IK 후보 전부 실패"); 
    if vw: wait(3.0)
    sys.exit(1)
px, py, P, _j = chosen; print("선택된 놓기 위치: (%.2f, %.2f), 병 중심 z %.3f" % (px, py, PLACE_Z))
move("transit", P["hi"], GRIP, 3.0); move("above_pl", P["above"], GRIP, 1.5); move("lower", P["place"], GRIP, 1.5)
move("release", P["place"], OPENR, 1.0)
for _k in range(6):
    wait(0.2); _z = d.xmat[bt].reshape(3, 3)[:, 2]
    print("   release+%.1fs: bottle z %.4f | tilt %.1f deg | contacts %s | force %s" % (0.2 * (_k + 1), d.xpos[bt][2], np.degrees(np.arccos(np.clip(_z[2], -1, 1))), finger_contacts(), grip_force()))
move("ret_back", P["back"], OPENR, 2.0); wait(2.0)
bz = d.xmat[bt].reshape(3, 3)[:, 2]; tilt = np.degrees(np.arccos(np.clip(bz[2], -1, 1)))
bj = m.body_jntadr[bt]; da = m.jnt_dofadr[bj]; spd = np.linalg.norm(d.qvel[da:da + 3])
print("최종 병 위치", np.round(d.xpos[bt], 3), "| 기울기 %.1f deg | 속도 %.4f m/s | 외부 충돌 기록 %s" % (tilt, spd, sorted(touched)))
ok = abs(d.xpos[bt][2] - PLACE_Z) < 0.006 and tilt < 5 and spd < 0.01
print("놓기 안정 판정(측정 기준):", "PASS" if ok else "FAIL")
print("== 복귀: 병을 놓은 뒤 그리퍼를 위로 빼고 홈 자세로 돌아옴")
_pb, _Rb = fk(P["back"]); _ql, _ep, _er = ik(_pb + np.array([0, 0, 0.25]), _Rb, P["back"])
if _ep > 0.01: print("   경고: 위로 빼는 IK 오차 %.1f mm" % (_ep * 1000))
move("lift_away", _ql, OPEN, 2.0); move("go_home", Q_HOME, OPEN, 4.0); wait(1.0)
_he = max(abs(d.qpos[QADR[i]] - Q_HOME[i]) for i in range(6))
_bz = d.xmat[bt].reshape(3, 3)[:, 2]; _tl = np.degrees(np.arccos(np.clip(_bz[2], -1, 1)))
print("홈 복귀 판정(측정 기준):", "PASS" if _he < 0.05 and abs(d.xpos[bt][2] - PLACE_Z) < 0.006 and _tl < 5 else "FAIL",
      "| 홈 대비 관절오차 %.3f rad | 병 위치 %s | 기울기 %.1f deg" % (_he, np.round(d.xpos[bt], 3), _tl))
if HRI and os.environ.get("HRI_SCEN") == "full":
    print("== 협업 구간: 작업자 접근 (감속 -> 보호정지 -> 외력 감지 -> 재개)")
    HRI.start()
    while d.time - HRI.t0 < 21.0:
        qa = Q_HOME.copy(); qb = Q_HOME.copy(); qa[5] -= 0.6; qb[5] += 0.6
        move("collab_a", qa, OPEN, 2.5); move("collab_b", qb, OPEN, 2.5)
    HRI.finale(); wait(20.0)
if vw: wait(3.0)
