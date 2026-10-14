import re
WD="/ros_ws/mujoco/claude_work/"
s=open(WD+"claude_closed_loop_clean.py").read()
assert "claude_clean_scene.xml" in s
s=s.replace("claude_clean_scene.xml","claude_walk_scene.xml",1)
PRE = r'''# ---- 보행 구간 (zip 스크립트 보행 파형 이식, 스크립트 보행) ----
import math
START_X, WALK_T, HIP, KNEE = -2.5, 14.0, 0.24, 0.32
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
print("== -1) TRON1 보행(스크립트 보행): x %.1f -> 0.0, %.0fs" % (START_X, WALK_T))
t0 = d.time
while d.time - t0 < WALK_T:
    t = d.time - t0; u = t / WALK_T
    d.ctrl[A_X] = START_X * (1 - smooth(u))
    bl = min(1.0, t / 1.0) * max(0.0, min(1.0, (1.0 - u) / 0.15))
    w = math.sin(2 * math.pi * 1.2 * t)
    d.ctrl[A_L["abad_L"]] = 0.0; d.ctrl[A_L["abad_R"]] = 0.0
    d.ctrl[A_L["hip_L"]] = HIP * w * bl; d.ctrl[A_L["hip_R"]] = HIP * w * bl
    d.ctrl[A_L["knee_L"]] = KNEE * max(0.0, w) * bl; d.ctrl[A_L["knee_R"]] = KNEE * max(0.0, -w) * bl
    wait(1/60.0)
d.ctrl[A_X] = 0.0
for n in LEGS: d.ctrl[A_L[n]] = 0.0
wait(2.0)
shift = np.linalg.norm((d.xpos[bt][:2] - d.xpos[tray][:2]) - rel0) * 1000
ok = d.xpos[bt][2] > 0.95 and shift < 30
print("보행 종료: 베이스 x %.3f | 병 z %.3f | 트레이 대비 병 이동 %.1f mm | 병 유지 판정(측정 기준): %s" % (d.qpos[m.jnt_qposadr[J_X]], d.xpos[bt][2], shift, "PASS" if ok else "FAIL"))
if not ok: print("병이 트레이에서 이탈 → 중단"); sys.exit(1)
'''
mt=re.search(r'^print\("== 0\)', s, re.M); assert mt, "== 0) 줄 못 찾음"
s=s[:mt.start()]+PRE+s[mt.start():]
open(WD+"claude_walk_grasp.py","w").write(s); print("생성 완료")
