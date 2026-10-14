#!/usr/bin/env python3
"""UR5e 통합 파지 / 들기 / 운반 / 놓기 테스트 v2.

- 기존 /tmp/ur5e_integrated_grasp_test.py 와 XML 은 수정하지 않는다.
- weld/equality 없음, teleport 없음, 충돌 비활성화 없음, actuator gain 변경 없음.
- 달라진 점은 '제어 입력 계산'뿐이다.
    * feedforward: ctrl = q_target + qfrc_bias / kp   (중력 처짐 보상)
    * 정지 중 적분 보정: 병 하중 같은 남은 정적 오차를 줄임
  --baseline 을 주면 기존 방식(ctrl = q_target)으로 실행해 비교할 수 있다.
- PASS 는 아래 물리 조건이 실제 측정값으로 모두 만족될 때만 출력한다.

사용:
  python3 ur5e_grasp_test_v2.py --baseline     # 기존 방식(실패 재현/비교용)
  python3 ur5e_grasp_test_v2.py                # feedforward 적용
  python3 ur5e_grasp_test_v2.py --grip 0.004   # 손가락 목표를 더 닫아 누르는 힘 증가(gain 변경 아님)
"""
import argparse
import sys

import mujoco
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--xml", default="/ros_ws/mujoco/scenes/warehouse_tron1_integration.xml")
ap.add_argument("--baseline", action="store_true", help="feedforward/적분 없이 기존 방식으로 실행")
ap.add_argument("--grip", type=float, default=0.012, help="닫을 때 손가락 ctrl (기존 0.012)")
ap.add_argument("--open", dest="open_ctrl", type=float, default=0.064, help="열 때 손가락 ctrl (기존 0.064)")
ap.add_argument("--ki", type=float, default=2.0, help="관절 적분 gain [1/s]")
ap.add_argument("--place-x", type=float, default=0.85, help="병을 놓을 목표 x (테이블 중심 확인 후 지정)")
ap.add_argument("--place-y", type=float, default=-3.0, help="병을 놓을 목표 y")
args = ap.parse_args()

# ---- 판정 기준 (낮추지 않는다. 필요하면 근거와 함께 여기서만 바꾼다) ----
TRACK_TOL = 0.03      # rad, 각 이동 단계 종료 시 6관절 오차 norm
LIFT_MIN = 0.04       # m, 병 최대 상승량
SLIP_TOL = 0.015      # m, 운반 중 병-그리퍼 상대 위치 변화
PLACE_TOL = 0.03      # m, 최종 병 xy 와 목표 xy 의 오차
REST_Z_TOL = 0.02     # m, 최종 병 z 와 초기 안착 z 의 차이
MIN_NORMAL = 0.5      # N, 손가락당 최소 수직항력
UPRIGHT_MIN = 0.98    # 병 z축과 월드 z축의 내적
CARRY_CONTACT_MIN = 0.95  # 운반 중 양쪽 손가락 접촉 유지 비율
Z_CLEAR = 0.003       # m, 놓을 때 테이블 위 여유

m = mujoco.MjModel.from_xml_path(args.xml)
d = mujoco.MjData(m)
ikd = mujoco.MjData(m)
dt = m.opt.timestep
O = mujoco.mjtObj


def nid(kind, name):
    i = mujoco.mj_name2id(m, kind, name)
    if i < 0:
        raise RuntimeError("NOT_FOUND: %s" % name)
    return i


arm_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
arm_j = np.array([nid(O.mjOBJ_JOINT, n) for n in arm_names])
arm_qadr = np.array([m.jnt_qposadr[j] for j in arm_j])
arm_dadr = np.array([m.jnt_dofadr[j] for j in arm_j])
arm_a = np.array([nid(O.mjOBJ_ACTUATOR, "act_" + n.replace("_joint", "")) for n in arm_names])
finger_a = np.array([nid(O.mjOBJ_ACTUATOR, "act_left_finger"), nid(O.mjOBJ_ACTUATOR, "act_right_finger")])
tool = nid(O.mjOBJ_BODY, "tool0")
bottle = nid(O.mjOBJ_BODY, "bottle_target")
bottle_geom = nid(O.mjOBJ_GEOM, "bottle_geom")
left_geom = nid(O.mjOBJ_GEOM, "left_finger_geom")
right_geom = nid(O.mjOBJ_GEOM, "right_finger_geom")

TARGET_R = np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]])
LO = m.jnt_range[arm_j, 0]
HI = m.jnt_range[arm_j, 1]
kp = np.array([m.actuator_gainprm[a, 0] for a in arm_a])


def r(x, n=4):
    return np.round(x, n)


# ------------------------------------------------------------------ 사전 점검 출력
def preflight():
    print("=== PREFLIGHT ===")
    print("MuJoCo %s dt=%g nq=%d nv=%d nu=%d neq=%d" % (mujoco.__version__, dt, m.nq, m.nv, m.nu, m.neq))
    print("모드:", "BASELINE(기존 방식)" if args.baseline else "FEEDFORWARD + 정지 중 적분")
    print("arm kp =", r(kp, 3), " biasprm[1] =", r([m.actuator_biasprm[a, 1] for a in arm_a], 3))
    for a in arm_a:
        print("  %s gain=%.4g ctrl=%s%s force=%s%s" % (
            mujoco.mj_id2name(m, O.mjOBJ_ACTUATOR, a), m.actuator_gainprm[a, 0],
            "lim" if m.actuator_ctrllimited[a] else "free", r(m.actuator_ctrlrange[a]),
            "lim" if m.actuator_forcelimited[a] else "free", r(m.actuator_forcerange[a])))
    for a in finger_a:
        jn = mujoco.mj_id2name(m, O.mjOBJ_JOINT, m.actuator_trnid[a, 0])
        print("  FINGER %s joint=%s gain=%.4g bias=%s ctrl=%s%s force=%s%s" % (
            mujoco.mj_id2name(m, O.mjOBJ_ACTUATOR, a), jn, m.actuator_gainprm[a, 0], r(m.actuator_biasprm[a, :3]),
            "lim" if m.actuator_ctrllimited[a] else "free", r(m.actuator_ctrlrange[a]),
            "lim" if m.actuator_forcelimited[a] else "free", r(m.actuator_forcerange[a])))
    g = bottle_geom
    print("BOTTLE mass=%.4g geom type=%d size=%s friction=%s condim=%d solref=%s solimp=%s" % (
        m.body_mass[bottle], m.geom_type[g], r(m.geom_size[g]), r(m.geom_friction[g]),
        m.geom_condim[g], r(m.geom_solref[g]), r(m.geom_solimp[g], 3)))
    for name, g in (("LEFT", left_geom), ("RIGHT", right_geom)):
        print("FINGER_GEOM %s type=%d size=%s friction=%s condim=%d solref=%s solimp=%s" % (
            name, m.geom_type[g], r(m.geom_size[g]), r(m.geom_friction[g]), m.geom_condim[g],
            r(m.geom_solref[g]), r(m.geom_solimp[g], 3)))
    for gi in range(m.ngeom):
        n = mujoco.mj_id2name(m, O.mjOBJ_GEOM, gi) or ""
        bn = mujoco.mj_id2name(m, O.mjOBJ_BODY, m.geom_bodyid[gi]) or ""
        if "table" in n.lower() or "table" in bn.lower():
            print("TABLE geom=%s body=%s type=%d size=%s pos=%s" % (n, bn, m.geom_type[gi], r(m.geom_size[gi]), r(d.geom_xpos[gi])))
    if m.neq:
        print("NOTE equality constraints present:", m.neq)


# ------------------------------------------------------------------ IK (별도 MjData, 시뮬레이션 상태 불변)
def pose_error(dd, target_p):
    ep = target_p - dd.xpos[tool]
    R = dd.xmat[tool].reshape(3, 3)
    E = TARGET_R @ R.T
    er = 0.5 * np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    return ep, er


def solve_ik(target_p, seed, label):
    ikd.qpos[:] = d.qpos
    ikd.qvel[:] = 0
    ikd.qpos[arm_qadr] = seed
    for _ in range(3500):
        mujoco.mj_forward(m, ikd)
        ep, er = pose_error(ikd, target_p)
        pe, re_ = np.linalg.norm(ep), np.linalg.norm(er)
        if pe < 0.006 and re_ < 0.025:
            print("IK_%s: OK pos=%.4fm rot=%.4frad" % (label, pe, re_))
            return ikd.qpos[arm_qadr].copy()
        jp = np.zeros((3, m.nv))
        jr = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, ikd, jp, jr, tool)
        J = np.vstack((jp[:, arm_dadr], jr[:, arm_dadr]))
        dq = J.T @ np.linalg.solve(J @ J.T + 0.025 * np.eye(6), np.r_[ep, er])
        ikd.qpos[arm_qadr] = np.clip(ikd.qpos[arm_qadr] + np.clip(dq, -0.035, 0.035), LO, HI)
    mujoco.mj_forward(m, ikd)
    ep, er = pose_error(ikd, target_p)
    raise RuntimeError("IK_%s: FAIL pos=%.4fm rot=%.4frad" % (label, np.linalg.norm(ep), np.linalg.norm(er)))


# ------------------------------------------------------------------ 제어
integ = np.zeros(6)


def arm_cmd(q_target):
    if args.baseline:
        c = np.array(q_target, dtype=float)
    else:
        if np.linalg.norm(d.qvel[arm_dadr]) < 0.02:      # 거의 정지했을 때만 적분
            integ[:] = np.clip(integ + args.ki * (q_target - d.qpos[arm_qadr]) * dt, -0.10, 0.10)
        c = q_target + d.qfrc_bias[arm_dadr] / kp + integ
    for i, a in enumerate(arm_a):
        if m.actuator_ctrllimited[a]:
            c[i] = np.clip(c[i], m.actuator_ctrlrange[a, 0], m.actuator_ctrlrange[a, 1])
    return c


def finger_contacts():
    force = {"L": 0.0, "R": 0.0}
    pairs = []
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        if bottle_geom not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == bottle_geom else c.geom1
        on = mujoco.mj_id2name(m, O.mjOBJ_GEOM, other) or "geom%d" % other
        if "finger" not in on.lower():
            continue
        mujoco.mj_contactForce(m, d, i, f6)
        side = "L" if "left" in on.lower() else "R"
        force[side] += abs(float(f6[0]))
        pairs.append((on, round(float(c.dist), 5), round(float(f6[0]), 3)))
    return force, pairs


stats = {"max_z": -1e9, "drift": 0.0, "carry_n": 0, "carry_both": 0}
track_rel = [None]
step_count = [0]
phase_errors = []


def sample():
    stats["max_z"] = max(stats["max_z"], float(d.xpos[bottle][2]))
    if track_rel[0] is not None:
        R = d.xmat[tool].reshape(3, 3)
        rel = R.T @ (d.xpos[bottle] - d.xpos[tool])
        stats["drift"] = max(stats["drift"], float(np.linalg.norm(rel - track_rel[0])))
        f, _ = finger_contacts()
        stats["carry_n"] += 1
        if f["L"] > 0 and f["R"] > 0:
            stats["carry_both"] += 1


def step(q_target, fing):
    d.ctrl[arm_a] = arm_cmd(q_target)
    d.ctrl[finger_a] = fing
    mujoco.mj_step(m, d)
    step_count[0] += 1
    if step_count[0] % 10 == 0:
        sample()


def hold(q_target, fing, secs):
    for _ in range(int(secs / dt)):
        step(q_target, fing)


def move(q1, secs, fing, label, target_p=None):
    q0 = d.qpos[arm_qadr].copy()
    n = max(1, int(secs / dt))
    for k in range(n):
        u = (k + 1) / n
        f = 0.5 - 0.5 * np.cos(np.pi * u)
        step(q0 + f * (q1 - q0), fing)
    hold(q1, fing, 1.5)
    mujoco.mj_forward(m, d)
    actual = d.qpos[arm_qadr].copy()
    err = actual - q1
    en = float(np.linalg.norm(err))
    msg = "MOVE_%s: joint_error=%.4frad" % (label, en)
    if target_p is not None:
        msg += " tool_pos_error=%.1fmm" % (1000 * np.linalg.norm(d.xpos[tool] - target_p))
    print(msg)
    for name, tg, val, de in zip(arm_names, q1, actual, err):
        print("  %s: target=%.4f actual=%.4f error=%+.4f" % (name, tg, val, de))
    print("  bottle_xyz=", r(d.xpos[bottle]).tolist())
    phase_errors.append((label, en))
    return actual


def ramp_fingers(q_hold, c0, c1, secs):
    n = max(1, int(secs / dt))
    for k in range(n):
        step(q_hold, c0 + (k + 1) / n * (c1 - c0))


# ------------------------------------------------------------------ 본 테스트
def main():
    preflight()
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    initial_bottle = d.xpos[bottle].copy()
    print("BOTTLE_INITIAL:", r(initial_bottle).tolist())
    start_q = d.qpos[arm_qadr].copy()
    open_c, grip_c = args.open_ctrl, args.grip

    hold(start_q, open_c, 0.5)
    print("BOTTLE_AFTER_SETTLE:", r(d.xpos[bottle]).tolist())

    bx, by, bz = initial_bottle
    DX, DZ = 0.083, 0.0825           # 기존 스크립트의 접근 오프셋(병 중심 기준)
    p_high = np.array([bx + DX, by, 1.22])
    p_grasp = np.array([bx + DX, by, bz + DZ])
    p_lift = np.array([bx + DX, by, 1.20])
    q_high = solve_ik(p_high, start_q, "HIGH")
    q_grasp = solve_ik(p_grasp, q_high, "GRASP")
    q_lift = solve_ik(p_lift, q_grasp, "LIFT")

    move(q_high, 6.0, open_c, "APPROACH_HIGH", p_high)
    move(q_grasp, 6.0, open_c, "DESCEND", p_grasp)

    # 그리퍼 좌표계/축 검증: 손가락 사이 축이 수평인가
    mujoco.mj_forward(m, d)
    lp, rp = d.geom_xpos[left_geom].copy(), d.geom_xpos[right_geom].copy()
    axis = lp - rp
    dist = float(np.linalg.norm(axis))
    axis /= dist
    mid = 0.5 * (lp + rp)
    off = d.xpos[bottle] - mid
    perp = off - np.dot(off, axis) * axis
    horizontal_ok = abs(axis[2]) < 0.2
    print("FINGER_AXIS(world)=%s center_distance=%.4fm bottle_dia=%.4fm" % (
        r(axis, 3).tolist(), dist, 2 * m.geom_size[bottle_geom][0]))
    print("BOTTLE_OFFSET_FROM_FINGER_MIDPOINT=%s (축 방향 성분 제외 거리 %.1fmm)" % (
        r(off).tolist(), 1000 * np.linalg.norm(perp)))

    # 파지: 접촉은 가정하지 않고 측정한다
    ramp_fingers(q_grasp, open_c, grip_c, 1.0)
    hold(q_grasp, grip_c, 1.0)
    mujoco.mj_forward(m, d)
    grasp_force, grasp_pairs = finger_contacts()
    closed_bottle = d.xpos[bottle].copy()
    print("GRASP_BOTTLE:", r(closed_bottle).tolist())
    print("GRASP_FINGER_CONTACT_PAIRS(name, dist, normal_force):", grasp_pairs[:12])
    print("GRASP_NORMAL_FORCE L=%.2fN R=%.2fN  finger_actuator_force=%s" % (
        grasp_force["L"], grasp_force["R"], r(d.actuator_force[finger_a], 2).tolist()))
    need = m.body_mass[bottle] * 9.81 / (2 * max(m.geom_friction[bottle_geom][0], 1e-6))
    print("참고: 병 무게를 마찰만으로 버티는 데 필요한 손가락당 최소 수직항력 ≈ %.2fN (안전계수 별도)" % need)

    # 이후 병-그리퍼 상대 위치 변화(미끄러짐) 추적 시작
    R0 = d.xmat[tool].reshape(3, 3)
    track_rel[0] = R0.T @ (d.xpos[bottle] - d.xpos[tool])
    grasp_off = d.xpos[tool] - d.xpos[bottle]
    stats["max_z"] = float(closed_bottle[2])

    # 놓을 위치: 측정된 파지 오프셋을 반영
    place_xy = np.array([args.place_x, args.place_y])
    p_carry = np.array([place_xy[0] + grasp_off[0], place_xy[1] + grasp_off[1], 1.12])
    p_lower = np.array([place_xy[0] + grasp_off[0], place_xy[1] + grasp_off[1], bz + Z_CLEAR + grasp_off[2]])
    p_retreat = p_lower + np.array([0.0, 0.0, 0.05])
    q_carry = solve_ik(p_carry, q_lift, "CARRY")
    q_lower = solve_ik(p_lower, q_carry, "LOWER")
    q_retreat = solve_ik(p_retreat, q_lower, "RETREAT")

    move(q_lift, 6.0, grip_c, "LIFT", p_lift)
    lifted_bottle = d.xpos[bottle].copy()
    print("BOTTLE_AFTER_LIFT:", r(lifted_bottle).tolist())
    move(q_carry, 6.0, grip_c, "CARRY", p_carry)
    move(q_lower, 6.0, grip_c, "LOWER", p_lower)

    # 해제: 상대 위치 추적 종료 후 손가락을 연다
    track_rel[0] = None
    ramp_fingers(q_lower, grip_c, open_c, 0.8)
    hold(q_lower, open_c, 0.5)
    move(q_retreat, 3.0, open_c, "RETREAT", p_retreat)

    pos_hist = []
    for _ in range(int(2.0 / dt)):
        step(q_retreat, open_c)
        if step_count[0] % 100 == 0:
            pos_hist.append(d.xpos[bottle].copy())
    mujoco.mj_forward(m, d)

    final_bottle = d.xpos[bottle].copy()
    final_force, final_pairs = finger_contacts()
    speed = float(np.linalg.norm(pos_hist[-1] - pos_hist[-6]) / (5 * 100 * dt)) if len(pos_hist) >= 6 else 0.0
    upright = float(d.xmat[bottle].reshape(3, 3)[2, 2])

    max_lift = stats["max_z"] - float(closed_bottle[2])
    place_err = float(np.linalg.norm(final_bottle[:2] - place_xy))
    carry_ratio = stats["carry_both"] / stats["carry_n"] if stats["carry_n"] else 0.0
    worst_track = max(e for _, e in phase_errors)

    print("BOTTLE_FINAL:", r(final_bottle).tolist())
    print("FINAL_FINGER_CONTACTS:", final_pairs[:12])
    print("\n=== METRICS ===")
    print("bottle_initial =", r(initial_bottle).tolist())
    print("bottle_final   =", r(final_bottle).tolist())
    print("worst_joint_error = %.4f rad  (phases: %s)" % (worst_track, [(l, round(e, 4)) for l, e in phase_errors]))
    print("finger_contact_pairs_at_grasp =", grasp_pairs[:12])
    print("grasp_normal_force L=%.2fN R=%.2fN" % (grasp_force["L"], grasp_force["R"]))
    print("max_lift = %.4f m" % max_lift)
    print("carry_slip_drift = %.4f m, carry_both_contact_ratio = %.3f" % (stats["drift"], carry_ratio))
    print("table_center_error(xy) = %.4f m (목표 %s)" % (place_err, place_xy.tolist()))
    print("final_dz = %+.4f m, upright = %.4f, final_speed = %.4f m/s" % (final_bottle[2] - initial_bottle[2], upright, speed))

    checks = [
        ("NO_EQUALITY_CONSTRAINTS(weld 없음)", m.neq == 0, "neq=%d" % m.neq),
        ("JOINT_TRACKING<=%.2frad" % TRACK_TOL, worst_track <= TRACK_TOL, "worst=%.4f" % worst_track),
        ("FINGER_AXIS_HORIZONTAL", horizontal_ok, "axis_z=%.3f" % axis[2]),
        ("BOTH_FINGER_CONTACT>=%.1fN" % MIN_NORMAL,
         grasp_force["L"] >= MIN_NORMAL and grasp_force["R"] >= MIN_NORMAL,
         "L=%.2f R=%.2f" % (grasp_force["L"], grasp_force["R"])),
        ("LIFT>=%.2fm" % LIFT_MIN, max_lift >= LIFT_MIN, "max_lift=%.4f" % max_lift),
        ("NO_SLIP<=%.3fm" % SLIP_TOL, stats["drift"] <= SLIP_TOL, "drift=%.4f" % stats["drift"]),
        ("CARRY_CONTACT_KEPT>=%.2f" % CARRY_CONTACT_MIN, carry_ratio >= CARRY_CONTACT_MIN, "ratio=%.3f" % carry_ratio),
        ("PLACE_XY<=%.2fm" % PLACE_TOL, place_err <= PLACE_TOL, "err=%.4f" % place_err),
        ("RESTING_HEIGHT<=%.2fm" % REST_Z_TOL, abs(final_bottle[2] - initial_bottle[2]) <= REST_Z_TOL,
         "dz=%+.4f" % (final_bottle[2] - initial_bottle[2])),
        ("UPRIGHT>=%.2f" % UPRIGHT_MIN, upright >= UPRIGHT_MIN, "z_dot=%.4f" % upright),
        ("RELEASED(손가락 접촉 없음)", len(final_pairs) == 0, "pairs=%d" % len(final_pairs)),
        ("STABLE(속도<0.01m/s)", speed < 0.01, "speed=%.4f" % speed),
    ]
    print("\n=== RESULT ===")
    for name, ok, info in checks:
        print("%-38s %s  (%s)" % (name, "PASS" if ok else "FAIL", info))
    overall = all(ok for _, ok, _ in checks)
    print("OVERALL:", "PASS" if overall else "FAIL")
    print("Note: 물리 시뮬레이션 테스트이며 실제 하드웨어 보장이 아님.")
    return 0 if overall else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("INTEGRATED_TEST_ABORTED:", type(e).__name__, str(e))
        print("PASS 아님. MuJoCo 파지/놓기 완료로 표시하지 말 것.")
        raise
