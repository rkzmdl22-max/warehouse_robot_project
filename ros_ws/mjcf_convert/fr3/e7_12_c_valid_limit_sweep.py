#!/usr/bin/env python3

import hashlib
import json
import math
import os
import random
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    import mujoco
except ImportError:
    print("ERROR: mujoco Python package is not installed.")
    sys.exit(2)


MJCF_PATH = "/ros_ws/mjcf_convert/fr3/output/fr3.xml"
URDF_PATH = "/ros_ws/mjcf_convert/fr3.urdf"
RESULT_PATH = "/ros_ws/mjcf_convert/fr3/e7_12_c_result.json"

JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
TCP_LINK = "fr3_hand_tcp"

RANDOM_SEED = 712
RANDOM_COUNT = 100

# Numerical tolerance for valid-limit membership.
LIMIT_EPS = 1e-10

# FK comparison tolerance.
POS_TOL = 1e-10
ROT_TOL = 1e-10


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_vec(text, n=3):
    if text is None:
        return np.zeros(n, dtype=float)
    v = np.fromstring(text, sep=" ", dtype=float)
    if len(v) != n:
        raise ValueError(f"Expected {n} values, got {text!r}")
    return v


def rpy_matrix(rpy):
    """
    URDF fixed-joint / origin convention:
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    roll, pitch, yaw = rpy

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr],
    ], dtype=float)

    Ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp],
    ], dtype=float)

    Rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1],
    ], dtype=float)

    return Rz @ Ry @ Rx


def homogeneous(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError("Zero joint axis")

    axis = axis / norm
    x, y, z = axis

    K = np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0],
    ])

    I = np.eye(3)
    return I + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def load_urdf():
    root = ET.parse(URDF_PATH).getroot()

    joints = {}

    for j in root.findall("joint"):
        name = j.get("name")
        if not name:
            continue

        parent = j.find("parent")
        child = j.find("child")
        origin = j.find("origin")
        axis = j.find("axis")
        limit = j.find("limit")

        joints[name] = {
            "name": name,
            "type": j.get("type"),
            "parent": parent.get("link") if parent is not None else None,
            "child": child.get("link") if child is not None else None,
            "xyz": parse_vec(origin.get("xyz") if origin is not None else None),
            "rpy": parse_vec(origin.get("rpy") if origin is not None else None),
            "axis": parse_vec(axis.get("xyz") if axis is not None else None),
            "lower": float(limit.get("lower")) if limit is not None and limit.get("lower") is not None else None,
            "upper": float(limit.get("upper")) if limit is not None and limit.get("upper") is not None else None,
        }

    # Follow the exact chain from the 7 arm joints to TCP.
    chain = []
    current_parent = "fr3_link0"

    for joint_name in JOINT_NAMES:
        if joint_name not in joints:
            raise RuntimeError(f"URDF joint not found: {joint_name}")

        j = joints[joint_name]

        if j["parent"] != current_parent:
            raise RuntimeError(
                f"Unexpected URDF chain at {joint_name}: "
                f"expected parent {current_parent}, got {j['parent']}"
            )

        chain.append(j)
        current_parent = j["child"]

    # Continue only along the explicit TCP fixed chain.
    #
    # fr3_hand has additional fixed finger branches:
    #   fr3_hand_tcp_joint
    #   fr3_finger_joint1
    #   fr3_finger_joint2
    #
    # For E-7-12-C we must follow the TCP branch only and must not
    # attempt to uniquely select among all children of fr3_hand.
    tcp_fixed_chain = [
        "fr3_joint8",
        "fr3_hand_joint",
        "fr3_hand_tcp_joint",
    ]

    for joint_name in tcp_fixed_chain:
        if joint_name not in joints:
            raise RuntimeError(
                f"Required TCP chain joint not found: {joint_name}"
            )

        j = joints[joint_name]

        if j["type"] != "fixed":
            raise RuntimeError(
                f"Expected fixed TCP-chain joint {joint_name}, "
                f"got type={j['type']}"
            )

        if j["parent"] != current_parent:
            raise RuntimeError(
                f"Unexpected TCP chain at {joint_name}: "
                f"expected parent {current_parent}, got {j['parent']}"
            )

        chain.append(j)
        current_parent = j["child"]

    if current_parent != TCP_LINK:
        raise RuntimeError(
            f"TCP chain ended at {current_parent}, expected {TCP_LINK}"
        )

    return root, joints, chain


def urdf_fk(q):
    """
    Independent URDF FK.

    Starts at the URDF root link frame (fr3_link0) and follows
    the complete chain through fr3_hand_tcp.
    """
    T = np.eye(4)

    q_index = {
        name: float(q[i])
        for i, name in enumerate(JOINT_NAMES)
    }

    for j in URDF_CHAIN:
        # URDF joint origin first.
        T = T @ homogeneous(
            rpy_matrix(j["rpy"]),
            j["xyz"]
        )

        if j["type"] == "revolute":
            Rj = axis_angle_matrix(j["axis"], q_index[j["name"]])
            T = T @ homogeneous(Rj, np.zeros(3))

    return T


def load_mjcf():
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    joint_ids = {}
    joint_ranges = {}

    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name
        )

        if jid < 0:
            raise RuntimeError(f"MJCF joint not found: {name}")

        joint_ids[name] = jid

        lo, hi = model.jnt_range[jid]

        joint_ranges[name] = (float(lo), float(hi))

    tcp_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        TCP_LINK
    )

    if tcp_body_id < 0:
        raise RuntimeError(f"MJCF TCP body not found: {TCP_LINK}")

    return model, data, joint_ids, joint_ranges, tcp_body_id


def mujoco_fk(model, data, joint_ids, tcp_body_id, q):
    # Reset all qpos, then assign the seven hinge joint values.
    data.qpos[:] = 0.0

    for i, name in enumerate(JOINT_NAMES):
        jid = joint_ids[name]
        qadr = model.jnt_qposadr[jid]
        data.qpos[qadr] = q[i]

    data.qvel[:] = 0.0

    mujoco.mj_forward(model, data)

    p = data.xpos[tcp_body_id].copy()
    R = data.xmat[tcp_body_id].reshape(3, 3).copy()

    return homogeneous(R, p)


def rotation_error(R_ref, R_test):
    """
    Numerically stable rotation-angle error.

    Uses atan2(sin(theta), cos(theta)) instead of acos(cos(theta)).
    This avoids severe loss of precision when theta is very small.
    """
    R_delta = R_ref.T @ R_test

    c = (np.trace(R_delta) - 1.0) / 2.0
    c = float(np.clip(c, -1.0, 1.0))

    skew = 0.5 * np.array([
        R_delta[2, 1] - R_delta[1, 2],
        R_delta[0, 2] - R_delta[2, 0],
        R_delta[1, 0] - R_delta[0, 1],
    ])

    s = float(np.linalg.norm(skew))

    return abs(math.atan2(s, c))


def generate_cases(intersections):
    """
    Valid-only deterministic cases plus fixed-seed random cases.

    No historical invalid values are injected.
    """
    lows = np.array([x[0] for x in intersections], dtype=float)
    highs = np.array([x[1] for x in intersections], dtype=float)
    mids = (lows + highs) / 2.0

    cases = []

    def add(label, q):
        q = np.asarray(q, dtype=float)

        for i, name in enumerate(JOINT_NAMES):
            lo, hi = intersections[i]
            if q[i] < lo - LIMIT_EPS or q[i] > hi + LIMIT_EPS:
                raise RuntimeError(
                    f"Generated invalid configuration {label}: "
                    f"{name}={q[i]} outside [{lo}, {hi}]"
                )

        cases.append({
            "label": label,
            "q": q.tolist(),
            "source": "deterministic",
        })

    # All-mid.
    add("mid_all", mids)

    # All-low / all-high.
    add("low_all", lows)
    add("high_all", highs)

    # One-joint-at-a-time low/high around midpoint.
    for i, name in enumerate(JOINT_NAMES):
        q = mids.copy()
        q[i] = lows[i]
        add(f"{name}_low", q)

        q = mids.copy()
        q[i] = highs[i]
        add(f"{name}_high", q)

    # Alternating boundaries.
    q = np.array([
        lows[i] if i % 2 == 0 else highs[i]
        for i in range(7)
    ])
    add("alternating_low_high", q)

    q = np.array([
        highs[i] if i % 2 == 0 else lows[i]
        for i in range(7)
    ])
    add("alternating_high_low", q)

    # 25/75 interior deterministic combinations.
    q25 = lows + 0.25 * (highs - lows)
    q75 = lows + 0.75 * (highs - lows)

    add("quarter_all", q25)
    add("three_quarter_all", q75)

    # Fixed-seed random valid configurations.
    rng = random.Random(RANDOM_SEED)

    for i in range(RANDOM_COUNT):
        q = np.array([
            rng.uniform(float(lows[j]), float(highs[j]))
            for j in range(7)
        ])

        add(f"random_{i+1:03d}", q)

        cases[-1]["source"] = "random"

    return cases


def main():
    global URDF_CHAIN

    print("============================================================")
    print("E-7-12-C : FR3 VALID-LIMIT FK SWEEP")
    print("============================================================")

    print(f"MJCF : {MJCF_PATH}")
    print(f"URDF : {URDF_PATH}")
    print(f"TCP  : {TCP_LINK}")
    print()

    sha_before = sha256(MJCF_PATH)

    print("[SHA256 BEFORE]")
    print(sha_before)
    print()

    root, urdf_joints, URDF_CHAIN = load_urdf()

    print("[URDF 7-DOF LIMITS]")
    urdf_limits = []

    for name in JOINT_NAMES:
        j = urdf_joints[name]

        if j["lower"] is None or j["upper"] is None:
            raise RuntimeError(f"Missing URDF joint limits: {name}")

        urdf_limits.append((j["lower"], j["upper"]))

        print(
            f"{name}: "
            f"{j['lower']:.10f} "
            f"{j['upper']:.10f}"
        )

    print()

    model, data, mj_joint_ids, mj_ranges, tcp_body_id = load_mjcf()

    print("[MJCF LIMITS]")
    for name in JOINT_NAMES:
        lo, hi = mj_ranges[name]
        print(
            f"{name}: "
            f"{lo:.10f} "
            f"{hi:.10f}"
        )

    print()

    intersections = []

    print("[VALID INTERSECTION LIMITS]")

    for i, name in enumerate(JOINT_NAMES):
        ulo, uhi = urdf_limits[i]
        mlo, mhi = mj_ranges[name]

        lo = max(ulo, mlo)
        hi = min(uhi, mhi)

        if lo > hi:
            raise RuntimeError(
                f"No valid intersection for {name}: "
                f"URDF=[{ulo},{uhi}] MJCF=[{mlo},{mhi}]"
            )

        intersections.append((lo, hi))

        print(
            f"{name}: "
            f"[{lo:.10f}, {hi:.10f}]"
        )

    print()

    cases = generate_cases(intersections)

    print(f"[CASE COUNT] {len(cases)}")
    print()

    results = []

    max_pos = -1.0
    max_rot = -1.0
    max_pos_case = None
    max_rot_case = None

    failures = []

    for idx, case in enumerate(cases, start=1):
        q = np.asarray(case["q"], dtype=float)

        # Explicit valid-limit check.
        valid = True
        violations = []

        for i, name in enumerate(JOINT_NAMES):
            lo, hi = intersections[i]
            if q[i] < lo - LIMIT_EPS or q[i] > hi + LIMIT_EPS:
                valid = False
                violations.append({
                    "joint": name,
                    "value": float(q[i]),
                    "lower": float(lo),
                    "upper": float(hi),
                })

        if not valid:
            failures.append({
                "case": case["label"],
                "reason": "INVALID_LIMIT",
                "violations": violations,
            })
            continue

        T_urdf = urdf_fk(q)
        T_mj = mujoco_fk(
            model,
            data,
            mj_joint_ids,
            tcp_body_id,
            q
        )

        dp = T_urdf[:3, 3] - T_mj[:3, 3]

        pos_err = float(np.linalg.norm(dp))

        rot_err = float(
            rotation_error(
                T_urdf[:3, :3],
                T_mj[:3, :3]
            )
        )

        passed = (
            pos_err <= POS_TOL
            and rot_err <= ROT_TOL
        )

        result = {
            "index": idx,
            "label": case["label"],
            "source": case["source"],
            "q": [float(x) for x in q],
            "position_error_m": pos_err,
            "rotation_error_rad": rot_err,
            "pass": passed,
        }

        results.append(result)

        if pos_err > max_pos:
            max_pos = pos_err
            max_pos_case = case["label"]

        if rot_err > max_rot:
            max_rot = rot_err
            max_rot_case = case["label"]

        if not passed:
            failures.append(result)

        print(
            f"[{idx:03d}/{len(cases):03d}] "
            f"{case['label']:28s} "
            f"pos={pos_err:.3e} m "
            f"rot={rot_err:.3e} rad "
            f"{'PASS' if passed else 'FAIL'}"
        )

    sha_after = sha256(MJCF_PATH)

    print()
    print("[SHA256 AFTER]")
    print(sha_after)
    print()

    sha_unchanged = (sha_before == sha_after)

    valid_case_count = len(results)
    invalid_case_count = len(cases) - valid_case_count

    fk_pass = (
        valid_case_count == len(cases)
        and len(failures) == 0
        and max_pos <= POS_TOL
        and max_rot <= ROT_TOL
    )

    overall_pass = (
        fk_pass
        and sha_unchanged
    )

    summary = {
        "test": "E-7-12-C",
        "description": "FR3 valid-limit URDF FK vs MuJoCo FK sweep",
        "mjcf": MJCF_PATH,
        "urdf": URDF_PATH,
        "tcp": TCP_LINK,
        "joint_names": JOINT_NAMES,
        "random_seed": RANDOM_SEED,
        "random_count": RANDOM_COUNT,
        "position_tolerance_m": POS_TOL,
        "rotation_tolerance_rad": ROT_TOL,
        "limit_epsilon": LIMIT_EPS,
        "sha256_before": sha_before,
        "sha256_after": sha_after,
        "sha256_unchanged": sha_unchanged,
        "case_count": len(cases),
        "valid_case_count": valid_case_count,
        "invalid_case_count": invalid_case_count,
        "max_position_error_m": max_pos,
        "max_position_error_case": max_pos_case,
        "max_rotation_error_rad": max_rot,
        "max_rotation_error_case": max_rot_case,
        "fk_pass": fk_pass,
        "overall_pass": overall_pass,
        "failures": failures,
        "results": results,
    }

    with open(RESULT_PATH, "w") as f:
        json.dump(summary, f, indent=2)

    print("============================================================")
    print("E-7-12-C SUMMARY")
    print("============================================================")
    print(f"Cases              : {len(cases)}")
    print(f"Valid cases        : {valid_case_count}")
    print(f"Invalid cases      : {invalid_case_count}")
    print(f"Max position error : {max_pos:.12e} m")
    print(f"Max rotation error : {max_rot:.12e} rad")
    print(f"FK comparison      : {'PASS' if fk_pass else 'FAIL'}")
    print(f"SHA256 unchanged   : {'PASS' if sha_unchanged else 'FAIL'}")
    print(f"RESULT             : {'PASS' if overall_pass else 'REVIEW REQUIRED / FAIL'}")
    print()
    print(f"Result JSON: {RESULT_PATH}")
    print()

    if not sha_unchanged:
        print("CRITICAL: output/fr3.xml changed during verification.")
        sys.exit(3)

    if not overall_pass:
        sys.exit(1)


if __name__ == "__main__":
    main()
