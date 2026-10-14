#!/usr/bin/env python3

import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer


PROJECT_ROOT = Path(__file__).resolve().parents[3]
TRON1_XML = (
    PROJECT_ROOT
    / "assets"
    / "vendor"
    / "objects_3da_v0"
    / "PF_TRON1A"
    / "xml"
    / "robot.xml"
)

UPDATE_RATE = 60.0

START_X = -1.2
TARGET_X = 0.0
TRANSPORT_DURATION = 6.0

STEP_FREQUENCY = 1.2
HIP_SWING = 0.24
KNEE_LIFT = 0.32
BODY_BOB = 0.012

ARRIVAL_HOLD_DURATION = 2.0
CYCLE_DURATION = TRANSPORT_DURATION + ARRIVAL_HOLD_DURATION


def build_model() -> mujoco.MjModel:
    robot_spec = mujoco.MjSpec.from_file(str(TRON1_XML))
    base_body = robot_spec.body("base_Link")

    if base_body is None:
        raise RuntimeError("TRON1 body 'base_Link' was not found")

    base_body.add_geom(
        name="transport_tray",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.0, 0.0, 0.16],
        size=[0.22, 0.18, 0.02],
        rgba=[0.10, 0.30, 0.80, 1.0],
        contype=0,
        conaffinity=0,
    )

    base_body.add_geom(
        name="transport_bottle",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0.0, 0.0, 0.30],
        size=[0.03, 0.12, 0.0],
        rgba=[0.00, 0.55, 1.00, 1.0],
        contype=0,
        conaffinity=0,
    )

    return robot_spec.compile()


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def set_transport_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    x_position: float,
    elapsed: float,
    progress: float,
) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    walk_blend = max(
        0.0,
        min(1.0, (1.0 - progress) / 0.15),
    )

    phase = (
        2.0
        * math.pi
        * STEP_FREQUENCY
        * elapsed
    )

    step_wave = math.sin(phase)

    # 좌우 hip 축이 모델에서 반대로 정의되어 있으므로
    # 같은 부호를 사용해야 화면에서는 반대 방향으로 움직입니다.
    left_hip = HIP_SWING * step_wave * walk_blend
    right_hip = HIP_SWING * step_wave * walk_blend

    # 앞으로 나가는 다리의 무릎만 굽힙니다.
    left_knee = (
        KNEE_LIFT
        * max(0.0, step_wave)
        * walk_blend
    )

    right_knee = (
        KNEE_LIFT
        * max(0.0, -step_wave)
        * walk_blend
    )

    body_height = (
        0.82
        + BODY_BOB
        * (0.5 - 0.5 * math.cos(phase * 2.0))
        * walk_blend
    )

    data.qpos[0:3] = [
        x_position,
        0.0,
        body_height,
    ]

    # Free-joint quaternion: w, x, y, z
    data.qpos[3:7] = [
        1.0,
        0.0,
        0.0,
        0.0,
    ]

    # 관절 순서:
    # abad_L, hip_L, knee_L,
    # abad_R, hip_R, knee_R
    data.qpos[7:13] = [
        0.0,
        left_hip,
        left_knee,
        0.0,
        right_hip,
        right_knee,
    ]

    mujoco.mj_forward(model, data)


def main() -> None:
    model = build_model()
    data = mujoco.MjData(model)

    print(
        f"TRON1 transport model loaded: "
        f"nq={model.nq}, nv={model.nv}, "
        f"nu={model.nu}, nbody={model.nbody}"
    )

    start_time = time.monotonic()
    current_cycle = -1
    arrival_reported = False

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
    ) as viewer:
        while viewer.is_running():
            total_elapsed = time.monotonic() - start_time

            cycle_number = int(
                total_elapsed // CYCLE_DURATION
            )

            elapsed = (
                total_elapsed % CYCLE_DURATION
            )

            if cycle_number != current_cycle:
                current_cycle = cycle_number
                arrival_reported = False
                print(
                    f"STATE: TRANSPORT_START "
                    f"cycle={cycle_number + 1}"
                )

            movement_elapsed = min(
                elapsed,
                TRANSPORT_DURATION,
            )

            progress = smoothstep(
                movement_elapsed / TRANSPORT_DURATION
            )

            x_position = (
                START_X
                + (TARGET_X - START_X) * progress
            )

            set_transport_pose(
                model=model,
                data=data,
                x_position=x_position,
                elapsed=movement_elapsed,
                progress=progress,
            )

            if progress >= 1.0 and not arrival_reported:
                print(
                    f"STATE: HANDOVER_READY "
                    f"cycle={cycle_number + 1}"
                )
                arrival_reported = True

            viewer.sync()
            time.sleep(1.0 / UPDATE_RATE)


if __name__ == "__main__":
    main()