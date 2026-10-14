#!/usr/bin/env python3

import math
import time

import mujoco
import mujoco.viewer
import numpy as np

from preview_handover_cell import (
    BOTTLE_TRAY_OFFSETS,
    TRON1_BOTTLE_POSITION,
    TRON1_POSITION,
    build_model,
    find_tron1_free_joint,
    set_initial_state,
)
from run_pick_place import solve_ik


ARM_DOF = 6
GRIPPER_ACTUATOR = 6

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 255.0

TRON1_START_X = -2.5
TRON1_TARGET_X = TRON1_POSITION[0]
TRANSPORT_DURATION = 12.0
ARRIVAL_HOLD_DURATION = 1.5

STEP_FREQUENCY = 1.2
HIP_SWING = 0.24
KNEE_LIFT = 0.32
BODY_BOB = 0.012

VIEWER_FPS = 20.0

BOTTLE_POSITION = np.array(
    TRON1_BOTTLE_POSITION,
    dtype=float,
)

BOTTLE_PICK_POSITIONS = [
    BOTTLE_POSITION + offset
    for offset in BOTTLE_TRAY_OFFSETS
]

BOTTLE_PLACE_POSITIONS = [
    np.array([0.50, -0.38, 0.865]),
    np.array([0.50, -0.25, 0.865]),
    np.array([0.50, -0.12, 0.865]),
]

PREGRASP_POSITION = (
    BOTTLE_POSITION
    + np.array([0.0, 0.0, 0.16])
)

GRASP_POSITION = (
    BOTTLE_POSITION
    + np.array([0.0, 0.0, 0.01])
)

LIFT_POSITION = (
    BOTTLE_POSITION
    + np.array([0.0, 0.0, 0.18])
)

PLACE_POSITION = np.array(
    [0.50, -0.25, 0.875],
)

PLACE_HIGH_POSITION = np.array(
    [0.50, -0.25, 1.05],
)

RETREAT_POSITION = np.array(
    [0.50, -0.25, 1.08],
)


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def configure_grasp_physics(
    model: mujoco.MjModel,
) -> None:
    gripper_actuator = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "gripper_fingers_actuator",
    )

    if gripper_actuator < 0:
        raise RuntimeError(
            "gripper actuator was not found"
        )

    if gripper_actuator != GRIPPER_ACTUATOR:
        raise RuntimeError(
            f"unexpected gripper actuator index: "
            f"{gripper_actuator}"
        )

    model.actuator_forcerange[
        gripper_actuator
    ] = [-15.0, 15.0]

    pad_names = [
        "gripper_right_pad1",
        "gripper_right_pad2",
        "gripper_left_pad1",
        "gripper_left_pad2",
    ]

    for pad_name in pad_names:
        pad_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            pad_name,
        )

        if pad_id < 0:
            raise RuntimeError(
                f"gripper pad was not found: "
                f"{pad_name}"
            )

        model.geom_friction[pad_id] = [
            2.0,
            0.05,
            0.01,
        ]

        model.geom_solref[pad_id] = [
            0.006,
            1.0,
        ]

    for bottle_number in range(1, 4):
        bottle_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            (
                f"transport_bottle_{bottle_number}"
                "_collision"
            ),
        )

        if bottle_id < 0:
            raise RuntimeError(
                "transport bottle geometry was "
                f"not found: {bottle_number}"
            )

        model.geom_friction[bottle_id] = [
            1.5,
            0.03,
            0.005,
        ]

    print("Grasp physics configured")


def build_kinematic_context(
    model: mujoco.MjModel,
) -> dict:
    tron1_free_joint = find_tron1_free_joint(
        model
    )

    bottle_free_joints = []

    for bottle_number in range(1, 4):
        bottle_free_joint = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            (
                f"transport_bottle_{bottle_number}"
                "_free_joint"
            ),
        )

        if bottle_free_joint < 0:
            raise RuntimeError(
                "transport bottle free joint was "
                f"not found: {bottle_number}"
            )

        bottle_free_joints.append(
            bottle_free_joint
        )

    leg_joint_names = [
        "tron1_abad_L_Joint",
        "tron1_hip_L_Joint",
        "tron1_knee_L_Joint",
        "tron1_abad_R_Joint",
        "tron1_hip_R_Joint",
        "tron1_knee_R_Joint",
    ]

    leg_joint_ids = []

    for joint_name in leg_joint_names:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )

        if joint_id < 0:
            raise RuntimeError(
                f"TRON1 joint was not found: "
                f"{joint_name}"
            )

        leg_joint_ids.append(joint_id)

    return {
        "tron1_qpos_address":
            model.jnt_qposadr[
                tron1_free_joint
            ],
        "tron1_qvel_address":
            model.jnt_dofadr[
                tron1_free_joint
            ],
        "bottle_qpos_addresses": [
            model.jnt_qposadr[joint_id]
            for joint_id in bottle_free_joints
        ],
        "bottle_qvel_addresses": [
            model.jnt_dofadr[joint_id]
            for joint_id in bottle_free_joints
        ],
        "leg_joint_ids":
            leg_joint_ids,
    }


def set_tron1_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    context: dict,
    x_position: float,
    elapsed: float = 0.0,
    walking: bool = False,
    progress: float = 1.0,
) -> float:
    if walking:
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

        left_hip = (
            HIP_SWING
            * step_wave
            * walk_blend
        )

        right_hip = (
            HIP_SWING
            * step_wave
            * walk_blend
        )

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

        base_height = (
            TRON1_POSITION[2]
            + BODY_BOB
            * (
                0.5
                - 0.5
                * math.cos(phase * 2.0)
            )
            * walk_blend
        )
    else:
        left_hip = 0.0
        right_hip = 0.0
        left_knee = 0.0
        right_knee = 0.0
        base_height = TRON1_POSITION[2]

    tron1_qpos_address = context[
        "tron1_qpos_address"
    ]

    tron1_qvel_address = context[
        "tron1_qvel_address"
    ]

    data.qpos[
        tron1_qpos_address:
        tron1_qpos_address + 7
    ] = [
        x_position,
        TRON1_POSITION[1],
        base_height,
        1.0,
        0.0,
        0.0,
        0.0,
    ]

    data.qvel[
        tron1_qvel_address:
        tron1_qvel_address + 6
    ] = 0.0

    leg_values = [
        0.0,
        left_hip,
        left_knee,
        0.0,
        right_hip,
        right_knee,
    ]

    for joint_id, joint_value in zip(
        context["leg_joint_ids"],
        leg_values,
    ):
        qpos_address = model.jnt_qposadr[
            joint_id
        ]

        qvel_address = model.jnt_dofadr[
            joint_id
        ]

        data.qpos[qpos_address] = joint_value
        data.qvel[qvel_address] = 0.0

    return base_height


def set_bottle_on_tray(
    data: mujoco.MjData,
    context: dict,
    bottle_index: int,
    x_position: float,
    base_height: float,
) -> None:
    bottle_qpos_address = context[
        "bottle_qpos_addresses"
    ][bottle_index]

    bottle_qvel_address = context[
        "bottle_qvel_addresses"
    ][bottle_index]

    offset = BOTTLE_TRAY_OFFSETS[
        bottle_index
    ]

    data.qpos[
        bottle_qpos_address:
        bottle_qpos_address + 7
    ] = [
        x_position + offset[0],
        TRON1_POSITION[1] + offset[1],
        base_height + 0.30 + offset[2],
        1.0,
        0.0,
        0.0,
        0.0,
    ]

    data.qvel[
        bottle_qvel_address:
        bottle_qvel_address + 6
    ] = 0.0


def set_bottle_at_gripper(
    data: mujoco.MjData,
    context: dict,
    bottle_index: int,
    pinch_site: int,
) -> None:
    bottle_qpos_address = context[
        "bottle_qpos_addresses"
    ][bottle_index]

    bottle_qvel_address = context[
        "bottle_qvel_addresses"
    ][bottle_index]

    bottle_position = (
        data.site_xpos[pinch_site]
        + np.array([0.0, 0.0, -0.07])
    )

    data.qpos[
        bottle_qpos_address:
        bottle_qpos_address + 3
    ] = bottle_position

    data.qpos[
        bottle_qpos_address + 3:
        bottle_qpos_address + 7
    ] = [
        1.0,
        0.0,
        0.0,
        0.0,
    ]

    data.qvel[
        bottle_qvel_address:
        bottle_qvel_address + 6
    ] = 0.0


def enforce_handover_scene(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    context: dict,
    bottle_index: int,
    hold_bottle: bool,
    attach_bottle: bool = False,
    pinch_site: int = -1,
) -> None:
    base_height = set_tron1_pose(
        model=model,
        data=data,
        context=context,
        x_position=TRON1_TARGET_X,
    )

    for waiting_index in range(
        bottle_index + 1,
        len(BOTTLE_TRAY_OFFSETS),
    ):
        set_bottle_on_tray(
            data=data,
            context=context,
            bottle_index=waiting_index,
            x_position=TRON1_TARGET_X,
            base_height=base_height,
        )

    if hold_bottle:
        set_bottle_on_tray(
            data=data,
            context=context,
            bottle_index=bottle_index,
            x_position=TRON1_TARGET_X,
            base_height=base_height,
        )
    elif attach_bottle:
        if pinch_site < 0:
            raise RuntimeError(
                "pinch site is required while "
                "the bottle is attached"
            )

        set_bottle_at_gripper(
            data=data,
            context=context,
            bottle_index=bottle_index,
            pinch_site=pinch_site,
        )


def sync_viewer_if_due(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
) -> None:
    steps_per_frame = max(
        1,
        int(
            round(
                1.0
                / (
                    VIEWER_FPS
                    * model.opt.timestep
                )
            )
        ),
    )

    step_number = int(
        round(
            data.time
            / model.opt.timestep
        )
    )

    if step_number % steps_per_frame == 0:
        viewer.sync()


def step_and_sync(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    context: dict,
    bottle_index: int,
    hold_bottle: bool,
    attach_bottle: bool = False,
    pinch_site: int = -1,
) -> None:
    enforce_handover_scene(
        model=model,
        data=data,
        context=context,
        bottle_index=bottle_index,
        hold_bottle=hold_bottle,
        attach_bottle=attach_bottle,
        pinch_site=pinch_site,
    )

    step_started = time.perf_counter()

    mujoco.mj_step(model, data)

    enforce_handover_scene(
        model=model,
        data=data,
        context=context,
        bottle_index=bottle_index,
        hold_bottle=hold_bottle,
        attach_bottle=attach_bottle,
        pinch_site=pinch_site,
    )

    mujoco.mj_forward(model, data)
    sync_viewer_if_due(
        model=model,
        data=data,
        viewer=viewer,
    )

    elapsed = time.perf_counter() - step_started
    remaining = model.opt.timestep - elapsed

    if remaining > 0.0:
        time.sleep(remaining)


def run_tron1_transport(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    context: dict,
) -> bool:
    print("STATE: TRANSPORT_START")

    start_time = data.time
    end_time = start_time + TRANSPORT_DURATION

    while (
        viewer.is_running()
        and data.time < end_time
    ):
        elapsed = data.time - start_time

        progress = smoothstep(
            elapsed / TRANSPORT_DURATION
        )

        x_position = (
            TRON1_START_X
            + (
                TRON1_TARGET_X
                - TRON1_START_X
            )
            * progress
        )

        base_height = set_tron1_pose(
            model=model,
            data=data,
            context=context,
            x_position=x_position,
            elapsed=elapsed,
            walking=True,
            progress=progress,
        )

        for bottle_index in range(3):
            set_bottle_on_tray(
                data=data,
                context=context,
                bottle_index=bottle_index,
                x_position=x_position,
                base_height=base_height,
            )

        step_started = time.perf_counter()

        mujoco.mj_step(model, data)

        base_height = set_tron1_pose(
            model=model,
            data=data,
            context=context,
            x_position=x_position,
            elapsed=elapsed,
            walking=True,
            progress=progress,
        )

        for bottle_index in range(3):
            set_bottle_on_tray(
                data=data,
                context=context,
                bottle_index=bottle_index,
                x_position=x_position,
                base_height=base_height,
            )

        mujoco.mj_forward(model, data)
        sync_viewer_if_due(
            model=model,
            data=data,
            viewer=viewer,
        )

        step_elapsed = (
            time.perf_counter()
            - step_started
        )

        remaining = (
            model.opt.timestep
            - step_elapsed
        )

        if remaining > 0.0:
            time.sleep(remaining)

    if not viewer.is_running():
        return False

    print("STATE: HANDOVER_READY")

    hold_start_time = data.time
    hold_end_time = (
        hold_start_time
        + ARRIVAL_HOLD_DURATION
    )

    while (
        viewer.is_running()
        and data.time < hold_end_time
    ):
        step_and_sync(
            model=model,
            data=data,
            viewer=viewer,
            context=context,
            bottle_index=0,
            hold_bottle=True,
        )

    return viewer.is_running()


def move_to_target(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    context: dict,
    bottle_index: int,
    arm_target: np.ndarray,
    gripper_target: float,
    duration: float,
    label: str,
    hold_bottle: bool,
    attach_bottle: bool,
    pinch_site: int,
) -> bool:
    print(f"STATE: {label}")

    start_time = data.time
    end_time = start_time + duration

    start_arm = data.ctrl[
        :ARM_DOF
    ].copy()

    start_gripper = float(
        data.ctrl[GRIPPER_ACTUATOR]
    )

    while (
        viewer.is_running()
        and data.time < end_time
    ):
        phase = (
            data.time - start_time
        ) / duration

        phase = np.clip(
            phase,
            0.0,
            1.0,
        )

        smooth_phase = (
            phase
            * phase
            * (3.0 - 2.0 * phase)
        )

        data.ctrl[:ARM_DOF] = (
            start_arm
            + smooth_phase
            * (arm_target - start_arm)
        )

        data.ctrl[GRIPPER_ACTUATOR] = (
            start_gripper
            + smooth_phase
            * (
                gripper_target
                - start_gripper
            )
        )

        step_and_sync(
            model=model,
            data=data,
            viewer=viewer,
            context=context,
            bottle_index=bottle_index,
            hold_bottle=hold_bottle,
            attach_bottle=attach_bottle,
            pinch_site=pinch_site,
        )

    return viewer.is_running()


def main() -> None:
    model = build_model()
    configure_grasp_physics(model)

    data = mujoco.MjData(model)
    set_initial_state(model, data)

    context = build_kinematic_context(
        model
    )

    pinch_site = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        "gripper_pinch",
    )

    if pinch_site < 0:
        raise RuntimeError(
            "gripper_pinch site was not found"
        )

    safe_qpos = data.qpos[
        :ARM_DOF
    ].copy()

    ik_seed = safe_qpos.copy()
    ik_seed[0] = -1.5708

    orientation_data = mujoco.MjData(model)
    orientation_data.qpos[:] = data.qpos
    orientation_data.qpos[
        :ARM_DOF
    ] = ik_seed

    mujoco.mj_forward(
        model,
        orientation_data,
    )

    grasp_rotation = (
        orientation_data.site_xmat[
            pinch_site
        ]
        .reshape(3, 3)
        .copy()
    )

    bottle_paths = []
    path_seed = ik_seed

    for bottle_index, pickup_position in enumerate(
        BOTTLE_PICK_POSITIONS
    ):
        place_position = BOTTLE_PLACE_POSITIONS[
            bottle_index
        ]

        pregrasp_qpos = solve_ik(
            model,
            path_seed,
            pickup_position
            + np.array([0.0, 0.0, 0.16]),
            grasp_rotation,
            pinch_site,
        )

        grasp_qpos = solve_ik(
            model,
            pregrasp_qpos,
            pickup_position
            + np.array([0.0, 0.0, 0.07]),
            grasp_rotation,
            pinch_site,
        )

        lift_qpos = solve_ik(
            model,
            grasp_qpos,
            pickup_position
            + np.array([0.0, 0.0, 0.18]),
            grasp_rotation,
            pinch_site,
        )

        place_high_qpos = solve_ik(
            model,
            lift_qpos,
            place_position
            + np.array([0.0, 0.0, 0.30]),
            grasp_rotation,
            pinch_site,
        )

        place_qpos = solve_ik(
            model,
            place_high_qpos,
            place_position
            + np.array([0.0, 0.0, 0.07]),
            grasp_rotation,
            pinch_site,
        )

        retreat_qpos = solve_ik(
            model,
            place_qpos,
            place_position
            + np.array([0.0, 0.0, 0.30]),
            grasp_rotation,
            pinch_site,
        )

        bottle_paths.append(
            (
                pregrasp_qpos,
                grasp_qpos,
                lift_qpos,
                place_high_qpos,
                place_qpos,
                retreat_qpos,
            )
        )

        path_seed = retreat_qpos

    data.ctrl[:ARM_DOF] = safe_qpos
    data.ctrl[GRIPPER_ACTUATOR] = GRIPPER_OPEN

    base_height = set_tron1_pose(
        model=model,
        data=data,
        context=context,
        x_position=TRON1_START_X,
    )

    for bottle_index in range(3):
        set_bottle_on_tray(
            data=data,
            context=context,
            bottle_index=bottle_index,
            x_position=TRON1_START_X,
            base_height=base_height,
        )

    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
    ) as viewer:
        viewer.cam.lookat[:] = [
            0.10,
            0.15,
            0.75,
        ]
        viewer.cam.distance = 2.3
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -22.0

        if not run_tron1_transport(
            model=model,
            data=data,
            viewer=viewer,
            context=context,
        ):
            return

        if not move_to_target(
            model=model,
            data=data,
            viewer=viewer,
            context=context,
            bottle_index=0,
            arm_target=safe_qpos,
            gripper_target=GRIPPER_OPEN,
            duration=1.5,
            label="IDLE",
            hold_bottle=True,
            attach_bottle=False,
            pinch_site=pinch_site,
        ):
            return

        for bottle_index, path in enumerate(
            bottle_paths
        ):
            (
                pregrasp_qpos,
                grasp_qpos,
                lift_qpos,
                place_high_qpos,
                place_qpos,
                retreat_qpos,
            ) = path

            bottle_number = bottle_index + 1
            print(
                f"STATE: BOTTLE_{bottle_number}_START"
            )

            sequence = [
                (
                    pregrasp_qpos,
                    GRIPPER_OPEN,
                    3.0,
                    "PREGRASP",
                    True,
                    False,
                ),
                (
                    grasp_qpos,
                    GRIPPER_OPEN,
                    2.0,
                    "APPROACH",
                    True,
                    False,
                ),
                (
                    grasp_qpos,
                    GRIPPER_CLOSED,
                    5.0,
                    "GRASP",
                    True,
                    False,
                ),
                (
                    grasp_qpos,
                    GRIPPER_CLOSED,
                    1.5,
                    "GRASP_HOLD",
                    True,
                    False,
                ),
                (
                    lift_qpos,
                    GRIPPER_CLOSED,
                    4.0,
                    "LIFT",
                    False,
                    True,
                ),
                (
                    place_high_qpos,
                    GRIPPER_CLOSED,
                    3.0,
                    "TRANSFER",
                    False,
                    True,
                ),
                (
                    place_qpos,
                    GRIPPER_CLOSED,
                    2.0,
                    "PLACE",
                    False,
                    True,
                ),
                (
                    place_qpos,
                    GRIPPER_OPEN,
                    2.0,
                    "RELEASE",
                    False,
                    True,
                ),
                (
                    place_qpos,
                    GRIPPER_OPEN,
                    1.5,
                    "OPEN_HOLD",
                    False,
                    True,
                ),
                (
                    place_qpos,
                    GRIPPER_OPEN,
                    1.5,
                    "SETTLE",
                    False,
                    False,
                ),
                (
                    retreat_qpos,
                    GRIPPER_OPEN,
                    2.0,
                    "RETREAT",
                    False,
                    False,
                ),
            ]

            for (
                arm_target,
                gripper_target,
                duration,
                label,
                hold_bottle,
                attach_bottle,
            ) in sequence:
                if not move_to_target(
                    model=model,
                    data=data,
                    viewer=viewer,
                    context=context,
                    bottle_index=bottle_index,
                    arm_target=arm_target,
                    gripper_target=gripper_target,
                    duration=duration,
                    label=(
                        f"BOTTLE_{bottle_number}_{label}"
                    ),
                    hold_bottle=hold_bottle,
                    attach_bottle=attach_bottle,
                    pinch_site=pinch_site,
                ):
                    return

            print(
                f"STATE: BOTTLE_{bottle_number}_COMPLETE"
            )

        print("STATE: ALL_COMPLETE")

        while viewer.is_running():
            step_and_sync(
                model=model,
                data=data,
                viewer=viewer,
                context=context,
                bottle_index=2,
                hold_bottle=False,
                attach_bottle=False,
                pinch_site=pinch_site,
            )


if __name__ == "__main__":
    main()
