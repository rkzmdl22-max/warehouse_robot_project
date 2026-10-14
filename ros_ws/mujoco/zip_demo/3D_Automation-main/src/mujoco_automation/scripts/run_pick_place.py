import time

import mujoco
import mujoco.viewer
import numpy as np

from preview_workcell import build_model, set_initial_state


ARM_DOF = 6
GRIPPER_ACTUATOR = 6

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 255.0

BOTTLE_POSITION = np.array([-0.15, 0.62, 0.325])

PREGRASP_POSITION = BOTTLE_POSITION + np.array([0.0, 0.0, 0.16])
GRASP_POSITION = BOTTLE_POSITION.copy()
LIFT_POSITION = BOTTLE_POSITION + np.array([0.0, 0.0, 0.18])

PLACE_POSITION = np.array([0.20, 0.62, 0.335])
PLACE_HIGH_POSITION = PLACE_POSITION + np.array([0.0, 0.0, 0.17])
RETREAT_POSITION = PLACE_POSITION + np.array([0.0, 0.0, 0.20])


def configure_grasp_physics(model: mujoco.MjModel) -> None:
    gripper_actuator = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "gripper_fingers_actuator",
    )

    if gripper_actuator < 0:
        raise RuntimeError("gripper actuator was not found")

    model.actuator_forcerange[gripper_actuator] = [-15.0, 15.0]

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
            raise RuntimeError(f"gripper pad was not found: {pad_name}")

        model.geom_friction[pad_id] = [2.0, 0.05, 0.01]
        model.geom_solref[pad_id] = [0.006, 1.0]

    bottle_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "water_bottle_collision",
    )

    if bottle_id < 0:
        raise RuntimeError("water bottle geometry was not found")

    model.geom_friction[bottle_id] = [1.5, 0.03, 0.005]

    print("Grasp physics configured")


def rotation_error(
    target_rotation: np.ndarray,
    current_rotation: np.ndarray,
) -> np.ndarray:
    error_matrix = target_rotation @ current_rotation.T

    return 0.5 * np.array(
        [
            error_matrix[2, 1] - error_matrix[1, 2],
            error_matrix[0, 2] - error_matrix[2, 0],
            error_matrix[1, 0] - error_matrix[0, 1],
        ]
    )


def solve_ik(
    model: mujoco.MjModel,
    initial_qpos: np.ndarray,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    site_id: int,
) -> np.ndarray:
    ik_data = mujoco.MjData(model)
    ik_data.qpos[:] = model.qpos0
    ik_data.qpos[:ARM_DOF] = initial_qpos

    damping = 0.02
    position_gain = 0.7
    rotation_gain = 0.35

    jacobian_position = np.zeros((3, model.nv))
    jacobian_rotation = np.zeros((3, model.nv))

    for _ in range(600):
        mujoco.mj_forward(model, ik_data)

        position_delta = target_position - ik_data.site_xpos[site_id]
        current_rotation = ik_data.site_xmat[site_id].reshape(3, 3)
        orientation_delta = rotation_error(
            target_rotation,
            current_rotation,
        )

        if (
            np.linalg.norm(position_delta) < 0.001
            and np.linalg.norm(orientation_delta) < 0.01
        ):
            break

        jacobian_position.fill(0.0)
        jacobian_rotation.fill(0.0)

        mujoco.mj_jacSite(
            model,
            ik_data,
            jacobian_position,
            jacobian_rotation,
            site_id,
        )

        jacobian = np.vstack(
            [
                position_gain * jacobian_position[:, :ARM_DOF],
                rotation_gain * jacobian_rotation[:, :ARM_DOF],
            ]
        )

        error = np.concatenate(
            [
                position_gain * position_delta,
                rotation_gain * orientation_delta,
            ]
        )

        system = (
            jacobian @ jacobian.T
            + damping * damping * np.eye(6)
        )

        joint_delta = jacobian.T @ np.linalg.solve(system, error)

        max_step = 0.08
        joint_delta = np.clip(joint_delta, -max_step, max_step)

        ik_data.qpos[:ARM_DOF] += joint_delta

        joint_ranges = model.jnt_range[:ARM_DOF]
        ik_data.qpos[:ARM_DOF] = np.clip(
            ik_data.qpos[:ARM_DOF],
            joint_ranges[:, 0],
            joint_ranges[:, 1],
        )

    mujoco.mj_forward(model, ik_data)

    final_error = np.linalg.norm(
        target_position - ik_data.site_xpos[site_id]
    )

    if final_error > 0.015:
        raise RuntimeError(
            f"IK failed: target={target_position}, "
            f"position error={final_error:.4f} m"
        )

    print(
        f"IK solved: target={target_position}, "
        f"error={final_error:.4f} m"
    )

    return ik_data.qpos[:ARM_DOF].copy()


def move_to_target(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    arm_target: np.ndarray,
    gripper_target: float,
    duration: float,
    label: str,
) -> bool:
    print(f"STATE: {label}")

    start_time = data.time
    end_time = start_time + duration

    start_arm = data.ctrl[:ARM_DOF].copy()
    start_gripper = float(data.ctrl[GRIPPER_ACTUATOR])

    while viewer.is_running() and data.time < end_time:
        phase = (data.time - start_time) / duration
        phase = np.clip(phase, 0.0, 1.0)

        smooth_phase = phase * phase * (3.0 - 2.0 * phase)

        data.ctrl[:ARM_DOF] = (
            start_arm
            + smooth_phase * (arm_target - start_arm)
        )

        data.ctrl[GRIPPER_ACTUATOR] = (
            start_gripper
            + smooth_phase * (gripper_target - start_gripper)
        )

        step_started = time.perf_counter()

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.perf_counter() - step_started
        remaining = model.opt.timestep - elapsed

        if remaining > 0.0:
            time.sleep(remaining)

    return viewer.is_running()


def main() -> None:
    model = build_model()
    configure_grasp_physics(model)
    data = mujoco.MjData(model)

    set_initial_state(model, data)

    pinch_site = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        "gripper_pinch",
    )

    if pinch_site < 0:
        raise RuntimeError("gripper_pinch site was not found")

    home_qpos = data.qpos[:ARM_DOF].copy()
    home_rotation = data.site_xmat[pinch_site].reshape(3, 3).copy()

    pregrasp_qpos = solve_ik(
        model,
        home_qpos,
        PREGRASP_POSITION,
        home_rotation,
        pinch_site,
    )

    grasp_qpos = solve_ik(
        model,
        pregrasp_qpos,
        GRASP_POSITION,
        home_rotation,
        pinch_site,
    )

    lift_qpos = solve_ik(
        model,
        grasp_qpos,
        LIFT_POSITION,
        home_rotation,
        pinch_site,
    )

    place_high_qpos = solve_ik(
        model,
        lift_qpos,
        PLACE_HIGH_POSITION,
        home_rotation,
        pinch_site,
    )

    place_qpos = solve_ik(
        model,
        place_high_qpos,
        PLACE_POSITION,
        home_rotation,
        pinch_site,
    )

    retreat_qpos = solve_ik(
        model,
        place_qpos,
        RETREAT_POSITION,
        home_rotation,
        pinch_site,
    )

    data.ctrl[:ARM_DOF] = home_qpos
    data.ctrl[GRIPPER_ACTUATOR] = GRIPPER_OPEN

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
    ) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.55, 0.30]
        viewer.cam.distance = 1.6
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -25.0

        sequence = [
            (
                home_qpos,
                GRIPPER_OPEN,
                1.5,
                "IDLE",
            ),
            (
                pregrasp_qpos,
                GRIPPER_OPEN,
                2.5,
                "PREGRASP",
            ),
            (
                grasp_qpos,
                GRIPPER_OPEN,
                2.0,
                "APPROACH",
            ),
            (
                grasp_qpos,
                GRIPPER_CLOSED,
                5.0,
                "GRASP",
            ),
            (
                grasp_qpos,
                GRIPPER_CLOSED,
                1.5,
                "GRASP_HOLD",
            ),
            (
                lift_qpos,
                GRIPPER_CLOSED,
                4.0,
                "LIFT",
            ),
            (
                place_high_qpos,
                GRIPPER_CLOSED,
                3.0,
                "TRANSFER",
            ),
            (
                place_qpos,
                GRIPPER_CLOSED,
                2.0,
                "PLACE",
            ),
            (
                place_qpos,
                GRIPPER_OPEN,
                2.0,
                "RELEASE",
            ),
            (
                retreat_qpos,
                GRIPPER_OPEN,
                2.0,
                "RETREAT",
            ),
        ]

        for arm_target, gripper_target, duration, label in sequence:
            if not move_to_target(
                model,
                data,
                viewer,
                arm_target,
                gripper_target,
                duration,
                label,
            ):
                return

        print("STATE: COMPLETE")

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()