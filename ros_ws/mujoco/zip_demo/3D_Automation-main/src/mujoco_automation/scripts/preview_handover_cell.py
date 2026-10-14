#!/usr/bin/env python3

import numpy as np
import time
from pathlib import Path

import mujoco
import mujoco.viewer


PROJECT_ROOT = Path(__file__).resolve().parents[3]

MENAGERIE_ROOT = (
    PROJECT_ROOT
    / "src"
    / "vendor"
    / "mujoco_menagerie"
)

UR5E_SCENE = (
    MENAGERIE_ROOT
    / "universal_robots_ur5e"
    / "scene.xml"
)

GRIPPER_MODEL = (
    MENAGERIE_ROOT
    / "robotiq_2f85"
    / "2f85.xml"
)

TRON1_XML = (
    PROJECT_ROOT
    / "assets"
    / "vendor"
    / "objects_3da_v0"
    / "PF_TRON1A"
    / "xml"
    / "robot.xml"
)

UR5E_BASE_HEIGHT = 0.70

TRON1_POSITION = [
    0.0,
    0.55,
    0.82,
]

TRON1_BOTTLE_POSITION = [
    0.0,
    0.55,
    1.12,
]

BOTTLE_TRAY_OFFSETS = [
    np.array([0.15, 0.0, 0.0]),
    np.array([0.00, 0.0, 0.0]),
    np.array([-0.15, 0.0, 0.0]),
]


def add_ur5e_pedestal(spec: mujoco.MjSpec) -> None:
    spec.worldbody.add_geom(
        name="ur5e_pedestal",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.0, 0.0, UR5E_BASE_HEIGHT / 2.0],
        size=[0.16, 0.16, UR5E_BASE_HEIGHT / 2.0],
        rgba=[0.25, 0.28, 0.32, 1.0],
    )


def add_receiving_table(spec: mujoco.MjSpec) -> None:
    table = spec.worldbody.add_body(
        name="receiving_table",
    )

    table.add_geom(
        name="receiving_tabletop",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.50, -0.25, 0.72],
        size=[0.22, 0.22, 0.025],
        rgba=[0.12, 0.28, 0.55, 1.0],
        friction=[1.0, 0.01, 0.001],
    )

    leg_positions = [
        [0.32, -0.43, 0.3475],
        [0.68, -0.43, 0.3475],
        [0.32, -0.07, 0.3475],
        [0.68, -0.07, 0.3475],
    ]

    for index, position in enumerate(leg_positions):
        table.add_geom(
            name=f"receiving_table_leg_{index + 1}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=position,
            size=[0.035, 0.035, 0.3475],
            rgba=[0.35, 0.35, 0.38, 1.0],
        )


def add_tron1_tray(tron1_spec: mujoco.MjSpec) -> None:
    base_body = tron1_spec.body("base_Link")

    if base_body is None:
        raise RuntimeError(
            "TRON1 body 'base_Link' was not found"
        )

    base_body.add_geom(
        name="transport_tray",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.0, 0.0, 0.16],
        size=[0.32, 0.18, 0.02],
        rgba=[0.10, 0.30, 0.80, 1.0],
        contype=0,
        conaffinity=0,
    )


def add_transport_bottles(spec: mujoco.MjSpec) -> None:
    base_position = np.array(
        TRON1_BOTTLE_POSITION,
        dtype=float,
    )

    for index, offset in enumerate(
        BOTTLE_TRAY_OFFSETS
    ):
        bottle_number = index + 1
        bottle_position = base_position + offset

        bottle = spec.worldbody.add_body(
            name=f"transport_bottle_{bottle_number}",
            pos=bottle_position,
        )

        bottle.add_freejoint(
            name=(
                f"transport_bottle_{bottle_number}"
                "_free_joint"
            )
        )

        bottle.add_geom(
            name=(
                f"transport_bottle_{bottle_number}"
                "_collision"
            ),
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.030, 0.12, 0.0],
            mass=0.3,
            rgba=[
                0.00,
                0.45 + 0.10 * index,
                1.00 - 0.15 * index,
                1.0,
            ],
            friction=[1.5, 0.03, 0.005],
        )


def build_model() -> mujoco.MjModel:
    cell_spec = mujoco.MjSpec.from_file(
        str(UR5E_SCENE)
    )

    ur5e_base = cell_spec.body("base")

    if ur5e_base is None:
        raise RuntimeError(
            "UR5e body 'base' was not found"
        )

    ur5e_base.pos = [
        0.0,
        0.0,
        UR5E_BASE_HEIGHT,
    ]

    gripper_spec = mujoco.MjSpec.from_file(
        str(GRIPPER_MODEL)
    )

    cell_spec.attach(
        gripper_spec,
        site="attachment_site",
        prefix="gripper_",
    )

    add_ur5e_pedestal(cell_spec)
    add_receiving_table(cell_spec)

    tron1_spec = mujoco.MjSpec.from_file(
        str(TRON1_XML)
    )

    add_tron1_tray(tron1_spec)

    cell_spec.worldbody.add_frame(
        name="tron1_mount"
    )

    cell_spec.attach(
        tron1_spec,
        frame="tron1_mount",
        prefix="tron1_",
    )

    add_transport_bottles(cell_spec)

    return cell_spec.compile()


def find_tron1_free_joint(
    model: mujoco.MjModel,
) -> int:
    base_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "tron1_base_Link",
    )

    if base_body_id < 0:
        raise RuntimeError(
            "TRON1 base body was not found"
        )

    for joint_id in range(model.njnt):
        if (
            model.jnt_bodyid[joint_id] == base_body_id
            and model.jnt_type[joint_id]
            == mujoco.mjtJoint.mjJNT_FREE
        ):
            return joint_id

    raise RuntimeError(
        "TRON1 free joint was not found"
    )


def set_initial_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    home_key = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_KEY,
        "home",
    )

    if home_key >= 0:
        data.qpos[:6] = model.key_qpos[
            home_key,
            :6,
        ]
        data.ctrl[:7] = model.key_ctrl[
            home_key,
            :7,
        ]

    # UR5e를 TRON1 반대쪽으로 돌린 안전 대기 자세
    data.qpos[0] = 0.0
    data.ctrl[0] = 0.0

    tron1_free_joint = find_tron1_free_joint(
        model
    )

    tron1_qpos_address = model.jnt_qposadr[
        tron1_free_joint
    ]

    data.qpos[
        tron1_qpos_address:
        tron1_qpos_address + 7
    ] = [
        TRON1_POSITION[0],
        TRON1_POSITION[1],
        TRON1_POSITION[2],
        1.0,
        0.0,
        0.0,
        0.0,
    ]

    mujoco.mj_forward(model, data)


def main() -> None:
    model = build_model()
    data = mujoco.MjData(model)

    set_initial_state(model, data)

    pinch_site = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        "gripper_pinch",
    )

    if pinch_site < 0:
        raise RuntimeError(
            "gripper_pinch site was not found"
        )

    print(
        f"Handover cell loaded: "
        f"nq={model.nq}, nv={model.nv}, "
        f"nu={model.nu}, nbody={model.nbody}"
    )
    print(
        "UR5e pinch position:",
        data.site_xpos[pinch_site],
    )
    print(
        "TRON1 bottle position:",
        TRON1_BOTTLE_POSITION,
    )
    print("STATE: HANDOVER_CELL_READY")

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
    ) as viewer:
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
