from pathlib import Path

import mujoco
import mujoco.viewer


PROJECT_ROOT = Path(__file__).resolve().parents[3]
MENAGERIE_ROOT = PROJECT_ROOT / "src" / "vendor" / "mujoco_menagerie"

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


def add_station_table(spec: mujoco.MjSpec) -> None:
    table = spec.worldbody.add_body(name="station_table")

    table.add_geom(
        name="tabletop",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.0, 0.65, 0.18],
        size=[0.45, 0.35, 0.025],
        rgba=[0.12, 0.28, 0.55, 1.0],
        friction=[1.0, 0.01, 0.001],
    )

    leg_positions = [
        [-0.38, 0.38, 0.0775],
        [0.38, 0.38, 0.0775],
        [-0.38, 0.92, 0.0775],
        [0.38, 0.92, 0.0775],
    ]

    for index, position in enumerate(leg_positions):
        table.add_geom(
            name=f"table_leg_{index + 1}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=position,
            size=[0.035, 0.035, 0.0775],
            rgba=[0.35, 0.35, 0.38, 1.0],
        )


def add_bottle(spec: mujoco.MjSpec) -> None:
    bottle = spec.worldbody.add_body(
        name="water_bottle",
        pos=[-0.15, 0.62, 0.325],
    )

    bottle.add_freejoint(name="water_bottle_free_joint")

    bottle.add_geom(
        name="water_bottle_collision",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[0.030, 0.12, 0.0],
        mass=0.3,
        rgba=[0.05, 0.45, 0.90, 1.0],
        friction=[1.2, 0.01, 0.001],
    )


def build_model() -> mujoco.MjModel:
    robot_spec = mujoco.MjSpec.from_file(str(UR5E_SCENE))
    gripper_spec = mujoco.MjSpec.from_file(str(GRIPPER_MODEL))

    robot_spec.attach(
        gripper_spec,
        site="attachment_site",
        prefix="gripper_",
    )

    add_station_table(robot_spec)
    add_bottle(robot_spec)

    model = robot_spec.compile()

    bottle_joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "water_bottle_free_joint",
    )
    bottle_dof_adr = model.jnt_dofadr[bottle_joint_id]

    model.dof_damping[bottle_dof_adr : bottle_dof_adr + 6] = [
        0.1,
        0.1,
        0.1,
        0.01,
        0.01,
        0.01,
    ]

    return model


def set_initial_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> None:
    data.qpos[:] = model.qpos0

    home_key = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_KEY,
        "home",
    )

    if home_key >= 0:
        data.qpos[:6] = model.key_qpos[home_key, :6]
        data.ctrl[:] = model.key_ctrl[home_key]

    mujoco.mj_forward(model, data)


def main() -> None:
    model = build_model()
    data = mujoco.MjData(model)

    set_initial_state(model, data)

    print(f"Workcell loaded: nq={model.nq}, nu={model.nu}")
    print("Bottle mass: 0.300 kg")
    print("UR5e + gripper + table + bottle viewer started")

    mujoco.viewer.launch(model=model, data=data)


if __name__ == "__main__":
    main()