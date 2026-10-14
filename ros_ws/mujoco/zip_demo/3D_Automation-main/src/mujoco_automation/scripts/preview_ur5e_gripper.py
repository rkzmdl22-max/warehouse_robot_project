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


def build_model() -> mujoco.MjModel:
    robot_spec = mujoco.MjSpec.from_file(str(UR5E_SCENE))
    gripper_spec = mujoco.MjSpec.from_file(str(GRIPPER_MODEL))

    robot_spec.attach(
        gripper_spec,
        site="attachment_site",
        prefix="gripper_",
    )

    return robot_spec.compile()


def main() -> None:
    model = build_model()
    data = mujoco.MjData(model)

    home_key = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_KEY,
        "home",
    )

    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)

    mujoco.mj_forward(model, data)

    print(f"Model loaded: nq={model.nq}, nu={model.nu}")
    print("UR5e + Robotiq 2F-85 viewer started")

    mujoco.viewer.launch(model=model, data=data)


if __name__ == "__main__":
    main()