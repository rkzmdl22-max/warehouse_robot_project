import mujoco

model = mujoco.MjModel.from_xml_path(
    "/ros_ws/mjcf_convert/output/tiago/scene.xml"
)

print("Actuator count:", model.nu)

for i in range(model.nu):
    print(
        i,
        mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            i
        )
    )
