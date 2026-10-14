import mujoco
import mujoco.viewer


model = mujoco.MjModel.from_xml_path(
    "/ros_ws/mjcf_convert/worlds/empty.xml"
)

data = mujoco.MjData(model)


print("START VIEWER")


mujoco.viewer.launch(model, data)
