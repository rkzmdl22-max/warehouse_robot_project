import mujoco
import mujoco.viewer
import time


model = mujoco.MjModel.from_xml_path(
    "/ros_ws/mjcf_convert/output/tiago/scene.xml"
)

data = mujoco.MjData(model)


with mujoco.viewer.launch_passive(model, data) as viewer:

    print("TIAGo wheel test")

    while viewer.is_running():

        # 오른쪽 바퀴
        data.ctrl[0] = 1.0

        # 왼쪽 바퀴
        data.ctrl[1] = 1.0

        mujoco.mj_step(model, data)

        time.sleep(0.002)
