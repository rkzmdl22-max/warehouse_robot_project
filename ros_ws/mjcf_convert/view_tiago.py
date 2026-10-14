import time
import mujoco
import mujoco.viewer


MODEL_PATH = "/ros_ws/mjcf_convert/worlds/empty.xml"


model = mujoco.MjModel.from_xml_path(
    MODEL_PATH
)

data = mujoco.MjData(model)


print("==========")
print("TIAGo MuJoCo Viewer START")
print("Model :", MODEL_PATH)
print("Bodies :", model.nbody)
print("Joints :", model.njnt)
print("Actuators :", model.nu)
print("==========")



with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    viewer.cam.distance = 3
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -20

    while viewer.is_running():

        mujoco.mj_step(
            model,
            data
        )

        viewer.sync()

        time.sleep(0.01)
