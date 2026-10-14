import time
import mujoco
import mujoco.viewer


model = mujoco.MjModel.from_xml_path(
    "/ros_ws/mjcf_convert/output/tiago/scene.xml"
)

data = mujoco.MjData(model)

print("================")
print("TIAGo MuJoCo")
print("================")
print("Bodies :", model.nbody)
print("Joints :", model.njnt)
print("Actuators :", model.nu)
print("Sensors :", model.nsensor)

print("\n--- Actuator Info ---")
for i in range(min(5, model.nu)):
    name = model.actuator(i).name
    gainprm = model.actuator_gainprm[i][:3]
    ctrlrange = model.actuator_ctrlrange[i]
    print(f"Actuator [{i}] {name} -> gainprm: {gainprm}, ctrlrange: {ctrlrange}")



with mujoco.viewer.launch_passive(model, data) as viewer:

    print("VIEWER STARTED")

    viewer.cam.distance = 3
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -20

    while viewer.is_running():

        mujoco.mj_step(model, data)

        time.sleep(0.002)

    print("VIEWER CLOSED")
