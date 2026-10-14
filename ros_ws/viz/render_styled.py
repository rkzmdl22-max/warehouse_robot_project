import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, "/ros_ws/viz")
import mujoco, imageio
from viz_style import apply_style

S = sys.argv[1] if len(sys.argv) > 1 else "/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
m = mujoco.MjModel.from_xml_path(S)
apply_style(m, audit=True)
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 720, 1280)
opt = mujoco.MjvOption(); opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]

views = {
    "overview": ([0, 0, 0.5], 24, 135, -42),
    "station":  ([0.5, -3.2, 1.0], 5.0, 150, -28),
    "tron1":    ([0.0, -3.0, 0.75], 2.4, 140, -18),
    "ur5e":     ([0.8, -3.3, 1.3], 3.0, 200, -25),
}
for name, (la, dist, az, el) in views.items():
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = la; cam.distance = dist; cam.azimuth = az; cam.elevation = el
    r.update_scene(d, camera=cam, scene_option=opt)
    imageio.imwrite(f"/ros_ws/viz/styled_{name}.png", r.render())
for i in range(m.ncam):
    r.update_scene(d, camera=i, scene_option=opt)
    imageio.imwrite(f"/ros_ws/viz/styled_cam{i}.png", r.render())
print("saved styled_*.png")
