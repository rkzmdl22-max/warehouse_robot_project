import mujoco
import numpy as np
import imageio
import time

XML_PATH = "/ros_ws/mujoco/scenes/warehouse.xml"
OUT_PATH = "warehouse_sim.mp4"
DURATION_SEC = 8
FPS = 20
WIDTH, HEIGHT = 640, 480

model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
frames = []

n_steps = int(DURATION_SEC / model.opt.timestep)
steps_per_frame = max(1, int(1 / (FPS * model.opt.timestep)))

print(f"Total steps: {n_steps}, steps_per_frame: {steps_per_frame}, expected frames: {n_steps // steps_per_frame}")

t0 = time.time()
for i in range(n_steps):
    mujoco.mj_step(model, data)
    if i % steps_per_frame == 0:
        renderer.update_scene(data, camera=-1)
        frames.append(renderer.render())
        if len(frames) % 10 == 0:
            print(f"frame {len(frames)} / step {i}/{n_steps}  ({time.time()-t0:.1f}s elapsed)")

imageio.mimsave(OUT_PATH, frames, fps=FPS)
print(f"Saved {OUT_PATH}, {len(frames)} frames, total time {time.time()-t0:.1f}s")
