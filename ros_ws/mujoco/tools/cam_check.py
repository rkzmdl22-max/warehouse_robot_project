import numpy as np
import mujoco

XML = "/ros_ws/mujoco/scenes/warehouse_tron1_integration.xml"
W, H = 960, 540
O = mujoco.mjtObj

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

targets = ["bottle_target", "bottle_place_table", "station_table", "tool0"]
tp = {}
for t in targets:
    i = mujoco.mj_name2id(m, O.mjOBJ_BODY, t)
    tp[t] = d.xpos[i].copy() if i >= 0 else None

for name in ("station_rgbd_camera", "gripper_rgbd_camera"):
    c = mujoco.mj_name2id(m, O.mjOBJ_CAMERA, name)
    if c < 0:
        print("CAM %s NOT_FOUND" % name)
        continue
    pos = d.cam_xpos[c].copy()
    R = d.cam_xmat[c].reshape(3, 3)
    right, up, fwd = R[:, 0], R[:, 1], -R[:, 2]
    fovy = float(m.cam_fovy[c])
    f = (H / 2) / np.tan(np.radians(fovy) / 2)
    print("CAM %s pos=%s forward=%s fovy=%.0f" % (
        name, np.round(pos, 3).tolist(), np.round(fwd, 3).tolist(), fovy))
    for t in targets:
        p = tp[t]
        if p is None:
            print("   %-20s NOT_FOUND" % t)
            continue
        v = p - pos
        zc = float(v @ fwd)
        if zc <= 0.01:
            print("   %-20s BEHIND_CAMERA depth=%.2fm" % (t, zc))
            continue
        u = W / 2 + f * float(v @ right) / zc
        w = H / 2 - f * float(v @ up) / zc
        inside = (0 <= u < W) and (0 <= w < H)
        print("   %-20s depth=%.2fm pixel=(%.0f,%.0f) %s" % (
            t, zc, u, w, "IN_VIEW" if inside else "OUT_OF_VIEW"))
