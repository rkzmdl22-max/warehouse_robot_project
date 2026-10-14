import os
os.environ.setdefault("MUJOCO_GL", "egl")
import json, numpy as np, mujoco
m = mujoco.MjModel.from_xml_path("/ros_ws/mujoco/scenes/claude_handover_scene.xml"); d = mujoco.MjData(m)
B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
Jn = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
sol = json.load(open("/ros_ws/mujoco/claude_work/ik_solution.json"))
arm = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
QADR = [m.jnt_qposadr[Jn(n)] for n in arm]
bt = B("carry_bottle_3")
H, W = 480, 640
R = mujoco.Renderer(m, H, W)
def detect(cam, label):
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    R.update_scene(d, camera=cam); R.enable_depth_rendering(); depth = R.render().copy(); R.disable_depth_rendering()
    R.update_scene(d, camera=cam); R.enable_segmentation_rendering(); seg = R.render().copy(); R.disable_segmentation_rendering()
    ids, typ = seg[:, :, 0], seg[:, :, 1]
    valid = typ == int(mujoco.mjtObj.mjOBJ_GEOM)
    mask = np.zeros((H, W), bool); mask[valid] = m.geom_bodyid[ids[valid]] == bt
    v, u = np.nonzero(mask)
    print("[%s | %s] 병 픽셀 수: %d" % (label, cam, len(u)))
    if len(u) < 30:
        print("   → 이 카메라에서는 병이 거의 안 보임 (가려짐 또는 시야 밖)"); return
    z = depth[v, u]; ok = np.isfinite(z) & (z > 0); u, v, z = u[ok], v[ok], z[ok]
    f = (H/2)/np.tan(np.radians(m.cam_fovy[cid])/2)
    pc = np.stack([(u-W/2)/f*z, -(v-H/2)/f*z, -z], 1)
    pw = d.cam_xpos[cid] + (d.cam_xmat[cid].reshape(3, 3) @ pc.T).T
    ray = pw.mean(0) - d.cam_xpos[cid]; h = ray[:2]/np.linalg.norm(ray[:2]); p = np.array([-h[1], h[0]])
    diam = np.ptp(pw[:, :2] @ p)
    front = pw[z <= z.min() + 0.004]
    cxy = front[:, :2].mean(0) + (diam/2)*h
    zr = (pw[:, 2].min(), pw[:, 2].max()); cz = 0.5*(zr[0]+zr[1])
    gt = d.xpos[bt]
    err = np.linalg.norm(np.array([cxy[0], cxy[1], cz]) - gt)*1000
    print("   추정 중심 (%.3f, %.3f, %.3f) | 정답 (%.3f, %.3f, %.3f) | 3D 오차 %.1f mm" % (cxy[0], cxy[1], cz, gt[0], gt[1], gt[2], err))
    print("   추정 지름 %.1f mm (모델 70.0) | 보이는 높이 %.0f mm" % (diam*1000, (zr[1]-zr[0])*1000))
mujoco.mj_forward(m, d)
detect("station_rgbd_camera", "스테이션, 초기 상태")
for k in ("above", "pregrasp"):
    for i, x in enumerate(QADR): d.qpos[x] = sol[k][i]
    mujoco.mj_forward(m, d)
    detect("gripper_rgbd_camera", "그리퍼, " + k)
