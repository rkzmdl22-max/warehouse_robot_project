import os
os.environ.setdefault("MUJOCO_GL", "egl")
import json, numpy as np, mujoco
m = mujoco.MjModel.from_xml_path("/ros_ws/mujoco/scenes/claude_handover_scene.xml"); d = mujoco.MjData(m)
B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
Jn = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
sol = json.load(open("/ros_ws/mujoco/claude_work/ik_solution.json"))
arm = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
QADR = [m.jnt_qposadr[Jn(n)] for n in arm]
bt = B("carry_bottle_3"); H, W = 480, 640
m.vis.map.znear = 0.0003; m.vis.map.zfar = 3.0
print("effective near/far (m): %.4f / %.1f" % (m.vis.map.znear*m.stat.extent, m.vis.map.zfar*m.stat.extent))
R = mujoco.Renderer(m, H, W)
def setpose(k):
    for i, x in enumerate(QADR): d.qpos[x] = sol[k][i]
    mujoco.mj_forward(m, d)
def aim(cam, target):
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam); mujoco.mj_forward(m, d)
    f = target - d.cam_xpos[cid]; f = f/np.linalg.norm(f); z = -f
    x = np.cross([0, 0, 1.0], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    Rl = d.xmat[m.cam_bodyid[cid]].reshape(3, 3).T @ np.stack([x, y, z], 1)
    q = np.zeros(4); mujoco.mju_mat2Quat(q, Rl.flatten()); m.cam_quat[cid] = q; mujoco.mj_forward(m, d)
def detect(cam, label):
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    R.update_scene(d, camera=cam); R.enable_depth_rendering(); depth = R.render().copy(); R.disable_depth_rendering()
    R.update_scene(d, camera=cam); R.enable_segmentation_rendering(); seg = R.render().copy(); R.disable_segmentation_rendering()
    ids, typ = seg[:, :, 0], seg[:, :, 1]; valid = typ == int(mujoco.mjtObj.mjOBJ_GEOM)
    mask = np.zeros((H, W), bool); mask[valid] = m.geom_bodyid[ids[valid]] == bt
    v, u = np.nonzero(mask)
    f = (H/2)/np.tan(np.radians(m.cam_fovy[cid])/2)
    pc = (d.cam_xmat[cid].reshape(3, 3).T @ (d.xpos[bt]-d.cam_xpos[cid]))
    pu, pv = W/2 + f*pc[0]/(-pc[2]), H/2 - f*pc[1]/(-pc[2])
    print("[%s | %s] 병 중심 투영 픽셀 (%.0f, %.0f) (화면 %dx%d, 음수 z면 뒤쪽) | 병 픽셀 수 %d" % (label, cam, pu, pv, W, H, len(u)))
    if len(u) < 30:
        print("   → 병이 안 보임"); return
    z = depth[v, u]; ok = np.isfinite(z) & (z > 0); u, v, z = u[ok], v[ok], z[ok]
    pw = d.cam_xpos[cid] + (d.cam_xmat[cid].reshape(3, 3) @ np.stack([(u-W/2)/f*z, -(v-H/2)/f*z, -z], 1).T).T
    ray = pw.mean(0) - d.cam_xpos[cid]; h = ray[:2]/np.linalg.norm(ray[:2]); p = np.array([-h[1], h[0]])
    diam = np.ptp(pw[:, :2] @ p); front = pw[z <= z.min()+0.004]
    cxy = front[:, :2].mean(0) + (diam/2)*h; zr = (pw[:, 2].min(), pw[:, 2].max()); cz = 0.5*(zr[0]+zr[1]); gt = d.xpos[bt]
    print("   추정 중심 (%.3f, %.3f, %.3f) | 정답 (%.3f, %.3f, %.3f) | 3D 오차 %.1f mm" % (cxy[0], cxy[1], cz, gt[0], gt[1], gt[2], np.linalg.norm(np.array([cxy[0], cxy[1], cz])-gt)*1000))
    print("   추정 지름 %.1f mm (모델 70.0) | 보이는 높이 %.0f mm (위에서 보면 의미 없음)" % (diam*1000, (zr[1]-zr[0])*1000))
def aim_along_tool(cam):
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam); mujoco.mj_forward(m, d)
    Rw = d.xmat[B("tool0")].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
    Rl = d.xmat[m.cam_bodyid[cid]].reshape(3, 3).T @ Rw
    q = np.zeros(4); mujoco.mju_mat2Quat(q, Rl.flatten()); m.cam_quat[cid] = q; mujoco.mj_forward(m, d)
mujoco.mj_forward(m, d)
aim("station_rgbd_camera", d.xpos[bt]); detect("station_rgbd_camera", "스테이션(마운트 보정)")
setpose("pregrasp"); aim_along_tool("gripper_rgbd_camera"); detect("gripper_rgbd_camera", "그리퍼 pregrasp (접근축 방향 마운트)")
setpose("above"); detect("gripper_rgbd_camera", "그리퍼 above (같은 마운트)")
