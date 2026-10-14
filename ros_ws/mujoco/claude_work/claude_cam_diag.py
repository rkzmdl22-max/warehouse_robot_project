import os
os.environ.setdefault("MUJOCO_GL", "egl")
import json, numpy as np, mujoco
from collections import Counter
m = mujoco.MjModel.from_xml_path("/ros_ws/mujoco/scenes/claude_handover_scene.xml"); d = mujoco.MjData(m)
B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
Jn = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
sol = json.load(open("/ros_ws/mujoco/claude_work/ik_solution.json"))
arm = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
QADR = [m.jnt_qposadr[Jn(n)] for n in arm]
bt = B("carry_bottle_3"); R = mujoco.Renderer(m, 480, 640)
def diag(cam, label):
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    R.update_scene(d, camera=cam); R.enable_segmentation_rendering(); seg = R.render().copy(); R.disable_segmentation_rendering()
    fwd = -d.cam_xmat[cid].reshape(3, 3)[:, 2]; to = d.xpos[bt] - d.cam_xpos[cid]; dist = np.linalg.norm(to)
    ang = np.degrees(np.arccos(np.clip(fwd @ to / dist, -1, 1)))
    print("[%s | %s] fovy %.0f | 카메라 pos %s | 시선 %s | 병까지 거리 %.2f m | 시선과 병 방향 각도 %.0f deg" %
          (label, cam, m.cam_fovy[cid], np.round(d.cam_xpos[cid], 2), np.round(fwd, 2), dist, ang))
    ids, typ = seg[:, :, 0], seg[:, :, 1]; v = typ == int(mujoco.mjtObj.mjOBJ_GEOM)
    c = Counter(m.geom_bodyid[ids[v]].tolist())
    print("   보이는 body 상위:", [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, k), n) for k, n in c.most_common(6)], "| 전체 geom 픽셀", int(v.sum()))
    print("   병 geom group:", sorted({int(m.geom_group[g]) for g in range(m.ngeom) if m.geom_bodyid[g] == bt}), "| 병 geom rgba alpha:", sorted({round(float(m.geom_rgba[g][3]), 2) for g in range(m.ngeom) if m.geom_bodyid[g] == bt}))
mujoco.mj_forward(m, d)
diag("station_rgbd_camera", "스테이션")
for k in ("above", "pregrasp", "grasp"):
    for i, x in enumerate(QADR): d.qpos[x] = sol[k][i]
    mujoco.mj_forward(m, d); diag("gripper_rgbd_camera", "그리퍼 " + k)
