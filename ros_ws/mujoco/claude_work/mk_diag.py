s = open("claude_vision_v5.py", encoding="utf-8").read().split("\n")
D = ['    cid = cam_id("gripper_rgbd_camera"); mujoco.mj_forward(m, d)',
     '    R.update_scene(d, camera="gripper_rgbd_camera"); R.enable_segmentation_rendering(); sg = R.render().copy(); R.disable_segmentation_rendering()',
     '    ids = sg[:, :, 0][sg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)]',
     '    for gid, n in zip(*np.unique(ids, return_counts=True)): print("GRIPCAM seg", int(gid), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(gid)), "body", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[gid])), int(n))',
     '    print("GRIPCAM cam", np.round(d.cam_xpos[cid], 4), "bottle", np.round(d.xpos[bt], 4), "dist", round(float(np.linalg.norm(d.cam_xpos[cid] - d.xpos[bt])), 4), "fovy", m.cam_fovy[cid])',
     '    sys.exit(0)']
i = [k for k, l in enumerate(s) if 'save_png(WD + "det_grasp.png"' in l]
assert len(i) == 1
s[i[0] + 1:i[0] + 1] = D
open("claude_vision_v5_diag.py", "w", encoding="utf-8").write("\n".join(s))
print("만듦: claude_vision_v5_diag.py")
