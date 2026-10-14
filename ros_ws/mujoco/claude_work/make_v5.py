import sys
s = open("claude_amr_walk_v4.py", encoding="utf-8").read()
def rep(old, new):
    global s
    if s.count(old) != 1:
        print("패치 실패:", old[:60]); sys.exit(1)
    s = s.replace(old, new)
CI = lambda cam, r: 'cam_info(d, cam_id("%s"), %s, d.xpos[bt])' % (cam, r)
rep("from claude_vis import ann, pump", "from claude_vis2 import ann, pump, ann_side, cam_info")
rep('ann("STATION RGB-D", res, 1380, 40)', 'ann_side("STATION RGB-D", res, ["STATIONARY: change < 2mm for %d/3 frames" % stable] + ' + CI("station_rgbd_camera", "res") + ', 1380, 40)')
rep('ann("GRIPPER RGB-D", gres, 1380, 420)', 'ann_side("GRIPPER RGB-D (pregrasp)", gres, ["pregrasp view"] + ' + CI("gripper_rgbd_camera", "gres") + ', 1380, 420)')
A = 'print("close 후 접촉 법선력(N) 좌/우:", grip_force())\n'
B = ['if not USE_SAVED:',
     '    gg = detect("gripper_rgbd_camera", radius=diam_st / 2)',
     '    L = ["open gap %.1f mm | closed inner gap %.1f mm | bottle dia(model) %.1f mm | diff %.1f mm" % (open_gap * 1000, cl_gap * 1000, dia_true * 1000, abs(cl_gap - dia_true) * 1000)]',
     '    save_png(WD + "det_grasp.png", ann_side("GRIPPER RGB-D (grasped)", gg, L + ' + CI("gripper_rgbd_camera", "gg") + ', 1380, 800))']
rep(A, A + "\n".join(B) + "\n")
open("claude_vision_v5.py", "w", encoding="utf-8").write(s)
print("만듦: claude_vision_v5.py")
