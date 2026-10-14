s = open("claude_vision_v5.py", encoding="utf-8").read().split("\n")
a = [k for k, l in enumerate(s) if l.startswith('print("close 후 접촉 법선력')]
assert len(a) == 1 and s[a[0] + 1] == 'if not USE_SAVED:' and 'det_grasp' in s[a[0] + 4]
del s[a[0] + 1:a[0] + 5]
b = [k for k, l in enumerate(s) if l.startswith('print("lift hold 1.5s')]
assert len(b) == 1
N = ['if not USE_SAVED:',
     '    gs = detect("station_rgbd_camera", radius=diam_st / 2)',
     '    L = ["open gap %.1f mm | closed inner gap %.1f mm | bottle dia(model) %.1f mm | diff %.1f mm" % (open_gap * 1000, cl_gap * 1000, dia_true * 1000, abs(cl_gap - dia_true) * 1000),',
     '         "held & lifted %.0f mm | slip %.1f mm | grip force L/R (N): %s" % ((d.xpos[bt][2] - z0[2]) * 1000, slip * 1000, grip_force())]',
     '    save_png(WD + "det_lift.png", ann_side("STATION RGB-D (bottle held, lifted)", gs, L + cam_info(d, cam_id("station_rgbd_camera"), gs, d.xpos[bt]), 1380, 800))']
s[b[0] + 1:b[0] + 1] = N
open("claude_vision_v6.py", "w", encoding="utf-8").write("\n".join(s))
print("만듦: claude_vision_v6.py")
