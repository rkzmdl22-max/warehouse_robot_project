import os, sys, runpy
import mujoco
import imageio

W, H, FPS = 960, 540, 20
OUT = os.environ.get("OUT", "mujoco_grasp.mp4")
_orig = mujoco.mj_step
_s = {"init": False, "next": 0.0, "frames": []}


def _patched(m, d, *a, **k):
    if not _s["init"]:
        _s["init"] = True
        m.opt.cone = int(mujoco.mjtCone.mjCONE_ELLIPTIC)
        m.opt.impratio = 10.0
        m.vis.global_.offwidth = W
        m.vis.global_.offheight = H
        _s["r"] = mujoco.Renderer(m, height=H, width=W)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [1.15, -3.0, 1.0]
        cam.distance = float(os.environ.get("DIST", "1.6"))
        cam.azimuth = float(os.environ.get("AZ", "135"))
        cam.elevation = float(os.environ.get("EL", "-15"))
        _s["cam"] = cam
        print("REC cone=1 impratio=10 -> %s" % OUT, flush=True)
    _orig(m, d, *a, **k)
    if d.time >= _s["next"]:
        _s["next"] += 1.0 / FPS
        _s["r"].update_scene(d, camera=_s["cam"])
        _s["frames"].append(_s["r"].render().copy())
        if len(_s["frames"]) % 50 == 0:
            print("REC frames=%d t=%.1fs" % (len(_s["frames"]), d.time), flush=True)


mujoco.mj_step = _patched
try:
    runpy.run_path("ur5e_grasp_test_v2.py", run_name="__main__")
except SystemExit:
    pass
finally:
    if _s["frames"]:
        imageio.mimsave(OUT, _s["frames"], fps=FPS)
        print("SAVED %s frames=%d (%.1fs)" % (OUT, len(_s["frames"]), len(_s["frames"]) / FPS), flush=True)
