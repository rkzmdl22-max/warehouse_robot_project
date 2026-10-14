import os, runpy, mujoco
_orig = mujoco.mj_step
_done = {"v": False}
def _patched(m, d, *a, **k):
    if not _done["v"]:
        _done["v"] = True
        ns = int(os.environ.get("NOSLIP", "0"))
        if ns:
            m.opt.noslip_iterations = ns
        if os.environ.get("ELLIPTIC") != "0":
            m.opt.cone = int(mujoco.mjtCone.mjCONE_ELLIPTIC)
            m.opt.impratio = float(os.environ.get("IMPRATIO", "10"))
        print("OPT cone=%d noslip=%d impratio=%.1f" % (m.opt.cone, m.opt.noslip_iterations, m.opt.impratio))
    return _orig(m, d, *a, **k)
mujoco.mj_step = _patched
runpy.run_path("ur5e_grasp_test_v2.py", run_name="__main__")
