import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco, imageio
T = mujoco.mjtObj
MESH = mujoco.mjtGeom.mjGEOM_MESH

def shot(m, d, r, name, lookat, dist, az, el, allgroups=True):
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat; cam.distance = dist; cam.azimuth = az; cam.elevation = el
    opt = mujoco.MjvOption()
    if allgroups: opt.geomgroup[:] = 1
    r.update_scene(d, camera=cam, scene_option=opt)
    imageio.imwrite(f"/ros_ws/viz/{name}.png", r.render())

m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
keys = ("base_Link","carry_","abad","hip_","knee","ur5e","finger","tool0","station","bottle")
print("== body별 geom 요약 (개수 / 종류 / group / alpha / mesh) ==")
for b in range(1, m.nbody):
    n = mujoco.mj_id2name(m, T.mjOBJ_BODY, b) or ""
    if not any(k in n for k in keys): continue
    gs = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b]
    if not gs:
        print(f"{n}: geom 0개"); continue
    types = sorted({int(m.geom_type[g]) for g in gs})
    groups = sorted({int(m.geom_group[g]) for g in gs})
    al = m.geom_rgba[gs, 3]
    meshes = sorted({mujoco.mj_id2name(m, T.mjOBJ_MESH, m.geom_dataid[g]) or "?" for g in gs if m.geom_type[g] == MESH})
    print(f"{n}: n={len(gs)} type={types} group={groups} alpha={al.min():.2f}~{al.max():.2f} mesh={meshes[:3]}")

r = mujoco.Renderer(m, 480, 640)
shot(m, d, r, "v_overview", [0.5,-3.0,1.0], 7.0, 120, -35)
shot(m, d, r, "v_tron1", [0.0,-3.0,0.8], 2.5, 150, -20)
shot(m, d, r, "v_ur5e", [0.8,-3.3,1.2], 3.0, 200, -25)
shot(m, d, r, "v_tron1_only_group0", [0.0,-3.0,0.8], 2.5, 150, -20, allgroups=False)
print("saved: v_overview, v_tron1, v_ur5e, v_tron1_only_group0")

try:
    tm = mujoco.MjModel.from_xml_path("/ros_ws/mujoco_menagerie/pal_tiago/tiago.xml")
    td = mujoco.MjData(tm); mujoco.mj_forward(tm, td)
    tr = mujoco.Renderer(tm, 480, 640)
    c = tm.stat.center.copy()
    shot(tm, td, tr, "v_tiago_front", c, tm.stat.extent*1.8, 140, -15)
    shot(tm, td, tr, "v_tiago_side", c, tm.stat.extent*1.8, 220, -15)
    print("TIAGo bodies:", tm.nbody, "saved: v_tiago_front, v_tiago_side")
except Exception as e:
    print("TIAGo 렌더 실패:", e)
