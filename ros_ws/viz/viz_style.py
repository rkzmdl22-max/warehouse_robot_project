import mujoco
T = mujoco.mjtObj
MESH = mujoco.mjtGeom.mjGEOM_MESH

def _b(m, g): return mujoco.mj_id2name(m, T.mjOBJ_BODY, m.geom_bodyid[g]) or ""
def _n(m, g): return mujoco.mj_id2name(m, T.mjOBJ_GEOM, g) or ""
def _mesh(m, g):
    if m.geom_type[g] != MESH: return ""
    return mujoco.mj_id2name(m, T.mjOBJ_MESH, m.geom_dataid[g]) or ""

def paint(m, g, rgba):
    m.geom_matid[g] = -1
    m.geom_rgba[g] = rgba

def apply_style(m, audit=False):
    for g in range(m.ngeom):
        if m.geom_rgba[g][3] < 0.01:      # 투명한 충돌용 geom은 그대로
            continue
        b, n, ms = _b(m, g), _n(m, g), _mesh(m, g)
        sz = m.geom_size[g]
        is_box = m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
        is_top = is_box and sz[2] < min(sz[0], sz[1])

        if n == "floor":
            mid = m.geom_matid[g]
            if mid >= 0 and (m.geom_matid == mid).sum() == 1:
                m.mat_texid[mid, :] = -1
                m.mat_rgba[mid] = [0.58, 0.59, 0.61, 1]
                m.mat_reflectance[mid] = 0.08
            else:
                paint(m, g, [0.58, 0.59, 0.61, 1])
        elif n.startswith("wall_"):
            paint(m, g, [0.90, 0.88, 0.82, 1])

        # TRON1 (시각용 메시만 칠함)
        elif b == "base_Link" and ms:
            paint(m, g, [0.94, 0.95, 0.97, 1])
        elif "abad" in b and ms:
            paint(m, g, [0.22, 0.24, 0.27, 1])
        elif "hip" in b and ms:
            paint(m, g, [0.90, 0.91, 0.93, 1])
        elif "knee" in b and ms:
            if ms.startswith("foot"): paint(m, g, [0.10, 0.10, 0.11, 1])
            else: paint(m, g, [0.88, 0.90, 0.92, 1])
        elif b == "carry_tray":
            paint(m, g, [0.25, 0.28, 0.33, 1])
        elif b == "carry_bottle":
            paint(m, g, [0.10, 0.45, 0.95, 1])
        elif b == "carry_bottle_2":
            paint(m, g, [0.15, 0.75, 0.35, 1])
        elif b == "carry_bottle_3":
            paint(m, g, [0.98, 0.55, 0.10, 1])

        # 스테이션
        elif b in ("station_table", "ur5e_worktable", "bottle_place_table") and is_box:
            paint(m, g, [0.80, 0.68, 0.50, 1] if is_top else [0.20, 0.21, 0.23, 1])
        elif b == "station_camera_stand":
            paint(m, g, [0.20, 0.20, 0.22, 1])
        elif b in ("left_finger_link", "right_finger_link", "tool0"):
            paint(m, g, [0.10, 0.10, 0.12, 1])

        # 창고 물류 물체
        elif b.startswith("box_"):
            paint(m, g, [0.78, 0.60, 0.38, 1])
        elif b.startswith("pallet_"):
            paint(m, g, [0.62, 0.46, 0.28, 1])
        elif b.startswith("tote_"):
            paint(m, g, [0.20, 0.45, 0.75, 1])
        elif b == "conveyor_frame":
            paint(m, g, [0.30, 0.32, 0.36, 1])

    # 조명
    for l in range(m.nlight):
        m.light_diffuse[l] = [0.70, 0.68, 0.64]
        m.light_specular[l] = [0.15, 0.15, 0.15]
        m.light_ambient[l] = [0.05, 0.05, 0.05]
        m.light_castshadow[l] = 1 if l < 2 else 0
    m.vis.headlight.ambient[:] = [0.32, 0.32, 0.32]
    m.vis.headlight.diffuse[:] = [0.40, 0.40, 0.40]
    m.vis.headlight.specular[:] = [0.10, 0.10, 0.10]

    # 렌더 품질
    m.vis.global_.offwidth = 1280
    m.vis.global_.offheight = 720
    m.vis.quality.offsamples = 8

    if audit:
        print("== rack geom 색 목록 ==")
        seen = {}
        for g in range(m.ngeom):
            if "rack" in _b(m, g):
                k = (tuple(m.geom_rgba[g].round(2)), int(m.geom_matid[g]), int(m.geom_type[g]))
                seen[k] = seen.get(k, 0) + 1
        for k, v in seen.items(): print(k, "x", v)
        print("== material 목록 ==")
        for i in range(m.nmat):
            print(i, mujoco.mj_id2name(m, T.mjOBJ_MATERIAL, i), m.mat_rgba[i].round(2))
