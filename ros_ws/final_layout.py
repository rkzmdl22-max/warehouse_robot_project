import re, math, shutil, time
import xml.etree.ElementTree as ET

PATH = "/ros_ws/src/warehouse_robot/worlds/warehouse_remodel.world"

PLACE = {
    "table_bottle_handover": (1.15, -3.00),
    "table_ur5e_handover":   (1.15, -3.90),
}
YAW = math.pi / 2
BARRIERS = {
    "safety_barrier_west":  (-0.30, -3.45, math.pi / 2),
    "safety_barrier_north": (1.70, -2.20, 0.0),
}
UR5E = (0.60, -3.45)

TOP_T, LEG, INSET = 0.05, 0.06, 0.06
TOP_RGB = "0.0 0.3 0.9 1"
LEG_RGB = "0.12 0.12 0.15 1"
NAME_RE = re.compile(r"table|station|desk|workbench", re.I)
SKIP_RE = re.compile(r"charging|conveyor|rack|pallet", re.I)


def pose_of(el):
    p = el.find("pose") if el is not None else None
    v = [float(t) for t in (p.text.split() if p is not None and p.text else [])]
    return (v + [0.0] * 6)[:6]


def set_pose(model, x, y, yaw):
    p = model.find("pose")
    if p is None:
        p = ET.Element("pose")
        model.insert(0, p)
    p.text = f"{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}"


def boxes(model):
    mp = pose_of(model)
    out = []
    for link in model.findall("link"):
        lp = pose_of(link)
        for e in link.findall("visual") + link.findall("collision"):
            b = e.find("geometry/box/size")
            if b is None:
                continue
            sx, sy, sz = map(float, b.text.split())
            ep = pose_of(e)
            out.append(dict(sx=sx, sy=sy, sz=sz,
                            ox=lp[0] + ep[0], oy=lp[1] + ep[1],
                            top=mp[2] + lp[2] + ep[2] + sz / 2))
    return out


def table_top(model):
    n = model.get("name", "")
    if not NAME_RE.search(n) or SKIP_RE.search(n):
        return None
    bs = boxes(model)
    if not bs:
        return None
    big = max(b["sx"] * b["sy"] for b in bs)
    cands = [b for b in bs if b["sx"] * b["sy"] >= 0.3 * big]
    t = max(cands, key=lambda b: b["top"])
    if not (0.5 <= t["top"] <= 1.1):
        return None
    return t


def part(kind, name, pose, size, rgb):
    sx, sy, sz = size
    geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    mat = f"<material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>"
    if kind == "col":
        return f'<collision name="{name}_col"><pose>{pose}</pose>{geom}</collision>'
    return f'<visual name="{name}_vis"><pose>{pose}</pose>{geom}{mat}</visual>'


def make_link(sx, sy, h):
    items = []
    tp = f"0 0 {h - TOP_T / 2:.4f} 0 0 0"
    items.append(part("col", "top", tp, (sx, sy, TOP_T), TOP_RGB))
    items.append(part("vis", "top", tp, (sx, sy, TOP_T), TOP_RGB))
    lh = h - TOP_T
    lx = sx / 2 - INSET - LEG / 2
    ly = sy / 2 - INSET - LEG / 2
    for k, (dx, dy) in enumerate([(-lx, -ly), (-lx, ly), (lx, -ly), (lx, ly)]):
        p = f"{dx:.3f} {dy:.3f} {lh / 2:.4f} 0 0 0"
        items.append(part("col", f"leg{k}", p, (LEG, LEG, lh), LEG_RGB))
        items.append(part("vis", f"leg{k}", p, (LEG, LEG, lh), LEG_RGB))
    return ET.fromstring('<link name="link">' + "".join(items) + "</link>")


def aabbs(m):
    mx, my, mz, _, _, myaw = pose_of(m)
    out = []
    for link in m.findall("link"):
        lp = pose_of(link)
        for c in link.findall("collision"):
            b = c.find("geometry/box/size")
            if b is None:
                continue
            sx, sy, sz = map(float, b.text.split())
            cp = pose_of(c)
            lx, ly = lp[0] + cp[0], lp[1] + cp[1]
            yaw = myaw + lp[5] + cp[5]
            cx = mx + lx * math.cos(myaw) - ly * math.sin(myaw)
            cy = my + lx * math.sin(myaw) + ly * math.cos(myaw)
            hx = (abs(sx * math.cos(yaw)) + abs(sy * math.sin(yaw))) / 2
            hy = (abs(sx * math.sin(yaw)) + abs(sy * math.cos(yaw))) / 2
            top = mz + lp[2] + cp[2] + sz / 2
            out.append((cx - hx, cx + hx, cy - hy, cy + hy, top))
    return out


parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
tree = ET.parse(PATH, parser)
world = tree.getroot().find("world")
backup = f"{PATH}.backup_{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy(PATH, backup)

final = {}
for m in world.findall("model"):
    name = m.get("name")
    t = table_top(m)
    if t is None:
        continue
    mp = pose_of(m)
    c, s = math.cos(mp[5]), math.sin(mp[5])
    x = mp[0] + t["ox"] * c - t["oy"] * s
    y = mp[1] + t["ox"] * s + t["oy"] * c
    yaw = mp[5]
    if name in PLACE:
        x, y = PLACE[name]
        yaw = YAW
    for link in m.findall("link"):
        m.remove(link)
    m.append(make_link(t["sx"], t["sy"], t["top"]))
    set_pose(m, x, y, yaw)
    final[name] = (t["sx"], t["sy"], t["top"], x, y, yaw)

for m in world.findall("model"):
    n = m.get("name")
    if n in BARRIERS:
        bx, by, byaw = BARRIERS[n]
        set_pose(m, bx, by, byaw)
        print("펜스 이동:", n, "->", (bx, by))

areas = {}
for n in PLACE:
    if n in final:
        sx, sy, h, x, y, yaw = final[n]
        hx = (abs(sx * math.cos(yaw)) + abs(sy * math.sin(yaw))) / 2
        hy = (abs(sx * math.sin(yaw)) + abs(sy * math.cos(yaw))) / 2
        areas[n] = (x - hx, x + hx, y - hy, y + hy)
areas["TRON1 접근 통로 (폭 1.0m)"] = (1.5, 4.5, -3.5, -2.5)
areas["UR5e 작업 반경"] = (UR5E[0] - 0.8, UR5E[0] + 0.8, UR5E[1] - 0.8, UR5E[1] + 0.8)

print()
for label, r in areas.items():
    hit = set()
    for m in world.findall("model"):
        n = m.get("name", "")
        if n in final or re.search(r"floor|ground|sun|marking", n, re.I):
            continue
        for (x0, x1, y0, y1, top) in aabbs(m):
            if top >= 0.15 and r[0] < x1 and r[1] > x0 and r[2] < y1 and r[3] > y0:
                hit.add(n)
    print(("[경고] " + label + " 와 겹침: " + ", ".join(sorted(hit))) if hit else "[OK]   " + label + ": 겹침 없음")

ET.indent(tree)
tree.write(PATH, encoding="utf-8", xml_declaration=True)
print()
print("백업:", backup)
for n, (sx, sy, h, x, y, yaw) in final.items():
    print(f"  {n}: {sx:.2f}x{sy:.2f} 높이 {h:.2f} 위치 ({x:.2f}, {y:.2f}) 각도 {math.degrees(yaw):.0f}도")
