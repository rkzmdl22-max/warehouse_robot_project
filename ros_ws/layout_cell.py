import sys, re, math, shutil, time, argparse
import xml.etree.ElementTree as ET

ap = argparse.ArgumentParser()
ap.add_argument("world")
ap.add_argument("--origin", nargs=2, type=float)
ap.add_argument("--yaw", type=float, default=0.0)
ap.add_argument("--height", type=float, default=0.8)
args = ap.parse_args()

WORLD_DIR = "/ros_ws/src/warehouse_robot/worlds/"
path = args.world if "/" in args.world else WORLD_DIR + args.world

TOP_T, LEG, INSET = 0.05, 0.06, 0.06
TOP_RGB = "0.0 0.3 0.9 1"
LEG_RGB = "0.12 0.12 0.15 1"
NAME_RE = re.compile(r"table|station|desk|workbench", re.I)
SKIP_RE = re.compile(r"charging|conveyor|rack|pallet", re.I)
CELL_RE = re.compile(r"table.*handover|handover.*table|station_table", re.I)
IGNORE_RE = re.compile(r"floor|ground|sun|ur5e|workcell", re.I)


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


def new_table(name, x, y, yaw, sx, sy, h):
    m = ET.fromstring(f'<model name="{name}"><static>true</static></model>')
    set_pose(m, x, y, yaw)
    m.append(make_link(sx, sy, h))
    return m


def world_aabbs(m):
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
tree = ET.parse(path, parser)
world = tree.getroot().find("world")
backup = f"{path}.backup_{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy(path, backup)

if args.origin:
    ox, oy = args.origin
else:
    ox = oy = None
    for m in world.findall("model"):
        if "ur5e" in m.get("name", "").lower():
            p = pose_of(m)
            ox, oy = p[0], p[1]
            break
    if ox is None:
        sys.exit("UR5e 위치를 못 찾았습니다. --origin X Y 로 지정해 주세요.")

yaw = math.radians(args.yaw)
H = args.height


def to_world(lx, ly):
    return (ox + lx * math.cos(yaw) - ly * math.sin(yaw),
            oy + lx * math.sin(yaw) + ly * math.cos(yaw))


insert_at = None
for m in list(world.findall("model")):
    if CELL_RE.search(m.get("name", "")):
        if insert_at is None:
            insert_at = list(world).index(m)
        world.remove(m)
        print("제거:", m.get("name"))

restyled = []
for m in world.findall("model"):
    t = table_top(m)
    if t is None:
        continue
    mp = pose_of(m)
    x, y, myaw = mp[0] + t["ox"], mp[1] + t["oy"], mp[5]
    for link in m.findall("link"):
        m.remove(link)
    m.append(make_link(t["sx"], t["sy"], t["top"]))
    set_pose(m, x, y, myaw)
    restyled.append(m.get("name"))

TW, TD = 0.6, 0.9
CX = 0.20 + TW / 2
A = ("handover_table", CX, -TD / 2)
B = ("station_table", CX, +TD / 2)
new_models = []
for name, lx, ly in (A, B):
    wx, wy = to_world(lx, ly)
    new_models.append(new_table(name, wx, wy, yaw, TW, TD, H))
pos = insert_at if insert_at is not None else len(list(world))
for i, nm in enumerate(new_models):
    world.insert(pos + i, nm)


def rect_world(lx0, lx1, ly0, ly1):
    pts = [to_world(a, b) for a in (lx0, lx1) for b in (ly0, ly1)]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


areas = {
    "테이블 셀": rect_world(0.20, 0.20 + TW, -TD, TD),
    "TRON1 접근 통로(폭 1.0m)": rect_world(0.80, 3.80, -TD / 2 - 0.5, -TD / 2 + 0.5),
    "UR5e 작업 반경(0.9m)": rect_world(-0.9, 0.9, -0.9, 0.9),
}
new_names = {nm.get("name") for nm in new_models}
obstacles = []
for m in world.findall("model"):
    n = m.get("name", "")
    if n in new_names or IGNORE_RE.search(n):
        continue
    for (x0, x1, y0, y1, top) in world_aabbs(m):
        if top >= 0.15:
            obstacles.append((n, x0, x1, y0, y1))


def overlaps(a, o):
    return a[0] < o[2] and a[1] > o[1] and a[2] < o[4] and a[3] > o[3]


print()
bad = False
for label, rect in areas.items():
    hit = sorted({o[0] for o in obstacles if overlaps(rect, o)})
    if hit:
        bad = True
        print(f"[경고] {label} 안에 다른 물체가 있음: {', '.join(hit)}")
    else:
        print(f"[OK]   {label}: 겹치는 물체 없음")

ET.indent(tree)
tree.write(path, encoding="utf-8", xml_declaration=True)

sx_, sy_ = to_world(1.35, -TD / 2)
cx_, cy_ = to_world(CX, -TD / 2)
print()
print("백업:", backup)
print("모양 변경:", ", ".join(restyled) if restyled else "(없음)")
print("셀 원점(UR5e): (%.2f, %.2f), 방향 %.0f도, 테이블 높이 %.2f" % (ox, oy, args.yaw, H))
for nm in new_models:
    p = pose_of(nm)
    print(f"  {nm.get('name')}: 위치 ({p[0]:.2f}, {p[1]:.2f})")
print("TRON1 정지 위치(서쪽을 바라봄): (%.2f, %.2f)" % (sx_, sy_))
print("병 정지/인계 지점 + 카메라 아래: (%.2f, %.2f)" % (cx_, cy_))
if bad:
    print("\n경고가 나온 방향은 --yaw 를 90/180/270 으로 바꿔 다시 실행해 보세요.")
