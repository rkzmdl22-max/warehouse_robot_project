import re

TABLES = {
    "handover_table": (0.95, -1.8, 0.9, 0.9, 0.85),
    "picking_station": (-3.5, 0.0, 2.0, 1.5, 0.90),
}
TOP_T, LEG, INSET = 0.05, 0.06, 0.06
TOP = "0.0 0.3 0.9 1"
LEGC = "0.12 0.12 0.15 1"


def part(kind, name, pose, size, rgb):
    sx, sy, sz = size
    geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    mat = f"<material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>"
    if kind == "col":
        return f'<collision name="{name}_col"><pose>{pose}</pose>{geom}</collision>'
    return f'<visual name="{name}_vis"><pose>{pose}</pose>{geom}{mat}</visual>'


def table(name, x, y, sx, sy, h):
    items = []
    top_pose = f"0 0 {h - TOP_T / 2} 0 0 0"
    top_size = (sx, sy, TOP_T)
    items.append(part("col", "top", top_pose, top_size, TOP))
    items.append(part("vis", "top", top_pose, top_size, TOP))
    lh = h - TOP_T
    lx = sx / 2 - INSET - LEG / 2
    ly = sy / 2 - INSET - LEG / 2
    corners = [(-lx, -ly), (-lx, ly), (lx, -ly), (lx, ly)]
    for k, (dx, dy) in enumerate(corners):
        pose = f"{dx:.3f} {dy:.3f} {lh / 2} 0 0 0"
        size = (LEG, LEG, lh)
        items.append(part("col", f"leg{k}", pose, size, LEGC))
        items.append(part("vis", f"leg{k}", pose, size, LEGC))
    body = "\n        ".join(items)
    return (
        f'<model name="{name}">\n'
        f"      <static>true</static>\n"
        f"      <pose>{x} {y} 0 0 0 0</pose>\n"
        f'      <link name="link">\n        {body}\n      </link>\n'
        f"    </model>"
    )


path = "/ros_ws/src/warehouse_robot/worlds/warehouse_simple.world"
text = open(path).read()
for name, (x, y, sx, sy, h) in TABLES.items():
    pat = re.compile(r'<model name="%s">.*?</model>' % name, re.S)
    text, n = pat.subn(lambda m: table(name, x, y, sx, sy, h), text, count=1)
    print(name, "OK" if n else "NOT FOUND")
open(path, "w").write(text)
