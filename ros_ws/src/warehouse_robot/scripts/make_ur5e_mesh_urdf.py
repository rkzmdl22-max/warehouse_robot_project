#!/usr/bin/env python3
import os, re, shutil, subprocess, sys
import xml.etree.ElementTree as ET

PKG = "/ros_ws/src/warehouse_robot"
UR = "/ros_ws/_public_models/ur_description"
SRC = f"{PKG}/urdf/ur5e_station.urdf.backup_before_ur5e_final"
DST = f"{PKG}/urdf/ur5e_station.urdf"

# 1) 메시 복사
for kind in ("visual", "collision"):
    d = f"{PKG}/meshes/ur5e/{kind}"
    if not os.path.isdir(d):
        os.makedirs(os.path.dirname(d), exist_ok=True)
        shutil.copytree(f"{UR}/meshes/ur5e/{kind}", d)

# 2) xacro가 ur_description을 찾도록 임시 ament 인덱스 생성
fake = "/tmp/fake_ament"
os.makedirs(f"{fake}/share/ament_index/resource_index/packages", exist_ok=True)
open(f"{fake}/share/ament_index/resource_index/packages/ur_description", "w").close()
lnk = f"{fake}/share/ur_description"
if os.path.lexists(lnk):
    os.remove(lnk)
os.symlink(UR, lnk)
env = dict(os.environ)
env["AMENT_PREFIX_PATH"] = fake + ":" + env.get("AMENT_PREFIX_PATH", "")
r = subprocess.run(["xacro", f"{UR}/urdf/ur.urdf.xacro", "ur_type:=ur5e", "name:=ur5e"],
                   capture_output=True, text=True, env=env)
if r.returncode != 0:
    print("xacro 실패:\n", r.stderr[-1500:]); sys.exit(1)
open("/tmp/ur5e_std.urdf", "w").write(r.stdout)

# 3) 표준 팔을 기존 이름 체계로 변환
LINK_MAP = {"base_link": "ur5e_base_link", "base_link_inertia": "ur5e_base_inertia_link",
            "shoulder_link": "ur5e_shoulder_link", "upper_arm_link": "ur5e_upper_arm_link",
            "forearm_link": "ur5e_forearm_link", "wrist_1_link": "ur5e_wrist_1_link",
            "wrist_2_link": "ur5e_wrist_2_link", "wrist_3_link": "ur5e_wrist_3_link",
            "tool0": "tool0"}
BIG = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"}
ARM = BIG | {"wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
ln = lambda n: LINK_MAP.get(n, "ur5e_" + n)
jn = lambda n: n if n in ARM else "ur5e_" + n.replace("-", "_")
FROM, TO = "package://ur_description/meshes/", f"file://{PKG}/meshes/"

out = []
for el in ET.fromstring(r.stdout):
    if el.tag == "link":
        if el.get("name") in ("world", "tool0"):
            continue
        el.set("name", ln(el.get("name")))
        for vis in el.findall("visual"):
            for m in vis.findall("material"):
                vis.remove(m)
        for mesh in el.iter("mesh"):
            mesh.set("filename", mesh.get("filename").replace(FROM, TO))
        out.append(el)
    elif el.tag == "joint":
        if el.find("parent").get("link") == "world":
            continue
        old = el.get("name")
        el.set("name", jn(old))
        p, c = el.find("parent"), el.find("child")
        p.set("link", ln(p.get("link"))); c.set("link", ln(c.get("link")))
        if old in ARM and el.find("dynamics") is None:
            d = ET.SubElement(el, "dynamics")
            d.set("damping", "0.5" if old in BIG else "0.1"); d.set("friction", "0.0")
        out.append(el)

parts = []
for e in out:
    ET.indent(e, space="  ")
    parts.append("  " + ET.tostring(e, encoding="unicode").strip().replace("\n", "\n  "))
arm_xml = "\n".join(parts)

old = open(SRC).read()
a = re.search(r'<link name="ur5e_base_link">', old)
b = re.search(r'<joint name="wrist_3_to_tool0".*?</joint>', old, re.S)
if not (a and b):
    print("원본에서 교체 구간을 못 찾음"); sys.exit(1)
new = (old[:a.start()] + "<!-- UR5e arm - 표준 ur_description 구조 + 메시 (scripts/make_ur5e_mesh_urdf.py 로 생성) -->\n"
       + arm_xml + old[b.end():])

# 4) 검증
try:
    root = ET.fromstring(new)
except ET.ParseError as e:
    print("생성된 URDF 파싱 실패:", e); sys.exit(1)
links = {l.get("name") for l in root.findall("link")}
bad = []
for j in root.findall("joint"):
    for t in ("parent", "child"):
        n = j.find(t).get("link")
        if n not in links:
            bad.append((j.get("name"), t, n))
for g in root.iter("gazebo"):
    ref = g.get("reference")
    if ref and ref not in links:
        bad.append(("gazebo", "reference", ref))
children = {j.find("child").get("link") for j in root.findall("joint")}
print("루트 링크(world 하나만 나와야 함):", sorted(links - children))
print("없는 링크를 가리키는 참조:", bad or "없음")
miss = [m.get("filename") for m in root.iter("mesh")
        if m.get("filename").startswith("file://") and not os.path.exists(m.get("filename")[7:])]
print("없는 메시 파일:", miss or "없음")

if not os.path.exists(DST + ".primitives_fixed"):
    shutil.copy(DST, DST + ".primitives_fixed")
open(DST, "w").write(new)
print("저장:", DST)
if shutil.which("check_urdf"):
    print(subprocess.run(["check_urdf", DST], capture_output=True, text=True).stdout[:600])
