import xml.etree.ElementTree as ET
import sys

tree = ET.parse(sys.argv[1])
root = tree.getroot()

for body in root.iter("body"):
    geoms = body.findall("geom")
    prev = None

    for geom in geoms:
        if prev is not None:
            if prev.attrib.get("name","").startswith("collision_"):
                for key in (
                    "type",
                    "size",
                    "mesh",
                    "rgba",
                    "quat",
                    "fromto",
                    "pos",
                ):
                    if key in prev.attrib and key not in geom.attrib:
                        geom.attrib[key] = prev.attrib[key]
        prev = geom

tree.write(sys.argv[1], encoding="utf-8")
print("Fixed:", sys.argv[1])
