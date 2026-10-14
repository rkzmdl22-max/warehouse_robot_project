#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET
import math


BASE_DIR = Path("/ros_ws/mjcf_convert/fr3")
URDF_PATH = Path("/ros_ws/mjcf_convert/fr3.urdf")
OUTPUT_DIR = BASE_DIR / "output"
MESH_DIR = BASE_DIR / "meshes"
OUTPUT_XML = OUTPUT_DIR / "fr3.xml"


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def parse_xyz(value):
    if not value:
        return [0.0, 0.0, 0.0]
    return [float(x) for x in value.split()]


def fmt(values):
    return " ".join(f"{v:.12g}" for v in values)


def rpy_to_quat(rpy):
    """
    URDF RPY -> MuJoCo quaternion.
    Returns [w, x, y, z].
    """
    r, p, y = parse_xyz(rpy)

    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, yy, z]


def add_xyz_rpy(parent, element, origin):
    if origin is None:
        return

    xyz = origin.get("xyz")
    rpy = origin.get("rpy")

    if xyz:
        element.set("pos", xyz)

    if rpy:
        quat = rpy_to_quat(rpy)
        element.set("quat", fmt(quat))


# ---------------------------------------------------------------------------
# URDF parsing
# ---------------------------------------------------------------------------

def parse_urdf():
    root = ET.parse(URDF_PATH).getroot()

    links = {}
    joints = {}

    for link in root.findall("link"):
        name = link.get("name")

        data = {
            "mass": None,
            "inertial_xyz": [0.0, 0.0, 0.0],
            "inertial_rpy": [0.0, 0.0, 0.0],
            "inertia": {},
            "collision": [],
        }

        inertial = link.find("inertial")

        if inertial is not None:
            mass = inertial.find("mass")
            origin = inertial.find("origin")
            inertia = inertial.find("inertia")

            if mass is not None:
                data["mass"] = float(mass.get("value"))

            if origin is not None:
                data["inertial_xyz"] = parse_xyz(
                    origin.get("xyz", "0 0 0")
                )
                data["inertial_rpy"] = parse_xyz(
                    origin.get("rpy", "0 0 0")
                )

            if inertia is not None:
                for key in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                    data["inertia"][key] = float(inertia.get(key))

        for collision in link.findall("collision"):
            origin = collision.find("origin")
            geometry = collision.find("geometry")

            if geometry is None:
                continue

            mesh = geometry.find("mesh")
            box = geometry.find("box")

            item = {
                "origin": origin,
                "mesh": None,
                "box": None,
            }

            if mesh is not None:
                filename = mesh.get("filename", "")
                filename = Path(filename).name
                item["mesh"] = filename

            if box is not None:
                item["box"] = parse_xyz(
                    box.get("size", "0 0 0")
                )

            data["collision"].append(item)

        links[name] = data

    for joint in root.findall("joint"):
        name = joint.get("name")

        data = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "origin": joint.find("origin"),
            "axis": None,
            "limit": {},
            "mimic": None,
        }

        axis = joint.find("axis")
        if axis is not None:
            data["axis"] = parse_xyz(axis.get("xyz", "0 0 1"))

        limit = joint.find("limit")
        if limit is not None:
            data["limit"] = limit.attrib.copy()

        mimic = joint.find("mimic")
        if mimic is not None:
            data["mimic"] = mimic.attrib.copy()

        joints[name] = data

    return links, joints


# ---------------------------------------------------------------------------
# MJCF construction
# ---------------------------------------------------------------------------

def build_mjcf(links, joints):

    mujoco = ET.Element(
        "mujoco",
        {
            "model": "fr3",
        },
    )

    # Compiler / options
    compiler = ET.SubElement(
        mujoco,
        "compiler",
        {
            "angle": "radian",
            "meshdir": "../meshes",
        },
    )

    option = ET.SubElement(
        mujoco,
        "option",
        {
            "timestep": "0.002",
            "iterations": "50",
            "integrator": "implicitfast",
        },
    )

    size = ET.SubElement(
        mujoco,
        "size",
        {
            "njmax": "1000",
            "nconmax": "200",
        },
    )

    # -----------------------------------------------------------------------
    # Assets
    # -----------------------------------------------------------------------

    asset = ET.SubElement(mujoco, "asset")

    for link_name in [
        "fr3_link0",
        "fr3_link1",
        "fr3_link2",
        "fr3_link3",
        "fr3_link4",
        "fr3_link5",
        "fr3_link6",
        "fr3_link7",
        "fr3_hand",
    ]:
        if link_name not in links:
            continue

        for collision in links[link_name]["collision"]:
            mesh = collision["mesh"]

            if mesh is None:
                continue

            mesh_path = MESH_DIR / mesh

            if not mesh_path.exists():
                print(f"[WARN] Missing mesh: {mesh_path}")
                continue

            ET.SubElement(
                asset,
                "mesh",
                {
                    "name": f"{link_name}_mesh",
                    "file": mesh,
                },
            )

    # -----------------------------------------------------------------------
    # World
    # -----------------------------------------------------------------------

    worldbody = ET.SubElement(mujoco, "worldbody")

    world = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "fr3_base",
            "pos": "0 0 0",
        },
    )

    # -----------------------------------------------------------------------
    # Recursive body construction
    # -----------------------------------------------------------------------

    children = {}

    for joint_name, joint in joints.items():
        parent = joint["parent"]
        children.setdefault(parent, []).append((joint_name, joint))

    def add_link_body(parent_body, link_name, incoming_joint=None):

        if link_name not in links:
            return

        data = links[link_name]

        attrs = {
            "name": link_name,
        }

        if incoming_joint is not None:
            add_xyz_rpy(
                parent_body,
                None,
                None,
            )

            origin = incoming_joint["origin"]

            if origin is not None:
                xyz = origin.get("xyz")
                rpy = origin.get("rpy")

                if xyz:
                    attrs["pos"] = xyz

                if rpy:
                    attrs["quat"] = fmt(
                        rpy_to_quat(rpy)
                    )

        body = ET.SubElement(
            parent_body,
            "body",
            attrs,
        )

        # Inertial
        if data["mass"] is not None:

            inertial_attrs = {
                "mass": str(data["mass"]),
                "pos": fmt(data["inertial_xyz"]),
            }

            # MuJoCo supports full inertia via diaginertia only.
            # Keep exact diagonal terms for first physical validation.
            inertia = data["inertia"]

            if inertia:
                inertial_attrs["diaginertia"] = fmt([
                    inertia["ixx"],
                    inertia["iyy"],
                    inertia["izz"],
                ])

            ET.SubElement(
                body,
                "inertial",
                inertial_attrs,
            )

        # Collision geoms
        for index, collision in enumerate(data["collision"]):

            mesh = collision["mesh"]

            if mesh:
                mesh_path = MESH_DIR / mesh

                if mesh_path.exists():
                    geom = ET.SubElement(
                        body,
                        "geom",
                        {
                            "name": f"{link_name}_collision_{index}",
                            "type": "mesh",
                            "mesh": f"{link_name}_mesh",
                            "contype": "1",
                            "conaffinity": "1",
                            "friction": "0.8 0.1 0.1",
                            "density": "0",
                        },
                    )

                    add_xyz_rpy(
                        geom,
                        geom,
                        collision["origin"],
                    )

            elif collision["box"]:
                geom = ET.SubElement(
                    body,
                    "geom",
                    {
                        "name": f"{link_name}_collision_{index}",
                        "type": "box",
                        "size": fmt([
                            x / 2.0 for x in collision["box"]
                        ]),
                        "density": "0",
                    },
                )

                add_xyz_rpy(
                    geom,
                    geom,
                    collision["origin"],
                )

        # Incoming joint
        if incoming_joint is not None:

            joint_name = None

            for candidate_name, candidate in joints.items():
                if candidate is incoming_joint:
                    joint_name = candidate_name
                    break

            if joint_name is not None:

                joint_type = incoming_joint["type"]

                if joint_type == "revolute":

                    limit = incoming_joint["limit"]

                    attrs = {
                        "name": joint_name,
                        "type": "hinge",
                        "axis": fmt(incoming_joint["axis"]),
                    }

                    if "lower" in limit and "upper" in limit:
                        attrs["range"] = (
                            f'{limit["lower"]} {limit["upper"]}'
                        )
                        attrs["limited"] = "true"

                    if "damping" not in attrs:
                        attrs["damping"] = "1.0"

                    ET.SubElement(
                        body,
                        "joint",
                        attrs,
                    )

                elif joint_type == "prismatic":

                    limit = incoming_joint["limit"]

                    attrs = {
                        "name": joint_name,
                        "type": "slide",
                        "axis": fmt(incoming_joint["axis"]),
                    }

                    if "lower" in limit and "upper" in limit:
                        attrs["range"] = (
                            f'{limit["lower"]} {limit["upper"]}'
                        )
                        attrs["limited"] = "true"

                    attrs["damping"] = "5.0"

                    ET.SubElement(
                        body,
                        "joint",
                        attrs,
                    )

        # Children
        for child_joint_name, child_joint in children.get(
            link_name, []
        ):
            child_link = child_joint["child"]

            # Skip fixed sensor-only links in first physical model
            if "accelerometer" in child_link:
                continue

            add_link_body(
                body,
                child_link,
                child_joint,
            )

    # Start with link0
    add_link_body(
        world,
        "fr3_link0",
        joints.get("fr3_base_joint"),
    )

    # -----------------------------------------------------------------------
    # Actuators
    # -----------------------------------------------------------------------

    actuator = ET.SubElement(mujoco, "actuator")

    for joint_name in [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
    ]:
        joint = joints[joint_name]
        effort = joint["limit"].get("effort", "50")

        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"{joint_name}_motor",
                "joint": joint_name,
                "ctrlrange": f"-{effort} {effort}",
                "gear": "1",
            },
        )

    # Gripper master actuator
    ET.SubElement(
        actuator,
        "motor",
        {
            "name": "fr3_gripper",
            "joint": "fr3_finger_joint1",
            "ctrlrange": "0 100",
            "gear": "1",
        },
    )

    # -----------------------------------------------------------------------
    # Sensors
    # -----------------------------------------------------------------------

    sensor = ET.SubElement(mujoco, "sensor")

    for joint_name in [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
        "fr3_finger_joint1",
    ]:
        ET.SubElement(
            sensor,
            "jointpos",
            {
                "name": f"{joint_name}_position",
                "joint": joint_name,
            },
        )

    # -----------------------------------------------------------------------
    # Compiler output
    # -----------------------------------------------------------------------

    return mujoco


# ---------------------------------------------------------------------------
# XML formatting
# ---------------------------------------------------------------------------

def indent_xml(element, level=0):

    indent = "\n" + level * "  "

    if len(element):

        if not element.text or not element.text.strip():
            element.text = indent + "  "

        for child in element:
            indent_xml(child, level + 1)

        if not child.tail or not child.tail.strip():
            child.tail = indent

    if level and (
        not element.tail or not element.tail.strip()
    ):
        element.tail = indent


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():

    print("=" * 70)
    print("FR3 MJCF GENERATOR")
    print("=" * 70)

    print(f"URDF    : {URDF_PATH}")
    print(f"Mesh    : {MESH_DIR}")
    print(f"Output  : {OUTPUT_XML}")

    if not URDF_PATH.exists():
        raise FileNotFoundError(
            f"URDF not found: {URDF_PATH}"
        )

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    links, joints = parse_urdf()

    print()
    print(f"Links  : {len(links)}")
    print(f"Joints : {len(joints)}")

    mujoco = build_mjcf(
        links,
        joints,
    )

    indent_xml(mujoco)

    tree = ET.ElementTree(mujoco)

    tree.write(
        OUTPUT_XML,
        encoding="utf-8",
        xml_declaration=True,
    )

    print()
    print(f"[OK] Generated: {OUTPUT_XML}")
    print("=" * 70)


if __name__ == "__main__":
    main()
