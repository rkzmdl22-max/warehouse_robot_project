#!/usr/bin/env python3

import os
import shutil
import xml.etree.ElementTree as ET
from xml.dom import minidom


INPUT_XML = "tiago.xml"

OUTPUT_DIR = "output/tiago"


def pretty_xml(element):
    """
    XML indentation
    """
    rough = ET.tostring(element, encoding="utf-8")

    parsed = minidom.parseString(rough)

    return parsed.toprettyxml(
        indent="  ",
        encoding="utf-8"
    )


def save_xml(filename, root):

    path = os.path.join(
        OUTPUT_DIR,
        filename
    )

    with open(path, "wb") as f:
        f.write(pretty_xml(root))

    print(f"[OK] created {path}")


def clone_element(element):

    return ET.fromstring(
        ET.tostring(element)
    )


def main():

    if not os.path.exists(INPUT_XML):
        raise FileNotFoundError(
            f"{INPUT_XML} not found"
        )


    #
    # output 초기화
    #
    if os.path.exists(OUTPUT_DIR):
        shutil.rmtree(OUTPUT_DIR)

    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )


    #
    # 원본 로드
    #
    tree = ET.parse(INPUT_XML)

    root = tree.getroot()


    print(
        f"Loaded MJCF model: {root.attrib}"
    )


    #
    # 기본 MJCF root
    #
    mujoco_root = ET.Element(
        "mujoco",
        root.attrib
    )


    #
    # model option/compiler copy
    #
    for child in root:

        if child.tag in [
            "compiler",
            "option",
            "size",
            "default"
        ]:
            mujoco_root.append(
                clone_element(child)
            )


    #
    # asset 분리
    #
    asset = root.find("asset")

    if asset is not None:

        save_xml(
            "assets.xml",
            asset
        )


        mujoco_root.append(
            clone_element(asset)
        )


    #
    # worldbody 분리
    #
    worldbody = root.find(
        "worldbody"
    )

    if worldbody is not None:

        save_xml(
            "bodies.xml",
            worldbody
        )


        mujoco_root.append(
            clone_element(worldbody)
        )


    #
    # actuator
    #
    actuator = root.find(
        "actuator"
    )

    if actuator is not None:

        save_xml(
            "actuators.xml",
            actuator
        )

        mujoco_root.append(
            clone_element(actuator)
        )


    #
    # sensor
    #
    sensor = root.find(
        "sensor"
    )

    if sensor is not None:

        save_xml(
            "sensors.xml",
            sensor
        )

        mujoco_root.append(
            clone_element(sensor)
        )


    #
    # scene.xml 생성
    #
    save_xml(
        "scene.xml",
        mujoco_root
    )


    #
    # 원본 백업
    #
    backup_dir = "backup"

    os.makedirs(
        backup_dir,
        exist_ok=True
    )


    backup_file = os.path.join(
        backup_dir,
        "tiago_original.xml"
    )


    if not os.path.exists(
        backup_file
    ):

        shutil.copy2(
            INPUT_XML,
            backup_file
        )

        print(
            "[OK] original backup created"
        )


    print("\n===== COMPLETE =====")
    print(
        "MuJoCo TIAGo split finished"
    )


if __name__ == "__main__":
    main()
