]#!/usr/bin/env python3

import os
import shutil
import xml.etree.ElementTree as ET
from xml.dom import minidom


INPUT_XML = "tiago.xml"

OUTPUT_DIR = "output/tiago"


def pretty_xml(element):

    rough = ET.tostring(
        element,
        encoding="utf-8"
    )

    parsed = minidom.parseString(
        rough
    )

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
        f.write(
            pretty_xml(root)
        )

    print(
        "[OK] created",
        path
    )


def clone_element(element):

    return ET.fromstring(
        ET.tostring(element)
    )


def main():

    if os.path.exists(
        OUTPUT_DIR
    ):
        shutil.rmtree(
            OUTPUT_DIR
        )


    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )


    tree = ET.parse(
        INPUT_XML
    )

    root = tree.getroot()


    print(
        "Loaded MJCF model:",
        root.attrib
    )


    # assets

    asset = root.find(
        "asset"
    )

    if asset is not None:

        save_xml(
            "assets.xml",
            clone_element(asset)
        )


    # bodies

    worldbody = root.find(
        "worldbody"
    )

    if worldbody is not None:

        save_xml(
            "bodies.xml",
            clone_element(worldbody)
        )


    # actuator

    actuator = root.find(
        "actuator"
    )

    if actuator is not None:

        save_xml(
            "actuators.xml",
            clone_element(actuator)
        )


    # sensor

    sensor = root.find(
        "sensor"
    )

    if sensor is not None:

        save_xml(
            "sensors.xml",
            clone_element(sensor)
        )


    #
    # scene 생성
    #

    scene = ET.Element(
        "mujoco",
        root.attrib
    )


    for child in root:

        if child.tag in [
            "size",
            "option",
            "compiler",
            "default"
        ]:

            scene.append(
                clone_element(child)
            )


    for filename in [
        "assets.xml",
        "bodies.xml",
        "actuators.xml",
        "sensors.xml"
    ]:

        scene.append(
            ET.Element(
                "include",
                {
                    "file": filename
                }
            )
        )


    save_xml(
        "scene.xml",
        scene
    )


    print()
    print("===== COMPLETE =====")
    print(
        "MuJoCo TIAGo split finished"
    )


if __name__ == "__main__":
    main()
