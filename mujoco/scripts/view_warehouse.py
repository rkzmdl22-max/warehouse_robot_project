#!/usr/bin/env python3
"""Interactive 3D MuJoCo Viewer for Realistic Warehouse Simulation Environment."""

import os
import sys
import time

if os.name == 'nt':
    anaconda_path = r'C:\Users\HKIT\anaconda3'
    if os.path.exists(anaconda_path):
        os.add_dll_directory(anaconda_path)
    mujoco_pkg_path = r'C:\Users\HKIT\AppData\Local\Programs\Python\Python313\Lib\site-packages\mujoco'
    if os.path.exists(mujoco_pkg_path):
        os.add_dll_directory(mujoco_pkg_path)

import mujoco
import mujoco.viewer

def main():
    ws_dir = r'c:\work\warehouse_robot_project'
    xml_path = os.path.join(ws_dir, 'mujoco', 'scenes', 'warehouse.xml')

    print("==================================================")
    print("LAUNCHING INTERACTIVE REALISTIC WAREHOUSE VIEWER")
    print(f"Scene XML: {xml_path}")
    print("==================================================")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print(f"Bodies    : {model.nbody}")
    print(f"Geoms     : {model.ngeom}")
    print(f"Materials : {model.nmat}")
    print(f"Sites     : {model.nsite}")
    print("==================================================")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 18.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -30
        viewer.cam.lookat[:] = [0.0, 0.0, 1.0]

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)

if __name__ == "__main__":
    main()
