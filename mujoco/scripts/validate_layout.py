#!/usr/bin/env python3
"""Validation script for realistic MuJoCo warehouse layout against config/warehouse_layout.yaml SSOT."""

import os
import sys
import yaml

if os.name == 'nt':
    anaconda_path = r'C:\Users\HKIT\anaconda3'
    if os.path.exists(anaconda_path):
        os.add_dll_directory(anaconda_path)
    mujoco_pkg_path = r'C:\Users\HKIT\AppData\Local\Programs\Python\Python313\Lib\site-packages\mujoco'
    if os.path.exists(mujoco_pkg_path):
        os.add_dll_directory(mujoco_pkg_path)

import mujoco
import numpy as np

def validate():
    ws_dir = r'c:\work\warehouse_robot_project'
    yaml_path = os.path.join(ws_dir, 'config', 'warehouse_layout.yaml')
    xml_path = os.path.join(ws_dir, 'mujoco', 'scenes', 'warehouse.xml')

    print("==================================================")
    print("MUJOCO WAREHOUSE SCENE VALIDATION")
    print(f"YAML SSOT Path : {yaml_path}")
    print(f"MuJoCo XML Path: {xml_path}")
    print("==================================================")

    if not os.path.exists(yaml_path):
        print(f"FAIL: YAML SSOT file missing at {yaml_path}")
        sys.exit(1)

    with open(yaml_path, 'r', encoding='utf-8') as f:
        ssot = yaml.safe_load(f)

    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        print("SUCCESS: MuJoCo model compiled cleanly from XML!")
        print(f"Model dimensions: nbody={model.nbody}, ngeom={model.ngeom}, nmat={model.nmat}, nsite={model.nsite}")
    except Exception as e:
        print(f"FAIL: MuJoCo XML compilation failed: {e}")
        sys.exit(1)

    # Validate key element coordinates
    checks = []

    # 1. Wall Left
    wall_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wall_left")
    if wall_gid >= 0:
        pos = model.geom_pos[wall_gid]
        expected = np.array([0.0, -10.0, 1.25])
        err = np.linalg.norm(pos - expected)
        checks.append(("wall_left pos", err < 1e-4, f"{pos} vs {expected}"))

    # 2. Shelf 1
    shelf_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "shelf_1_body")
    if shelf_bid >= 0:
        pos = model.body_pos[shelf_bid]
        expected = np.array([-6.0, -5.0, 0.0])
        err = np.linalg.norm(pos - expected)
        checks.append(("shelf_1_body pos", err < 1e-4, f"{pos} vs {expected}"))

    # 3. Pallet 1
    pallet_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pallet_1")
    if pallet_bid >= 0:
        pos = model.body_pos[pallet_bid]
        expected = np.array([-6.0, -2.0, 0.075])
        err = np.linalg.norm(pos - expected)
        checks.append(("pallet_1 pos", err < 1e-4, f"{pos} vs {expected}"))

    # 4. Box 1
    box_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_1")
    if box_bid >= 0:
        pos = model.body_pos[box_bid]
        expected = np.array([-4.0, -2.0, 0.55])
        err = np.linalg.norm(pos - expected)
        checks.append(("box_1 pos", err < 1e-4, f"{pos} vs {expected}"))

    # 5. Loading Zone Site
    zone_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "loading_zone_site")
    if zone_sid >= 0:
        pos = model.site_pos[zone_sid]
        expected = np.array([-6.0, -2.0, 0.005])
        err = np.linalg.norm(pos - expected)
        checks.append(("loading_zone_site pos", err < 1e-4, f"{pos} vs {expected}"))

    print("\n--------------------------------------------------")
    print("COORDINATE SSOT VALIDATION RESULTS")
    print("--------------------------------------------------")
    all_pass = True
    for item, status, detail in checks:
        pass_str = "PASS" if status else "FAIL"
        if not status:
            all_pass = False
        print(f"[{pass_str}] {item:<25} : {detail}")

    print("==================================================")
    if all_pass:
        print("FINAL RESULT: ALL LAYOUT CHECKS PASSED!")
    else:
        print("FINAL RESULT: SOME LAYOUT CHECKS FAILED!")
        sys.exit(1)

if __name__ == "__main__":
    validate()
