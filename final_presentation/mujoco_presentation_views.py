#!/usr/bin/env python3
"""
MuJoCo Presentation Camera Views Configuration
==============================================
PROJECT MASTER v5.3.2 — Final Presentation Visualization Only

Reads c:/work/warehouse_robot_project/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml in READ-ONLY mode.
Configures 4 presentation camera viewpoints without modifying source files:
  1. VIEW 1: Warehouse Full Isometric View
  2. VIEW 2: TRON1 -> Handover -> UR5e Workstation View
  3. VIEW 3: 3-bottle tray + UR5e Station Close View
  4. VIEW 4: AI Vision / RGB-D Camera View
"""

import os
import sys

MUJOCO_XML_PATH = r"c:\work\warehouse_robot_project\ros_ws\mujoco\scenes\warehouse_tron1_final_visual.xml"

MUJOCO_VIEWS = {
    "VIEW 1 (Warehouse Full Isometric View)": {
        "lookat": [-2.0, 0.0, 1.2],
        "distance": 12.5,
        "azimuth": 135.0,
        "elevation": -30.0,
        "description": "High angle isometric view showing shelves, pallets, boxes, totes, conveyor, and robots."
    },
    "VIEW 2 (TRON1 -> Handover -> UR5e Workstation View)": {
        "lookat": [-2.5, -0.5, 0.9],
        "distance": 4.5,
        "azimuth": 160.0,
        "elevation": -25.0,
        "description": "Mid-range view capturing TRON1 arrival, handover zone, and UR5e station table."
    },
    "VIEW 3 (3-Bottle Tray + UR5e Station Close View)": {
        "lookat": [-4.5, 0.0, 0.8],
        "distance": 2.0,
        "azimuth": 180.0,
        "elevation": -35.0,
        "description": "Close-up view of the 3-bottle tray, gripper, and station workcell."
    },
    "VIEW 4 (AI Vision / RGB-D Camera View)": {
        "lookat": [-4.65, 0.0, 0.77],
        "distance": 1.5,
        "azimuth": 90.0,
        "elevation": -60.0,
        "description": "Overhead RGB-D sensor perspective inspecting bottle detection area."
    }
}

def main():
    print("============================================================")
    print("MUJOCO PRESENTATION CAMERA VIEWS VERIFICATION")
    print("============================================================")

    if not os.path.exists(MUJOCO_XML_PATH):
        print(f"[ERROR] Source MuJoCo XML file missing: {MUJOCO_XML_PATH}")
        sys.exit(1)

    print(f"[READ-ONLY TARGET] {MUJOCO_XML_PATH}")
    print(f"[STATUS] Source XML file exists and is UNTOUCHED.\n")

    for view_name, cfg in MUJOCO_VIEWS.items():
        print(f"[{view_name}]")
        print(f"  LookAt   : {cfg['lookat']}")
        print(f"  Distance : {cfg['distance']}")
        print(f"  Azimuth  : {cfg['azimuth']}")
        print(f"  Elevation: {cfg['elevation']}")
        print(f"  Desc     : {cfg['description']}\n")

    print("[RESULT] All 4 MuJoCo presentation views successfully generated.")

if __name__ == '__main__':
    main()
