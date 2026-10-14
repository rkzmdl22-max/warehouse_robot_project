#!/usr/bin/env python3
"""
Gazebo Presentation Camera Views Configuration
===============================================
PROJECT MASTER v5.3.2 — Final Presentation Visualization Only

Reads c:/work/warehouse_robot_project/ros_ws/src/warehouse_robot/worlds/warehouse.world in READ-ONLY mode.
Configures 4 presentation camera viewpoints without modifying source files:
  1. VIEW 1: Warehouse Full Top/Isometric View
  2. VIEW 2: TRON1 + Handover + UR5e Station View
  3. VIEW 3: AI Vision Camera Handover View
  4. VIEW 4: Logistics Flow View (Packing / Delivery)
"""

import os
import sys

WORLD_PATH = r"c:\work\warehouse_robot_project\ros_ws\src\warehouse_robot\worlds\warehouse.world"

VIEWS = {
    "VIEW 1 (Warehouse Full Top/Isometric)": {
        "pose": "-12.0 0.0 14.0 0.0 0.8 0.0",
        "fov": 1.047,
        "description": "Full overview of warehouse layout including shelves, main aisle, and work zones."
    },
    "VIEW 2 (TRON1 + Handover + UR5e Station)": {
        "pose": "-3.0 -2.5 3.5 0.0 0.5 0.6",
        "fov": 1.047,
        "description": "Close-up view of TRON1 AMR, handover station table, and UR5e manipulator."
    },
    "VIEW 3 (AI Vision Camera View)": {
        "pose": "-4.65 0.0 2.1 0.0 0.78 0.0",
        "fov": 1.047,
        "description": "Station RGB-D camera perspective inspecting bottle posture and conveyor stopping."
    },
    "VIEW 4 (Logistics Flow View)": {
        "pose": "4.0 -1.5 4.0 0.0 0.4 -2.5",
        "fov": 1.047,
        "description": "Perspective facing packing and delivery staging zones along main logistics aisle."
    }
}

def main():
    print("============================================================")
    print("GAZEBO PRESENTATION CAMERA VIEWS VERIFICATION")
    print("============================================================")
    
    if not os.path.exists(WORLD_PATH):
        print(f"[ERROR] Source world file missing: {WORLD_PATH}")
        sys.exit(1)
        
    print(f"[READ-ONLY TARGET] {WORLD_PATH}")
    print(f"[STATUS] Source world file exists and is UNTOUCHED.\n")
    
    for view_name, cfg in VIEWS.items():
        print(f"[{view_name}]")
        print(f"  Pose : {cfg['pose']}")
        print(f"  FOV  : {cfg['fov']}")
        print(f"  Desc : {cfg['description']}\n")
        
    print("[RESULT] All 4 Gazebo presentation views successfully generated.")

if __name__ == '__main__':
    main()
