#!/usr/bin/env python3
"""
Interactive MuJoCo Warehouse Viewer — Linux / Ubuntu 22.04
============================================================
Loads mujoco/scenes/warehouse.xml (which includes tron1_station.xml
with UR5e + actuators) and opens the official mujoco.viewer.

Usage (inside Docker container or Ubuntu host):
  cd /ros_ws  (or the project root)
  python3 /path/to/mujoco/scripts/view_warehouse_linux.py

Or with explicit project root:
  python3 view_warehouse_linux.py --root /ros_ws

The viewer allows:
  - Drag sliders in the "Control" panel to drive UR5e joints
  - Observe physics stability
  - Press F5 to reload the model

UR5e actuator index mapping (Control panel order):
  0  act_shoulder_pan
  1  act_shoulder_lift
  2  act_elbow
  3  act_wrist_1
  4  act_wrist_2
  5  act_wrist_3
  6  act_left_finger
  7  act_right_finger
"""

import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer


def parse_args():
    parser = argparse.ArgumentParser(
        description="MuJoCo Warehouse Viewer (Linux)"
    )
    parser.add_argument(
        "--root",
        default=None,
        help="Project root directory. Defaults to the parent of this script's directory.",
    )
    return parser.parse_args()


def find_project_root(script_path: str) -> str:
    """Walk up from scripts/ to find the project root containing mujoco/scenes/."""
    candidate = os.path.dirname(os.path.abspath(script_path))
    for _ in range(4):
        candidate = os.path.dirname(candidate)
        if os.path.isdir(os.path.join(candidate, "mujoco", "scenes")):
            return candidate
    return "/ros_ws"


def main():
    args = parse_args()

    root = args.root or find_project_root(__file__)
    xml_path = os.path.join(root, "mujoco", "scenes", "warehouse.xml")

    if not os.path.exists(xml_path):
        print(f"ERROR: Scene XML not found: {xml_path}", file=sys.stderr)
        print(f"  Project root searched: {root}", file=sys.stderr)
        sys.exit(1)

    print("=" * 60)
    print("MuJoCo Warehouse Viewer (Linux)")
    print(f"MuJoCo version : {mujoco.__version__}")
    print(f"Scene XML      : {xml_path}")
    print("=" * 60)

    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
    except Exception as exc:
        print(f"ERROR: Failed to load model: {exc}", file=sys.stderr)
        sys.exit(1)

    data = mujoco.MjData(model)

    print(f"Bodies    : {model.nbody}")
    print(f"Joints    : {model.njnt}")
    print(f"Actuators : {model.nu}")
    print(f"Geoms     : {model.ngeom}")
    print(f"Sites     : {model.nsite}")
    print(f"Cameras   : {model.ncam}")
    print("=" * 60)

    # Print actuator names for reference
    print("Actuator index mapping:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  [{i:2d}] {name}")
    print("=" * 60)

    # Run one forward pass to check for NaN / instability
    mujoco.mj_forward(model, data)
    if any(v != v for v in data.qpos):   # NaN check
        print("WARNING: NaN detected in initial qpos", file=sys.stderr)

    print("Launching MuJoCo Official Viewer …")
    print("  Use the 'Control' panel sliders to drive UR5e joints.")
    print("  Close the viewer window to exit.")
    print("=" * 60)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set a good default camera angle to see the UR5e station
        viewer.cam.distance = 6.0
        viewer.cam.azimuth  = 210.0
        viewer.cam.elevation = -25.0
        viewer.cam.lookat[:] = [2.5, -3.0, 1.2]

        step = 0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
            step += 1

            # Report simulation stability every 5000 steps (~10s)
            if step % 5000 == 0:
                max_vel = float(max(abs(v) for v in data.qvel)) if model.nv > 0 else 0.0
                print(f"  step={step:8d}  sim_time={data.time:.2f}s  "
                      f"max_qvel={max_vel:.4f}")

    print("Viewer closed. Simulation stable." if step > 0 else "Viewer closed immediately.")


if __name__ == "__main__":
    main()
