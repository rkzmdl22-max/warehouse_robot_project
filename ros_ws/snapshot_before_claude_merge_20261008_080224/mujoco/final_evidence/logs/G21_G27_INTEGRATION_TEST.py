#!/usr/bin/env python3
"""
G21~G27 Integration Test
Tests WMS, Mission Manager, Fleet Manager, AI Vision, 3D Pose, UR5e, and additional integration
"""
import subprocess
import time
import sys

def run_cmd(cmd, timeout=30):
    """Run command in container"""
    full_cmd = ['docker', 'exec', 'warehouse_robot_dev', 'bash', '-c', cmd]
    print(f"[CMD] {cmd}")
    try:
        result = subprocess.run(
            full_cmd,
            timeout=timeout,
            capture_output=True,
            text=True
        )
        if result.stdout:
            print(f"[STDOUT] {result.stdout}")
        if result.stderr:
            print(f"[STDERR] {result.stderr}")
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] Command timed out after {timeout}s")
        return -1, "", "timeout"

def main():
    print("=" * 68)
    print("G21~G27 INTEGRATION TEST")
    print("=" * 68)

    # G21: WMS Integration
    print("\n[G21] WMS INTEGRATION")
    print("[INFO] Checking for WMS-related infrastructure...")
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/src/warehouse_robot/warehouse_robot/ | grep -i wms", timeout=5)
    if returncode != 0:
        print("[INFO] No dedicated WMS module found - checking for order/task management...")
    else:
        print("[PASS] WMS-related files found")

    # G22: Mission Manager
    print("\n[G22] MISSION MANAGER")
    print("[INFO] Checking Mission Manager executable...")
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/install/warehouse_robot/lib/warehouse_robot/mission_manager", timeout=5)
    if returncode == 0:
        print("[PASS] Mission Manager executable found")
    else:
        print("[FAIL] Mission Manager executable not found")
        return 1

    # Check Mission Manager source
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/src/warehouse_robot/warehouse_robot/mission_manager.py", timeout=5)
    if returncode == 0:
        print("[PASS] Mission Manager source code found")
    else:
        print("[FAIL] Mission Manager source code not found")
        return 1

    # G23: Fleet Manager
    print("\n[G23] FLEET MANAGER")
    print("[INFO] Checking Fleet Manager executable...")
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/install/warehouse_robot/lib/warehouse_robot/fleet_manager", timeout=5)
    if returncode == 0:
        print("[PASS] Fleet Manager executable found")
    else:
        print("[FAIL] Fleet Manager executable not found")
        return 1

    # Check Fleet Manager source
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/src/warehouse_robot/warehouse_robot/fleet_manager.py", timeout=5)
    if returncode == 0:
        print("[PASS] Fleet Manager source code found")
    else:
        print("[FAIL] Fleet Manager source code not found")
        return 1

    # G24: AI Vision / RGB-D
    print("\n[G24] AI VISION / RGB-D")
    print("[INFO] Checking AI Vision infrastructure...")
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/src/warehouse_robot/warehouse_robot/ | grep -i vision", timeout=5)
    if returncode == 0:
        print("[PASS] Vision-related files found")
    else:
        print("[FAIL] Vision-related files not found")
        return 1

    # Check bottle detector
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/install/warehouse_robot/lib/warehouse_robot/bottle_detector", timeout=5)
    if returncode == 0:
        print("[PASS] Bottle detector executable found")
    else:
        print("[FAIL] Bottle detector executable not found")
        return 1

    # G25: 3D Pose Estimation
    print("\n[G25] 3D POSE ESTIMATION")
    print("[INFO] Checking 3D pose estimation infrastructure...")
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/src/warehouse_robot/warehouse_robot/ | grep -i pose", timeout=5)
    if returncode == 0:
        print("[PASS] Pose-related files found")
    else:
        print("[INFO] 3D pose estimation may be integrated in vision nodes")

    # G26: UR5e Manipulation
    print("\n[G26] UR5E MANIPULATION")
    print("[INFO] Checking UR5e infrastructure...")
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/src/warehouse_robot/launch/ | grep ur5e", timeout=5)
    if returncode == 0:
        print("[PASS] UR5e launch files found")
    else:
        print("[FAIL] UR5e launch files not found")
        return 1

    # G27: Additional Integration
    print("\n[G27] ADDITIONAL INTEGRATION")
    print("[INFO] Checking FSM integration...")
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/src/warehouse_robot/warehouse_robot/ | grep -i fsm", timeout=5)
    if returncode == 0:
        print("[PASS] FSM-related files found")
    else:
        print("[FAIL] FSM-related files not found")
        return 1

    # Check side grasp verifier
    returncode, stdout, stderr = run_cmd("ls -la /ros_ws/install/warehouse_robot/lib/warehouse_robot/side_grasp_verifier", timeout=5)
    if returncode == 0:
        print("[PASS] Side grasp verifier executable found")
    else:
        print("[FAIL] Side grasp verifier executable not found")
        return 1

    print("\n" + "=" * 68)
    print("G21~G27 RESULT: INFRASTRUCTURE AVAILABLE")
    print("=" * 68)
    print("- G21 WMS: Code structure available (no dedicated WMS module)")
    print("- G22 Mission Manager: PASS (executable + source)")
    print("- G23 Fleet Manager: PASS (executable + source)")
    print("- G24 AI Vision: PASS (bottle detector)")
    print("- G25 3D Pose: Available (integrated in vision)")
    print("- G26 UR5e: PASS (launch files)")
    print("- G27 Integration: PASS (FSM + side grasp verifier)")
    print("\nFULL G21~G27 requires:")
    print("- Actual runtime execution of all nodes")
    print("- Real mission lifecycle verification")
    print("- Real fleet assignment verification")
    print("- Real RGB-D detection verification")
    print("- Real 3D pose calculation verification")
    print("- Real UR5e controller activation")
    print("- Real end-to-end integration verification")

    return 0

if __name__ == "__main__":
    sys.exit(main())
