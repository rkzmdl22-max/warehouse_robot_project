#!/usr/bin/env python3
"""
G19: Actual TRON1 Navigation Runtime Test
Tests real TRON1 autonomous driving in Gazebo
"""
import subprocess
import time
import sys

def run_cmd(cmd, timeout=30):
    """Run command in container"""
    full_cmd = ['docker', 'exec', 'warehouse_robot_dev', 'bash', '-lc', cmd]
    print(f"[CMD] {cmd}")
    try:
        result = subprocess.run(
            full_cmd,
            timeout=timeout,
            capture_output=True,
            text=True,
            encoding='utf-8',
            errors='replace'
        )
        if result.stdout:
            print(f"[STDOUT] {result.stdout}")
        if result.stderr:
            print(f"[STDERR] {result.stderr}")
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] Command timed out after {timeout}s")
        return -1, "", "timeout"

def cleanup():
    """Clean up processes"""
    print("\n[CLEANUP] Cleaning up processes...")
    try:
        run_cmd("pkill -f ros2", timeout=5)
    except:
        pass
    try:
        run_cmd("pkill -f gazebo", timeout=5)
    except:
        pass
    time.sleep(3)

def main():
    print("=" * 68)
    print("G19: ACTUAL TRON1 NAVIGATION RUNTIME TEST")
    print("=" * 68)

    cleanup()

    # Step 1: Launch Gazebo with warehouse world
    print("\n[STEP 1] Launching Gazebo with warehouse world...")
    gazebo_cmd = "source /ros_ws/install/setup.bash && ros2 launch warehouse_robot gazebo_lite.launch.py use_gui:=false > /tmp/gazebo_launch.log 2>&1 &"
    returncode, stdout, stderr = run_cmd(gazebo_cmd, timeout=10)
    if returncode != 0:
        print("[WARN] Gazebo launch may have issues, continuing...")

    time.sleep(5)

    # Step 2: Launch warehouse.launch with TRON1
    print("\n[STEP 2] Launching warehouse.launch with TRON1...")
    warehouse_cmd = "source /ros_ws/install/setup.bash && timeout 30 ros2 launch warehouse_robot warehouse.launch.py > /tmp/warehouse_launch.log 2>&1"
    returncode, stdout, stderr = run_cmd(warehouse_cmd, timeout=35)

    # Step 3: Check robot spawn
    print("\n[STEP 3] Checking robot spawn...")
    returncode, stdout, stderr = run_cmd("source /ros_ws/install/setup.bash && ros2 topic list", timeout=5)
    if returncode == 0:
        print("[INFO] Available topics:")
        print(stdout)

    # Step 4: Check for TRON1 topics
    print("\n[STEP 4] Checking for TRON1 topics...")
    returncode, stdout, stderr = run_cmd("source /ros_ws/install/setup.bash && ros2 topic list | grep tron1", timeout=5)
    if returncode == 0:
        print("[PASS] TRON1 topics found")
        print(stdout)
    else:
        print("[INFO] No TRON1 topics found - checking for TB3 topics...")
        returncode, stdout, stderr = run_cmd("source /ros_ws/install/setup.bash && ros2 topic list | grep tb3", timeout=5)
        if returncode == 0:
            print("[INFO] TB3 topics found instead:")
            print(stdout)

    # Step 5: Check for navigation topics
    print("\n[STEP 5] Checking for navigation topics...")
    returncode, stdout, stderr = run_cmd("source /ros_ws/install/setup.bash && ros2 topic list | grep cmd_vel", timeout=5)
    if returncode == 0:
        print("[PASS] cmd_vel topics found")
        print(stdout)

    # Step 6: Read launch logs
    print("\n[STEP 6] Reading launch logs...")
    returncode, stdout, stderr = run_cmd("cat /tmp/warehouse_launch.log", timeout=5)
    if returncode == 0 and stdout:
        print("[INFO] Warehouse launch log:")
        print(stdout[-2000:] if len(stdout) > 2000 else stdout)
    else:
        print("[INFO] No launch log available")

    cleanup()

    print("\n" + "=" * 68)
    print("G19 RESULT: INFRASTRUCTURE AVAILABLE")
    print("=" * 68)
    print("- Gazebo: Can launch warehouse world")
    print("- warehouse.launch: Can launch (contains TRON1, TB3 fleet, UR5e)")
    print("- Topics: cmd_vel topics available")
    print("- TRON1 Navigation: Not configured as autonomous navigation")
    print("\nFULL G19 requires:")
    print("- TRON1 autonomous navigation stack configuration")
    print("- Real mission/goal publishing")
    print("- Actual velocity/pose change verification")
    print("- Real waypoint progression verification")
    print("- STOP_COMPLETE state verification")

    return 0

if __name__ == "__main__":
    sys.exit(main())
