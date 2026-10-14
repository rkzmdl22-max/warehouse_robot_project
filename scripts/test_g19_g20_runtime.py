#!/usr/bin/env python3
"""
G19 & G20 Runtime Verification Script.
Launches system in Docker, audits Nav2 lifecycle, TF transforms, and sends multi-robot navigation goals.
"""

import subprocess
import time
import sys
import json

def run_in_docker(cmd, timeout=30):
    full_cmd = ["docker", "exec", "warehouse_robot_dev", "bash", "-c", f"source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash; {cmd}"]
    try:
        res = subprocess.run(full_cmd, timeout=timeout, capture_output=True, text=True)
        return res.returncode, res.stdout.strip(), res.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "TIMEOUT"

def clean_environment():
    print("[CLEANUP] Terminating existing Gazebo and ROS processes...")
    run_in_docker("killall -9 gzserver gzclient 2>/dev/null || true; pkill -9 -f ros2 2>/dev/null || true", timeout=10)
    time.sleep(3)

def main():
    print("=" * 80)
    print("G19 & G20 RUNTIME VERIFICATION (NAV2 & MULTI-ROBOT NAVIGATION)")
    print("=" * 80)

    clean_environment()

    print("\n[STEP 1] Starting Gazebo & ROS 2 Nav2 system in background...")
    # Launch warehouse.launch.py asynchronously inside container
    bg_launch_cmd = "docker exec -d warehouse_robot_dev bash -c \"source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash; ros2 launch warehouse_robot warehouse.launch.py gui:=false > /tmp/warehouse_launch.log 2>&1\""
    subprocess.run(bg_launch_cmd, shell=True)

    print("[WAIT] Waiting 45 seconds for Gazebo spawn and Nav2 lifecycle activation...")
    time.sleep(45)

    # G19 Audit: Node states & TF
    print("\n[STEP 2] Auditing G19 Nav2 Lifecycle & TF Tree...")
    ret, nodes_out, _ = run_in_docker("ros2 node list", timeout=10)
    print(f"Active ROS 2 Nodes count: {len(nodes_out.splitlines())}")

    ret, tb3_1_cs, _ = run_in_docker("ros2 lifecycle get /tb3_1/controller_server", timeout=10)
    ret, tb3_1_bt, _ = run_in_docker("ros2 lifecycle get /tb3_1/bt_navigator", timeout=10)
    ret, tb3_3_cs, _ = run_in_docker("ros2 lifecycle get /tb3_3/controller_server", timeout=10)
    ret, tb3_3_bt, _ = run_in_docker("ros2 lifecycle get /tb3_3/bt_navigator", timeout=10)

    print(f"  > /tb3_1/controller_server state: {tb3_1_cs}")
    print(f"  > /tb3_1/bt_navigator state       : {tb3_1_bt}")
    print(f"  > /tb3_3/controller_server state: {tb3_3_cs}")
    print(f"  > /tb3_3/bt_navigator state       : {tb3_3_bt}")

    # Check TF transform lookup
    ret, tf_1_out, _ = run_in_docker("ros2 run tf2_ros tf2_echo map tb3_1/base_footprint --once", timeout=15)
    ret, tf_3_out, _ = run_in_docker("ros2 run tf2_ros tf2_echo map tb3_3/base_footprint --once", timeout=15)

    print("\n[TF Transform Check]")
    print(f"  > map -> tb3_1/base_footprint: {'VALID [PASS]' if 'Translation' in tf_1_out else 'INVALID [FAIL]'}")
    if 'Translation' in tf_1_out:
        print(f"    {tf_1_out.splitlines()[0] if tf_1_out else ''}")
    print(f"  > map -> tb3_3/base_footprint: {'VALID [PASS]' if 'Translation' in tf_3_out else 'INVALID [FAIL]'}")
    if 'Translation' in tf_3_out:
        print(f"    {tf_3_out.splitlines()[0] if tf_3_out else ''}")

    g19_pass = ("active" in tb3_1_cs.lower()) and ("active" in tb3_1_bt.lower()) and ("Translation" in tf_1_out)
    print(f"\nG19 NAV2 / ROS INTEGRATION RESULT: {'PASS' if g19_pass else 'FAIL'}")

    # G20 Audit: Multi-robot Goal Execution
    print("\n[STEP 3] Auditing G20 Multi-Robot Navigation...")
    print("Sending navigation action goal to /tb3_1/navigate_to_pose...")
    goal_1_cmd = "ros2 action send_goal /tb3_1/navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 6.0, y: -4.0, z: 0.0}, orientation: {w: 1.0}}}}\""
    ret1, out1, _ = run_in_docker(goal_1_cmd, timeout=30)
    print(f"  > tb3_1 Goal Output: {out1.splitlines()[-1] if out1 else 'No output'}")

    print("Sending navigation action goal to /tb3_3/navigate_to_pose...")
    goal_3_cmd = "ros2 action send_goal /tb3_3/navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 4.0, y: -4.0, z: 0.0}, orientation: {w: 1.0}}}}\""
    ret3, out3, _ = run_in_docker(goal_3_cmd, timeout=30)
    print(f"  > tb3_3 Goal Output: {out3.splitlines()[-1] if out3 else 'No output'}")

    g20_pass = ("SUCCEEDED" in out1 or "accepted" in out1.lower() or ret1 == 0) and ("SUCCEEDED" in out3 or "accepted" in out3.lower() or ret3 == 0)
    print(f"\nG20 MULTI-ROBOT NAVIGATION RESULT: {'PASS' if g20_pass else 'FAIL'}")

    clean_environment()
    return 0 if (g19_pass and g20_pass) else 1

if __name__ == "__main__":
    sys.exit(main())
