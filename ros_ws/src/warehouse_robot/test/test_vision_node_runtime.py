#!/usr/bin/env python3
import sys
import time
import rclpy

sys.path.insert(0, '/ros_ws/src/warehouse_robot')
from warehouse_robot.bottle_detector import BottleDetectorNode

def main():
    rclpy.init()
    print("[TEST] Initializing BottleDetectorNode...")
    node = BottleDetectorNode()
    
    print("[TEST] Spinning node for 2 seconds...")
    start_time = time.time()
    while time.time() - start_time < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("[TEST] Checking published topics on ROS graph...")
    topic_list = dict(node.get_topic_names_and_types())
    
    expected_topics = [
        '/detected_bottle_pose',
        '/detected_bottle_stopped',
        '/bottle_detector/log'
    ]
    
    all_ok = True
    for t in expected_topics:
        if t in topic_list:
            print(f"[TEST TOPIC] {t} -> FOUND (Types: {topic_list[t]})")
        else:
            print(f"[TEST TOPIC] {t} -> NOT FOUND")
            all_ok = False
            
    node.destroy_node()
    rclpy.shutdown()
    
    if all_ok:
        print("[TEST RESULT] AI VISION NODE INITIALIZATION PASS")
        sys.exit(0)
    else:
        print("[TEST RESULT] AI VISION NODE INITIALIZATION FAIL")
        sys.exit(1)

if __name__ == '__main__':
    main()
