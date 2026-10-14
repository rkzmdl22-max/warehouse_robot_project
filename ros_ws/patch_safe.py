p = '/ros_ws/walk_to_table.py'
s = open(p, encoding='utf-8').read()
def rep(old, new):
    global s
    assert s.count(old) == 1, f'패치 위치를 못 찾음({s.count(old)}개): {old[:40]!r}'
    s = s.replace(old, new)
rep("    finally:\n        stop(3.0)\n",
    "    finally:\n        try:\n            stop(3.0)\n        except BaseException:      # Ctrl+C 등으로 rclpy가 죽어도 정지 명령은 보낸다\n"
    "            subprocess.run(['ros2', 'topic', 'pub', '-t', '20', '-r', '10', f'/{ROBOT}/cmd_vel',\n"
    "                            'geometry_msgs/msg/Twist', '{}'], capture_output=True)\n")
rep("        node.destroy_node(); rclpy.shutdown()",
    "        try:\n            node.destroy_node(); rclpy.shutdown()\n        except Exception:\n            pass")
open(p, 'w', encoding='utf-8').write(s)
print('패치3 완료')
