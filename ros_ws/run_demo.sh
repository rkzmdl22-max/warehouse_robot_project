#!/bin/bash
source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash 2>/dev/null
for m in tb3_1 tb3_2 tb3_3; do timeout 5 ros2 topic pub -t 5 -r 10 /$m/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; done
timeout 10 gz model -m tb3_1 -x -2.0 -y -2.0 -z 0.02 -R 0 -P 0 -Y 3.1416
timeout 10 gz model -m tb3_2 -x -2.0 -y  2.0 -z 0.02 -R 0 -P 0 -Y 3.1416
timeout 10 gz model -m tb3_3 -x  2.0 -y  0.0 -z 0.02 -R 0 -P 0 -Y 3.1416
timeout 10 gz camera -c gzclient_camera -f tron1a || echo "카메라 따라가기 실패(수동으로 TRON1을 더블클릭하세요)"
sleep 2
python3 /ros_ws/walk_to_table.py
echo "== 터틀봇 위치 (x y z roll pitch yaw)"
for m in tb3_1 tb3_2 tb3_3; do echo -n "$m: "; gz model -m $m -p | head -1; done
