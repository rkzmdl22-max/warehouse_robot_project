#!/bin/bash
source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash 2>/dev/null
cd /ros_ws/mujoco/claude_work
pkill -f claude_tron_walk.py; pkill -f claude_tb3_nav; pkill -f spawn_entity; pkill -9 -f gzclient; pkill -9 -f gzserver; sleep 3
bash claude_gz_up.sh > /tmp/claude_up5.log 2>&1; tail -n 1 /tmp/claude_up5.log
U=/ros_ws/src/warehouse_robot/urdf/claude_tron1a_walk.urdf
ros2 run gazebo_ros spawn_entity.py -entity tron_walk -file $U -x 7.0 -y -7.5 -z 0.7 -Y 2.356 2>&1 | tail -n 2
for i in 1 2 3; do ros2 topic list | grep -qx /tb3_$i/cmd_vel || echo "경고: tb3_$i cmd_vel 없음"; done
echo "지금 View > Laser Scans 끄고 호스트 녹화 시작. 10초 뒤 동시 출발"; sleep 10
python3 claude_tb3_nav.py > nav_run.log 2>&1 & NAV=$!
timeout ${WALK_SEC:-40} python3 claude_tron_walk.py > /tmp/claude_walk.log 2>&1 &
WALK=$!
wait $WALK
sleep ${TAIL:-5}
kill $NAV 2>/dev/null
for i in 1 2 3; do ros2 topic pub --once /tb3_$i/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; done
echo "끝. 녹화를 멈추세요."
