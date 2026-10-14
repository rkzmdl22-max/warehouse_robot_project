#!/bin/bash
pkill -9 -f gzclient; pkill -9 -f gzserver; pkill -f "ros2 launch"; pkill -f spawn_entity; pkill -f robot_state_publisher; sleep 3
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash 2>/dev/null
export ROBOT_TYPE=PF_TRON1A
nohup ros2 launch warehouse_robot warehouse_remodel.launch.py > /tmp/demo_launch.log 2>&1 &
echo "launch pid $!, 로그: /tmp/demo_launch.log"
sleep ${1:-90}
/ros_ws/check_demo.sh
