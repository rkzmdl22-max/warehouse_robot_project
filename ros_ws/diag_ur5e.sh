#!/bin/bash
source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash 2>/dev/null
LOG=/tmp/demo_launch.log
echo "== ur5e 관련 경고/에러 원문 =="; grep -n "ur5e" $LOG | grep -iE "skipping|configur|error|warn|param|already" | cut -c1-230 | head -30
echo "== 중복 노드 =="; ros2 node list --no-daemon 2>/dev/null | sort | uniq -d
echo "== ur5e 컨트롤러 상태 =="; timeout 25 ros2 control list_controllers -c /ur5e/controller_manager 2>&1 | tail -6
echo "== ur5e 하드웨어 인터페이스 =="; timeout 25 ros2 control list_hardware_interfaces -c /ur5e/controller_manager 2>&1 | head -25
echo "== launch에서 ur5e 포함 횟수 =="; grep -c "ur5e_station.launch.py" /ros_ws/src/warehouse_robot/launch/warehouse_remodel.launch.py /ros_ws/src/warehouse_robot/launch/warehouse.launch.py
