#!/bin/bash
source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash 2>/dev/null
pkill -9 -f gzclient; pkill -9 -f gzserver; pkill -f tb3_fleet; pkill -f tiago_fleet; pkill -f ur5e_station; sleep 2
export GAZEBO_MODEL_PATH=/ros_ws/install/tiago_description/share:$GAZEBO_MODEL_PATH
(gzserver /ros_ws/install/warehouse_robot/share/warehouse_robot/worlds/warehouse_remodel.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/claude_gz.log 2>&1 &)
sleep 12; (gzclient > /tmp/claude_gzc.log 2>&1 &); sleep 5
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: safety_barrier_north}" > /dev/null
(ros2 launch warehouse_robot ur5e_station.launch.py > /tmp/claude_ur5e.log 2>&1 &)
(ros2 launch warehouse_robot tb3_fleet.launch.py > /tmp/claude_tb3.log 2>&1 &)
sleep 45; echo UP
