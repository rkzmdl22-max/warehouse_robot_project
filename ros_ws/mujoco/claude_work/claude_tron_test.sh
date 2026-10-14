source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash 2>/dev/null
pkill -f spawn_tron1; pkill -9 -f gzclient; pkill -9 -f gzserver; sleep 3
cd /ros_ws/mujoco/claude_work; bash claude_gz_up.sh > /tmp/claude_up3.log 2>&1
tail -n 1 /tmp/claude_up3.log
(ros2 launch /ros_ws/src/warehouse_robot/launch/spawn_tron1_classic.launch.py > /tmp/claude_tron1h.log 2>&1 &)
(timeout 50 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" > /dev/null 2>&1 &)
for t in 10 18 26 34 42; do sleep 8; echo t=$t; timeout 6 gz model -m tron1a -p; done
