source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash 2>/dev/null
pkill -f claude_tron_walk.py; pkill -f spawn_entity; pkill -9 -f gzclient; pkill -9 -f gzserver; sleep 3
cd /ros_ws/mujoco/claude_work; bash claude_gz_up.sh > /tmp/claude_up4.log 2>&1; tail -n 1 /tmp/claude_up4.log
U=/ros_ws/src/warehouse_robot/urdf/claude_tron1a_walk.urdf
ros2 run gazebo_ros spawn_entity.py -entity tron_walk -file $U -x 7.0 -y -7.5 -z 0.7 -Y 2.356 2>&1 | tail -n 2
(python3 claude_tron_walk.py > /tmp/claude_walk.log 2>&1 &)
for t in 1 2 3 4; do sleep 6; echo t=$t; timeout 6 gz model -m tron_walk -p; done
