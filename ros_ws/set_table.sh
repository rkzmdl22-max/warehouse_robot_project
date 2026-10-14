#!/bin/bash
W=/ros_ws/src/warehouse_robot/worlds/warehouse_remodel.world
sed -i "845s|<pose>[^<]*</pose>|<pose>$1 $2 0 0 0 $3</pose>|" $W
sed -n '845p' $W
cd /ros_ws && colcon build --packages-select warehouse_robot >/dev/null 2>&1
diff $W /ros_ws/install/warehouse_robot/share/warehouse_robot/worlds/warehouse_remodel.world >/dev/null && echo SAME || echo NOT_SAME
