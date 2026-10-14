#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash 2>/dev/null
LOG=/tmp/demo_launch.log
echo "== 프로세스 =="; pgrep -a "gzserver|gzclient" | cut -c1-70
echo "== controller_manager 노드 =="; ros2 node list --no-daemon 2>/dev/null | grep controller_manager
echo "== 스폰 결과 =="; grep -E "Spawn status" $LOG | sed 's/.*Spawn status: //' | sort | uniq -c
echo "== 경고/에러 요약 =="; grep -E "Skipping joint|update_rate|already loaded|\[ERROR\]" $LOG | cut -c1-160 | sed 's/^[0-9.]* //' | sort | uniq -c | sort -rn | head -12
echo "== 노드 namespace별 개수 =="; ros2 node list --no-daemon 2>/dev/null | awk -F/ '{print "/"$2}' | sort | uniq -c
