#!/bin/bash
# 보행 + 터틀봇 자율주행 동시 실행. TAIL=보행 종료 후 터틀봇을 더 돌릴 초(기본 5)
TAIL=${TAIL:-5}
source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash 2>/dev/null
cd /ros_ws/mujoco/claude_work
WALK=$(find /ros_ws -name walk_final.py -not -path '*/install/*' | head -1)
[ -z "$WALK" ] && { echo "walk_final.py 없음"; exit 1; }
NAV=""
stop(){ [ -n "$NAV" ] && kill $NAV 2>/dev/null; for i in 1 2 3; do ros2 topic pub --once /tb3_$i/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; done; }
trap stop EXIT
echo "호스트에서 녹화를 켜세요. 5초 뒤 동시 출발"
for s in 5 4 3 2 1; do echo $s; sleep 1; done
python3 claude_tb3_nav.py > nav_run.log 2>&1 &
NAV=$!
python3 "$WALK" &
WPID=$!
sleep 3
kill -0 $NAV 2>/dev/null || { echo "터틀봇 노드가 죽음:"; tail -5 nav_run.log; }
wait $WPID
echo "보행 끝. 터틀봇 ${TAIL}초 더 주행"
sleep $TAIL
stop; sleep 1
echo "== 터틀봇 자세 (roll/pitch 30도 이상이면 뒤집힘)"
for i in 1 2 3; do
  ros2 topic echo --once /tb3_$i/odom 2>/dev/null | python3 -c "
import sys,re,math
s=sys.stdin.read()
g=lambda sec,k: float(re.search(sec+r':.*?'+k+r': (-?[\d.e-]+)',s,re.S).group(1))
qx,qy,qz,qw=[g('orientation',k) for k in 'xyzw']
print('tb3_$i','x=%.2f y=%.2f'%(g('position','x'),g('position','y')),'roll=%.1f pitch=%.1f'%(math.degrees(math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))),math.degrees(math.asin(max(-1,min(1,2*(qw*qy-qz*qx)))))))" || echo "tb3_$i 읽기 실패"
done
