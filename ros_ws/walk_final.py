import math, subprocess, sys, threading, time
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
sys.path.insert(0, '/ros_ws')
import walk_to_table as W

R, Z = 'tron1a', 0.82
START = (6.5, -5.0)
GOAL = (2.0, -3.0)          # 노란 핸드오버 패드
UR5E = (1.75, -3.737)       # UR5e 받침대
FINAL_YAW = math.atan2(UR5E[1]-GOAL[1], UR5E[0]-GOAL[0])
V, F, HIP, KNEE, KS = 0.30, 1.0, 0.35, 0.5, 1.0   # 무릎이 뒤로 꺾이면 KS=-1.0
J = ["abad_L_Joint","hip_L_Joint","knee_L_Joint","abad_R_Joint","hip_R_Joint","knee_R_Joint"]
S = {'amp': 0.0, 'run': True, 'frame': 'base_Link'}

def sh(*a):
    return subprocess.run(['gz','model','-m',R,*a],capture_output=True,text=True,timeout=8).stdout
def put(x,y,yaw):
    sh('-x',f'{x:.3f}','-y',f'{y:.3f}','-z',str(Z),'-R','0','-P','0','-Y',f'{yaw:.3f}')
def pose():
    v = sh('-p').split(); return float(v[0]), float(v[1]), float(v[5])
def hip():
    o = sh('-i').splitlines()
    for i,l in enumerate(o):
        if 'tron1a::hip_L_Joint' in l:
            for k in o[i+1:i+4]:
                if 'angle:' in k: return float(k.split()[-1])
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi

def gait(pubs):
    t0 = time.time()
    while S['run']:
        a = S['amp']; s = math.sin(2*math.pi*F*(time.time()-t0))
        p = JointTrajectoryPoint()
        p.positions = [0.0, HIP*a*s, KS*KNEE*a*max(0.0,s), 0.0, HIP*a*s, KS*KNEE*a*max(0.0,-s)]
        p.time_from_start = Duration(sec=0, nanosec=20000000)
        m = JointTrajectory(); m.header.frame_id = S['frame']; m.joint_names = J; m.points = [p]
        for pub in pubs: pub.publish(m)
        time.sleep(0.02)

def turn(x,y,y0,y1,w=0.8):
    d = wrap(y1-y0); T = max(abs(d)/w, 0.01); t0 = time.time()
    while True:
        k = min(1.0,(time.time()-t0)/T); put(x,y,y0+d*k)
        if k >= 1.0: break

def main():
    W.START = (START[0],START[1],Z,0.0); W.WAYPOINTS = [GOAL]
    hits = W.check_path(W.obstacles())
    for k,d in hits.items(): print(f'  경고: {k} 경로 최소거리 {d:.2f} m')
    if hits and '--force' not in sys.argv:
        print('경로에 장애물이 있어 중단합니다(--force 로 강행).'); return
    rclpy.init(); node = rclpy.create_node('walk_final'); time.sleep(2.0)
    names = [n for n,_ in node.get_topic_names_and_types() if n.endswith('/set_joint_trajectory')]
    print('관절 토픽:', names)
    if not names:
        print('set_joint_trajectory 토픽이 없습니다. 이 모델에는 관절 플러그인이 없는 것입니다.'); return
    pubs = [node.create_publisher(JointTrajectory,n,10) for n in names]
    threading.Thread(target=gait, args=(pubs,), daemon=True).start()
    try:
        yaw = math.atan2(GOAL[1]-START[1], GOAL[0]-START[0])
        put(START[0],START[1],yaw); time.sleep(0.5)
        ok = None
        for fr in ('base_Link','tron1a::base_Link','base_link'):
            S['frame'] = fr; S['amp'] = 1.0; time.sleep(0.5)
            v = [a for a in (hip() for _ in range(5)) if a is not None]
            print(f'frame_id={fr}  hip_L 변화폭={max(v)-min(v):.3f}' if v else f'frame_id={fr} 각도 읽기 실패')
            if v and max(v)-min(v) > 0.05: ok = fr; break
        if not ok:
            print('다리 관절이 움직이지 않습니다. 걷기를 중단합니다.'); S['amp'] = 0.0; return
        print('다리 동작 확인, frame_id =', ok)
        dx, dy = GOAL[0]-START[0], GOAL[1]-START[1]; D = math.hypot(dx,dy); t0 = time.time()
        while True:
            S['amp'] = min(1.0,(time.time()-t0)/1.0)
            s = min(V*(time.time()-t0), D)
            put(START[0]+dx*s/D, START[1]+dy*s/D, yaw)
            if s >= D: break
        S['amp'] = 0.6
        turn(GOAL[0],GOAL[1],yaw,FINAL_YAW)
        S['amp'] = 0.0; time.sleep(1.5)
        x,y,yw = pose()
        print(f'도착 실제 ({x:.2f}, {y:.2f}, yaw {yw:.2f}) / 목표 ({GOAL[0]}, {GOAL[1]}, yaw {FINAL_YAW:.2f})')
        if math.hypot(x-GOAL[0],y-GOAL[1]) > 0.2: print('경고: 목표와 어긋났습니다(플러그인이 몸통 위치를 잡고 있을 수 있음).')
    finally:
        S['amp'] = 0.0; time.sleep(0.3); S['run'] = False
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass
main()
