#!/usr/bin/env python3
"""TRON1을 테이블을 피해 물병 테이블 앞까지 걷게 한다 (폐루프 + 끝에서 Twist 0 반복).
사용: python3 walk_to_table.py --dry   (충돌 검사만)
      python3 walk_to_table.py         (검사 통과 시 실행)"""
import math, subprocess, sys, time
import xml.etree.ElementTree as ET

WORLD = '/ros_ws/src/warehouse_robot/worlds/warehouse_remodel.world'
ROBOT = 'tron1a'
START = (6.5, -5.0, 0.82, 2.49)                 # x, y, z, yaw (분홍=delivery_zone 안)
WAYPOINTS = [(1.26, -1.00), (1.26, -2.40)]       # 마지막 = 물병 테이블 앞 정지 위치
V_MAX, W_MAX, CLEAR = 0.40, 0.40, 0.35           # m/s, rad/s, 로봇 반경+여유(m)
SKIP = ('safety_barrier_north', 'ground_plane', 'marking_', ROBOT, 'tb3_')

def nums(el, n=6):
    p = el.find('pose') if el is not None else None
    v = [float(t) for t in p.text.split()] if p is not None and p.text else []
    return (v + [0.0] * n)[:n]

def tf(pt, pose):
    x, y, _, _, _, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * pt[0] - s * pt[1], y + s * pt[0] + c * pt[1])

def obstacles():
    out = []
    for m in ET.parse(WORLD).getroot().iter('model'):
        name = m.get('name', '')
        if name.startswith(SKIP):
            continue
        mp = nums(m)
        for link in m.iter('link'):
            lp = nums(link)
            for col in link.iter('collision'):
                cp, g = nums(col), col.find('geometry')
                if g is None:
                    continue
                if g.find('box') is not None:
                    sx, sy, _ = [float(t) for t in g.find('box/size').text.split()]
                    poly = [(-sx/2, -sy/2), (sx/2, -sy/2), (sx/2, sy/2), (-sx/2, sy/2)]
                elif g.find('cylinder') is not None:
                    r = float(g.find('cylinder/radius').text)
                    poly = [(r*math.cos(a*math.pi/4), r*math.sin(a*math.pi/4)) for a in range(8)]
                else:
                    continue
                out.append((name, [tf(tf(tf(q, cp), lp), mp) for q in poly]))
    return out

def seg_d(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    t = 0 if dx == dy == 0 else max(0, min(1, ((p[0]-ax)*dx + (p[1]-ay)*dy) / (dx*dx + dy*dy)))
    return math.hypot(p[0] - ax - t*dx, p[1] - ay - t*dy)

def inside(p, poly):
    c, n = False, len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i+1) % n]
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1]) + a[0]:
            c = not c
    return c

def check_path(obs):
    pts, hits = [(START[0], START[1])] + WAYPOINTS, {}
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / 0.05))
        for i in range(n + 1):
            p = (a[0] + (b[0]-a[0])*i/n, a[1] + (b[1]-a[1])*i/n)
            for name, poly in obs:
                d = 0.0 if inside(p, poly) else min(seg_d(p, poly[k], poly[(k+1) % len(poly)]) for k in range(len(poly)))
                if d < CLEAR and name not in ('table_bottle_handover',) or (d < 0.1 and name == 'table_bottle_handover'):
                    hits[name] = min(hits.get(name, 9), d)
    return hits

def pose():
    r = subprocess.run(['gz', 'model', '-m', ROBOT, '-p'], capture_output=True, text=True, timeout=5)
    v = [float(t) for t in r.stdout.split()[:6]]
    global LAST, LAST_T
    LAST, LAST_T = v, time.time()
    return v[0], v[1], v[5]

LAST, LAST_T, _LT, VX = None, 0.0, 0.0, [0.0]

def level(force=False):      # 위치/yaw 유지(명령 속도로 지연 보정), roll/pitch만 0으로
    global _LT
    if LAST is None or (not force and time.time() - _LT < 0.5):
        return
    _LT = time.time()
    x, y, z, _, _, yaw = LAST
    dt = time.time() - LAST_T + 0.1
    x += VX[0] * dt * math.cos(yaw)
    y += VX[0] * dt * math.sin(yaw)
    subprocess.run(['gz', 'model', '-m', ROBOT, '-x', str(x), '-y', str(y), '-z', str(START[2]),
                    '-R', '0', '-P', '0', '-Y', str(yaw)], timeout=5)

def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def main():
    obs = obstacles()
    print(f'충돌 검사 대상 {len(obs)}개 collision')
    hits = check_path(obs)
    for k, d in sorted(hits.items(), key=lambda kv: kv[1]):
        print(f'  경고: {k} 와 경로 최소거리 {d:.2f} m')
    if not hits:
        print('  경로 상 충돌 없음 (반경 %.2f m 기준)' % CLEAR)
    if '--dry' in sys.argv:
        return
    if hits and '--force' not in sys.argv:
        print('충돌 경고가 있어 중단합니다. 웨이포인트를 조정하거나 --force 로 강행하세요.')
        return
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init()
    node = rclpy.create_node('walk_to_table')
    pub = node.create_publisher(Twist, f'/{ROBOT}/cmd_vel', 10)
    def send(vx=0.0, wz=0.0):
        t = Twist(); t.linear.x = vx; t.angular.z = wz; pub.publish(t)
    def stop(sec=3.0):
        VX[0] = 0.0
        for _ in range(int(sec * 20)):
            send(); time.sleep(0.05)
    try:
        stop(1.0)
        subprocess.run(['gz', 'model', '-m', ROBOT, '-x', str(START[0]), '-y', str(START[1]), '-z', str(START[2]),
                        '-R', '0', '-P', '0', '-Y', str(START[3])], timeout=10)
        time.sleep(1.5)
        for gx, gy in WAYPOINTS:
            t0 = time.time()
            while time.time() - t0 < 25:
                x, y, yaw = pose()
                err = wrap(math.atan2(gy - y, gx - x) - yaw)
                if abs(err) < 0.06:
                    break
                send(0.0, math.copysign(clamp(0.8*abs(err), 0.12, W_MAX), err))
            else:
                print('회전이 25초 안에 끝나지 않아 중단 (angular.z가 안 먹는 것일 수 있음)'); break
            stop(0.5)
            pose(); level(True)
            t0 = time.time()
            while time.time() - t0 < 60:
                x, y, yaw = pose(); level()
                dist = math.hypot(gx - x, gy - y)
                if dist < 0.10:
                    break
                err = wrap(math.atan2(gy - y, gx - x) - yaw)
                VX[0] = clamp(0.8*dist, 0.12, V_MAX)
                send(VX[0], clamp(1.0*err, -0.3, 0.3))
            else:
                print('직진이 60초 안에 끝나지 않아 중단'); break
            stop(0.5)
            pose(); level(True)
            print(f'도착: ({gx}, {gy}) 실제 {pose()[0]:.2f}, {pose()[1]:.2f}')
    finally:
        try:
            stop(3.0)
        except BaseException:      # Ctrl+C 등으로 rclpy가 죽어도 정지 명령은 보낸다
            subprocess.run(['ros2', 'topic', 'pub', '-t', '20', '-r', '10', f'/{ROBOT}/cmd_vel',
                            'geometry_msgs/msg/Twist', '{}'], capture_output=True)
        pose(); level(True)
        print('최종 위치(x, y, yaw):', pose())
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
