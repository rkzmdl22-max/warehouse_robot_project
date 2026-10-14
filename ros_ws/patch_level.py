import re
p = '/ros_ws/walk_to_table.py'
s = open(p, encoding='utf-8').read()
def rep(old, new, count=1):
    global s
    assert s.count(old) == count, f'패치 위치를 못 찾음({s.count(old)}개): {old[:40]!r}'
    s = s.replace(old, new)
rep("    v = [float(t) for t in r.stdout.split()[:6]]\n    return v[0], v[1], v[5]\n",
    "    v = [float(t) for t in r.stdout.split()[:6]]\n    global LAST, LAST_T\n    LAST, LAST_T = v, time.time()\n    return v[0], v[1], v[5]\n\n"
    "LAST, LAST_T, _LT, VX = None, 0.0, 0.0, [0.0]\n\n"
    "def level(force=False):      # 위치/yaw 유지(명령 속도로 지연 보정), roll/pitch만 0으로\n"
    "    global _LT\n"
    "    if LAST is None or (not force and time.time() - _LT < 0.5):\n"
    "        return\n"
    "    _LT = time.time()\n"
    "    x, y, z, _, _, yaw = LAST\n"
    "    dt = time.time() - LAST_T + 0.1\n"
    "    x += VX[0] * dt * math.cos(yaw)\n"
    "    y += VX[0] * dt * math.sin(yaw)\n"
    "    subprocess.run(['gz', 'model', '-m', ROBOT, '-x', str(x), '-y', str(y), '-z', str(START[2]),\n"
    "                    '-R', '0', '-P', '0', '-Y', str(yaw)], timeout=5)\n")
rep("                x, y, yaw = pose()\n", "                x, y, yaw = pose(); level()\n", 2)
rep("                send(clamp(0.8*dist, 0.12, V_MAX), clamp(1.0*err, -0.3, 0.3))",
    "                VX[0] = clamp(0.8*dist, 0.12, V_MAX)\n                send(VX[0], clamp(1.0*err, -0.3, 0.3))")
rep("    def stop(sec=3.0):\n", "    def stop(sec=3.0):\n        VX[0] = 0.0\n")
s, n = re.subn(r"(        stop\(3\.0\))[^\n]*\n(        print\('최종 위치)", r"\1\n        pose(); level(True)\n\2", s)
assert n == 1, '종료부 패치 위치를 못 찾음'
open(p, 'w', encoding='utf-8').write(s)
print('패치 완료')
