p = '/ros_ws/walk_to_table.py'
s = open(p, encoding='utf-8').read()
def rep(old, new):
    global s
    assert s.count(old) == 1, f'패치 위치를 못 찾음({s.count(old)}개): {old[:40]!r}'
    s = s.replace(old, new)
rep("            while time.time() - t0 < 25:\n                x, y, yaw = pose(); level()\n",
    "            while time.time() - t0 < 25:\n                x, y, yaw = pose()\n")
rep("            stop(0.5)\n            t0 = time.time()\n            while time.time() - t0 < 60:",
    "            stop(0.5)\n            pose(); level(True)\n            t0 = time.time()\n            while time.time() - t0 < 60:")
rep("            stop(0.5)\n            print(f'도착",
    "            stop(0.5)\n            pose(); level(True)\n            print(f'도착")
open(p, 'w', encoding='utf-8').write(s)
print('패치2 완료')
