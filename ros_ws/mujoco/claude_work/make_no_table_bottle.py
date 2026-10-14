import re
W="/ros_ws/mujoco/warehouse/"; S="/ros_ws/mujoco/scenes/"
x=open(W+"claude_tron1_station_walk.xml").read()
new,n=re.subn(r'\n[ \t]*<body name="bottle_target".*?</body>', '', x, count=1, flags=re.S)
assert n==1
open(W+"claude_tron1_station_walk_nobt.xml","w").write(new)
s=open(S+"claude_walk_full_scene.xml").read()
assert "claude_tron1_station_walk.xml" in s
open(S+"claude_walk_full_scene_nobt.xml","w").write(s.replace("claude_tron1_station_walk.xml","claude_tron1_station_walk_nobt.xml"))
p=open("claude_walk_grasp_full.py").read()
p=p.replace("claude_walk_full_scene.xml","claude_walk_full_scene_nobt.xml")
assert 'bt_tgt = B("bottle_target"); ' in p
p=p.replace('bt_tgt = B("bottle_target"); ','')
open("claude_walk_grasp_nobt.py","w").write(p)
print("생성 완료")
