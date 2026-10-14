import re, mujoco
W="/ros_ws/mujoco/warehouse/"; S="/ros_ws/mujoco/scenes/"
src=open(W+"claude_tron1_station_handover.xml").read()
src,c=re.subn(r'(<body name="base_Link" pos="0 -3 0\.82">)',
  r'\1\n      <joint name="tron1_slide_x" type="slide" axis="1 0 0" range="-3.2 0.3" damping="50" />', src, count=1)
assert c==1, "base_Link 못 찾음"
def nocol(mt):
    t=mt.group(0)
    return t if "contype" in t else t.replace('_collision"','_collision" contype="0" conaffinity="0"',1)
src=re.sub(r'<geom name="(?:abad|hip|knee|foot)_[LR]_collision"[^>]*>', nocol, src)
def fr(mt):
    t=mt.group(0)
    return t if "friction" in t else t.replace('name="carry_tray_collision"','name="carry_tray_collision" friction="1.5 0.05 0.01"',1)
src=re.sub(r'<geom name="carry_tray_collision"[^>]*>', fr, src)
legs=[("abad_L",-0.38397,1.39626),("hip_L",-1.01229,1.39626),("knee_L",-0.872665,1.36136),
      ("abad_R",-1.39626,0.38397),("hip_R",-1.39626,1.01229),("knee_R",-1.36136,0.872665)]
act='    <position name="act_tron1_x" joint="tron1_slide_x" kp="20000" kv="2500" ctrlrange="-3.2 0.3" />\n'
for n,lo,hi in legs:
    act+=f'    <position name="act_leg_{n}" joint="{n}_Joint" kp="200" kv="10" ctrlrange="{lo} {hi}" />\n'
assert "  </actuator>" in src
src=src.replace("  </actuator>", act+"  </actuator>",1)
open(W+"claude_tron1_station_walk.xml","w").write(src)
sc=open(S+"claude_clean_scene.xml").read()
assert "claude_tron1_station_handover.xml" in sc
open(S+"claude_walk_scene.xml","w").write(sc.replace("claude_tron1_station_handover.xml","claude_tron1_station_walk.xml"))
m=mujoco.MjModel.from_xml_path(S+"claude_walk_scene.xml")
print("로드 성공 nq",m.nq,"nu",m.nu)
