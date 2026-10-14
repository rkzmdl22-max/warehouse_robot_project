W="/ros_ws/mujoco/warehouse/"; S="/ros_ws/mujoco/scenes/"
open(W+"claude_estop.xml","w").write('''<mujoco>
  <worldbody>
    <body name="claude_estop" pos="0.9 -2.45 0">
      <geom name="claude_estop_post" type="cylinder" pos="0 0 0.45" size="0.05 0.45" rgba="0.95 0.8 0.1 1" contype="0" conaffinity="0"/>
      <geom name="claude_estop_box" type="box" pos="0 0 0.95" size="0.09 0.09 0.06" rgba="0.95 0.8 0.1 1" contype="0" conaffinity="0"/>
      <geom name="claude_estop_btn" type="cylinder" pos="0 0 1.03" size="0.05 0.025" rgba="0.6 0 0 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
''')
s=open(S+"claude_walk_full_scene_nobt.xml").read(); i=s.rfind("</mujoco>"); assert i>0
open(S+"claude_walk_full_scene_estop.xml","w").write(s[:i]+'  <include file="../warehouse/claude_estop.xml" />\n'+s[i:])
p=open("claude_walk_grasp_nobt.py").read()
a="claude_walk_full_scene_nobt.xml"; assert a in p
p=p.replace(a,"claude_walk_full_scene_estop.xml")
old="def step():\n    mujoco.mj_step(m, d)\n"; assert old in p
new='''ESTOP = [False]; ESTOP_REQ = [False]
ESTOP_AT = float(__import__("os").environ.get("ESTOP_AT", "0") or 0)
G_BTN = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "claude_estop_btn")
def estop_halt():
    ESTOP[0] = True
    m.geom_rgba[G_BTN] = (1.0, 0.1, 0.1, 1); m.geom_pos[G_BTN, 2] -= 0.015
    jx = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "tron1_slide_x")]
    print("!! E-STOP 작동: t=%.2fs | 베이스 x %.3f | 병 위치 %s | 이후 모든 구동 정지(동결)" % (d.time, d.qpos[jx], d.xpos[bt].round(3)))
    mujoco.mj_forward(m, d)
    if vw is not None:
        while vw.is_running(): vw.sync(); time.sleep(0.03)
    sys.exit(0)
def step():
    if ESTOP_REQ[0] or (ESTOP_AT > 0 and d.time >= ESTOP_AT): estop_halt()
    mujoco.mj_step(m, d)
'''
p=p.replace(old,new)
v="vw = mujoco.viewer.launch_passive(m, d)"; assert v in p
p=p.replace(v,"vw = mujoco.viewer.launch_passive(m, d, key_callback=lambda k: ESTOP_REQ.__setitem__(0, True) if k in (ord('X'), ord('x')) else None)")
open("claude_walk_grasp_estop.py","w").write(p); print("생성 완료")
