import mujoco
S="/ros_ws/mujoco/scenes/"
base=open(S+"claude_walk_scene.xml").read(); i=base.rfind("</mujoco>"); assert i>0
ok=[]
for a in ["pallets","boxes","zones","conveyor"]:
    inc="".join('  <include file="../warehouse/%s.xml" />\n'%x for x in ok+[a])
    open(S+"_probe.xml","w").write(base[:i]+inc+base[i:])
    try:
        mujoco.MjModel.from_xml_path(S+"_probe.xml"); ok.append(a); print("추가 가능:",a)
    except Exception as e:
        print("충돌/오류:",a,str(e).splitlines()[0][:120])
inc="".join('  <include file="../warehouse/%s.xml" />\n'%x for x in ok)
open(S+"claude_walk_full_scene.xml","w").write(base[:i]+inc+base[i:])
m=mujoco.MjModel.from_xml_path(S+"claude_walk_full_scene.xml"); print("최종 로드 성공 nq",m.nq,"nu",m.nu,"추가:",ok)
