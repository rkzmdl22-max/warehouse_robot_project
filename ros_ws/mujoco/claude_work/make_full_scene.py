import mujoco, numpy as np
S="/ros_ws/mujoco/scenes/"
sc=open(S+"claude_walk_scene.xml").read()
adds=["shelves_realistic_test","pallets","boxes","zones","conveyor"]
inc="".join('  <include file="../warehouse/%s.xml" />\n'%a for a in adds)
i=sc.rfind("</mujoco>"); assert i>0
open(S+"claude_walk_full_scene.xml","w").write(sc[:i]+inc+sc[i:])
m=mujoco.MjModel.from_xml_path(S+"claude_walk_full_scene.xml"); d=mujoco.MjData(m); mujoco.mj_forward(m,d)
print("로드 성공 nq",m.nq,"nu",m.nu,"ngeom",m.ngeom)
lo=np.array([-3.3,-3.35,0.05]); hi=np.array([0.5,-2.65,1.45]); tr=m.body("base_Link").id
def under(b,root):
    while b>0:
        if b==root: return True
        b=m.body_parentid[b]
    return False
hits={}
for g in range(m.ngeom):
    if m.geom_type[g]==0 or (m.geom_contype[g]==0 and m.geom_conaffinity[g]==0): continue
    b=m.geom_bodyid[g]; nm=m.body(b).name
    if under(b,tr) or "bottle" in nm or nm.startswith("ur5e"): continue
    p=d.geom_xpos[g]; r=m.geom_rbound[g]
    if np.all(p+r>=lo) and np.all(p-r<=hi): hits.setdefault(nm,[]).append(round(float(r),2))
print("보행 통로 후보 간섭(구 근사, 보수적):", {k:len(v) for k,v in hits.items()})
