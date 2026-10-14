p=open("claude_walk_grasp_estop.py").read()
old="    mujoco.mj_forward(m, d)\n    if vw is not None:\n"; assert old in p
brake='''    ja = lambda a: int(m.actuator_trnid[a, 0])
    acts = [A_X] + list(A_L.values())
    dx = m.jnt_dofadr[ja(A_X)]; v0 = float(d.qvel[dx]); x0 = float(d.qpos[jx]); t0 = d.time
    for a in acts: d.ctrl[a] = d.qpos[m.jnt_qposadr[ja(a)]]
    mujoco.mj_forward(m, d); pb0 = (d.xpos[bt] - d.xpos[tray]).copy(); ts = None
    for k in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
        if ts is None and abs(d.qvel[dx]) < 0.002: ts = d.time - t0
        if vw is not None and k % RE == 0: vw.sync()
    mujoco.mj_forward(m, d); pb1 = d.xpos[bt] - d.xpos[tray]
    print("== 제동 측정(위치 서보 유지, 이 모델 기준): 정지 직전 속도 %.3f m/s | 제동거리 %.1f mm | 속도<2mm/s까지 %s | 병 트레이 대비 이동 %.1f mm"
          % (v0, (float(d.qpos[jx]) - x0) * 1000, ("%.2fs" % ts) if ts is not None else "2s 내 미도달", float(np.linalg.norm(pb1 - pb0)) * 1000))
'''
p=p.replace(old,brake+old)
open("claude_walk_grasp_estop2.py","w").write(p); print("생성 완료")
