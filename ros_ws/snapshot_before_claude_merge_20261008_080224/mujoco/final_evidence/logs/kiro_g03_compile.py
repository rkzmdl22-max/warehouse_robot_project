import mujoco, sys, math
XML = r"/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
try:
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print("COMPILE = PASS")
    print("nbody =", m.nbody)
    print("ngeom =", m.ngeom)
    print("njnt  =", m.njnt)
    print("nsite =", m.nsite)
    print("ncam  =", m.ncam)
    print("nu    =", m.nu)
    print("nq    =", m.nq)
    nan_q = sum(1 for v in d.qpos if math.isnan(v))
    print("NaN in qpos =", nan_q)
    print("Actuators:")
    for i in range(m.nu):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print("  act[" + str(i) + "] =", nm)
    print("Cameras:")
    for i in range(m.ncam):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i)
        print("  cam[" + str(i) + "] =", nm)
    # Check key bodies
    for bname in ["base_Link","carry_tray","carry_bottle","carry_bottle_2","carry_bottle_3","ur5e_base","tool0","left_finger_link","right_finger_link","bottle_target"]:
        bi = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bname)
        print("  body", bname, "=", bi)
    # Equality constraints
    print("neq =", m.neq)
    for i in range(m.neq):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, i)
        print("  eq[" + str(i) + "] =", nm, " active=", d.eq_active[i])
    print("G03 = PASS")
except Exception as e:
    print("G03 = FAIL:", e)
    import traceback; traceback.print_exc()
    sys.exit(1)
