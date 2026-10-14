import os
import sys
import xml.etree.ElementTree as ET
import numpy as np

# Setup DLL paths for Windows
if os.name == 'nt':
    anaconda_path = r'C:\Users\HKIT\anaconda3'
    if os.path.exists(anaconda_path):
        os.add_dll_directory(anaconda_path)
    mujoco_pkg_path = r'C:\Users\HKIT\AppData\Local\Programs\Python\Python313\Lib\site-packages\mujoco'
    if os.path.exists(mujoco_pkg_path):
        os.add_dll_directory(mujoco_pkg_path)

ws_dir = r'c:\work\warehouse_robot_project'
sys.path = [p for p in sys.path if p not in ('', ws_dir, os.path.join(ws_dir, 'mujoco'))]

import mujoco

urdf_src = os.path.join(ws_dir, 'ros_ws', 'tiago_fixed.urdf')
urdf_win = os.path.join(ws_dir, 'ros_ws', 'tiago_win_fixed.urdf')

with open(urdf_src, 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace('/ros_ws/', 'C:/work/warehouse_robot_project/ros_ws/')

tree = ET.fromstring(content)

# Preprocess inertias / masses for valid MuJoCo loading
for link in tree.findall('.//link'):
    inertial = link.find('inertial')
    if inertial is None:
        inertial = ET.SubElement(link, 'inertial')
        mass_elem = ET.SubElement(inertial, 'mass')
        mass_elem.attrib['value'] = '0.01'
        inertia = ET.SubElement(inertial, 'inertia')
        inertia.attrib['ixx'] = '1e-4'
        inertia.attrib['ixy'] = '0'
        inertia.attrib['ixz'] = '0'
        inertia.attrib['iyy'] = '1e-4'
        inertia.attrib['iyz'] = '0'
        inertia.attrib['izz'] = '1e-4'
    else:
        mass_elem = inertial.find('mass')
        if mass_elem is None:
            mass_elem = ET.SubElement(inertial, 'mass')
            mass_elem.attrib['value'] = '0.01'
        else:
            m_val = float(mass_elem.attrib.get('value', 0))
            if m_val < 1e-4:
                mass_elem.attrib['value'] = '0.01'
        
        inertia = inertial.find('inertia')
        if inertia is None:
            inertia = ET.SubElement(inertial, 'inertia')
            inertia.attrib['ixx'] = '1e-4'
            inertia.attrib['ixy'] = '0'
            inertia.attrib['ixz'] = '0'
            inertia.attrib['iyy'] = '1e-4'
            inertia.attrib['iyz'] = '0'
            inertia.attrib['izz'] = '1e-4'
        else:
            ixx = float(inertia.attrib.get('ixx', 0))
            ixy = float(inertia.attrib.get('ixy', 0))
            ixz = float(inertia.attrib.get('ixz', 0))
            iyy = float(inertia.attrib.get('iyy', 0))
            iyz = float(inertia.attrib.get('iyz', 0))
            izz = float(inertia.attrib.get('izz', 0))
            mat = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
            eigvals = np.linalg.eigvalsh(mat)
            if np.any(eigvals <= 1e-12) or max(abs(ixx), abs(iyy), abs(izz)) < 1e-6:
                d_val = max(1e-4, abs(ixx), abs(iyy), abs(izz))
                inertia.attrib['ixx'] = f"{d_val:.8e}"
                inertia.attrib['ixy'] = "0"
                inertia.attrib['ixz'] = "0"
                inertia.attrib['iyy'] = f"{d_val:.8e}"
                inertia.attrib['iyz'] = "0"
                inertia.attrib['izz'] = f"{d_val:.8e}"

mujoco_elem = tree.find('mujoco')
if mujoco_elem is None:
    mujoco_elem = ET.Element('mujoco')
    tree.insert(0, mujoco_elem)
compiler_elem = mujoco_elem.find('compiler')
if compiler_elem is None:
    compiler_elem = ET.SubElement(mujoco_elem, 'compiler')
compiler_elem.attrib['balanceinertia'] = 'true'
compiler_elem.attrib['discardvisual'] = 'false'

fixed_xml = ET.tostring(tree, encoding='unicode')
with open(urdf_win, 'w', encoding='utf-8') as f:
    f.write(fixed_xml)

model = mujoco.MjModel.from_xml_path(urdf_win)
data = mujoco.MjData(model)

# E-7 J7 TRANSFORM / AXIS ALIGNMENT INSPECTION
joint_name = "arm_7_joint"
jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

# Line updated as requested by user:
body_id = int(model.jnt_bodyid[jid])

body_name = mujoco.mj_id2name(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    body_id
)

# Forward kinematics at q = 0
data.qpos[:] = 0
mujoco.mj_forward(model, data)

body_pos = data.xpos[body_id]
body_rot = data.xmat[body_id].reshape(3, 3)
jnt_axis_body = model.jnt_axis[jid]

# Axis alignment calculation
# Unit norm check & expected local Z-axis alignment check
axis_norm = np.linalg.norm(jnt_axis_body)
expected_axis = np.array([0.0, 0.0, 1.0])
axis_error = float(np.linalg.norm(jnt_axis_body / axis_norm - expected_axis))

alignment_pass = "PASS" if axis_error < 1e-5 else "FAIL"

print("==================================================")
print(f"J7 BODY FRAME @ q=0")
print(f"Associated Body Name : {body_name} (ID: {body_id})")
print(f"body world position :")
print(body_pos)
print(f"\nbody rotation matrix :")
print(body_rot)
print(f"\nJ7 axis in body frame :")
print(jnt_axis_body)
print(f"\nBODY ↔ LOCAL AXIS ERROR : {axis_error:.6e}")
print(f"J7 AXIS ALIGNMENT       : {alignment_pass}")
print("==================================================")
print(f"E-7 J7 TRANSFORM / AXIS ALIGNMENT = {alignment_pass}")
