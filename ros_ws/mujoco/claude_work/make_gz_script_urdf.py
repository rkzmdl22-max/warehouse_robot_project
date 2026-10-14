import re
src="/ros_ws/src/warehouse_robot/urdf/tron1a_classic.urdf"
dst="/ros_ws/src/warehouse_robot/urdf/claude_tron1a_script.urdf"
s=open(src).read()
s,n1=re.subn(r'<ros2_control .*?</ros2_control>','',s,flags=re.S)
s,n2=re.subn(r'<gazebo>\s*<plugin filename="libgazebo_ros2_control\.so".*?</gazebo>','',s,flags=re.S)
add='''<gazebo><plugin name="tron1_planar_move" filename="libgazebo_ros_planar_move.so">
<ros><namespace>/tron1a</namespace></ros>
<update_rate>100</update_rate><publish_odom>false</publish_odom><publish_odom_tf>false</publish_odom_tf>
</plugin></gazebo>
<gazebo><plugin name="tron1_joint_pose" filename="libgazebo_ros_joint_pose_trajectory.so">
<ros><namespace>/tron1a</namespace></ros>
<update_rate>100</update_rate>
</plugin></gazebo>
'''
i=s.rfind("</robot>"); assert i>0
open(dst,"w").write(s[:i]+add+s[i:])
print("ros2_control 제거",n1,"| gz ros2_control 플러그인 제거",n2,"| 저장",dst)
