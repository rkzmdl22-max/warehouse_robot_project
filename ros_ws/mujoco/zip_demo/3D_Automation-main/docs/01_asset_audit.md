# 제공 3D 자산 감사

## 확인 결과

- TRON1: URDF/Xacro, STL 9개, USD 계열 파일이 있습니다.
- 매니퓰레이터/그리퍼: `ur_gripper` 및 `ur_Gripper_robotiq_2f_140.usd`가 있으나 Gazebo용 URDF/SRDF는 없습니다.
- 물병: USD와 MDL/PNG 재질이 있으며 Gazebo가 직접 사용하는 SDF/URDF 및 단순 collision mesh는 없습니다.

## 통합 위험

1. TRON1 URDF의 mesh URI가 현재 폴더 구조와 다른 `robot_description/pointfoot/...`를 가리킵니다.
2. TRON1 URDF는 ROS 1 스타일 `gazebo_ros_control`, `EffortJointInterface`, 전용 `libpointfoot_hw_sim.so`에 의존합니다.
3. 제공된 매니퓰레이터와 물병의 주 자산은 USD 중심이어서 Gazebo Classic에 바로 spawn할 수 없습니다.
4. PRD에는 UR8이라 적혀 있으나 파일만으로 정확한 UR 모델/kinematics를 확정할 수 없습니다.
5. 물병 질량, 관성, 마찰, collision 형상과 실제 지름이 없어 파지·미끄럼 물리 검증값을 확정할 수 없습니다.

## 결정

원본은 변경하지 않습니다. 기업의 전체 Gazebo 패키지를 우선 요청하고, 받지 못할 때만 별도 변환 패키지에서 ROS 2 control 및 Gazebo 모델을 작성합니다. USD 원본을 직접 덮어쓰지 않고 파생 SDF/URDF와 단순 collision geometry를 별도 관리합니다.

