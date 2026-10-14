# FR3 E-8 Verification Handover
## Date: 2026-08-20
## Workspace: /ros_ws
## Container: warehouse_robot_dev
## ROS: ROS 2 Humble
## Target: Franka FR3 MuJoCo Adapter / Gripper Verification

---

# 1. Verification Scope

이번 검증은 FR3 MuJoCo Adapter의 다음 항목을 대상으로 한다.

- FR3 MuJoCo Adapter 실행 상태
- FR3 joint / actuator mapping
- FR3 gripper joint mapping
- `/fr3/command/joint_trajectory` command path
- `/fr3/joint_states` feedback path
- gripper target position response
- finger convergence
- E-8 gripper controller 변경사항
- MuJoCo MJCF finger joint / actuator 구조

기준 파일:

/ros_ws/mjcf_convert/fr3/output/fr3.xml

Adapter:

/ros_ws/src/warehouse_robot/warehouse_robot/fr3_mujoco_adapter.py

---

# 2. FR3 Adapter Process Verification

실행 확인 명령:

ps aux | grep -E 'fr3_mujoco_adapter|ros2' | grep -v grep

확인 결과:

root        3960  0.0  0.2  35048 23836 pts/5    S+   05:26   0:00 /usr/bin/python3 /opt/ros/humble/bin/ros2 run warehouse_robot fr3_mujoco_adapter

root        3961 22.6  0.9 1208136 78084 pts/5   Sl+  05:26   3:51 /usr/bin/python3 /ros_ws/install/warehouse_robot/lib/warehouse_robot/fr3_mujoco_adapter

판정:

PASS — FR3 MuJoCo Adapter 프로세스가 실제 실행 중임.

---

# 3. ROS 2 Node Verification

명령:

ros2 node list | grep -i fr3

결과:

/fr3_mujoco_adapter

판정:

PASS — FR3 MuJoCo Adapter ROS 2 node가 등록되어 있음.

---

# 4. ROS 2 Interface Verification

명령:

ros2 node info /fr3_mujoco_adapter

확인된 Subscriber:

/fr3/command/joint_trajectory
trajectory_msgs/msg/JointTrajectory

확인된 Publishers:

/fr3/joint_states
sensor_msgs/msg/JointState

확인된 Action Server:

/fr3_arm_controller/follow_joint_trajectory
control_msgs/action/FollowJointTrajectory

판정:

PASS — FR3 command / feedback ROS 2 interface가 연결되어 있음.

---

# 5. FR3 Joint Mapping Verification

Adapter source 확인 결과:

FR3 arm joints:

fr3_joint1
fr3_joint2
fr3_joint3
fr3_joint4
fr3_joint5
fr3_joint6
fr3_joint7

FR3 finger joints:

fr3_finger_joint1
fr3_finger_joint2

FR3 actuators:

fr3_joint1_motor
fr3_joint2_motor
fr3_joint3_motor
fr3_joint4_motor
fr3_joint5_motor
fr3_joint6_motor
fr3_joint7_motor
fr3_gripper

Adapter 내부 확인 로그:

FR3 joint map verified
FR3 actuator map verified

판정:

PASS — FR3 7-DOF arm + 2-finger joint/actuator mapping이 구성되어 있음.

---

# 6. MJCF Gripper Structure Verification

기준 MJCF:

/ros_ws/mjcf_convert/fr3/output/fr3.xml

확인된 내용:

fr3_finger_joint1:

type="slide"
axis="0 1 0"
range="0.0 0.04"
limited="true"
damping="5.0"

fr3_finger_joint2:

type="slide"
axis="0 1 0"
range="0.0 0.04"
limited="true"
damping="5.0"

Gripper actuator:

<motor name="fr3_gripper"
       joint="fr3_finger_joint1"
       ctrlrange="-100 100"
       gear="1" />

Mimic relation:

<jointpos/mimic relation>
fr3_finger_joint2 follows fr3_finger_joint1

확인된 실제 XML lines:

61: fr3_finger_joint1
69: fr3_finger_joint2
91: fr3_gripper
94: fr3_finger_mimic
104: fr3_finger_joint1_position

판정:

PASS — MJCF gripper joint range, actuator, mimic structure가 존재함.

---

# 7. E-8 Gripper Controller Verification

Adapter source의 gripper control branch:

if actuator_name == "fr3_gripper":

gripper_kp = 50.0
gripper_kd = 2.0

raw_torque =
    gripper_kp * (target[qpos_adr] - q)
    - gripper_kd * qd

중요 사항:

E-8-15 변경사항으로 gripper에는 gravity compensation을 추가하지 않음.

주석 기준:

"Gripper uses dedicated position PD control."

"Do not add gravity compensation here because
the previous gravity + PD formulation prevented
the gripper from moving toward smaller targets."

판정:

PASS — 현재 코드에는 E-8-15의 dedicated gripper position-PD control 구조가 반영되어 있음.

---

# 8. Gripper Command Test #1

명령:

ros2 topic pub --once /fr3/command/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [fr3_finger_joint1, fr3_finger_joint2], points: [{positions: [0.01, 0.01], time_from_start: {sec: 2}}]}"

실제 publish 결과:

joint_names:
- fr3_finger_joint1
- fr3_finger_joint2

positions:
- 0.01
- 0.01

time_from_start:
sec: 2

판정:

PASS — gripper joint trajectory command가 ROS 2 topic으로 정상 publish됨.

---

# 9. Gripper Feedback After 0.01 Command

명령 후:

ros2 topic echo /fr3/joint_states --once

확인된 finger position:

fr3_finger_joint1:
0.010000000000000071

fr3_finger_joint2:
0.010000726080500368

목표:

0.01
0.01

오차:

finger1 ≈ 7.1e-17
finger2 ≈ 7.26e-7

판정:

PASS — 0.01 목표에 대해 두 finger 모두 사실상 목표 위치로 수렴함.

---

# 10. Gripper Convergence / Upper Limit Test

후속:

ros2 topic echo /fr3/joint_states --once

확인된 finger position:

fr3_finger_joint1:
0.03999932651655777

fr3_finger_joint2:
0.04000005221530291

MJCF joint range upper limit:

0.04

목표 위치:

0.04

오차:

finger1:
0.04 - 0.03999932651655777
≈ 6.7348344223e-7

finger2:
0.04000005221530291 - 0.04
≈ 5.221530291e-8

판정:

PASS — 두 finger 모두 0.04 upper target에 사실상 수렴함.

주의:

finger2의 측정값이 0.04를 약 5.22e-8 초과하지만 이는 매우 작은 수치 오차이며,
MJCF joint limit / simulation numerical tolerance 범위에서 관측된 값으로 기록한다.

---

# 11. Finger Velocity at Convergence

최종 feedback:

fr3_finger_joint1 velocity:
-7.980639286146582e-16

fr3_finger_joint2 velocity:
-4.835246985252554e-16

판정:

PASS — 최종 상태에서 finger velocity가 사실상 0에 수렴하여 정착 상태를 확인함.

---

# 12. Final Gripper State Evidence

최종 관측:

finger1 = 0.03999932651655777
finger2 = 0.04000005221530291

target/range upper boundary = 0.04

velocities:

finger1 ≈ -7.98e-16
finger2 ≈ -4.84e-16

결론:

FR3 gripper 두 finger가 0.04 위치로 안정적으로 수렴한 것이
실제 `/fr3/joint_states` feedback으로 확인됨.

---

# 13. GRIPPER_DIAG Log Investigation

검색 명령:

grep -Rni "GRIPPER_DIAG" /ros_ws/log/2026-08-20* 2>/dev/null | tail -20

결과:

출력 없음.

해석:

현재 ROS log 파일에서 GRIPPER_DIAG 문자열은 확인되지 않았다.

따라서:

- GRIPPER_DIAG 로그 자체는 증거로 사용하지 않는다.
- 이를 FAIL로 판정하지 않는다.
- 실제 `/fr3/joint_states` feedback을 primary evidence로 사용한다.

현재 gripper convergence 판정은 실제 joint state 값으로 수행한다.

---

# 14. Gazebo Errors — E-8 Scope Exclusion

/ros_ws/launch_output.log에는 다음과 같은 기존 Gazebo 관련 오류가 존재함.

- pmb2_description mesh missing
- tiago_description mesh missing
- libgazebo_world_odometry.so missing
- libgazebo_underactuated_finger.so missing
- pal_hey5_description finger mesh 관련 오류

이 오류들은 현재 FR3 MuJoCo E-8 gripper verification 대상이 아니다.

특히:

pal_hey5
hey5
finger_flex

관련 오류는 FR3 MuJoCo gripper convergence 판정에서 제외한다.

---

# 15. Current E-8 Evidence Status

| 항목 | 상태 | 증거 |
|---|---|---|
| FR3 Adapter 실행 | PASS | ps aux |
| FR3 ROS2 node | PASS | ros2 node list |
| Command subscriber | PASS | ros2 node info |
| Joint state publisher | PASS | ros2 node info |
| FR3 7DOF mapping | PASS | adapter source |
| Finger joint mapping | PASS | adapter source |
| MJCF finger range | PASS | fr3.xml |
| MJCF gripper actuator | PASS | fr3.xml |
| Finger mimic | PASS | fr3.xml |
| Dedicated gripper PD | PASS | adapter source |
| 0.01 command response | PASS | /fr3/joint_states |
| 0.01 convergence | PASS | joint state |
| 0.04 convergence | PASS | joint state |
| Final velocity convergence | PASS | joint state |
| GRIPPER_DIAG log | NOT USED | grep 결과 없음 |
| Gazebo PAL/Tiago errors | EXCLUDED | E-8 범위 외 |

---

# 16. Important Evidence Values

## Command

target:
[0.01, 0.01]

## Response

position:
[0.010000000000000071,
 0.010000726080500368]

## Final convergence

position:
[0.03999932651655777,
 0.04000005221530291]

## Final velocity

velocity:
[-7.980639286146582e-16,
 -4.835246985252554e-16]

---

# 17. Current Conclusion

현재까지 실제 터미널 증거로 확인된 범위에서:

FR3 MuJoCo Adapter는 실행 중이며,
ROS 2 command path와 joint-state feedback path가 정상적으로 연결되어 있다.

FR3 gripper는:

0.01 target → 0.01 근처 수렴

그리고

0.04 target → 0.04 근처 수렴

을 실제 `/fr3/joint_states`를 통해 확인했다.

특히 최종 상태에서 finger velocity가 약 10^-16 수준으로 감소했으므로
gripper가 목표 위치에서 안정적으로 정착한 상태를 확인했다.

따라서 현재 E-8 gripper convergence 관련 검증은
실측 joint-state evidence 기준으로 PASS 상태로 기록한다.

단, 각 세부 E-8 번호의 공식 PASS 판정은
MASTER에서 정의된 해당 E-8 항목의 정확한 acceptance criteria와 대조하여
최종 확정해야 한다.

---

# 18. Next Step

다음 작업에서는 E-8 결과를 임의로 확장하지 않고,
MASTER의 E-8 acceptance criteria와 현재 증거를 1:1 대조한다.

순서:

1. E-8 공식 항목 목록 확인
2. 각 항목별 acceptance criteria 확인
3. 현재 증거 연결
4. 부족한 항목만 추가 검증
5. E-8 최종 PASS/FAIL 확정
6. 이후 E-9 또는 MASTER가 지정한 다음 검증 단계로 이동

---

# 19. Critical Rule

이 문서의 PASS는 실제 현재 터미널에서 확인된 증거에만 기반한다.

특히 다음은 임의로 PASS 처리하지 않는다.

- 로그에 존재하지 않는 GRIPPER_DIAG
- 확인하지 않은 E-8 세부 항목
- Gazebo 오류를 FR3 MuJoCo 오류로 간주하는 것
- 실제 측정하지 않은 convergence 결과
- MASTER에 없는 새로운 acceptance criteria

기존 프로젝트 자산은 삭제하거나 새 프로젝트로 재구성하지 않는다.


---

# 20. Recovered E-8 Official Verification Results

The following E-8 results are preserved from the prior FR3 verification record and are added without redefining the acceptance criteria.

## E-8-15 — Gripper Bidirectional Convergence

Status: PASS

Recorded verification:

- Gripper convergence was tested in both directions.
- `0.03 -> 0.04` convergence verified.
- `0.04 -> 0.01` convergence verified.
- Both finger joints converged toward the commanded target.
- MJCF gripper actuator remains `fr3_gripper`.
- Finger mimic relationship remains active.
- Actuator control range remains `-100 ~ 100`.

Related current implementation evidence:

- Dedicated gripper position-PD controller.
- `gripper_kp = 50.0`
- `gripper_kd = 2.0`
- No gravity compensation in the dedicated gripper branch.

Conclusion:

E-8-15 PASS is retained from the verified prior test record.

## E-8-16 — FollowJointTrajectory Round Trip

Status: PASS

Recorded verification:

- FollowJointTrajectory round-trip test executed.
- Command sequence included `0.00 -> 0.10 -> 0.00`.
- Goal accepted.
- Final action result reported `error_code = 0` / `SUCCEEDED`.
- Final joint state returned to the commanded endpoint.

Conclusion:

E-8-16 PASS is retained from the verified prior test record.

## E-8-17 — 9-Joint Integrated FollowJointTrajectory

Status: PASS

Scope:

- FR3 arm 7 joints
- FR3 finger 2 joints
- Total = 9 joints

Recorded verification:

- Integrated 9-joint FollowJointTrajectory goal accepted.
- Action result: `error_code = 0` / `SUCCEEDED`.
- `/fr3/joint_states` confirmed the 7 arm joints and 2 finger joints.
- Final finger state recorded:
  - `fr3_finger_joint1 = 0.039999326516557764`
  - `fr3_finger_joint2 = 0.0400000522153029`

Conclusion:

E-8-17 PASS is retained from the verified prior test record.

## E-8-14

Status: PENDING / NOT RECONSTRUCTED

The original acceptance criterion for E-8-14 is not currently present in `/ros_ws` documentation, and no exact criterion is reconstructed here.

Do not infer or invent an E-8-14 acceptance criterion.

---

# 21. Updated E-8 Status Matrix

| Item | Status | Evidence basis |
|---|---|---|
| E-8-14 | PENDING | Original acceptance criterion not reconstructed |
| E-8-15 | PASS | Bidirectional gripper convergence record |
| E-8-16 | PASS | FollowJointTrajectory round trip, error_code 0 / SUCCEEDED |
| E-8-17 | PASS | Integrated 9-joint FollowJointTrajectory, error_code 0 / SUCCEEDED |

Important:

The E-8-15, E-8-16 and E-8-17 PASS states are preserved from previously verified project records. No new PASS criterion has been invented.

---

# 22. Current Baseline / Next Action

Current verified E-8 baseline:

- E-8-15: PASS
- E-8-16: PASS
- E-8-17: PASS
- E-8-14: pending reconstruction of the original acceptance criterion

Do not modify the FR3 adapter or MJCF based on these results.

The next verification step is to recover the exact E-8-14 acceptance criterion from the project/master record if available. If it cannot be recovered, leave E-8-14 explicitly marked as pending rather than inventing a criterion.

