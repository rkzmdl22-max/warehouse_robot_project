# 3D Automation Codex 인수인계

작성일: 2026-09-09 (Asia/Seoul)  
기준 프로젝트 루트: `C:\work\3D_Automation_git`  
기준 브랜치/커밋: `main` / `94b751a` (`feat: support sequential three-bottle handover`)  
원격 저장소: `https://github.com/JoHJ3753/3D_Automation.git`

## 1. 이 문서의 기준과 확인 범위

이 문서는 기존 Codex 대화 기록과 2026-09-09에 실제로 clone한 GitHub `main`을 대조해 작성했다. 내용이 충돌할 때는 실제 clone본을 우선했다.

- 문서 작성 직전 `git status --short --branch`: `## main...origin/main` (clean)
- 확인된 최근 커밋:
  - `94b751a feat: support sequential three-bottle handover`
  - `23b012e feat: add integrated TRON1 UR5e handover demo`
  - `2af3dba feat: add TRON1 transport animation`
  - `428de6d feat: add reliable MuJoCo bottle pick and place`
  - `0fa1a69 feat: add MuJoCo bottle workcell`
  - `ba33215 feat: add MuJoCo UR5e gripper preview`
  - `79718f3 chore: save Gazebo prototype baseline`
- Google Drive 압축본을 풀어 만든 `C:\work\3D_Automation`은 일부 파일과 루트 `.git` 내용이 누락된 불완전 복원본이다. 다만 이 폴더의 `src/vendor/mujoco_menagerie`는 온전한 것으로 확인되어 외부 의존성 복구 원본으로 사용할 수 있다.
- 실제 동적 MuJoCo GUI 재실행은 이 문서 작성 시점에 하지 않았다. 현재 Windows clone본에는 Git에서 제외된 `src/vendor`가 없기 때문이다.

## 2. 프로젝트 목적과 최종 목표

프로젝트의 원래 ROS 2/Gazebo MVP 목표는 TurtleBot3 Waffle Pi와 OpenMANIPULATOR-X를 이용해 300 g 이하 물체를 인식하고 안전하게 Pick & Place하는 모바일 매니퓰레이션 데모를 만드는 것이다. 안전 원칙은 `SAFE_STOP -> REVALIDATE -> RESUME/REPLAN`이며, 베이스 정지 후 팔 동작, 파지 검증, 낙하/인식 실패 시 정지가 핵심이다.

실제 최근 개발의 중심은 별도의 MuJoCo 통합 데모로 확장되었다.

1. TRON1 이족 로봇이 트레이에 물병을 싣고 먼 거리에서 인계 위치까지 이동한다.
2. TRON1의 좌우 다리가 번갈아 움직이는 보행 애니메이션을 보여준다.
3. 고정형 UR5e와 Robotiq 2F-85 그리퍼가 물병을 집는다.
4. 물병을 들어 올리고 수신 테이블로 옮겨 세운 상태로 배치한다.
5. 현재 코드는 물병 3개를 순서대로 처리하도록 확장되어 있다.

최종적으로는 Ubuntu 22.04 네이티브 환경에서 Windows + Docker + X server보다 부드러운 MuJoCo 렌더링을 확보하고, 세 병 순차 인계가 앞서 놓은 병을 건드리지 않으면서 반복 성공하도록 만드는 것이 당면 목표다.

## 3. 개발 환경과 도구

### 기존 확인 환경

- 호스트: Windows 10
- IDE: Antigravity IDE / Codex
- 컨테이너 베이스: `osrf/ros:humble-desktop-full-jammy`
- 컨테이너 OS: Ubuntu 22.04 (Jammy)
- ROS: ROS 2 Humble
- 시뮬레이터: Gazebo Classic 11 및 MuJoCo
- 컨테이너 GUI: Windows X server(VcXsrv/X410 계열)와 `DISPLAY=host.docker.internal:0.0`
- GitHub: `origin/main`을 사용

### 전환 예정 환경

- Ubuntu 22.04를 Windows와 듀얼부팅으로 설치했다/설치 중이다.
- 새 Codex 환경은 Ubuntu 22.04에서 구성할 예정이다.
- Ubuntu 네이티브에서 MuJoCo를 우선 실행해 FPS 개선 여부를 확인해야 한다.

### 주요 도구

- Python
- MuJoCo Python API (`mujoco`, `mujoco.viewer`, `MjSpec`)
- NumPy
- ROS 2 Humble (`rclpy`, TF2, sensor/geometry/nav/std messages)
- OpenCV / `cv_bridge` / `message_filters`
- Gazebo Classic, Nav2, MoveIt 2, ros2_control
- Docker Compose
- `colcon`, `rosdep`, `vcstool`, `pytest`

## 4. 설치된/선언된 주요 패키지와 버전

`docker/Dockerfile`에서 정확히 확인되는 항목:

- Ubuntu 22.04 계열 이미지: `osrf/ros:humble-desktop-full-jammy`
- MuJoCo: `mujoco==3.3.7`
- NumPy: `numpy<2.0`
- ROS 2 Humble
- Gazebo Classic 패키지
- MoveIt 2: `ros-humble-moveit`
- Nav2: `ros-humble-navigation2`, `ros-humble-nav2-bringup`
- `ros-humble-cv-bridge`
- `ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-ros2-control`
- `ros-humble-ros2-control`, `ros-humble-ros2-controllers`
- `ros-humble-gripper-controllers`, `ros-humble-joint-trajectory-controller`
- `ros-humble-image-transport`, `ros-humble-robot-state-publisher`, `ros-humble-xacro`
- 시스템 그래픽 라이브러리: `libegl1`, `libgl1`, `libglew2.2`, `libglfw3`, `libosmesa6`, `patchelf`
- 개발 도구: Git, pip, pytest, rosdep, vcstool

정확한 패치 버전이 선언되지 않은 apt/ROS 패키지는 임의로 기록하지 않는다. 별도 `requirements.txt`/`pyproject.toml` 기반 Python 의존성 파일은 없다. `pyrefly.toml`은 타입 검사 범위/오류 억제 설정이며 패키지 잠금 파일이 아니다.

문서 작성 당시 Windows의 현재 Python 환경은 Python 3.13.9, MuJoCo 3.6.0, NumPy 2.3.5, pytest 8.4.2였지만, 이것은 프로젝트가 검증된 컨테이너 버전과 다르므로 기준 실행 환경으로 간주하지 않는다.

## 5. 현재 디렉터리 구조

```text
3D_Automation_git/
├── .env.example
├── .gitattributes
├── .gitignore
├── .vscode/settings.json
├── README.md
├── CODEX_HANDOFF.md
├── assets/vendor/objects_3da_v0/
│   ├── Bottle/
│   ├── Manipulator_Gripper/
│   └── PF_TRON1A/
│       ├── meshes/
│       ├── urdf/
│       ├── xacro/
│       └── xml/robot.xml
├── config/handover.yaml
├── deps/turtlebot3.repos
├── docker/Dockerfile
├── docker-compose.yml
├── docs/
│   ├── 00_project_baseline.md
│   ├── 01_asset_audit.md
│   └── 02_architecture_decision.md
├── scripts/bootstrap_dependencies.sh
└── src/
    ├── automation_cell_bringup/
    ├── bottle_perception/
    ├── handover_fsm/
    ├── mujoco_automation/scripts/
    │   ├── preview_ur5e_gripper.py
    │   ├── preview_workcell.py
    │   ├── run_pick_place.py
    │   ├── preview_tron1.py
    │   ├── preview_handover_cell.py
    │   └── run_handover_demo.py
    └── vendor/                 # .gitignore 대상; clone 직후에는 없음
```

`build/`, `install/`, `log/`, `.env`, `src/vendor/`는 `.gitignore` 대상이다.

## 6. 주요 파일과 역할

- `README.md`: 원래 ROS 2/Gazebo MVP의 목적, Docker 실행법, 기업 측에 필요한 자료를 설명한다. 최근 MuJoCo 통합 데모 실행법은 아직 충분히 반영되지 않았다.
- `docker/Dockerfile`: ROS 2 Humble, Gazebo, MoveIt/Nav2, MuJoCo 3.3.7, NumPy 1.x 계열을 설치하는 재현 환경이다.
- `docker-compose.yml`: 저장소를 `/workspace/3D_Automation`에 마운트하고 ROS 컨테이너를 실행한다. Windows GUI를 위해 `DISPLAY`를 전달하며 현재 `LIBGL_ALWAYS_SOFTWARE="1"`이다.
- `scripts/bootstrap_dependencies.sh`: `deps/turtlebot3.repos`를 `src/vendor`에 `vcs import`하고 rosdep 의존성을 설치한다.
- `deps/turtlebot3.repos`: Humble 브랜치의 `turtlebot3_msgs`, `turtlebot3`, `turtlebot3_simulations`, `turtlebot3_manipulation` 저장소 목록이다.
- `config/handover.yaml`: 인식 임계값, 0.30 kg 최대 질량, 조작 속도/가속도, 파지 폭/미끄럼 한계, 적재 이동 속도, 재검증 정책을 담는다.
- `docs/00_project_baseline.md`: 원래 Gazebo MVP 수용 기준과 단계 계획이다.
- `docs/01_asset_audit.md`: 제공된 TRON1/UR gripper/병 자산과 Gazebo 통합 위험을 기록한다.
- `docs/02_architecture_decision.md`: 원래 MVP가 TurtleBot3 + OpenMANIPULATOR-X를 채택했다는 ADR이다. "TRON1 동적 보행과 UR5e는 향후 확장"이라는 내용은 이후 MuJoCo 데모가 실제 구현된 현재 상황과 시점상 맞지 않으므로 역사적 결정으로 읽어야 한다.
- `src/handover_fsm/handover_fsm/machine.py`: 안전 상태기계와 정상/오류 전이를 구현한다.
- `src/handover_fsm/handover_fsm/node.py`: `/handover/event`를 받아 상태를 변경하고 `/handover/state` JSON을 발행한다.
- `src/handover_fsm/handover_fsm/approach_controller.py`: odom 좌표의 병 위치에서 0.42 m standoff 지점까지 저속 접근하고 `APPROACH_OK`를 발행한다.
- `src/bottle_perception/bottle_perception/bottle_detector.py`: RGB/depth를 시간 동기화하고 HSV 파란색 병을 분할해 카메라 및 odom 좌표의 `PoseStamped`와 디버그 영상을 발행한다.
- `src/automation_cell_bringup/launch/demo.launch.py`: TurtleBot3 manipulation Gazebo world, FSM, 카메라 정적 TF를 실행한다.
- `preview_ur5e_gripper.py`: MuJoCo Menagerie의 UR5e scene에 Robotiq 2F-85를 붙여 정적 미리보기를 연다.
- `preview_workcell.py`: UR5e + 그리퍼 + 테이블 + 단일 원통형 병 작업셀을 구성한다.
- `run_pick_place.py`: 단일 병의 IK 기반 `PREGRASP -> APPROACH -> GRASP -> GRASP_HOLD -> LIFT -> TRANSFER -> PLACE -> RELEASE -> RETREAT -> COMPLETE` 동작을 수행한다.
- `preview_tron1.py`: TRON1에 비충돌 트레이/병 시각 형상을 추가하고, free joint 이동과 좌우 교대 다리 애니메이션을 반복한다.
- `preview_handover_cell.py`: UR5e/그리퍼/받는 테이블/TRON1/트레이/물병 3개를 하나의 MuJoCo 모델로 합친다.
- `run_handover_demo.py`: TRON1 이동과 세 병 순차 pick/place를 통합한 현재 핵심 데모다.

## 7. 지금까지 구현한 기능과 중요한 변경사항

### ROS 2/Gazebo 기반

- 안전 중심 handover FSM 구현.
- FSM ROS 노드와 문자열 이벤트/JSON 상태 토픽 구현.
- 파란색 병 HSV 검출, RGB-D 중심 깊이의 median 계산, 카메라 좌표 3D 복원, odom TF 변환 구현.
- TurtleBot3 저속 접근 제어기 구현.
- Gazebo bringup launch와 station world/카메라 TF 구성.

### MuJoCo 단일 로봇/작업셀

- MuJoCo Menagerie UR5e와 Robotiq 2F-85를 `MjSpec.attach`로 결합.
- 테이블과 질량 0.3 kg 원통형 병을 추가.
- 위치+회전 오차를 사용하는 damped least-squares IK 구현. 최대 600회 반복, damping 0.02, 관절 step ±0.08, 최종 위치 오차 0.015 m 초과 시 실패한다.
- 그리퍼 actuator force range를 ±15로 제한하고 pad 마찰/접촉 파라미터와 병 마찰을 강화.
- grasp 이후 hold 단계를 추가해 들어 올릴 때 병이 빠지는 현상을 완화.
- 단일 병 pick/place는 대화 중 연속 실행에서 여러 차례 성공했고, 사용자가 마지막으로 2회 추가 실행 모두 성공했다고 확인했다.

### TRON1

- 제공 TRON1 MuJoCo XML을 로드하고 트레이를 추가.
- 초기에는 베이스 좌표만 움직여 미끄러져 보였으나, hip/knee 보행 애니메이션을 추가.
- 처음 구현에서는 두 다리가 같이 움직였고, 이후 모델의 좌우 hip 축 정의를 고려해 화면상 교대 보행이 되도록 수정. 사용자가 좌우 다리가 번갈아 움직임을 확인했다.
- 독립 미리보기는 `START_X=-1.2`, 통합 데모는 더 먼 거리인 `TRON1_START_X=-2.5`를 사용한다.

### 통합 handover 및 3병 확장

- UR5e 베이스를 0.70 m 받침대 위로 올려 TRON1 트레이와 그리퍼 충돌/관통을 해소하고 인계 높이에 대한 IK 도달성을 확보.
- 통합 경로 IK가 다음 경유점을 포함해 통과한 기록이 있다: 인계 측 `[0, 0.55, 1.28]`, `[0, 0.55, 1.13]`, `[0, 0.55, 1.30]`; 배치 측 `[0.5, -0.25, 1.05]`, `[0.5, -0.25, 0.865]`, `[0.5, -0.25, 1.08]`.
- 단일 통합 handover 데모는 사용자가 여러 번 완벽하게 동작한다고 확인했다.
- 병 3개를 트레이 x 방향 `[-0.08, 0.00, +0.08]` 오프셋에 배치.
- 세 병을 각각 `[0.34, -0.25, 0.875]`, `[0.50, -0.25, 0.875]`, `[0.66, -0.25, 0.875]`에 놓도록 분리.
- 이전에 놓은 병을 다음 동작에서 건드리는 문제를 줄이기 위해 place-high를 각 목표보다 0.30 m, retreat를 0.30 m 높게 계산하도록 변경.
- 렌더 호출 비용을 줄이기 위해 `VIEWER_FPS=20.0`과 `sync_viewer_if_due()`를 도입. 물리 step은 계속 수행하되 `viewer.sync()` 호출 빈도를 제한한다.
- 실제 contact만으로 병을 끝까지 유지하기 어려운 구간은 `set_bottle_at_gripper()`와 `enforce_handover_scene()`가 병 free joint를 pinch site에 맞추는 방식으로 안정적인 데모 연출을 보조한다. 이는 순수 물리 기반 grasp 검증이 아니라 의도적인 kinematic attachment/scene enforcement 설계다.

## 8. 실행 환경 및 실행 방법

### A. 기존 Windows + Docker 방식

프로젝트 루트에서:

```powershell
Copy-Item .env.example .env
docker compose build
docker compose run --rm ros
```

컨테이너에서 vendor/ROS 의존성 설치 및 빌드:

```bash
./scripts/bootstrap_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

ROS 테스트/실행:

```bash
pytest -q src/handover_fsm/test
ros2 run handover_fsm handover_fsm_node
ros2 launch automation_cell_bringup demo.launch.py
```

MuJoCo 핵심 데모:

```bash
python3 src/mujoco_automation/scripts/preview_ur5e_gripper.py
python3 src/mujoco_automation/scripts/preview_workcell.py
python3 src/mujoco_automation/scripts/run_pick_place.py
python3 src/mujoco_automation/scripts/preview_tron1.py
python3 src/mujoco_automation/scripts/preview_handover_cell.py
python3 src/mujoco_automation/scripts/run_handover_demo.py
```

### B. 새 Ubuntu 22.04 네이티브 방식

우선 GitHub clone 후 `src/vendor/mujoco_menagerie`를 복원해야 한다. 방법은 다음 둘 중 하나다.

1. 기존 압축 해제본 `C:\work\3D_Automation\src\vendor`를 Ubuntu로 옮긴다.
2. MuJoCo Menagerie와 TurtleBot3 저장소를 다시 clone/import한다. 단, 현재 `bootstrap_dependencies.sh`는 TurtleBot3 네 저장소만 가져오며 MuJoCo Menagerie는 가져오지 않는다.

네이티브 Python 환경에서는 최소한 프로젝트 기준 버전을 맞춘다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install "numpy<2.0" "mujoco==3.3.7" pytest
```

주의: `preview_tron1.py`, `preview_handover_cell.py`, 그리고 이를 import하는 `run_handover_demo.py`는 `PROJECT_ROOT = Path("/workspace/3D_Automation")`를 하드코딩한다. 이 경로는 Docker 마운트에는 맞지만 일반적인 Ubuntu clone 위치(예: `~/work/3D_Automation`)에는 맞지 않는다. 네이티브 실행 전에 `Path(__file__).resolve().parents[3]` 방식으로 바꾸거나 `/workspace/3D_Automation` 경로를 마련해야 한다. 새 Codex는 임의 수정 전에 사용자에게 변경 내용을 먼저 설명해야 한다.

## 9. 테스트 방법과 현재 결과

2026-09-09 clone본에서 실행한 비GUI 검사:

```powershell
python -m compileall -q `
    src\mujoco_automation\scripts `
    src\handover_fsm\handover_fsm `
    src\bottle_perception\bottle_perception `
    src\automation_cell_bringup\launch
```

결과: **PASS**.

FSM 테스트:

```powershell
$env:PYTHONPATH=(Resolve-Path 'src\handover_fsm').Path
python -m pytest -q src\handover_fsm\test\test_machine.py
```

결과: **1 failed, 2 passed**.

실패 원인은 `test_happy_path_reaches_complete()`의 이벤트 목록이 `BOTTLE_STABLE` 다음에 `APPROACH_OK`를 보내지 않고 바로 `PLAN_OK`를 보내기 때문이다. 현재 FSM은 `BOTTLE_STABLE -> APPROACH` 후 `APPROACH_OK -> MOTION_PLAN`을 요구하므로 테스트가 구현 변경을 따라가지 못한 상태다.

실제 MuJoCo GUI 결과(기존 대화에서 사용자 확인):

- UR5e + 2F-85 미리보기: 정상.
- 단일 병 작업셀: 정상.
- 단일 병 pick/place: 초기 slip/파지 실패를 수정한 뒤 반복 성공.
- TRON1 교대 보행 애니메이션: 정상 확인.
- 단일 병 통합 TRON1-UR5e handover: 반복 성공 확인.
- 세 병 순차 handover: 구현은 커밋되었으나, 이전 배치 병을 다음 동작이 건드리는 문제를 수정한 최종 버전에 대해 반복 성공이 대화에서 명시적으로 확인되지는 않았다. 따라서 **미검증**으로 취급한다.

## 10. 주요 오류와 해결 과정

1. IDE에서 MuJoCo import/attribute 진단이 대량 표시됨
   - `.vscode/settings.json`과 `pyrefly.toml`에서 vendor/build/install/log 제외 및 missing import/module/attribute 진단을 억제.
   - 이는 정적 분석 노이즈를 줄인 것이며 런타임 오류를 고친 것은 아니다.

2. 병이 grasp 후 lift 중 그리퍼에서 빠짐
   - gripper force range, pad/bottle friction, `geom_solref`, 긴 `GRASP`, `GRASP_HOLD`를 적용.
   - 통합 데모에서는 추가로 병을 pinch site에 kinematic하게 유지하는 보조 로직을 사용.

3. 수정 후 병을 아예 제대로 잡지 못하거나 grasp 도중 이탈
   - 접근/파지 높이와 hold 시간을 재조정하고 마찰 설정을 정착시킴.

4. TRON1이 걸어오지 않고 미끄러짐
   - free joint x 이동뿐 아니라 hip/knee 주기 운동과 body bob을 추가.

5. TRON1 두 다리가 동시에 움직임
   - 좌우 축 방향을 확인하고 knee lift를 양/음 반주기로 분리해 시각적으로 교대 보행하도록 수정.

6. `MjSpec.find_body` 오류
   - 오류: `AttributeError: 'mujoco._specs.MjSpec' object has no attribute 'find_body'`.
   - 현재 코드는 `spec.body("base_Link")` 또는 `spec.body("base")` API를 사용한다.

7. 모델 결합 시 frame/site 미지정
   - 오류: `ValueError: One of frame or site must be specified.`
   - `cell_spec.attach(gripper_spec, site="attachment_site", ...)`와 TRON1용 frame을 명시.

8. body에 attach 호출
   - 오류: `AttributeError: 'mujoco._specs.MjsBody' object has no attribute 'attach'`.
   - body 객체가 아니라 `MjSpec.attach()`를 사용하도록 변경.

9. UR5e 그리퍼가 TRON1 트레이를 관통
   - UR5e 베이스 높이를 0.70 m로 올리고 안전 초기 shoulder pan과 인계 위치를 조정.

10. 통합 IK 실패
    - 오류 예: target `[0.0, 0.55, 1.28]`, error 0.1150 m 또는 0.5455 m.
    - UR5e 베이스 높이와 IK seed/경유점 재설정 후 전체 인계/배치 경로 IK가 통과.

11. 물병이 마지막에 눕거나 불안하게 놓임
    - 파지 중심, kinematic hold, release 후 `SETTLE` 단계와 수직 자세 강제를 사용.

12. 두 번째/세 번째 병을 옮기며 이미 놓은 병을 건드림
    - 세 배치 x 좌표 간격을 0.16 m로 넓히고 place-high/retreat 높이를 각각 +0.30/+0.35 m로 상향. 최종 반복 검증은 아직 필요.

13. Windows Docker MuJoCo FPS가 약 1 FPS 수준으로 매우 낮음
    - 원인 후보는 Docker Desktop + Windows X server + `LIBGL_ALWAYS_SOFTWARE=1` 소프트웨어 렌더링, 고해상도 STL, 매 physics step의 렌더 동기화.
    - `VIEWER_FPS=20`으로 sync 호출을 줄였지만 근본 해결은 Ubuntu 네이티브 OpenGL 실행을 우선 시험하는 것.

14. 백업 ZIP 해제 문제
    - Bandizip가 `log/latest`, `log/latest_build` 심볼릭 링크 생성에 실패했고, 처음 해제된 사본에서는 루트 `.git` 내부와 여러 추적 파일/자산도 누락됨.
    - 해결: GitHub에서 새로 clone한 `C:\work\3D_Automation_git`을 기준 저장소로 사용.

## 11. 시도했지만 실패했거나 폐기한 방법

- TRON1 베이스 위치만 직접 이동: 보행이 아니라 미끄러지는 모양이어서 폐기.
- 좌우 다리에 동일하게 보이는 단순 관절 파형: 두 다리가 함께 움직여 교대 knee lift 방식으로 수정.
- 물리 접촉만 믿고 lift/transfer: 병 slip이 반복되어 마찰/hold 및 통합 데모의 kinematic attachment 보조를 사용.
- `MjSpec.find_body`: 현재 MuJoCo API에 없어 실패.
- `MjsBody.attach`: 지원하지 않아 실패.
- frame/site 없이 `MjSpec.attach`: 실패.
- 너무 높은/잘못된 베이스 또는 seed 조건의 통합 IK: 목표 오차가 허용치를 크게 초과해 실패.
- 매 physics step마다 `viewer.sync()`: Windows Docker/X 환경에서 지나치게 느려져 통합 데모에 sync rate 제한을 적용.
- Google Drive 프로젝트 ZIP만 완전한 Git 사본으로 복원: 추출본이 불완전해 GitHub clone본으로 교체.

## 12. 현재 정상 작동하는 부분

- GitHub `main`은 `94b751a`에서 clean하게 clone됨.
- 모든 자체 Python 파일의 구문 컴파일이 통과함.
- FSM의 오류/재검증 테스트와 invalid transition 테스트는 통과함.
- MuJoCo Menagerie UR5e/Robotiq 모델은 기존 압축 해제본의 `src/vendor`에 존재함.
- clone본에는 TRON1 `assets/vendor/objects_3da_v0/PF_TRON1A/xml/robot.xml`과 STL이 존재함.
- 대화상 확인된 단일 병 pick/place, TRON1 교대 보행, 단일 병 통합 handover는 정상 동작함.
- 세 병 순차 동작 코드와 충돌 회피용 배치 간격/상향 경유점이 실제 `main`에 반영되어 있음.

## 13. 현재 미해결 문제

1. **Ubuntu 네이티브 환경 구축 및 실행 검증 미완료**
   - MuJoCo 3.3.7/NumPy<2.0 설치, OpenGL renderer 확인, vendor 복구가 필요.

2. **하드코딩된 프로젝트 루트**
   - 통합/TRON1 스크립트가 `/workspace/3D_Automation`을 가정해 네이티브 clone 경로에서 바로 실행되지 않는다.

3. **`src/vendor`가 clone본에 없음**
   - `.gitignore` 대상이며 bootstrap은 TurtleBot3만 가져온다. MuJoCo Menagerie 설치 절차를 추가하거나 수동 복구해야 한다.

4. **FSM happy-path 테스트 실패**
   - 테스트 이벤트에 `Event.APPROACH_OK`가 누락됨. 구현 또는 테스트 중 무엇이 의도인지 확인하되 현재 상태 정의상 테스트 수정이 자연스럽다.

5. **세 병 최종 반복 안정성 미검증**
   - 앞서 배치된 병과의 충돌, place 후 upright 유지, 3개 전체 완료를 최소 여러 회 반복 확인해야 한다.

6. **FPS 문제의 근본 해결 미검증**
   - Ubuntu 네이티브 GPU/OpenGL renderer에서 측정해야 한다. 내장 그래픽이라도 Windows 소프트웨어 렌더링보다 개선될 가능성이 있지만 보장되지 않는다.

7. **MuJoCo 통합과 원래 ROS/Gazebo FSM의 연결 부족**
   - MuJoCo 데모는 자체 print 상태 시퀀스이며 ROS 2 FSM/인식/MoveIt과 실제로 연결되어 있지 않다.

8. **물리적 정합성**
   - 통합 데모가 병 pose를 강제로 유지하므로 실제 contact/grasp 안정성 평가와는 다르다. 발표용 안정 데모인지 물리 검증용인지 목표를 분리해야 한다.

9. **README/ADR 최신화 필요**
   - 최근 MuJoCo/TRON1/UR5e 및 세 병 데모가 문서에 충분히 반영되어 있지 않다.

## 14. 유지해야 할 설계 결정과 제약사항

- 제공된 원본 3D 자산을 직접 수정하지 않는다. 필요한 변환/단순 collision은 별도 파생 파일이나 코드로 관리한다.
- 안전 우선 원칙 `SAFE_STOP -> REVALIDATE -> RESUME/REPLAN`을 유지한다.
- 베이스가 완전히 정지한 뒤 팔을 움직이고, 베이스 이동 전 팔을 stow해야 한다.
- 물체 최대 질량은 0.30 kg이며 현재 MuJoCo 원통 병도 0.3 kg이다.
- TF 변환 실패나 검증 실패 시 계획/동작을 진행하지 않는 것이 원래 시스템 계약이다.
- MuJoCo 결합은 `MjSpec.attach()`에 정확한 `site` 또는 `frame`을 지정한다.
- 그리퍼 actuator 이름은 `gripper_fingers_actuator`, pinch site는 `gripper_pinch`를 전제로 한다.
- 통합 모델의 prefix(`gripper_`, `tron1_`)와 이름 조회 로직을 함께 유지해야 한다.
- 현재 세 병 로직은 정확히 3개를 가정하는 `range(3)`/`range(1, 4)`가 여러 곳에 있다. 병 수를 바꿀 때 상수 하나만 바꾸면 되는 구조가 아니다.
- FPS를 위해 렌더 sync 빈도와 physics timestep을 구분한다. sync를 줄이더라도 physics state update는 유지한다.
- 현재 kinematic attachment는 안정적인 시각 데모를 위한 의도적 선택이다. 이를 제거하면 slip 문제가 재발할 수 있다.

## 15. 사용자가 대화 중 결정하거나 요구한 중요한 사항

- 앞으로 코드를 제공할 때 들여쓰기를 완벽하게 맞춰야 한다.
- 사용자는 토큰 사용량을 아끼기 위해 일반적인 코드 수정은 직접 하고 싶어 한다.
- Codex가 파일을 직접 수정해야 할 때는 **먼저 무엇을 수정할지 알리고 허락을 받아야 한다**.
- 이번 `CODEX_HANDOFF.md` 생성은 사용자가 명시적으로 직접 생성을 요청했으므로 허가된 작업이다.
- 사용자는 무조코 시각 동작을 직접 여러 번 실행해 성공 여부를 확인하는 방식으로 검증한다.
- 단일 병 인계는 이미 만족스러운 수준으로 확인되었다.
- TRON1은 미끄러지는 이동이 아니라 좌우 다리를 번갈아 움직여야 한다.
- 세 병은 하나씩 순차적으로 옮겨야 하며, 새 병을 처리할 때 기존 배치 병을 건드리면 안 된다.
- Windows Docker의 낮은 FPS가 매우 큰 불편이어서 Ubuntu 22.04 네이티브 실행을 추진했다.
- 프로젝트는 GitHub와 Google Drive에 백업했다. WSL 전체 export도 Google Drive에 백업했다.
- 팀 공유는 GitHub clone/브랜치/PR 방식이 기본이며, Git에 없는 vendor는 별도 복구가 필요하다.

## 16. 다음 작업 우선순위

1. **Ubuntu clone본과 저장소 상태 확인**
   - `git status --short --branch`, `git log -1 --oneline`, remote를 확인한다.

2. **외부 의존성 복구**
   - `src/vendor/mujoco_menagerie/universal_robots_ur5e/scene.xml`
   - `src/vendor/mujoco_menagerie/robotiq_2f85/2f85.xml`
   - TurtleBot3 vendor는 필요 시 `scripts/bootstrap_dependencies.sh` 사용.

3. **네이티브 경로 처리**
   - `/workspace/3D_Automation` 하드코딩을 portable root 계산으로 바꿀지 사용자에게 먼저 설명하고 승인받는다.

4. **MuJoCo 3.3.7 네이티브 환경 구축 및 renderer 확인**
   - `numpy<2.0`, `mujoco==3.3.7`을 별도 venv에 설치.
   - `glxinfo -B` 등으로 실제 renderer가 `llvmpipe`가 아닌 내장 GPU인지 확인.

5. **단계별 smoke test**
   - UR5e/gripper preview → workcell → single pick/place → TRON1 preview → integrated cell → full handover 순서.

6. **FPS 측정**
   - 기존 체감 약 1 FPS와 비교하고, viewer sync/mesh/renderer 병목을 구분한다.

7. **FSM 테스트 수정 및 전체 테스트**
   - happy path에 `APPROACH_OK`를 반영한 뒤 pytest/colcon test를 실행한다.

8. **세 병 반복 안정성 검증**
   - 최소 3~5회 실행하며 각 병의 grasp, lift, place, upright, 기존 병 비접촉 여부를 기록한다.

9. **필요 시 궤적 개선**
   - 기존 병 충돌이 남으면 배치 순서, x 간격, lateral escape waypoint, retreat 높이를 조정한다.

10. **문서 최신화 및 커밋**
    - README와 ADR을 현재 MuJoCo 통합 상태에 맞게 갱신하고 `CODEX_HANDOFF.md`와 함께 사용자 확인 후 커밋/푸시한다.

## 17. 새 Codex가 가장 먼저 확인할 사항

새 세션은 다음 순서로 시작해야 한다.

```bash
pwd
git status --short --branch
git log -5 --oneline
git remote -v
```

그다음 필수 파일을 확인한다.

```bash
test -f docker/Dockerfile
test -f assets/vendor/objects_3da_v0/PF_TRON1A/xml/robot.xml
test -f src/vendor/mujoco_menagerie/universal_robots_ur5e/scene.xml
test -f src/vendor/mujoco_menagerie/robotiq_2f85/2f85.xml
```

Python/그래픽 환경을 확인한다.

```bash
python3 --version
python3 -c "import mujoco, numpy; print(mujoco.__version__, numpy.__version__)"
glxinfo -B
```

마지막으로 코드를 고치기 전에 반드시 다음을 지킨다.

1. 사용자가 직접 수정할지 Codex가 수정할지 먼저 확인한다.
2. Codex가 직접 수정한다면 대상 파일과 변경 내용을 먼저 알린다.
3. 제공하는 모든 코드 블록은 복사 즉시 사용할 수 있도록 들여쓰기를 정확히 유지한다.
4. 실행 전 현재 `main`과 working tree를 확인하고, 사용자 변경을 덮어쓰지 않는다.
5. 먼저 단일 병의 기존 성공 동작을 보존한 뒤 세 병 기능을 검증한다.

## 18. 백업/복구 참고

- 프로젝트 GitHub: `https://github.com/JoHJ3753/3D_Automation`
- 프로젝트 ZIP 백업 당시 SHA-256:
  `4598C891EDC0AF21A0835C001CABC0E1C97CA8DA2001AAE1A3A41F2C145F1FAA`
- 백업 ZIP 당시 크기: `1,170,670,654` bytes
- WSL export 백업: `Ubuntu-22.04-D_2026-09-08.tar`
- WSL export 당시 크기: `21,028,976,640` bytes

GitHub clone본을 코드/이력의 기준으로 사용하고, Google Drive ZIP은 Git에서 제외된 `src/vendor` 같은 대용량 외부 자료 복구용으로 사용한다.
