# 3D Automation

TurtleBot3 Waffle Pi와 OpenMANIPULATOR-X가 300g 이하의 물체를 인식하고 Pick & Place하는 ROS 2/Gazebo 학습·발표용 모바일 매니퓰레이션 프로젝트입니다.

## 기준 범위

1. TurtleBot3가 목표 작업 위치까지 이동하고 완전히 정지한다.
2. RGB-D 카메라가 300g 이하의 대상 물체를 검출한다.
3. 물체의 3D 중심과 파지 접근 자세를 계산한다.
4. OpenMANIPULATOR-X가 접근·파지하고 파지 상태를 확인한다.
5. 팔을 안전 자세로 복귀한 뒤 지정 위치로 이동하여 물체를 배치한다.

원본 PRD의 안전 우선, `SAFE_STOP -> REVALIDATE -> RESUME/REPLAN` 원칙은 유지합니다. 다만 MVP에서는 위 정상 시나리오를 먼저 구현하고 작업자 접근·장애물·낙하 예외는 후속 단계에서 통합합니다.

## 개발 환경

- Host: Windows 10 + Docker Desktop
- Container: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, Nav2, MoveIt 2
- IDE: Antigravity에서 이 폴더를 열고 통합 터미널 사용
- GUI: Windows에 VcXsrv/X410 등 X server가 필요합니다. `.env.example`을 `.env`로 복사한 뒤 접근 허용 범위를 로컬로 제한하세요.

## 시작

```powershell
Copy-Item .env.example .env
docker compose build
docker compose run --rm ros
```

컨테이너 안에서:

```bash
./scripts/bootstrap_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
pytest -q src/handover_fsm/test
ros2 run handover_fsm handover_fsm_node
```

공식 시뮬레이션은 `ros2 launch turtlebot3_manipulation_gazebo gazebo.launch.py`로 실행합니다. 프로젝트 통합 실행은 `ros2 launch automation_cell_bringup demo.launch.py`를 사용합니다.

## 저장소 구조

```text
assets/vendor/objects_3da_v0/  제공받은 원본 3D 자산
config/                        셀·카메라·파지 파라미터
docker/                        재현 가능한 ROS 2 환경
docs/                          요구사항과 자산 적합성 기록
src/handover_fsm/              작업 상태기계 ROS 2 패키지
```

## 즉시 필요한 기업 자료

- 요청에 언급된 전체 Gazebo 시뮬레이션 패키지
- 매니퓰레이터의 정확한 모델명과 Gazebo/MoveIt 2용 URDF·SRDF·컨트롤러 설정
- TRON1 트레이 프레임과 인계 정지 포즈
- 물병 실물 치수·질량·마찰계수, 허용 파지 높이
- 사용할 RGB-D 카메라 모델 또는 최소 FOV·해상도·깊이 범위

상세 판단은 `docs/00_project_baseline.md`와 `docs/01_asset_audit.md`에 정리했습니다.
