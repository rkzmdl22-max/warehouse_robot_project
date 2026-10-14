# AI Multi-Robot Smart Warehouse

**한국IT교육원 기업연계 프로젝트**

## 폴더 구역 (파일 꼬임 방지)

| 구역 | 경로 | 설명 |
|------|------|------|
| **Gazebo 레거시** | `ros_ws/` | ROS2 Humble + Gazebo (기존 작업) |
| **MuJoCo TIAGo** | `ros_ws/mjcf_convert/` | Stage 01~02 MJCF |
| **Jazzy 통합 (신규)** | `ros2_jazzy_ws/` | ROS2 Jazzy + mujoco_bridge |
| **MuJoCo Scene** | `mujoco/` | 창고 scene (최종 Simulation Core) |
| **설정** | `config/` | 좌표·로봇·fleet 설정 |
| **문서** | `docs/` | 재배치·갭분석·ChatGPT 규칙 |

## 빠른 시작

```bash
# Gazebo (기존)
ros2 launch warehouse_robot warehouse_lite.launch.py

# MuJoCo TIAGo viewer
python3 ros_ws/mjcf_convert/view_tiago.py

# Jazzy bridge (Jazzy Docker 내)
cd ros2_jazzy_ws && colcon build && source install/setup.bash
ros2 launch warehouse_bringup warehouse_jazzy.launch.py
```

## 필수 문서

- `docs/CHATGPT_WORKFLOW.md` — ChatGPT 병행 규칙
- `docs/REORGANIZATION_PLAN.md` — 폴더 재배치 계획
- `docs/GAP_ANALYSIS.md` — 스펙 대비 현황
- `docs/GAZEBO_TO_MUJOCO.md` — 좌표 이식 체크리스트
- `config/warehouse_layout.yaml` — 좌표 Single Source of Truth

## FINAL RULE

```
MuJoCo = Simulation Core
ROS2 Jazzy = Integration Layer
Gazebo ros_ws = Legacy (삭제 금지)
```
