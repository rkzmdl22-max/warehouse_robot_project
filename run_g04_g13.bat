@echo off
cd /d c:\work\warehouse_robot_project
.venv_mujoco360\Scripts\python.exe ros_ws\mujoco\final_evidence\logs\kiro_g04_g13.py > ros_ws\mujoco\final_evidence\logs\kiro_g04_g13_result.txt 2>&1
echo Exit code: %errorlevel% >> ros_ws\mujoco\final_evidence\logs\kiro_g04_g13_result.txt
