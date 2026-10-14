@echo off
cd /d c:\work\warehouse_robot_project
.venv_mujoco360\Scripts\python.exe ros_ws\mujoco\final_evidence\logs\G14_G18_FINAL_HOLD_TO_RELEASE.py > ros_ws\mujoco\final_evidence\logs\G14_G18_RESULT.txt 2>&1
echo Exit code: %errorlevel% >> ros_ws\mujoco\final_evidence\logs\G14_G18_RESULT.txt
