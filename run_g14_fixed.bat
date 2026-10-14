@echo off
cd /d c:\work\warehouse_robot_project
.venv_mujoco360\Scripts\python.exe ros_ws\mujoco\final_evidence\logs\G14_TRON1_FIXED_TEST.py > ros_ws\mujoco\final_evidence\logs\G14_TRON1_FIXED_RESULT.txt 2>&1
echo Exit: %errorlevel% >> ros_ws\mujoco\final_evidence\logs\G14_TRON1_FIXED_RESULT.txt
