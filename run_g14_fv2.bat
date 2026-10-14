@echo off
cd /d c:\work\warehouse_robot_project
.venv_mujoco360\Scripts\python.exe ros_ws\mujoco\final_evidence\logs\G14_FIXED_V2.py > ros_ws\mujoco\final_evidence\logs\G14_FIXED_V2_RESULT.txt 2>&1
echo Exit: %errorlevel% >> ros_ws\mujoco\final_evidence\logs\G14_FIXED_V2_RESULT.txt
