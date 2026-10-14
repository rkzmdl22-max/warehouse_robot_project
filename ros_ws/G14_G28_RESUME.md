# G14_G28_RESUME CHECKPOINT
# PROJECT MASTER v5.3.2

## RESUME CHECKPOINT

PROJECT: PROJECT MASTER v5.3.2
LAST VERIFIED PASS: G13
CURRENT GATE: G14
CURRENT ACTION: wrist scan for bilateral contact stability
LAST COMMAND: run_g14_fv3.bat (G14_FIXED_V3.py)
LAST RESULT:
  G14_FIXED_V2_RESULT.txt:
    drift=0.0m (bottle did not move -- TRON1 fixed working)
    contact_loss at step=2000 t=4.0s (rc=False)
    rq near range max (0.07) -- right finger pushed out

FILES MODIFIED:
  G14_FIXED_V2.py (new)
  G14_FIXED_V3.py (new, wrist scan)
  run_g14_fv2.bat (new)
  run_g14_fv3.bat (new)
  CURRENT_STATUS.md (updated)

BACKUPS:
  G14_G18_V3_TEST.bak_ctrl0_20260916
  G14_G18_V2_FINAL.py
  G14_G18_FINAL_HOLD_TO_RELEASE.py

LATEST EVIDENCE:
  G14_FIXED_V2_RESULT.txt -- drift=0.0m, contact_loss t=4s
  G14_CAUSE_RESULT.txt -- root cause confirmed
  G14_STEPB_RESULT.txt -- 10s diagnosis
  G17_IK_RESULT.txt -- carry pan=-2.72rad
  kiro_g04_g13_result.txt -- G13 PASS

FAILURE/BLOCKER:
  Right contact lost at t=4.0s
  rq oscillates near range max (0.07)
  Cause: approach angle causes asymmetric force on right finger

NEXT ACTION:
  Wait for G14_FIXED_V3_RESULT.txt
  If scan finds score=4 (g13+contact_sustained+drift<0.025):
    Use that Q as new best_q
    Run official G14 5s hold
    If PASS -> proceed to G15

DO NOT RESTART:
  G01~G13 all PASS -- do not rerun
  drift=0.0m is achieved -- TRON1 fix works
  Only need stable bilateral contact for 5s

CURRENT PROGRESS:
  overall: 12/28 = 42.86%
  MuJoCo: 10/16 = 62.5%
  Gazebo: not started (after G18)

---

## COPY-READY RESUME PROMPT

RESUME PROJECT MASTER v5.3.2 FROM CURRENT CHECKPOINT

Continue the existing project. DO NOT rebuild.

MASTER: PROJECT MASTER v5.3.2 FINAL

Verified PASS: G01,G02,G03,G04,G05,G06,G07,G08,G09,G10,G11,G12,G13

Current Gate: G14 Dynamic Grasp Hold

Current status: FAIL -- right contact lost at t=4.0s

Last verified command:
  .venv_mujoco360\Scripts\python.exe ros_ws\mujoco\final_evidence\logs\G14_FIXED_V3.py

Last verified result (G14_FIXED_V2_RESULT.txt):
  drift=0.0m (TRON1 fixed -- bottle stationary)
  contact_loss=2000 (t=4.0s, rc=False)
  rq near max 0.07 -- right finger asymmetric

Current files:
  ros_ws/mujoco/final_evidence/logs/G14_FIXED_V2.py (completed)
  ros_ws/mujoco/final_evidence/logs/G14_FIXED_V3.py (running/pending result)
  ros_ws/mujoco/final_evidence/logs/G14_G18_V3_TEST.py (G15-G18 script, ready after G14)
  ros_ws/mujoco/final_evidence/logs/G17_IK_RESULT.txt (carry pan=-2.72rad)

Backups:
  G14_G18_V3_TEST.bak_ctrl0_20260916

Current blocker:
  Right contact asymmetric. rq pushed to range max.
  Need wrist angle adjustment for symmetric bilateral contact.

NEXT ACTION:
  1. Read G14_FIXED_V3_RESULT.txt
  2. Find best wrist config (score=4: g13+contact_sustained+drift<0.025)
  3. Run official G14 5s hold with best config
  4. If G14 PASS -> update CURRENT_STATUS.md
  5. Proceed to G15 (G14_G18_V3_TEST.py with TRON1-fixed + best Q)

G14 official condition (LOCKED -- do not change):
  5-second actual runtime hold
  contact_loss < 0 (never lost)
  drift < 0.025m
  left contact maintained
  right contact maintained
  bottle finite, no hold failure

Key parameters:
  Q0 = [-1.0356, 1.3959, 0.3225, -0.1977, -0.9749, 0.0]
  TRON1_HANDOVER = [0.0, -3.0, 0.82]
  finger_ctrl = -0.015
  TRON1 fix: qpos[root:root+7] = [0,−3,0.82,1,0,0,0], qvel[dof:dof+6]=0
  XML = warehouse_tron1_final_visual.xml (FROZEN, nbody=54)

Protect existing PASS. Do not modify Gate definitions.
Do not modify frozen XML unless proven necessary.
Continue autonomously. Update CURRENT_STATUS.md after every major state change.
If message limit approaches, update this file and output copy-ready prompt.
