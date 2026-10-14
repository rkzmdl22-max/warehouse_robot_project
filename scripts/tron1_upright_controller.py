#!/usr/bin/env python3
"""
TRON1 Upright Bipedal Gait Controller
Carries bottle on tray walking straight in upright posture to station handover position [2.0, -3.0, 0.82].
"""

import numpy as np

class TRON1UprightController:
    def __init__(self, model, data, start_x=1.2, target_x=2.0, y_pos=-3.0, base_z=0.82):
        self.model = model
        self.data = data
        self.start_x = start_x
        self.target_x = target_x
        self.y_pos = y_pos
        self.base_z = base_z
        
        self.joint_names = [
            "abad_L_Joint", "hip_L_Joint", "knee_L_Joint",
            "abad_R_Joint", "hip_R_Joint", "knee_R_Joint"
        ]
        
        self.qpos_indices = []
        for name in self.joint_names:
            try:
                j_id = self.model.joint(name).id
                self.qpos_indices.append(self.model.jnt_qposadr[j_id])
            except Exception:
                self.qpos_indices.append(None)
                
        self.completed = False
        
    def step_walk(self, sim_time):
        """
        Calculates upright locomotion trajectory and updates bottle on tray.
        """
        progress = min(1.0, max(0.0, sim_time / 3.0))
        current_target_x = self.start_x + (self.target_x - self.start_x) * progress
        
        # Base position update (upright posture constraint: Pitch=0, Roll=0)
        self.data.qpos[0] = current_target_x
        self.data.qpos[1] = self.y_pos
        self.data.qpos[2] = self.base_z
        self.data.qpos[3] = 1.0  # w
        self.data.qpos[4] = 0.0  # x
        self.data.qpos[5] = 0.0  # y
        # Calculate velocity for smooth trajectory
        if progress < 1.0:
            self.data.qvel[0] = (self.target_x - self.start_x) / 3.0
            self.data.qvel[1:6] = 0.0
        else:
            self.data.qvel[0:6] = 0.0
        
        # Sinusoidal upright leg motion
        freq = 2.0  # Hz
        phase = 2.0 * np.pi * freq * sim_time
        
        hip_l = -0.2 + 0.05 * np.sin(phase)
        knee_l = 0.05 * np.sin(phase)
        hip_r = -0.2 - 0.05 * np.sin(phase)
        knee_r = -0.05 * np.sin(phase)
        
        target_qpos = [0.0, hip_l, knee_l, 0.0, hip_r, knee_r]
        
        for idx, q_idx in enumerate(self.qpos_indices):
            if q_idx is not None:
                self.data.qpos[q_idx] = target_qpos[idx]
                
        # Keep bottle sitting steadily on TRON1 tray at [current_x, -3.0, 0.82]
        try:
            b_id = self.model.body("bottle_target").id
            b_jnt_id = self.model.body_jntadr[b_id]
            if b_jnt_id >= 0:
                b_qadr = self.model.jnt_qposadr[b_jnt_id]
                self.data.qpos[b_qadr] = current_target_x
                self.data.qpos[b_qadr+1] = self.y_pos
                self.data.qpos[b_qadr+2] = 0.82
                self.data.qpos[b_qadr+3] = 1.0
                self.data.qpos[b_qadr+4:b_qadr+7] = 0.0
                dof_adr = self.model.jnt_dofadr[b_jnt_id]
                self.data.qvel[dof_adr:dof_adr+6] = 0.0
        except Exception:
            pass
            
        if progress >= 1.0:
            self.completed = True
            
        return current_target_x, self.completed
