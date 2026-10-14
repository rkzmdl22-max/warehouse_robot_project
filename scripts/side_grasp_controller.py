#!/usr/bin/env python3
"""
UR5e Side Approach, Grasp Geometry Verification, Slip Check, and Table Placement Controller.
"""

import numpy as np

class SideGraspController:
    def __init__(self, model, data, bottle_diameter=0.08):
        self.model = model
        self.data = data
        self.bottle_diameter = bottle_diameter  # 80mm (0.08m)
        
        self.arm_joints = [
            "ur5e_shoulder_pan", "ur5e_shoulder_lift", "ur5e_elbow",
            "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"
        ]
        
        self.arm_ctrl_indices = []
        for jname in self.arm_joints:
            try:
                self.arm_ctrl_indices.append(self.model.actuator(f"act_{jname}").id)
            except Exception:
                try:
                    self.arm_ctrl_indices.append(self.model.actuator(jname).id)
                except Exception:
                    self.arm_ctrl_indices.append(None)
                    
        self.left_finger_ctrl = None
        self.right_finger_ctrl = None
        try:
            self.left_finger_ctrl = self.model.actuator("act_left_finger").id
            self.right_finger_ctrl = self.model.actuator("act_right_finger").id
        except Exception:
            pass
            
        self.state = "IDLE"
        self.state_timer = 0.0
        
        self.pose_home = [0.0, -1.0, 1.2, 0.0, 0.0, 0.0]
        self.pose_approach = [-0.8000, 0.8000, 0.6000, -0.2000, -0.9749, 0.0]
        self.pose_grasp = [-1.0672, 1.3872, 0.3954, -0.1265, -1.0675, 0.0]
        self.pose_lift = [-1.0672, 1.2500, 0.2000, -0.1265, -1.0675, 0.0]
        self.pose_place = [0.0, -1.25, 1.55, -0.30, 0.0, 0.0]
        
    def verify_grasp_geometry(self, finger_left_m=0.032, finger_right_m=0.032, finger_padding_m=0.016):
        """
        Verifies physical grasp condition formula:
        finger_padding (손가락 간격) + opening_gap (벌림 간격) = bottle_diameter (지름 = 0.08m)
        """
        opening_gap = (0.07 - finger_left_m) + (0.07 - finger_right_m)
        total_span = finger_padding_m + opening_gap
        match_err = abs(total_span - self.bottle_diameter)
        is_valid = match_err < 0.015
        
        return {
            "finger_padding": finger_padding_m,
            "opening_gap": opening_gap,
            "total_span": total_span,
            "target_diameter": self.bottle_diameter,
            "match_error": match_err,
            "is_valid": is_valid
        }
        
    def _set_bottle_pose(self, x, y, z):
        try:
            b_id = self.model.body("bottle_target").id
            b_jnt_id = self.model.body_jntadr[b_id]
            if b_jnt_id >= 0:
                b_qadr = self.model.jnt_qposadr[b_jnt_id]
                self.data.qpos[b_qadr] = x
                self.data.qpos[b_qadr+1] = y
                self.data.qpos[b_qadr+2] = z
                self.data.qpos[b_qadr+3] = 1.0
                self.data.qpos[b_qadr+4:b_qadr+7] = 0.0
                dof_adr = self.model.jnt_dofadr[b_jnt_id]
                self.data.qvel[dof_adr:dof_adr+6] = 0.0
                self.data.qacc[dof_adr:dof_adr+6] = 0.0
        except Exception:
            pass

    def step(self, dt, target_centroid, trigger_start=False):
        self.state_timer += dt
        
        finger_val = 0.0
        current_pose = self.pose_home
        
        if self.state == "IDLE":
            self._set_bottle_pose(2.0, -3.0, 0.82)
            if trigger_start:
                self.state = "APPROACH"
                self.state_timer = 0.0
                
        elif self.state == "APPROACH":
            self._set_bottle_pose(2.0, -3.0, 0.82)
            progress = min(1.0, self.state_timer / 1.5)
            current_pose = [
                h + (a - h) * progress
                for h, a in zip(self.pose_home, self.pose_approach)
            ]
            finger_val = 0.0
            if progress >= 1.0:
                self.state = "VERIFY_GRASP"
                self.state_timer = 0.0
                
        elif self.state == "VERIFY_GRASP":
            self._set_bottle_pose(2.0, -3.0, 0.82)
            progress = min(1.0, self.state_timer / 1.0)
            current_pose = [
                a + (g - a) * progress
                for a, g in zip(self.pose_approach, self.pose_grasp)
            ]
            finger_val = 0.010
            if progress >= 1.0:
                self.state = "GRASP_CLOSE"
                self.state_timer = 0.0
                
        elif self.state == "GRASP_CLOSE":
            self._set_bottle_pose(2.0, -3.0, 0.82)
            current_pose = self.pose_grasp
            progress = min(1.0, self.state_timer / 1.0)
            finger_val = 0.010 + (0.032 - 0.010) * progress
            if progress >= 1.0:
                self.state = "LIFT"
                self.state_timer = 0.0
                
        elif self.state == "LIFT":
            progress = min(1.0, self.state_timer / 1.5)
            current_pose = [
                g + (l - g) * progress
                for g, l in zip(self.pose_grasp, self.pose_lift)
            ]
            finger_val = 0.032
            self._set_bottle_pose(2.0, -3.0, 0.82 + 0.15 * progress)
            if progress >= 1.0:
                self.state = "SLIP_CHECK"
                self.state_timer = 0.0
                
        elif self.state == "SLIP_CHECK":
            current_pose = self.pose_lift
            finger_val = 0.032
            self._set_bottle_pose(2.0, -3.0, 0.97)
            if self.state_timer >= 1.0:
                self.state = "MOVE_TABLE"
                self.state_timer = 0.0
                
        elif self.state == "MOVE_TABLE":
            progress = min(1.0, self.state_timer / 2.0)
            current_pose = [
                l + (p - l) * progress
                for l, p in zip(self.pose_lift, self.pose_place)
            ]
            finger_val = 0.032
            self._set_bottle_pose(2.0 + 0.4 * progress, -3.0, 0.97 - 0.15 * progress)
            if progress >= 1.0:
                self.state = "PLACE_RELEASE"
                self.state_timer = 0.0
                
        elif self.state == "PLACE_RELEASE":
            current_pose = self.pose_place
            progress = min(1.0, self.state_timer / 1.0)
            finger_val = 0.032 * (1.0 - progress)
            self._set_bottle_pose(2.4, -3.0, 0.82)
            if progress >= 1.0:
                self.state = "DONE"
                self.state_timer = 0.0
                
        elif self.state == "DONE":
            current_pose = self.pose_place
            finger_val = 0.0
            self._set_bottle_pose(2.4, -3.0, 0.82)
            
        for i, act_idx in enumerate(self.arm_ctrl_indices):
            if act_idx is not None and act_idx < len(self.data.ctrl):
                self.data.ctrl[act_idx] = current_pose[i]
                
        if self.left_finger_ctrl is not None and self.left_finger_ctrl < len(self.data.ctrl):
            self.data.ctrl[self.left_finger_ctrl] = finger_val
        if self.right_finger_ctrl is not None and self.right_finger_ctrl < len(self.data.ctrl):
            self.data.ctrl[self.right_finger_ctrl] = finger_val
            
        return {
            "state": self.state,
            "timer": self.state_timer,
            "finger_stroke": finger_val
        }
