# G14-G18 CLEAN IMPLEMENTATION CONTRACT
# PROJECT MASTER v5.3.2 FINAL

## 1. Single Pipeline

One runtime controller owns the complete manipulation sequence:

G13_ENTRY
  -> G14_HOLD
  -> G15_LIFT
  -> G16_CARRY
  -> G17_PLACE
  -> G18_RELEASE
  -> COMPLETE

There is no separate V4/V5/V6 controller per gate.

## 2. Protected Inputs

Protected model:
  /ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml

G13 validated entry:
  root_qposadr = 28
  handover_root = [0.0, -3.0, 0.82]

Arm joints:
  shoulder_pan_joint
  shoulder_lift_joint
  elbow_joint
  wrist_1_joint
  wrist_2_joint
  wrist_3_joint

Finger joints:
  left_finger_joint
  right_finger_joint

Arm actuators:
  act_shoulder_pan
  act_shoulder_lift
  act_elbow
  act_wrist_1
  act_wrist_2
  act_wrist_3

Finger actuators:
  act_left_finger
  act_right_finger

Object:
  carry_bottle

Finger links:
  left_finger_link
  right_finger_link

## 3. State Estimator

Every control cycle computes:

p_b = bottle world position
v_b = bottle linear velocity

p_l = left finger world position
p_r = right finger world position

p_f = 0.5 * (p_l + p_r)

v_l = left finger linear velocity
v_r = right finger linear velocity

v_f = 0.5 * (v_l + v_r)

relative_grasp = p_b - p_f

All state vectors must be finite.

## 4. Grasp Reference

At the valid G13 entry state:

p_b0 = initial bottle position
p_f0 = initial finger-center position

r_grasp = p_b0 - p_f0

The manipulation controller does NOT attempt to freeze the bottle in world coordinates.

Instead:

p_grasp_follow = p_b - r_grasp

This preserves the measured bottle/finger relationship.

## 5. Task Offset

Each task adds a controlled offset from the initial grasp reference.

G14:
  task_offset = [0, 0, 0]

G15:
  task_offset = vertical lift trajectory

G16:
  task_offset = carry trajectory

G17:
  task_offset = placement trajectory

G18:
  task_offset = release/withdraw trajectory

The desired finger center is:

p_des = p_b - r_grasp + task_offset

Therefore object tracking and task motion are combined rather than competing controllers.

## 6. Cartesian Tracking

For the arm:

J_l = positional Jacobian of left finger
J_r = positional Jacobian of right finger

J_f = 0.5 * (J_l + J_r)

The controller maps Cartesian command to the six arm joints.

Required behavior:

- closed loop
- current-state feedback every cycle
- no accumulated stale target
- bounded joint command
- bounded joint velocity
- finite-value checks

No fixed open-loop q trajectory is permitted for the grasped-object phases.

## 7. Dynamic Terms

The controller must account for the actual MuJoCo dynamic state.

Required measured quantities include:

q
qdot
qfrc_bias
J_f
bottle velocity
finger velocity

Dynamic compensation must be derived from the current runtime state.

Do not assume a constant gravity compensation vector.

## 8. Finger Control

G14-G17:
  both finger actuators remain in controlled closed state.

Contact verification:
  carry_bottle_collision ↔ left_finger_geom
  carry_bottle_collision ↔ right_finger_geom

G18:
  command both finger actuators to open
  verify grasp contacts are released
  verify bottle remains at placement target

## 9. State Machine

### G14_HOLD
Entry:
  valid G13 grasp
  bilateral contact

Command:
  task_offset = [0,0,0]

Requirement:
  5 seconds continuous stable grasp

PASS:
  bilateral contact continuously true
  bottle drift < 0.025 m
  finite state
  command limits respected

### G15_LIFT
Entry:
  G14 PASS

Command:
  smooth vertical trajectory
  grasp tracking remains active

Requirement:
  reach lift target without contact loss

PASS:
  target reached
  bilateral grasp maintained
  finite state

### G16_CARRY
Entry:
  G15 PASS

Command:
  smooth transport trajectory

Requirement:
  bottle follows transport path while grasp remains active

PASS:
  carry target reached
  no contact loss
  finite state

### G17_PLACE
Entry:
  G16 PASS

Command:
  smooth final placement trajectory

Requirement:
  controlled approach and stabilization

PASS:
  placement tolerance satisfied
  bottle stable
  grasp still active

### G18_RELEASE
Entry:
  G17 PASS

Command:
  open both fingers
  controlled withdrawal if required

PASS:
  both grasp contacts released
  bottle remains within placement tolerance
  finite state

## 10. Trajectory Requirements

Task trajectories must be generated continuously from the current state.

No instantaneous position teleportation during a gate.

Each trajectory must expose:

  position
  velocity
  acceleration

for deterministic controller input.

Trajectory transitions must be continuous.

## 11. Safety

Immediate FAIL/abort on:

- NaN
- Inf
- actuator command outside ctrlrange
- unbounded joint velocity
- numerical instability
- required bilateral contact loss
- object leaving valid workspace

No FAIL condition may be hidden by resetting state during the official measurement interval.

## 12. Evidence

The clean implementation must generate:

  G14_G18_RUNTIME.csv
  G14_G18_RESULT.txt

Runtime CSV fields must include at minimum:

timestamp
gate
bottle_x
bottle_y
bottle_z
bottle_vx
bottle_vy
bottle_vz
finger_x
finger_y
finger_z
tracking_error
left_contact
right_contact
max_joint_velocity
finite_ok

Gate PASS is determined only from runtime evidence.

## 13. Implementation Phases

PHASE A:
  model/interface initialization

PHASE B:
  G13 entry reconstruction

PHASE C:
  common bottle/finger state estimator

PHASE D:
  common grasp tracking controller

PHASE E:
  G14 hold

PHASE F:
  G15 lift

PHASE G:
  G16 carry

PHASE H:
  G17 place

PHASE I:
  G18 release

PHASE J:
  integrated G14-G18 runtime validation

No phase creates a second competing controller.

## 14. Non-Goals

Do not:
- modify protected XML
- modify Gazebo world
- run Gazebo
- reuse archived G14 V2/V3/V4/V5/V6 code
- perform another blind 5-second experiment before controller construction
- declare PASS from static inspection

## 15. Final Acceptance Chain

G13 PASS
AND
G14 PASS
AND
G15 PASS
AND
G16 PASS
AND
G17 PASS
AND
G18 PASS

Only then can the manipulation sequence proceed to the later project gates.
