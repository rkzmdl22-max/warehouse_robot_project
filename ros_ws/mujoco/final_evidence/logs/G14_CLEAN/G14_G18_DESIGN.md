# G14-G18 CLEAN DESIGN — PROJECT MASTER v5.3.2

## 0. Scope
- 대상: MuJoCo TRON1 handover manipulation sequence
- 범위: G14 Dynamic Grasp Hold → G15 Lift → G16 Carry → G17 Place → G18 Release
- G13 PASS 상태를 입력 조건으로 사용한다.
- 기존 G14 V2~V6 및 과거 실험 controller는 구현 근거로 재사용하지 않는다.
- Protected XML 및 기존 G13 자산은 변경하지 않는다.
- Gazebo는 본 단계에서 실행하지 않는다.

## 1. Common Runtime State

### Object State
At every control cycle estimate:
- bottle position p_b ∈ R^3
- bottle linear velocity v_b ∈ R^3
- bottle validity / finite state

### Grasp State
Maintain:
- initial bottle position p_b0
- initial finger-center position p_f0
- relative grasp offset r_g = p_b0 - p_f0
- bilateral contact state
- contact-loss counter
- tracking error

Desired finger-center:
    p_f* = p_b - r_g

This makes the grasp target follow the measured bottle state rather than fixing the bottle or fixing the world coordinate.

## 2. Controller Architecture

Control loop:

1. read current MuJoCo state
2. forward dynamics/kinematics
3. estimate bottle position and velocity
4. estimate left/right finger positions and velocities
5. calculate finger-center state
6. calculate desired finger-center trajectory
7. calculate Cartesian tracking error
8. calculate Cartesian desired velocity/acceleration
9. map Cartesian command to the six arm DOFs using the current finger-center Jacobian
10. apply bounded joint command
11. maintain bilateral finger closure
12. evaluate safety/contact/finite conditions
13. record runtime evidence

The controller must be closed-loop. No open-loop fixed joint trajectory is sufficient for G14-G16.

## 3. G14 — Dynamic Grasp Hold

### Entry
- G13 PASS
- bottle and both fingers in grasp configuration
- gravity enabled for official hold
- finite state

### Objective
Maintain the bottle inside the grasp while the system responds to its measured motion.

### Required
- real-time bottle position
- real-time bottle velocity
- real-time finger-center position/velocity
- relative grasp offset
- Jacobian-based tracking
- bilateral finger closure
- bounded commands
- continuous contact monitoring

### PASS
For the complete 5.0 s interval:
- bilateral bottle/finger contact continuously maintained
- no contact-loss event
- bottle drift < 0.025 m
- finite/stable state
- controller remains within command limits

## 4. G15 — Lift

### Entry
- G14 PASS state
- bilateral grasp maintained

### Objective
Lift the grasped bottle vertically while preserving the relative grasp.

### Required
- dynamic grasp controller remains active
- commanded lift trajectory
- real-time bottle state feedback
- continuous bilateral contact

### PASS
- complete prescribed lift trajectory
- no contact loss
- grasp-relative error remains bounded
- bottle reaches lift target
- finite/stable state

## 5. G16 — Carry

### Entry
- G15 PASS

### Objective
Transport the grasped bottle from lift state to the designated carry/place approach region.

### Required
- dynamic tracking remains active during motion
- bottle position/velocity feedback
- continuous grasp maintenance
- bounded joint commands

### PASS
- complete carry trajectory
- no contact loss
- bottle remains within grasp envelope
- target approach state reached
- finite/stable state

## 6. G17 — Place

### Entry
- G16 PASS
- bottle remains grasped

### Objective
Move the bottle to the designated placement pose.

### Required
- dynamic tracking until placement pose
- controlled final approach
- placement position verification
- contact state monitoring

### PASS
- placement target reached within defined tolerance
- bottle stable at placement location
- no unintended release before placement
- finite/stable state

## 7. G18 — Release

### Entry
- G17 PASS
- bottle stable at placement location

### Objective
Release the bottle by opening both fingers without disturbing the placed bottle.

### Required
- controlled finger opening
- arm withdrawal/hold as required
- bottle separation verification

### PASS
- both fingers open
- bottle no longer held by grasp contacts
- bottle remains at placement target
- no unintended displacement beyond placement tolerance
- final state finite/stable

## 8. Shared Safety Rules

Abort the runtime test if:
- NaN/Inf appears
- joint command exceeds actuator range
- uncontrolled joint velocity occurs
- bilateral grasp is lost when grasp is required
- object leaves the valid workspace
- simulation becomes numerically unstable

An aborted run is FAIL evidence, not PASS evidence.

## 9. Evidence Rules

Each gate requires runtime evidence.

Static XML/code inspection can establish configuration only.
It cannot establish G14-G18 PASS.

Each gate record must contain:
- timestamp
- gate
- start state
- end state
- relevant positions
- relevant velocities
- tracking error
- contact state
- maximum command/velocity
- PASS/FAIL reason

## 10. Implementation Rule

Implementation proceeds in this order:

A. freeze this design
B. inspect only the minimum existing G13 runtime interface required
C. create one clean common manipulation controller
D. validate controller initialization/state estimation without claiming gate PASS
E. validate G14
F. extend the same controller for G15
G. extend for G16
H. extend for G17
I. extend for G18
J. run regression

No V4/V5/V6 patching.
No arbitrary controller replacement after every failed 5 s run.
No XML modification unless a demonstrated requirement makes it unavoidable.

## 11. Official Gate Chain

G13 PASS
  ↓
G14 Dynamic Grasp Hold
  ↓
G15 Lift
  ↓
G16 Carry
  ↓
G17 Place
  ↓
G18 Release

A later gate cannot be marked PASS using an unverified earlier gate.

