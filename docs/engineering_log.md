## 2026-04-22 — RK4 Actuator Instability

**Context**  
During off-axis thrust testing, the gimbal pitch state exhibited high-frequency oscillations under RK4 integration. The same system behaved correctly under explicit Euler integration.

**Expected Behavior**  
A first-order actuator model should produce a smooth, monotonic response toward the commanded angle.

**Observed Behavior**
- Rapid oscillation in `gimbal_pitch`
- Instability only under RK4 (Euler stable)

**Root Cause**  
Incorrect RK4 averaging in `addRK4()`:
- Actuator derivatives used `/ (1.0/6.0)` instead of `* (1.0/6.0)`
- This amplified actuator derivatives by a factor of 6

**Fix**
- Corrected RK4 weighting for:
    - `throttle_dot`
    - `gimbal_pitch_dot`
    - `gimbal_yaw_dot`

**Validation**
- Actuator-only test with frozen dynamics is stable
- RK4 and Euler now produce consistent behavior
- Off-axis thrust tests no longer oscillate

---

## 2026-04-22 — Missing State Update in RK4 Intermediate Steps

**Context:**
While debugging actuator behavior, inconsistencies appeared between intermediate RK4 states and final integrated state.

**Expected Behavior:**
All state variables must be updated consistently during intermediate RK4 stage construction.

**Observed Behavior:**
- Intermediate states did not reflect mass depletion
- Downstream inconsistencies in force/moment calculations

**Root Cause:**
`addScaled()` did not update mass

**Fix:**
Added mass update to `addScaled()`

**Validation:**
- Intermediate RK4 states now physically consistent
- Mass evolution matches expected linear depletion
- No discrepancies between intermediate and final states

---

## 2026-04-22 — TVC Validation — Pitch Axis

**Scenario:**  
- Engine location: `(-1, 0, 0)` (behind COM)
- Command: `gimbal_pitch = +5°`
- No gravity, no aero

**Expected:**  
- Thrust vector gains negative z-component
- Moment about y-axis:
  - `M_y < 0`
- Angular velocity:
  - `omega_y < 0`

**Observed:**   
- `force_body ≈ (Fx, 0, Fz<0)`
- `moment_body ≈ (0, My<0, 0)`
- `omega_body.y` evolves negative

**Conclusion:**  
Pitch-axis TVC implementation correct

---

## 2026-04-22 — TVC Validation — Yaw Axis

**Scenario:** 
- `gimbal_yaw = +5°`
- `gimbal_pitch = 0`

**Expected:**  
- Thrust vector gains y-component
- Moment about z-axis:
    - `M_z ≠ 0`

**Observed:**  
- `force_body.y ≠ 0`
- `moment_body.z ≠ 0`
- `omega_body.z` evolves accordingly

**Conclusion:**  
- Yaw-axis TVC implementation correct
- Axis conventions consistent

___

## 2026-04-22 — Combined Pitch + Yaw TVC Validation

**Scenario:**  
- `gimbal_pitch = +5°`
- `gimbal_yaw = +5°`

**Expected:**  
- Combined off-axis thrust
- Coupled rotational response

**Observed:**  
- Both `Fy` and `Fz` nonzero
- Multi-axis moment generation
- Smooth coupled angular response

**Conclusion:**  
Full 2-axis TVC geometry validated

___

## 2026-04-22 — Euler Equation Bug in Rotational Dynamics

**Context:**  
Unexpected angular motion observed in straight-thrust baseline case.

**Expected Behavior:**  
With zero external moment and zero initial angular velocity:
- omega_dot = 0
- omega = 0
- attitude remains constant

**Observed Behavior:**  
Nonzero angular acceleration
Quaternion components oscillating

**Root Cause:**  
Error in implementation of Euler rigid-body equation, incorrect computation in inertia / cross-product term

**Fix:**  
Corrected rotational dynamics implementation

**Validation:**  
- Straight-thrust test produces zero angular motion
- Quaternion remains stable
- No unintended rotation observed

---

## 2026-04-22 — CSV Logging Format Error

**Context:**  
Simulation outputs appeared inconsistent (e.g., missing inertia values).

**Expected Behavior:**  
Logged values should match internal simulation state.

**Observed Behavior:**
- Misaligned columns in CSV
- Missing or incorrect values (e.g., `Izz`)

**Root Cause:**  
Missing comma in CSV output caused column misalignment.

**Fix:**
Corrected CSV formatting

**Validation:**
- Logged values match in-memory debug values
- All fields correctly aligned

---

## 2026-04-22 — RK4 vs Euler Diagnostic Insight

**Context:**  
Actuator behavior differed between Euler and RK4.

**Insight:**
Euler stability with RK4 instability indicated:
issue in RK4 implementation, not actuator model

**Outcome:**  
- Focused debugging on RK4 combination logic
- Avoided unnecessary model changes

___

## 2026-04-22 — Constant Gravity Model Integration

**Context:**  
Added constant gravity model for near-ground approximation.

**Expected Behavior:**  
- Gravity affects translation only
- No induced torque (force applied at COM)

**Observed:**  
- Trajectory curvature consistent with gravity
- No change in rotational behavior
- Stable integration with propulsion

**Conclusion:**  
- Gravity correctly integrated into force model
- Frame consistency maintained

___

## Summary of Current Status

The simulator now correctly models:

- 6-DOF rigid body dynamics
- quaternion-based attitude propagation
- actuator dynamics with rate limits
- thrust vector control (pitch + yaw)
- variable mass and inertia
- gravity effects
- RK4 time integration (validated)