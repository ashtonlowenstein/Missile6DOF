# Validation

This document describes the validation process for the 6-DOF flight simulator.
Validation is performed incrementally, from low-level mathematical correctness
to full closed-loop system behavior.

## 1. Mathematical Validation

### 1.1 Vector Operations

**Test:**
- cross(ex, ey) = ez
- cross(ey, ex) = -ez
- dot(ex, ey) = 0

**Result:**
- All vector operations produced expected results

**Conclusion:**
- Basic vector algebra is implemented correctly

### 1.2 Matrix Operations

**Test:**
- Matrix inverse: A * A⁻¹ = I

**Result:**
- Diagonal elements of A * A⁻¹ ≈ 1
- Off-diagonal elements ≈ 0

**Conclusion:**
- Matrix inversion and multiplication are correct

### 1.3 Quaternion Rotation

**Test:**
- 90° rotation about z-axis applied to x-axis vector

**Expected:**
- x → y

**Observed:**
- Rotated vector matches expected result

**Conclusion:**
- Quaternion rotation is implemented correctly

## 2. Actuator Model Validation

### Test: First-Order Actuator Response

**Scenario:**
- Initial actuator state = 0
- Command step input applied:
    - throttle = 1.0
    - gimbal_pitch = 5°
    - gimbal_yaw = 0

**Expected:**
- Positive throttle derivative
- Positive pitch derivative
- Zero yaw derivative

**Observed:**
- throttle_dot > 0
- gimbal_pitch_dot > 0
- gimbal_yaw_dot ≈ 0

**Conclusion:**
- Actuator model responds correctly to command inputs
- Axis decoupling is preserved

## 3. Dynamics Validation

### 3.1 Zero Moment Test

**Scenario:**
- No forces or moments applied
- Initial angular velocity = 0

**Expected:**
- ω̇ = 0
- q̇ = 0

**Observed:**
- Angular acceleration is zero
- Quaternion derivative is zero

**Conclusion:**
- Rotational dynamics are consistent with rigid-body equations

## 3. Dynamics Validation

### 3.1 Zero Moment Test

**Scenario:**
- No forces or moments applied
- Initial angular velocity = 0

**Expected:**
- ω̇ = 0
- q̇ = 0

**Observed:**
- Angular acceleration is zero
- Quaternion derivative is zero

**Conclusion:**
- Rotational dynamics are consistent with rigid-body equations

## 4. GNC Validation

### 4.1 Attitude Hold — Pitch Axis

**Scenario:**
- Initial pitch error ≈ 5°
- Desired attitude = identity
- Closed-loop control active

**Expected:**
- Attitude error decreases over time

**Observed:**
- Error norm decreases to zero
- Gimbal pitch command active
- Yaw channel remains inactive

**Conclusion:**
- Pitch-axis control is stable and correct

### 4.2 Attitude Hold — Yaw Axis

**Scenario:**
- Initial yaw error ≈ 5°

**Observed:**
- Error decreases to zero
- Only yaw gimbal is active

**Conclusion:**
- Yaw-axis control behaves correctly

### 4.3 Combined Pitch + Yaw

**Scenario:**
- Initial combined attitude error

**Expected:**
- Coupled rotational response
- Error converges to near zero

**Observed:**
- Error decreases and stabilizes
- Small steady-state residual (~0.2°)

**Explanation:**
- Residual error is due to:
  - PD control (no integral term)
  - approximate mapping from control input to gimbal angles

**Conclusion:**
- Closed-loop system is stable and performs as expected

## Summary

The simulator has been validated across multiple layers:

- mathematical correctness (vectors, matrices, quaternions)
- actuator dynamics
- rigid-body dynamics
- force and moment generation
- gravity effects
- closed-loop GNC behavior

All tests confirm that the system behaves consistently with expected physical and control principles.