
# System Assumptions and Tests for Two-Wheeled Balancing Robot (TWR)

This document lists the **implicit assumptions** your balancing robot system relies on, along with **tests** you can perform to verify each one. These assumptions span sensors, kinematics, dynamics, actuation, filtering, timing, and modeling.

---

## 1. Geometry & Kinematics Assumptions

### **A1. IMU tilt = body tilt**
**Assumption:** IMU pitch matches true robot pitch.  
**Test:** Tilt robot on a measured board (5°, 10°, 15°) and confirm logged pitch matches.

### **A2. Wheel angle ↔ linear displacement**
**Assumption:** `x_wheel = radius × wheel_angle` matches real displacement.  
**Test:** Move robot along marked distances; compare encoder-based estimate.

### **A3. Unwrapping is correct**
**Assumption:** Wheel angles don’t jump at ±π.  
**Test:** Spin wheel by hand through multiple revolutions and plot `x_wheel`.

### **A4. COM and inertia match model**
**Assumption:** Physical COM and I_body are close to modeled values.  
**Test:** Measure small oscillation frequency or verify static torque-angle relationship.

---

## 2. Sensor Assumptions

### **S1. IMU rate = actual angular rate**
**Assumption:** `pitch_rate` matches physical angular velocity.  
**Test:** High-frame-rate video comparison.

### **S2. Correct gyro axis for pitch**
**Assumption:** The selected gyro axis truly corresponds to pitch.  
**Test:** Rotate robot around each axis; check which gyro axis responds.

### **S3. Wheel velocity reading is accurate**
**Assumption:** `vel_rad_s` reflects true wheel speed.  
**Test:** Compare measured time per revolution with logged velocity.

### **S4. Latency is small/constant**
**Assumption:** Loop delay IMU → control → torque is small.  
**Test:** Send step torque, observe IMU acceleration start time.

---

## 3. Actuation / ESC Assumptions

### **E1. Torque command ∝ actual torque**
**Assumption:** Sent torque produces proportional torque at wheel.  
**Test:** Use spring scale or lever arm to measure torque.

### **E2. Torque direction matches model**
**Assumption:** +u makes robot tilt in expected way.  
**Test:** Command small u and observe initial tilt direction.

### **E3. Motors behave symmetrically**
**Assumption:** Left/right torque for same u are similar.  
**Test:** Free-spin both with same torque; compare velocities.

### **E4. No large torque deadzone**
**Assumption:** Small non-zero u produces motion/torque.  
**Test:** Slowly ramp u from 0 → upward and check first movement.

---

## 4. Timing / Loop Assumptions

### **T1. Loop runs at 500 Hz**
**Assumption:** `CONTROL_PERIOD_US` matches actual dt.  
**Test:** Log `dt_us` and inspect mean & variance.

### **T2. Mahony dt is correct**
**Assumption:** dt passed to Mahony matches actual IMU interrupt spacing.  
**Test:** Log `(now - last_update)` each time IMU updates.

---

## 5. Model / LQR Assumptions

### **M1. Linearization is valid**
**Assumption:** Small-angle dynamics approximate model.  
**Test:** With motors off, gently perturb robot in a rig and compare oscillation characteristics.

### **M2. Q and R reflect your priorities**
**Assumption:** Weighting properly emphasizes angle over position.  
**Test:** Simulate model-based step response and compare with desired behavior.

---

## 6. Coordinate Frame & Sign Conventions

### **C1. Pitch sign matches model**
**Assumption:** Positive tilt matches direction used in LQR model.  
**Test:** Tilt robot forward/back and confirm sign.

### **C2. Wheel motion corrects tilt**
**Assumption:** Forward wheel motion stabilizes a forward-leaning body.  
**Test:** Suspend robot, apply small forward torque, observe body response.

---

## 7. Noise & Filtering Assumptions

### **N1. Filters introduce acceptable lag**
**Assumption:** LPFs on θ̇ and ẋ don’t add excessive phase delay.  
**Test:** Compare raw vs filtered during slow oscillations.

### **N2. Noise is bounded/stationary**
**Assumption:** Motor vibration doesn’t explode noise magnitude.  
**Test:** Log RMS noise under various torque conditions.

---

## 8. Safety & Saturation Assumptions

### **S1. Torque limits are respected**
**Assumption:** `TORQUE_CLAMP` prevents saturation.  
**Test:** Log u + observe ESC current draw.

### **S2. Sufficient wheel traction**
**Assumption:** Wheels don’t slip under normal control forces.  
**Test:** High-torque tests on actual surface.

---

## Summary Table

| Category | Assumption | What You Expect | How to Test |
|---------|------------|-----------------|-------------|
| Geometry | A1 | IMU pitch = real pitch | Measured tilt board |
| Geometry | A2 | Wheel angle → displacement | Tape measure test |
| Sensors | S1 | IMU rate = angular rate | Phone video |
| Actuation | E1 | u → linear torque | Spring scale |
| Control | T1 | 500 Hz loop | Log `dt_us` |
| Model | M1 | Linearization valid | Rig-based oscillation test |

---

This list is meant to be actionable: each assumption has a corresponding test.  
Validating these reduces uncertainty and makes successful balancing almost inevitable.
