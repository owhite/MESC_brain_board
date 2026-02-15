# Failure Modes & Watchdogs Checklist

Common issues that can break balancing in the brain board project, along with what to watch for and how to mitigate them.

---

## Model–Reality Gaps
- **Cause:** Wrong mass, inertia, COM height, motor constants, friction, or linearization errors.
- **Watch:** Compare predicted vs measured response, recovery time, deviations from simulation.
- **Mitigation:** Log commanded vs actual response, refine A/B matrices via system identification.

---

## IMU Problems
- **Cause:** Gyro bias drift, accelerometer noise
- **Watch:** Sudden jumps in angle/velocity, gyro bias while stationary, encoder inconsistency.
- **Mitigation:** Log vibration, [calibrate IMU](../teensy40/vibration_testing/README.md), add filtering (complementary/Kalman)

---

## ESC Problems
- **Cause:** Bad current limits, encoder measurement, motor settings.
- **Watch:** Motors behave poorly, assymetric motor behavior
- **Mitigation:** Log movement, test CAN commands, set out-of-range values, check torques, calibrate encoder offsets

---

## Actuator Saturation
- **Cause:** Motors can’t provide commanded torque at high tilt or disturbance.
- **Watch:** Torque command clipping at ±max, battery sag under load.
- **Mitigation:** Log % time saturated, trigger warnings, penalize torque in LQR (larger R), add anti-windup logic.

---

## Timing Jitter & Delays
- **Cause:** Control loop not running exactly at 500 Hz due to blocking I/O or CAN bursts.
- **Watch:** Control loop dt distribution, flag dt > 3–4 ms.
- **Mitigation:** Keep control loop deterministic, throttle telemetry, measure jitter with GPIO/DWT, abort if timing drift exceeds threshold.

---

## Communication Latency / Loss
- **Cause:** CAN congestion, delayed encoder updates, ESP32 UART backpressure.
- **Watch:** Gaps in encoder/velocity updates, node alive status toggling.
- **Mitigation:** Add timeout counters, log packet inter-arrival times, fail safe to idle if CAN stalls.

---

## Estimator Errors
- **Cause:** State estimator provides wrong tilt/velocity due to noise or drift.
- **Watch:** Innovation/residuals (measurement − prediction), divergence in estimated vs measured tilt.
- **Mitigation:** Log innovations, abort if residuals exceed threshold.

---

## Startup Conditions
- **Cause:** Robot starts too tilted (>15°) for linearization to hold.
- **Watch:** Initial tilt angle when enabling balance mode.
- **Mitigation:** Require tilt < 5° before enabling control, otherwise refuse to engage.

---

## Asymmetry & Cross-Coupling
- **Cause:** Left/right motors behave differently, leading to yaw drift.
- **Watch:** Persistent yaw when balancing in place.
- **Mitigation:** Calibrate wheel gains, extend state model to include yaw if necessary.

---

## Physical Robot Issues
- **Frame Flexibility / Looseness**  
  - **Cause:** Weak materials, loose bolts, frame compliance.  
  - **Watch:** Buzzing or oscillation not predicted by model.  
  - **Mitigation:** Rigid frame, thread-lock, stiffening braces.

- **Wheel Slippage**  
  - **Cause:** Low friction tires/surfaces.  
  - **Watch:** Encoder motion without tilt correction.  
  - **Mitigation:** High-friction tires, textured surfaces.

- **Wheel Backlash / Play**  
  - **Cause:** Loose coupling or gears.  
  - **Watch:** Delay between torque and wheel motion.  
  - **Mitigation:** Eliminate play, use direct drive.

- **Asymmetric Hardware**  
  - **Cause:** Unequal wheel/motor behavior.  
  - **Watch:** Constant yaw drift.  
  - **Mitigation:** Match components, calibrate offsets.

- **Center of Mass Shifts**  
  - **Cause:** Payload or battery off-center.  
  - **Watch:** Falls faster than predicted.  
  - **Mitigation:** Keep COM low/centered, secure loads.

- **Structural Resonances**  
  - **Cause:** Flexible members vibrating.  
  - **Watch:** High-frequency IMU oscillations.  
  - **Mitigation:** Stiffen frame, add damping. See [this discussion](../teensy40/vibration_testing/README.md)

- **Loose Encoders / Misalignment**  
  - **Cause:** Poor encoder mount.  
  - **Watch:** Phantom encoder ticks while stationary.  
  - **Mitigation:** Rigid mounting, use Z-index pulse.

- **Electrical / Power Issues**  
  - **Cause:** Loose connectors, voltage sag, brownouts.  
  - **Watch:** Sudden resets, dropouts.  
  - **Mitigation:** Secure wiring, proper gauge, capacitors.

---

# Robustness Strategy
- **Measure:** Collect metrics on dt, saturation, packet loss, sensor bias.
- **Log:** Histograms and distributions to spot drift and load trends.
- **React:** Use thresholds, timeouts, alarms, and safe torque cutoffs.

**Principle:** Anticipate → Instrument → React.
