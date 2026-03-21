# Position Control Project Goals

## System Intent

Build a two-wheeled balancing robot with an actuated knee joint (Ascento-like concept), where the knee BLDC is driven through a planetary gearbox.

## Supervisory Control (Outer Loop)

- A custom brain board (Teensy 4.0) runs:
  - High-level control logic
  - State estimation
  - Knee position control
- Brain board communicates with MESC over CAN bus.
- Brain board sends torque commands.
- Brain board receives position/velocity feedback.

## Control Goal: Knee Position Control

Target behavior:

- Move to a commanded position.
- Hold position reliably.
- Be reproducible (same final angle each time).
- Work at low speeds.
- Be robust to gearbox friction/backlash.
- "Reliable and practical" performance is the objective, not precision servo-grade behavior.

## Position Control Strategy (Brain Board)

Use torque mode on MESC and implement position logic on Teensy:

`tau = Kp * (theta_ref - theta) - Kd * omega + tau_friction`

Where:

- `Kp`: position gain
- `Kd`: damping gain
- `theta_ref`: target position
- `theta`: measured position
- `omega`: measured velocity

## Friction Compensation (Required)

Planetary gearbox friction is significant and must be compensated.

- Static friction compensation (near-zero speed, nonzero error):
  - `tau_stiction * sign(error)`
- Coulomb friction compensation (during motion):
  - `tau_c * sign(omega)`
- Optional viscous term:
  - `b * omega`

## Backlash Handling

To improve repeatability:

- Always approach final target from one direction, or
- Intentionally overshoot and return to target from a consistent direction.

## Key Design Decisions

- Do not modify MESC to add position control.
- Use MESC as:
  - High-speed torque controller
  - Sensor reporting device
- Keep all position logic on Teensy.
- Run outer loop over CAN at approximately 200 to 500 Hz (start lower, then increase).

## Current CAN TX/RX Limitations (Must Be Treated As Requirements)

- Shared-bus dual-ESC robustness is not yet a required success mode for this phase.
- Accepted mode for project completion: single active ESC node (node 11).
- Prior testing indicates likely shared-bus interaction limits under higher load:
  - Arbitration/timing collisions between multiple telemetry publishers.
  - Phase alignment effects causing repeated collisions.
  - Added Teensy command traffic reducing telemetry fidelity at unlucky timing.
  - Physical-layer sensitivity (termination, wiring, stubs, transceiver behavior, unpowered-node behavior).
- Serial1 activity on Teensy has previously degraded CAN receive quality and should be minimized during control validation.
- "TX success" (`CAN_TXQ_SUM ok/fail`) confirms Teensy enqueue/write success, not guaranteed actuator torque application.
- Per-frame counter continuity is the preferred drop-confidence metric when available; loop-sampled "missed" estimates alone are insufficient.

## Practical Operating Constraints for V1

- Single ESC telemetry publisher at 500 Hz (node 11).
- Teensy command TX decimated below control loop rate (currently capped at ~500 Hz with 2 ms min TX spacing).
- Control loop may remain 1 kHz internally, but command updates should be rate-limited.
- If feedback age exceeds timeout threshold (currently 10 ms), command zero torque and enter safe state.

## Implementation Learnings (Added from Bench Testing)

- ESC telemetry contract for position mode:
  - POSVEL position payload must be radians (counter-test payload is not valid for control operation).
- Direction/sign calibration is a first-class requirement:
  - Torque sign and encoder sign must be verified on each rig before position sweeps.
- Large sweep behavior:
  - `timed_sweep` targets are interpreted relative to current position at command start.
  - `timed_sweep` command range is `-10000..10000` degrees.
  - Multi-turn commands are supported and require unwrap safety limits that scale with requested move size.
- Estimation robustness:
  - Controller damping should use velocity estimated from position deltas.
  - Implausible per-sample position jumps should be rejected before unwrap integration.
- Command shaping:
  - Torque clamp and slew-rate limiting are required to avoid runaway during fast error sign changes and backlash transitions.
- Stale feedback policy:
  - Brief CAN hiccups can trip strict stale thresholds; stale-fault policy should be tuned (timeout value and/or consecutive-stale logic).
- Validation workflow:
  - Prefer scripted sweep logs (`run_sweep.py`) and evaluate `TIMED_POS_DONE`, `final_err_rad`, `jump_total`, and `CAN_TXQ_SUM fail=0`.

## What We Care About

- Reproducible final position.
- Stable position hold.
- No sustained oscillation.
- No runaway while lowering.
- Minimal implementation complexity.

## Out of Scope (For Now)

- High-speed trajectory control.
- Industrial precision motion control.
- Perfect speed regulation.


## Implemented Control Constants (Current Code)

These values are defined in `src/timed_pos_control.cpp` and should be kept in sync with that file.

- Sweep/input:
  - `SWEEP_DEG_LIMIT = 10000`
  - Target is relative: `target = pos_start + sweep_deg * pi/180`
- Outer-loop gains/limits:
  - `POS_KP_DEFAULT = 0.32`
  - `POS_KD_DEFAULT = 0.012`
  - `POS_TORQUE_CLAMP_NM = 0.90`
  - `POS_TORQUE_SLEW_NM_PER_S = 45.0`
  - `POS_TORQUE_SIGN = -1.0` (rig-specific)
  - `POS_DONE_BAND_RAD = 3 deg`
- Friction/backlash helpers:
  - `STICT_TORQUE_NM = 0.32`
  - `COULOMB_TORQUE_NM = 0.10`
  - `STATIC_VEL_THRESH_RAD_S = 0.20`
  - `STATIC_ERR_THRESH_RAD = 0.03`
  - `BREAKAWAY_ERR_RAD = 0.20`
  - `BREAKAWAY_TAU_MIN_NM = 0.45`
- Estimation/shaping:
  - `VEL_FROM_POS_ALPHA = 0.25`
  - `CMD_TX_MIN_DT_US = 2000` (about 500 Hz max TX)
- Safety/fault thresholds:
  - `POSVEL_TIMEOUT_US = 10000`
  - `POS_RAW_ABS_MAX_RAD = 100`
  - `VEL_RAW_ABS_MAX_RAD_S = 500`
  - `UNWRAP_BASE_LIMIT_RAD = 40`
  - `UNWRAP_TARGET_MARGIN_RAD = 12`
  - `INVALID_FB_MAX_CONSEC = 3`
  - `POS_JUMP_BASE_RAD = 0.30`
  - `POS_JUMP_SPEED_RAD_S = 350`
  - `POS_JUMP_MAX_CONSEC = 8`

-----------------------------------------
## Thoughts about lower friction compensation:

This was set up for a gearbox. These are parameters to tweak for other setups:
- `STICT_TORQUE_NM` down substantially (often near zero)
- `COULOMB_TORQUE_NM` down substantially
- `BREAKAWAY_TAU_MIN_NM` down or disable
- `BREAKAWAY_ERR_RAD` likely smaller

## Re-tune control gains for lower reflected inertia/friction:
- `POS_KP_DEFAULT` usually lower than geared case (start conservative)
- `POS_KD_DEFAULT` usually higher relative damping to prevent ringing

## Tighten torque limits:
- `POS_TORQUE_CLAMP_NM` lower (direct drive can accelerate fast)
- `POS_TORQUE_SLEW_NM_PER_S` lower for safer command shaping

## Tighten completion criteria:
- `POS_DONE_BAND_RAD` smaller (better natural precision without backlash)

## Revisit unwrap/jump guards:
- `POS_JUMP_BASE_RAD` and POS_JUMP_SPEED_RAD_S likely lower (motion is cleaner/less stick-slip)

## Possibly tighten stale feedback behavior:
- `POSVEL_TIMEOUT_US` can often be stricter if motion is smoother and less disturbance-sensitive

## Re-evaluate sign and direction constants:
- `POS_TORQUE_SIGN` and any encoder inversion still must be validated on the new mechanical setup

----------------------
## Testing
- parameters are set for gearbox motor
- launch: `$ ./run_sweep.py -3600`
- sweep value does not include gear ratio
- creates `sweep_output.log`
- codex can open the output and review
- try: `timed_sweep 3600` to turn the motor
- try: `pwm_control` to use an RC transmitter for input
-------------------------
[Watch the demo video on YouTube](https://www.youtube.com/shorts/7sQ_XWcPYlw)

[![Demo video thumbnail](https://img.youtube.com/vi/7sQ_XWcPYlw)/hqdefault.jpg)](https://www.youtube.com/shorts/7sQ_XWcPYlw)

