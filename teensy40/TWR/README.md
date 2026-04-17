# TWR Testing

This folder currently contains two firmware trees:

- Active TWR firmware: `TWR/src`
- CAN-focused reference copy: `TWR/can_testing/src`

Important: recent CAN reliability work was developed and validated in the CAN-focused tree. Do not assume `TWR/src` and `TWR/can_testing/src` are identical.

## CAN Status (Latest)

Recent CAN-focused runs (dual ESC, dual bus) have been strong:

- 60 s runs at 500 Hz IQREQ (`tx_period_us=2000`) completed with:
  - zero TX failures,
  - zero RX overflow,
  - healthy CAN event dispatch timing,
  - near-zero to zero sequence misses (occasional single-frame blips).

This is good enough to proceed with robot integration testing.

## IMU Filter (Current `TWR/src`)

- Filter type: 6-DOF Mahony quaternion update.
- Gains:
  - `Kp = 0.5` (`twoKp = 1.0`)
  - `Ki = 0.1` (`twoKi = 0.2`)
- Accel correction gating:
  - full correction for low accel-magnitude error,
  - tapered correction until `|acc|-1` reaches `0.06 g`,
  - no accel correction beyond that.
  - Practical band is tighter than `0.85..1.15 g` (roughly around `0.94..1.06 g` with taper).
- `dt` guard:
  - measured from IMU timestamps,
  - clamped to nominal 1 ms when outside `0.5..5 ms`.
- Reported states:
  - `pitch_rad = asin(2*(q0*q2 - q3*q1))`
  - `pitch_rate_raw` from gyro Y,
  - low-pass filtered `pitch_rate` (`alpha = 0.15`).

## IMU Test Tool

Run:

```bash
python3 /Users/owhite/balancing-robot-notes/teensy40/TWR/IMU_test.py \
  -p /dev/cu.usbmodemXXXX \
  -b 921600 \
  --expect-n 200
```

Outputs and plot fields in `VERIFY_ANGLE` mode remain as documented by the script (`pitch`, `pitch_rate`, raw/filtered RMS, IMU age/valid, loop dt, optional verify motor torque).

## Open Control Note

The balance controller still needs continued tuning/validation for translational terms (`x`, `x_dot`, integral hold behavior) under real disturbance cases.

## Recent Code Changes (April 2026)

- Added a virtual-axle synchronization path in `balance_TWR_mode.cpp`:
  - computes yaw mismatch from wheel unwrapped distance/speed (`g_unwrap_l - g_unwrap_r`),
  - stores a tare-time yaw reference (`g_yaw_ref`),
  - computes relative yaw error and optional sync torque (`u_sync`) that is mixed into left/right torque commands.
- Added/kept the tare-time yaw reference capture so heading correction is relative to the start posture, not absolute wheel offsets.
- Added a button-release entry latch in `main.cpp` for balance start:
  - if entry angle is valid while pressed (green LED on), release still starts balance/tare even if angle shifts slightly at release.
- Updated IMU anti-drift handling in `main.cpp`:
  - Mahony integral remains enabled with moderated `Ki`,
  - integral clamp + ungated decay behavior,
  - bump holdoff window for accel correction during disturbances,
  - quiet-motion gating for accel correction while balancing.
- Balance command cadence defaults are aligned with the stable CAN profile used in testing (`tx_period_us = 2000`, 500 Hz) unless overridden by runtime command.

