# Remaining Work for Teensy Balancing Robot

## 0. IMU MUST BE RESTORED FIRST (BLOCKER)

Before anything else, your IMU must output stable values: - ax, ay, az
must reflect gravity direction. - gx, gy, gz near 0 dps when
stationary. - Orientation must change when tilted.

## Fix 1 --- Reduce torque scale

Status: DONE.

## Fix 2 --- Improve LQR wheel velocity handling

Status: PARTIALLY DONE. Ensure dt is supervisor dt_us \* 1e-6.

## Fix 3 --- IMU filtering and rate stabilization

Status: PARTIALLY COMPLETE. Pending hardware recovery. After raw stable,
confirm imu.roll, imu.roll_rate are correct.

## Fix 4 --- Limit maximum angle before shutting off

Add cutoff at ±45 deg.

## Fix 5 --- Add angle offset calibration

Add theta_offset and subtract from roll.

## Fix 6 --- Add roll-rate deadband and startup ramp

## Timing / Supervisor Infrastructure

A. Confirm supervisor uses radians. B. Signature mismatch fixed. C. Tune
LQR after IMU fixed.

## Summary

Critical: fix IMU hardware first. Then implement fixes 4--6 and validate
dt, LPF, IMU readings.
