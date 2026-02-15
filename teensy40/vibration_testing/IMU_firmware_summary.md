# Balancing Robot IMU & Control Architecture Summary

## 1. main()

### Purpose

-   Entry point of the firmware (Teensy 4.0).
-   Initializes hardware and system-level components.
-   Sets up the supervisor and control loop timing.

### Responsibilities

-   Configure clocks, Serial, CAN, SPI, and timers.
-   Call `init_supervisor()`.
-   Start the 1 kHz control ISR.
-   Monitor conditions and set:
    -   `sup->mode = SUP_MODE_BALANCE_TWR` when ready.
-   Run main loop that executes `controlLoop()` when `g_control_due` is
    set.
-   Program enters `supervisor.mode = SUP_MODE_BALANCE_TWR` and initiates logging
-   State entry is done through a button press or external python program

------------------------------------------------------------------------

## 2. supervisor.h / supervisor.cpp

### Purpose

Central system manager. Owns: - ESC objects - IMU state - RC input -
Loop timing statistics - Mode state machine

### Responsibilities

#### IMU Handling

-   Configures ICM42688 over SPI.
-   Uses DRDY interrupt (`INT_PIN`) to set `dataReady` flag.
-   Calls `imu.getAGT()` inside control loop.
-   Runs Mahony filter.
-   Updates:
    -   `sup->imu.pitch_rad`
    -   `sup->imu.pitch_rate`
    -   `sup->imu.valid`
    -   `sup->imu.last_update_us`

#### Mahony Filter

Implemented in:

    static void mahonyUpdateIMU(...)

-   Fuses gyro + accel.
-   Uses proportional + integral correction.
-   Normalizes quaternion.
-   Extracts pitch from quaternion.

#### Timing Management

Inside `controlLoop()`: - Measures `dt_us` - Tracks `min_dt_us`,
`max_dt_us` - Counts overruns - Measures `exec_time_us`

Typical results: - \~21--22 µs execution at 1 kHz - 0 overruns

#### RC Input

-   Interrupt-based pulse capture
-   Normalized inside `updateRC()`

------------------------------------------------------------------------

## 3. balance_TWR_mode.cpp

### Purpose

Implements balancing controller for the two-wheeled robot.

### Responsibilities

#### Sensor Use

Reads: - `sup->imu.pitch_rad` - `sup->imu.pitch_rate` - ESC wheel
positions and velocities

#### Wheel Unwrapping

Function:

    updateWheelUnwrap(...)

-   Handles encoder wrap-around
-   Low-pass filters velocity
-   Computes forward position and velocity

#### LQR Controller

State:

    x = [theta, theta_dot, x_wheel, x_dot]

Control:

    u = -K_disc * x

-   Applies safety scaling
-   Clamps torque
-   Sends symmetric torque over CAN

#### Safety

-   If tilt exceeds threshold → zero torque and exit mode.
-   If ESC not alive → exit to IDLE.

------------------------------------------------------------------------

## 4. IMU Noise Testing

### What Was Verified

-   IMU SPI working correctly.
-   DRDY interrupt functioning.
-   `exec_us` stable and small.
-   No loop overruns.
-   Pitch rate RMS extremely low at rest.
-   Raw RMS spikes when striking table.
-   Filtered RMS attenuates spikes.

### RMS Computation

Used Welford's online variance algorithm:

    struct RunningStats {
        uint32_t n;
        float mean;
        float m2;
        ...
    };

-   Computes variance without storing samples.
-   Used for noise characterization.
-   NOT a filter --- statistical estimator.

------------------------------------------------------------------------

## 5. Filtering in System

### 1. Mahony Filter

-   Fuses gyro + accel.
-   Primary orientation estimator.

### 2. Accel Gating

    if (accMag > 0.85f && accMag < 1.15f)

-   Rejects accel when not near 1g.
-   Prevents vibration corruption.

### 3. Velocity Low Pass Filter (LPF)

First-order filter on wheel velocity.

------------------------------------------------------------------------

## 6. Mechanical Isolation Strategy

IMU mounting: - Drum gel pad on battery. - 3M VHB between gel and IMU. -
Battery mechanically isolated from frame.

Result: - Dramatically reduced vibration coupling. - Clean pitch-rate
measurements. - Valid tilt detection for balancing.

------------------------------------------------------------------------

## 7. Key Insight

Unless you address noise and vibration in the tilt detection system,\
you don't have a balancing robot --- you have a random torque generator.

------------------------------------------------------------------------
