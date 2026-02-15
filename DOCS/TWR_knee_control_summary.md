# System Summary -- Two-Wheel Robot with Actuated Knee (MESC + Brain Board Architecture)

## ChatGPT Prompt Overview

I am building a two-wheeled balancing robot with an additional actuated
"knee" joint (similar in concept to Ascento). The knee is driven by a
BLDC motor through a planetary gearbox.

------------------------------------------------------------------------

## Architecture Overview

### Motor Control (Inner Loop)

-   Motor controllers run **MESC firmware**.
-   MESC provides:
    -   High-bandwidth **torque (current) control**
    -   Position and velocity feedback
    -   CAN bus communication
-   MESC does **not** reliably perform speed or position control
    internally.
-   It reports:
    -   Position (rad)
    -   Velocity (rad/s)
-   It accepts:
    -   Torque commands via CAN

### Supervisory Control (Outer Loop)

-   A custom **brain board (Teensy 4.0)** runs:
    -   High-level control logic
    -   State estimation
    -   Position control for the knee
-   Brain board communicates with MESC over **CAN bus**
-   Brain board sends torque commands
-   Brain board receives position and velocity feedback

System structure:

Brain Board (Teensy)\
↓ torque command (CAN)\
MESC (FOC current control)\
↓ phase currents\
Motor + Planetary Gearbox\
↑ position/velocity (CAN feedback)

------------------------------------------------------------------------

## Control Goal -- Knee Position Control

The knee joint must:

-   Move to a commanded position
-   Hold position reliably
-   Be reproducible (same final angle each time)
-   Work at low speeds
-   Be robust to gearbox friction and backlash

This is not a precision motion-control application. It just needs to
"work reliably."

------------------------------------------------------------------------

## Mechanical Characteristics of the Knee

-   BLDC motor
-   Planetary gearbox
-   Significant:
    -   Static friction (stiction)
    -   Coulomb friction
    -   Possible backlash
-   Likely asymmetric torque requirements:
    -   More torque required to **raise** (against gravity)
    -   Less torque required to **lower**
    -   May require active braking when lowering

------------------------------------------------------------------------

## Position Control Strategy (Implemented on Brain Board)

Position control is implemented on the brain board using torque mode on
MESC.

Control law:

τ = Kp (θ_ref − θ) − Kd ω + τ_friction

Where:

-   Kp = position gain
-   Kd = damping gain
-   θ_ref = target position
-   θ = measured position
-   ω = measured velocity

------------------------------------------------------------------------

## Friction Compensation (Required)

Because of the planetary gearbox, friction compensation is mandatory:

### Static Friction Compensation

When position error exists but velocity ≈ 0:

τ_stiction · sign(error)

### Coulomb Friction Compensation

During motion:

τ_c · sign(ω)

### Optional Viscous Term

b · ω

------------------------------------------------------------------------

## Backlash Handling

To improve reproducibility:

-   Always approach the final position from the same direction
-   Or overshoot slightly and return to the target

------------------------------------------------------------------------

## Key Design Decisions

-   Do not modify MESC to add position control.
-   Use MESC purely as:
    -   High-speed torque controller
    -   Sensor reporting device
-   Implement all position logic on the brain board.
-   Run outer loop at \~200--500 Hz over CAN.

------------------------------------------------------------------------

## What We Care About

-   Reproducible final position
-   Stable position hold
-   No oscillation
-   No runaway when lowering
-   Minimal complexity

We are not optimizing for:

-   High-speed trajectory control
-   Precision industrial servo performance
-   Perfect speed regulation

------------------------------------------------------------------------

## Current Status

-   Torque control over CAN works.
-   Position and velocity feedback from MESC works.
-   Mechanical damping has been experimentally measured for other
    motors.
-   Next step: implement robust low-speed knee position hold using
    torque mode + friction compensation.
-   You may use other strategies than what was described in the position control section

------------------------------------------------------------------------

