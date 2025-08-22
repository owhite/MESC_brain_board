# MESC Brain Board

[Chatgpt discussion](https://chatgpt.com/c/68653d84-b634-8011-b055-3476bfa95f52)

## Teensy 4.0
* 600+ MHz ARM Cortex-M7 — plenty of headroom for filtering, control loops, gait logic, telemetry
* Tons of I/O for IMU, encoders, RC input, WiFi (via ESP add-on), etc.
* Precise timing with elapsedMicros or interrupts for deterministic control
* Great for real-time control, sensor fusion, and behavior modeling

## MESC motor controller
* Handles current, velocity, or position control
* Built-in support for encoders, FOC, and torque estimation
* UART/CAN interface
* Closed-loop torque control
* Accurate low-level motor control using current sensing
* Velocity control for trajectory and balance-level control
* Encoder support (AB, ABI, SPI), position feedback
* CAN interface (RX + TX)	Real-time communication with the Teensy
* PWM / FOC commutation w/ efficient, smooth motor operation

## Why This Combo Works So Well for a Walking or Balancing Robot
* Teensy: Runs the high-level brain → gait generator, balance estimator, RC inputs
* MESC: Handles the low-level muscle control → torque or velocity loops per joint
* ESP32: Easy to log and analyze later

## FEATURES

* Teensy 4.0
* ESP32
* RC receiver connector (for PWM/PPM/SBUS input)
* ESP32 UART serial programmer
* Power input connector
* Voltage divider (to monitor battery voltage)
* Push-button E-stop connector
* Power button (for system on/off)
* IMU connector (e.g., MPU-6050)
* Buzzer (for system alerts or E-stop signal)
* MESC I/O
* Reserved 2 GPIO pins on Teensy for emergency shutoff to MESC
* CAN connector (for MESC motor control)

## Chapters to my story
* Hardware considerations [[LINK](DOCS/hardware.md)]
* FreeRTOS considerations [[LINK](DOCS/free_rtos.md)]
* Brain board firmware specs [[LINK](DOCS/software_specs.md)]
* Using the MT6701 [[LINK](DOCS/MT6701.md)]

## NOT IMPLEMENTED

* Optional I²C header (for sensors like INA219 or IMU)
* Encoder inputs (if reading position directly)
* SWD debug access for Teensy
* Big red e-stop button
* Voltage divider (to monitor battery voltage)

## NEEDS WORK

* Not great design w/ DC-DC conversion from high voltage VBat

## VERSIONS

* See github tag [pcb-v1](https://github.com/owhite/MESC_brain_board/releases/tag/pcb-v1) for V1.0
* Retreive with this command: "git checkout pcb-v1"

## NOTES

ever wanted to change all the names of kicad files at once? use this:
```
python3 replace_kicad_names.py VESC_brain_board MESC_brain_board
```
