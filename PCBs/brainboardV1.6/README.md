# MESC BRAIN BOARD 

A PCB and software development project for a balancing robot. 

The goal is to use a hobby Electronic Speed Controller (one open source [example](https://github.com/owhite/MP2-DFN), and this circuit for a [brain board](https://github.com/owhite/MESC_brain_board/blob/main/brainboardV1.0/MESC_brain_board.pdf), a teensy 4.0, and an ESP-32, in combination with [MESC firmware](https://github.com/davidmolony/MESC_Firmware).

## GENERAL APPROACH

A separation of concerns is divided across these architectures:

* **Teensy:** Runs the high-level brain → gait generator, balance estimator, RC inputs
* **MESC:** The firmware on the ESC for control → torque or velocity loops per joint
* **ESP32:** Easy to log and analyze later → pumps UDP to the computer
* **Desktop computer** → perfectly adequate way of viewing high speed data with UDP

## TEENSY 4.0
* 600+ MHz ARM Cortex-M7 — plenty of headroom for filtering, control loops, gait logic, telemetry
* Tons of I/O for IMU, encoders, RC input, WiFi (via ESP add-on), etc.
* Precise timing with elapsedMicros or interrupts for deterministic control
* Great for real-time control, sensor fusion, and behavior modeling

## MESC MOTOR CONTROLLER
* Deploys on simple DIY or commercially available ESC boards
* Executes low-level Field-Oriented Control (FOC) motor control 
* Built-in support for encoders, FOC, and torque estimation
* Accurate low-level motor control using current sensing
* Reads angular position from the MT6701 encoder via SPI or PWM
* UART/CAN interface (RX + TX)	Real-time communication with the Teensy
* PWM / FOC commutation w/ efficient, smooth motor operation

## OTHER BRAIN BOARD FEATURES
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

## CAN BUS
* Full-duplex communication between Teensy and motor controllers
* Carries:
  * Commands: set_torque, set_velocity, enable_motor, zero_encoder, etc.
  * Telemetry: encoder position, estimated torque, velocity, fault codes
* Real-time performance with minimal latency
* Can support multiple motor controllers on the same bus

## MANY THANKS
This project would never happen without:
* [MESC firmware](https://github.com/davidmolony/MESC_Firmware). 
* [Netzpfuscher's](https://github.com/Netzpfuscher/TTerm) incredible TTerm and CAN work.
* [MP2](https://github.com/badgineer/MP2-ESC) an open source motor controller from badgineer. 

## NOTES
Ever wanted to change all the names of kicad files at once?

use this:
```
python3 replace_kicad_names.py VESC_brain_board MESC_brain_board
```

[[LINK](replace_kicad_names.py)]

## STILL NEEDS WORK
* Not great design w/ DC-DC conversion from high voltage VBat
* Replace out the IMU for a ICM-42688-P
* Order / revise pushbuttons for programming the ESP32
* Ensure 3.3V CAN transciever

## VERSIONS
* See github tag [pcb-v1](https://github.com/owhite/MESC_brain_board/releases/tag/pcb-v1) for V1.0
* Retreive with this command: "git checkout pcb-v1"

