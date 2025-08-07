# Hardware considerations

## First a out to friends:
* A fork from badgineer: [MP2] (https://github.com/badgineer/MP2-ESC). 
* MESC firmware [MESC](https://github.com/davidmolony/MESC_Firmware). 
* PCB sites for PCB and PCBA: [pcbway.com](https://www.pcbway.com/) and [jlcpcb.com](https://jlcpcb.com/).

## Project plan:

* Use my open source motor controller, the [MP2-DFN](https://github.com/owhite/MP2-DFN), and this circuit for a [brain board](https://github.com/owhite/MESC_brain_board/blob/main/brainboardV1.0/MESC_brain_board.pdf), a teensy 4.0, and an ESP-32. 
* In combination with [MESC](https://github.com/davidmolony/MESC_Firmware).

## Hardware
### Brain Board (Teensy 4.0)
* Acts as the central controller
* Runs real-time control algorithms (e.g., balancing, walking gaits, velocity planning)
* Sends motor commands (e.g., desired torque, velocity, or position) over CAN bus
* Receives telemetry (velocity, torque, encoder angle) from motor controllers
* Integrates data from the IMU and other sensors
* Optionally logs data or interfaces with a host computer

### Motor Controller (MESC-based with Custom Firmware)
* Executes low-level motor control (Field-Oriented Control)
* Supports:
** Torque mode
** Velocity mode
** Position mode
* Reads angular position from the MT6701 encoder via SPI or PWM
* Computes:
** Electrical angle for FOC
** Velocity (from encoder delta over time)
** Torque (via current sensing or estimation)
* Sends back telemetry via CAN (e.g., torque estimate, velocity, encoder angle)

### CAN Bus
* Full-duplex communication between Teensy and motor controllers
* Carries:
** Commands: set_torque, set_velocity, enable_motor, zero_encoder, etc.
** Telemetry: encoder position, estimated torque, velocity, fault codes
* Real-time performance with minimal latency
* Can support multiple motor controllers on the same bus

### MT6701 Encoder
* High-resolution magnetic rotary encoder (up to 14-bit)
* Measures rotor position for the BLDC
* Output via SPI, PWM, or ABI 
* Provides absolute angle, ideal for FOC and position tracking
