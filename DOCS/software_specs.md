# MESC Brain Board Documentation

### Overview
The MESC Brain Board is a Teensy 4.0-based microcontroller platform responsible for high-level control of a balancing robot using BLDC motors. It interfaces with motor controllers over CAN, receives motion data from an MPU6050 IMU, accepts user commands from an RC transceiver via PWM, and communicates with an ESP32 over UART. The brain board includes status indicators such as LEDs and a speaker. 

This document just helping to think through firmware overview, and create really useful prompts for chatGPT.

#
### Hardware Interfaces / Peripherals
- Status indicators
  - Speaker
  - LEDs
    - Red
    - Green
- MPU6050 IMU
  - Interface: I2C
  - Tracks orientation and angular velocity
- PWM Inputs (from RC transceiver)
  - Used for manual control input
- CAN Bus (2 ports)
  - Connects to dual motor controllers (left and right)
  - Receives motor encoder, torque, velocity, and state information
  - Sends control commands (e.g. torque or velocity setpoints)
- UART to ESP32
  - ESP32 handles WiFi communications and remote telemetry
- UART to USB
  - term connection to computer

### Threads
- CAN
- Terminal
- User input
- Telemetry
- Speaker
- LEDs
- IMU

### Priority loop
- Calculating desired corrective torque/velocity
  - Process user input
  - Translating target to motor commands
- State management

### Control State Management
- CONTROL_IDLE
- CONTROL_ARMED
- CONTROL_MANUAL_RESET
- CONTROL_BALANCING
- CONTROL_TIPPED
- CONTROL_SHUTTING_DOWN

### Data structure for brain board
- Config
  - network name
  - node list
  - tipping_angle
  - max_turn
  - max_speed
  - ESC_num
- Raw input
  - PWM CH1-CH6
  - IMU I2C
  - User strings
  - UDP strings
- User_req
  - tilt_offset
  - speed
  - direction
  - left
  - right
  - P_input
  - I_input
  - D_input
- Params
  - obs_angle
  - obs_speed
  - tilt_offset
  - state
  - speed
  - direction
  - left
  - right
  - P_input
  - I_input
  - D_input
- Status
  - error
  - angle_initiated
- Error codes
  - TERM_FAILURE
  - TELEMETRY_FAILURE
  - IMU_FAILURE
  - RC_PWM_FAILURE
  - ANGLE_INIT_FAILURE
- LED1 control state
  - LED1_ON
  - LED1_FAST_BLINK
  - LED1_SLOW_BLINK
  - LED1_OFF
- LED2 control state
  - LED2_ON
  - LED2_FAST_BLINK
  - LED2_SLOW_BLINK
  - LED2_OFF
- Speaker control state
  - SPK_WARN1
  - SPK_WARN2
  - SPK_TRACK_BALANCE

### Data structure for each ESC
- Config
  - name 
  - node_id
  - pole_pairs
  - encoder_offset
  - max_amps
  - max_volts
  - min_volts
  - direction

- Raw input from controller, fast loop
  - adc1_2_req
  - speed
  - bus_volt
  - bus_current
  - motor_current
  - motor_voltage
  - FOC_angle

- Raw input from controller, slow loop
  - tmos
  - tmot
  - wanted: error_code

- Output to controller, fast loop
   - AMPS
   - DIRECTION

- Status
  - error

- Error codes
  - ESC_ERROR
  - ESC_OVERHEAT
  - ESC_OVERCURRENT
  - ESC_CAN
  - ESC_RESPONSE

