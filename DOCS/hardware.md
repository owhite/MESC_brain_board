# Hardware considerations

## Project plan:

Gonna try to use my open source motor controller, the [MP2-DFN](https://github.com/owhite/MP2-DFN), and this circuit for a [brain board](https://github.com/owhite/MESC_brain_board/blob/main/brainboardV1.0/MESC_brain_board.pdf), a teensy 4.0, and an ESP-32, in combination with [MESC firmware](https://github.com/davidmolony/MESC_Firmware).

## First a shout out to friends:
* [MP2](https://github.com/badgineer/MP2-ESC), an open source motor controller from badgineer. 
* [MESC firmware](https://github.com/davidmolony/MESC_Firmware). 
* PCB and PCBA sites: [pcbway.com](https://www.pcbway.com/) and [jlcpcb.com](https://jlcpcb.com/).

## Hardware
### Brain Board 
* Uses a teensy 4.0 and acts as the central controller
* Runs real-time control algorithms (e.g., balancing, walking gaits, velocity planning)
* Sends motor commands (e.g., desired torque, velocity, or position) over CAN bus
* Receives telemetry (velocity, torque, encoder angle) from motor controllers
* Integrates data from the IMU and other sensors
* Optionally logs data or interfaces with a host computer

### Motor Controller (with custom firmware)
* Executes low-level motor control (Field-Oriented Control)
* Supports:
  * Torque mode
  * Velocity mode
  * Position mode
* Reads angular position from the MT6701 encoder via SPI or PWM
* Computes:
  * Electrical angle for FOC
  * Velocity (from encoder delta over time)
  * Torque (via current sensing or estimation)
* Sends back telemetry via CAN (e.g., torque estimate, velocity, encoder angle)

### CAN Bus
* Full-duplex communication between Teensy and motor controllers
* Carries:
  * Commands: set_torque, set_velocity, enable_motor, zero_encoder, etc.
  * Telemetry: encoder position, estimated torque, velocity, fault codes
* Real-time performance with minimal latency
* Can support multiple motor controllers on the same bus

### MT6701 Encoder
* High-resolution magnetic rotary encoder (up to 14-bit)
* Measures rotor position for the BLDC
* Output via SPI, PWM, or ABI 
* Provides absolute angle, ideal for FOC and position tracking

Suppose you bought a MT6701 board and you wanted to figure the mode it is set in. Use this:
```
#include <Wire.h>

#define MT6701_ADDR 0x06  // Replace with your MT6701 I2C address

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  delay(100);

}

void loop() {

  delay(1000);

  int modeByte = readRegister(0x38);
  if (modeByte < 0) {
    Serial.println("Failed to read register 0x38.");
    return;
  }

  int outMode = modeByte & 0b11;       // Bits 1:0
  int uvwRes  = (modeByte >> 4) & 0b111; // Bits 6:4

  Serial.print("OUT_MODE bits: ");
  Serial.println(outMode, BIN);

  switch (outMode) {
    case 0b00:
      Serial.println("🟢 MT6701 is in ABZ mode (quadrature encoder output).");
      break;
    case 0b01:
      Serial.println("🟡 MT6701 is in PWM or Analog output mode.");
      break;
    case 0b10:
      Serial.print("🔵 MT6701 is in UVW (commutation) mode. Pole pairs setting: ");
      Serial.println(uvwRes + 1);
      break;
    case 0b11:
      Serial.println("🔴 Invalid or reserved OUT_MODE value (0b11).");
      break;
  }
}


int readRegister(uint8_t reg) {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return -1;
  }

  Wire.requestFrom(MT6701_ADDR, 1);
  if (Wire.available() < 1) {
    return -1;
  }

  return Wire.read();
}
```