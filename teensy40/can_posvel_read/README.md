## Teensy CAN Test Programs

**Teensy sketches** that can be loaded to send commands over CAN to the MESC-based ESC. Each one uses a different control channel built into the firmware.  

---

### `src/main.cpp`: POSVEL reader

The active `src/main.cpp` sketch is a POSVEL receiver/diagnostic for the current two-bus Teensy 4.0 bench wiring.

Key behavior:

- Configures CAN1 with the TWR mapping: RX pin 23, TX pin 22.
- Configures CAN2 with the TWR mapping: RX pin 0, TX pin 1.
- Reads extended MESC CAN traffic from both controllers.
- Tracks `CAN_ID_POSVEL` (`0x2D0`) counts and latest position/velocity for nodes 11 and 12.
- Emits a compact 1 Hz JSON summary rather than printing every POSVEL frame.
- Can optionally send zero-torque `CAN_ID_IQREQ` (`0x001`) to ESC node 11 on both CAN buses for sync/legacy testing.

Why the zero-IQREQ keepalive existed:

Older ESC firmware synchronized POSVEL publication to incoming `CAN_ID_IQREQ` frames. Bench testing showed autonomous POSVEL could stop after about 25.56 seconds without periodic IQREQ traffic because the ESC POSVEL scheduler used `DWT_CYCCNT / 168` as a 32-bit microsecond timebase, and that raw cycle-derived timestamp wraps at about 25.56 seconds on the F405. Incoming IQREQ frames re-anchored the scheduler, so a zero-torque IQREQ keepalive preserved low-dropout synchronized POSVEL behavior.

The ESC firmware has now been repaired: POSVEL scheduling uses a wrap-safe monotonic microsecond timebase while retaining the IQREQ-triggered sync/nudge path. A 60s `tx off` CAN test validated autonomous 500 Hz POSVEL from both ESCs across the old DWT wrap point with no missed, duplicate, or out-of-order POSVEL frames.

The keepalive in this sketch is therefore disabled by default. If it is re-enabled for sync experiments or for older ESC firmware, the sketch sends it on both CAN buses because TWR bench wiring routes ESC nodes across CAN1/CAN2; sending on only one bus can silently miss the target ESC.

Firmware status:

The long-term ESC-side fix has been completed in `MESCinterface.c`: keep IQREQ-triggered sync/nudge behavior, but use a wrap-safe monotonic time source for autonomous POSVEL scheduling. Real POSVEL payloads have also been restored: slot0 is wrapped mechanical position in radians, and slot1 is velocity in rad/s.

Useful summary fields:

- `rx_can1`, `rx_can2`: which Teensy CAN controller is receiving frames.
- `pos11_count`, `pos12_count`: per-node decoded POSVEL frame counts.
- `last_mid`: most recent MESC message ID seen, useful for spotting when traffic changes from POSVEL to other telemetry such as `0x2B6` motor current.

---

### 1. `can_id_adc1_2_req.cpp`
- Use **`can_id_adc1_2_req.cpp`** for simple normalized throttle testing.  
- **CAN ID:** `CAN_ID_ADC1_2_REQ (0x010)`  
- **Payload:**  
  - Bytes 0–3 = `throttle_mapped` (float, normalized between **-1.0 and +1.0**)  
  - Bytes 4–7 = unused (`0.0f`)  
- **ESC Behavior:**  
  - Treated like a remote throttle input.  
  - Internally assigned to `motor_curr->input_vars.remote_ADC1_req`.  
  - Negative values = reverse torque, positive = forward torque.  
- **How to use:**  
  - Open Serial Monitor at 115200 baud.  
  - Type a float between `-1.0` and `+1.0`.  
  - Teensy sends that throttle value over CAN once.  
  - The ESC will respond if its **`remote_ADC_can_id`** matches the Teensy’s node ID (set via terminal: `set can_adc <id>`).  

---

### 2. `can_id_iqreq.cpp`
- Use **`can_id_iqreq.cpp`** for direct current/torque control.  
- **CAN ID:** `CAN_ID_IQREQ (0x001)`  
- **Payload:**  
  - Bytes 0–3 = `Iq_req` (float, in **amperes of q-axis current**)  
  - Bytes 4–7 = unused (`0.0f`)  
- **ESC Behavior:**  
  - Written directly into `motor_curr->FOC.Idq_req.q`.  
  - This is raw torque control, bypassing throttle mapping.  
- **How to use:**  
  - Open Serial Monitor at 115200 baud.  
  - Type a float (e.g. `5.0` or `-3.0`).  
  - Teensy will continuously stream that torque request at 500 Hz.  
  - The ESC will respond if its **`remote_Iq_can_id`** matches the Teensy’s node ID (set via terminal: `set can_iq <id>`).  

---

### Debugging in STM32CubeIDE
If you want to trace the command as it flows through the ESC firmware, good debug breakpoints are:  

- **`CAN1_RX0_IRQHandler()`** → Confirms the packet arrived in the ISR and was queued.  
- **`TASK_CAN_packet_cb()`** → Confirms the packet was dispatched to the right handler (ADC1_2_REQ or IQREQ).  
- **`MESCinput_Collect()`** → Confirms the chosen input value (CAN vs UART vs ADC) is being forwarded into the motor control loop.  
