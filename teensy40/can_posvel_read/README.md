## Teensy CAN Test Programs

**Teensy sketches** that can be loaded to send commands over CAN to the MESC-based ESC. Each one uses a different control channel built into the firmware.  

---

### `src/main.cpp`: POSVEL reader with keepalive

The active `src/main.cpp` sketch is a POSVEL receiver/diagnostic for the current two-bus Teensy 4.0 bench wiring.

Key behavior:

- Configures CAN1 with the TWR mapping: RX pin 23, TX pin 22.
- Configures CAN2 with the TWR mapping: RX pin 0, TX pin 1.
- Reads extended MESC CAN traffic from both controllers.
- Tracks `CAN_ID_POSVEL` (`0x2D0`) counts and latest position/velocity for nodes 11 and 12.
- Emits a compact 1 Hz JSON summary rather than printing every POSVEL frame.
- Sends zero-torque `CAN_ID_IQREQ` (`0x001`) to ESC node 11 at 10 Hz on both CAN buses.

Why the zero-IQREQ keepalive exists:

Current ESC firmware synchronizes POSVEL publication to incoming `CAN_ID_IQREQ` frames. Bench testing showed autonomous POSVEL can stop after about 25.56 seconds without periodic IQREQ traffic, likely because the ESC POSVEL scheduler uses `DWT_CYCCNT / 168` as a 32-bit microsecond timebase and that counter wraps at about 25.56 seconds on the F405. Incoming IQREQ frames re-anchor the scheduler, so a zero-torque IQREQ keepalive preserves the current low-dropout synchronized POSVEL behavior.

This sketch sends the keepalive on both CAN buses because TWR bench wiring routes ESC nodes across CAN1/CAN2; sending on only one bus can silently miss the target ESC.

Long-term firmware note:

The preferred ESC-side fix is to keep the IQREQ-triggered sync/nudge behavior, but replace the autonomous POSVEL scheduler's raw DWT-derived `uint32_t` timebase with a wrap-safe monotonic time source. Before making that ESC firmware change, repeat CAN dropout testing for both autonomous POSVEL and IQREQ-synchronized POSVEL.

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
