# Intro
## CAN POS/VEL, Telemetry, and Scheduling — Project Synopsis
- Goal: (Teensy 4.0 “brain board” & ESCs over CAN)
- Deterministic control at 1 kHz on Teensy.
- Low-jitter I/O: CAN setpoints and fast encoder feedback for balancing.
- Implement: control-plane (fast) vs telemetry-plane (slow).

## Control plane
- Control-plane (fast, 500–1000 Hz).
- Runs on Teensy: IMU + estimator + LQR/PID at 1 kHz.
- Sends setpoints (torque/velocity/position) to ESCs each tick.
- Reads high-rate encoder pos/vel from ESCs (new fast stream; see below).

## Why 500 Hz (or 1 kHz) Makes Sense
- 2 kHz sampling in a classic LQR balancing system ([Park et al.](https://link.springer.com/article/10.1007/s10015-011-0897-9?utm_source=chatgpt.com)) establishes an upper benchmark—suggesting that balance control benefits from high-frequency updates.
  - See also: [Ref1](https://indusedu.org/pdfs/IJREISS/IJREISS_3094_97729.pdf?utm_source=chatgpt.com) and [Ref2](https://www.mdpi.com/1424-8220/25/4/1056?utm_source=chatgpt.com)
- Running at 500–1000 Hz provides plenty of bandwidth to implement LQR-based stabilization effectively—while being manageable for embedded systems like Teensy + ESCs.

## There already is a telemetry-plane (slow, ~10–50 Hz)
- MESC “fast” telemetry is actually ~10 Hz (every 100 ms) — too slow for balancing.
- Useful for monitoring/logging: Vbus, Ibus, eHz, Idq/Vdq, state, etc.
- Not for stabilization; too slow for 1 kHz balancing.

## Bus Load Reality Check (CAN 2.0 @ 1 Mb/s)
- 2 ESCs × 1 kHz commands (~128 b per frame) ≈ 256 kb/s.
- 2 ESCs × 1 kHz POS/VEL ≈ 256 kb/s.
- Total ≈ 50% bus load (before additional telemetry). At 500 Hz POS/VEL, ≈ 25%.

## Commands vs Telemetry (Priority)
- Commands: highest priority, periodic/event-driven; send right after compute.
  - Coalesce (latest-wins) if back-pressured; never block.
- Telemetry: lower priority, best-effort; okay to drop when bus busy.
- Fast POS/VEL: own tiny task on ESC; independent of slow telemetry.

# Inserting CAN code into MESC
## From looking at `MESC_F405RG/MESC_F405RG.ioc`:
- PWM frequency = ~41 kHz
- MESC_ADC_IRQ_handler() execution rate = ~82 kHz
- `fastLoop()` runs at ~82 kHz

## Sending encoder position and motor velocity over CAN
- Putting it inside `fastLoop()` (82 kHz ISR) is not appropriate.
- It risks jitter in a time-critical ISR.
- The correct place is a separate FreeRTOS task.
- Schedule the task at 500–1000 Hz.
- Use `vTaskDelayUntil()` for cadence.
- Enqueue one frame each tick.
- TX handled by `TASK_CAN_tx`.
- This ensures no extra jitter in FOC timing.

## MESC code reads encoder counts (TIMx->CNT) every ISR
- It computes difference in counts (`diff`) from the previous sample.
- Scales that by encoder resolution and sampling period to estimate velocity.
- The logic looks like this (simplified):

```c
diff = new_count - last_count;
_motor->FOC.mechanical_angle += diff * (2π / encoder_ticks_per_rev);
_motor->FOC.mechanical_omega = diff * (2π / encoder_ticks_per_rev) * sample_rate;
```

- MESC normalizes by encoder CPR (counts per revolution) and multiplies by the loop rate (82 kHz ISR).
  - As a result:
    - `mechanical_angle` → radians (can wrap or accumulate).
    - `mechanical_omega` → radians/second (rad/s).
  - Therefore `_motor->FOC.mechanical_omega` reflects angular velocity of the rotor/wheel in physical units.

## Implementation plan

### 1. Current CAN tasks (in `task_can.c`)
```c
xTaskCreate(TASK_CAN_rx,        "task_rx_can", 256, (void*)port, osPriorityAboveNormal, &handle->rx_task_handle);
xTaskCreate(TASK_CAN_tx,        "task_tx_can", 256, (void*)port, osPriorityAboveNormal, &handle->tx_task_handle);
xTaskCreate(TASK_CAN_telemetry, "can_metry",   256, (void*)port, osPriorityAboveNormal, NULL);
```

### 2. New FreeRTOS Task (in `task_can.c`)
```c
#ifdef POSVEL_PLANE
static void TASK_CAN_posvel(void *arg) {
    TASK_CAN_handle *h = (TASK_CAN_handle*)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / POSVEL_HZ);
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        TASK_CAN_telemetry_posvel(h);
        vTaskDelayUntil(&last, period ? period : 1);
    }
}
#endif
```

Add alongside the other tasks:
```c
#ifdef POSVEL_PLANE
xTaskCreate(TASK_CAN_posvel, "can_posvel", 256, (void*)port, osPriorityAboveNormal, NULL);
#endif
```

### 3. New CAN ID (in `can_ids.h`)
```c
#ifdef POSVEL_PLANE
#define CAN_ID_POSVEL            0x2D0   // new high-rate position/velocity ID
#endif
```

### 4. Feature Toggle (in project header)
Add to one of these headers:
- `MESC_F405RG/Core/Inc/main.h`
- `MESC_F405RG/Core/Inc/MP2_V0_1.h`
- `MESC_F405RG/Core/Inc/MESC_F405.h`

```c
#define POSVEL_PLANE 1
```

### 5. Telemetry Function (in `MESCinterface.c`)
```c
#ifdef POSVEL_PLANE
void TASK_CAN_telemetry_posvel(TASK_CAN_handle *handle) {
    MESC_motor_typedef *motor_curr = &mtr[0];
    TASK_CAN_add_float(handle, CAN_ID_POSVEL, CAN_BROADCAST,
                       motor_curr->FOC.mechanical_angle,
                       motor_curr->FOC.mechanical_omega,
                       0);
}
#endif
```

## Receiving CAN commands from Teensy
- MESC already spawns TASK_CAN_rx at high priority (osPriorityAboveNormal) to process incoming frames.
- Torque setpoint commands are mapped to CAN_ID_IQREQ, which updates _motor->FOC.Idq_req.q.
- _motor->FOC.Idq_req.q is a signed float:
  - Positive → forward torque
  - Negative → reverse torque
- FOC loop (82 kHz) consumes this value each cycle; if no new frame arrives, it reuses the last command.
- Teensy should send one new command per 1 kHz control tick, ensuring ESC always has a fresh reference.

# Teensy side considerations
## Teensy Architecture (not using RTOS)
- Timing & Priority
  - IntervalTimer ISR: sets `control_due` + timestamp; no math in ISR.
  - Superloop priority order:
    - If `control_due` → run `control_step()` (compute + enqueue CAN command).
    - `Can.events()` → RX callback → drain/parse ring.
    - Low-priority chores: LEDs (state machine), pushbutton debounce, serial pump.

- CAN on Teensy (FlexCAN_T4)
  - Use `onReceive()` + tiny RX callback that copies frames into a ring buffer.
  - Parse in loop (non-blocking), update snapshots for control to read.
  - Commands: enqueue and send immediately (or via a small TX pump).
  - Do not block or print inside `control_step()`.

## CAN Framing & Decoding
- MESC ID Scheme (Extended 29-bit)
  - ESC packs {msg_id, sender, receiver} into the extended ID.
  - Working assumption:

```c
msg_id: bits 28..16 (13 b), sender: 15..8 (8 b), receiver: 7..0 (8 b; 0=broadcast).

static inline uint32_t mesc_pack_id(uint16_t msg, uint8_t snd, uint8_t rcv) {
  return ((uint32_t)(msg & 0x1FFF) << 16) | ((uint32_t)snd << 8) | rcv;
}
static inline void mesc_unpack_id(uint32_t id, uint16_t &msg, uint8_t &snd, uint8_t &rcv) {
  msg = (id >> 16) & 0x1FFF; snd = (id >> 8) & 0xFF; rcv = id & 0xFF;
}
```

### Key IDs
- Existing telemetry (slow): **CAN_ID_SPEED**, **CAN_ID_BUS_VOLT_CURR**, **CAN_ID_MOTOR_CURRENT**, **CAN_ID_MOTOR_VOLTAGE**, **CAN_ID_ADC1_2_REQ**, **CAN_ID_STATUS**.
- Control/utility: **CAN_ID_PING** (8-byte ASCII name, ~1 Hz), **CAN_ID_CONNECT**, **CAN_ID_TERMINAL**.
- New (proposed): **CAN_ID_POSVEL** (8 bytes) → pos, vel at 500–1000 Hz.

### Payloads
- Helpers in ESC send 8-byte payloads as 2×float32 or 2×uint32.
- Likely little-endian (STM32 default). Teensy decodes with memcpy.

```c
static inline void parse_two_floats(const uint8_t *b, float &a, float &c) {
  uint32_t u0 = b[0]|(b[1]<<8)|(b[2]<<16)|(b[3]<<24);
  uint32_t u1 = b[4]|(b[5]<<8)|(b[6]<<16)|(b[7]<<24);
  memcpy(&a,&u0,4); memcpy(&c,&u1,4);
}
```

### Teensy handling of POS/VEL
RX path:
- FlexCAN RX callback → ring push → main loop parse → pos/vel snapshot (per ESC) with timestamp/seq.

```c
struct PosVel { float pos_rad, vel_rad_s; uint32_t t_us, seq; };
volatile PosVel esc_posvel[2];

void handle_mesc(const CAN_message_t &m) {
  uint16_t id; uint8_t snd, rcv; mesc_unpack_id(m.id, id, snd, rcv);
  if (rcv != 0 && rcv != MY_NODE) return;
  if (id == CAN_ID_POSVEL && m.len == 8) {
    float p,v; parse_two_floats(m.buf, p, v);
    uint8_t i = (snd == LEFT_ESC_ID) ? 0 : 1;
    esc_posvel[i].pos_rad = p; esc_posvel[i].vel_rad_s = v;
    esc_posvel[i].t_us = micros(); esc_posvel[i].seq++;
  }
}
```

In `control_step()`, copy latest snapshot into local variables. Optionally extrapolate:

```
pos_est = pos + vel * dt; // dt = time since RX
```
