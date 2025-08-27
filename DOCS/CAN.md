## CAN POS/VEL, Telemetry, and Scheduling — Project Synopsis
- Goal: (Teensy 4.0 “brain board” & ESCs over CAN)
- Deterministic control at 1 kHz on Teensy.
- Low-jitter I/O: CAN setpoints and fast encoder feedback for balancing.
- Implement: control-plane (fast) vs telemetry-plane (slow).

## Control plane
- Control-Plane (fast, 500–1000 Hz)
- Runs on Teensy: IMU + estimator + LQR/PID at 1 kHz.
- Sends setpoints (torque/velocity/position) to ESCs each tick.
- Reads high-rate encoder pos/vel from ESCs (new fast stream; see below).

## Telemetry-Plane (slow, ~10–50 Hz)
- MESC “fast” telemetry is actually ~10 Hz (every 100 ms) too slow
- Useful for monitoring/logging: Vbus, Ibus, eHz, Idq/Vdq, state, etc.
- Not for stabilization; too slow for 1 kHz balancing.

## Teensy Architecture (not using RTOS)
- Timing & Priority
  - IntervalTimer ISR: sets control_due + timestamp; no math in ISR.
  - Superloop priority order:
    - If control_due → run control_step() (compute + enqueue CAN command).
    - Can.events() → RX callback → drain/parse ring.
    - Low-priority chores: LEDs (state machine), pushbutton debounce, serial pump.

- CAN on Teensy (FlexCAN_T4)
  - Use onReceive() + tiny RX callback that copies frames into a ring buffer.
  - Parse in loop (non-blocking), update snapshots for control to read.
  - Commands: enqueue and send immediately (or via a small TX pump).
  - Do not block or print inside control_step().

## ESC Side (FreeRTOS) — Adding High-Rate POS/VEL
- Add a minimal, periodic “posvel” task at 500–1000 Hz that:
- Reads latest encoder pos/vel (from FOC state),
- Enqueues one 8-byte CAN frame (TASK_CAN_add_float/_uint32),
- Uses vTaskDelayUntil() for cadence,
- Never blocks (timeout 0), leaving TX to existing CAN TX task.

Example (ESC — new task)
```
#ifndef POSVEL_HZ
#define POSVEL_HZ 1000
#endif

static void TASK_CAN_posvel(void *arg) {
  TASK_CAN_handle *h = (TASK_CAN_handle*)arg;
  const TickType_t period = pdMS_TO_TICKS(1000 / POSVEL_HZ);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    float pos = mtr[0].FOC.mechanical_angle;   // define exact source/units
    float vel = mtr[0].FOC.mechanical_omega;   // rad/s
    TASK_CAN_add_float(h, CAN_ID_POSVEL, CAN_BROADCAST, pos, vel, 0);
    vTaskDelayUntil(&last, period ? period : 1);
  }
}
```

Create this alongside existing CAN tasks:
```
xTaskCreate(TASK_CAN_rx,        "task_rx_can", 256, (void*)port, osPriorityAboveNormal, &handle->rx_task_handle);
xTaskCreate(TASK_CAN_tx,        "task_tx_can", 256, (void*)port, osPriorityAboveNormal, &handle->tx_task_handle);
xTaskCreate(TASK_CAN_telemetry, "can_metry",   256, (void*)port, osPriorityAboveNormal, NULL);
// NEW:
xTaskCreate(TASK_CAN_posvel,    "can_posvel",  256, (void*)port, osPriorityAboveNormal, NULL);
```

Guardrails: keep priority below anything time-critical for FOC; let the existing TX task handle mailbox availability.

## CAN Framing & Decoding
- MESC ID Scheme (Extended 29-bit)
  - ESC packs {msg_id, sender, receiver} into the extended ID.
  - Working assumption (fits code usage):

```
msg_id: bits 28..16 (13 b), sender: 15..8 (8 b), receiver: 7..0 (8 b; 0=broadcast).

static inline uint32_t mesc_pack_id(uint16_t msg, uint8_t snd, uint8_t rcv) {
  return ((uint32_t)(msg & 0x1FFF) << 16) | ((uint32_t)snd << 8) | rcv;
}
static inline void mesc_unpack_id(uint32_t id, uint16_t &msg, uint8_t &snd, uint8_t &rcv) {
  msg = (id >> 16) & 0x1FFF; snd = (id >> 8) & 0xFF; rcv = id & 0xFF;
}

```

### Key IDs
- Existing telemetry (slow): **CAN_ID_SPEED**, **CAN_ID_BUS_VOLT_CURR**, **CAN_ID_MOTOR_CURRENT**, **CAN_ID_MOTOR_VOLTAGE**, **CAN_ID_ADC1_2_REQ**, **CAN_ID_STATUS.

Control/utility: **CAN_ID_PING** (8-byte ASCII name**, ~1 Hz), **CAN_ID_CONNECT**, **CAN_ID_TERMINAL**.

New (proposed): **CAN_ID_POSVEL** (8 bytes) → pos, vel at 500–1000 Hz.

### Payloads
- Helpers in ESC send 8-byte payloads as 2×float32 or 2×uint32.
- Likely little-endian (STM32 default). Teensy decodes with memcpy.

```
static inline void parse_two_floats(const uint8_t *b, float &a, float &c) {
  uint32_t u0 = b[0]|(b[1]<<8)|(b[2]<<16)|(b[3]<<24);
  uint32_t u1 = b[4]|(b[5]<<8)|(b[6]<<16)|(b[7]<<24);
  memcpy(&a,&u0,4); memcpy(&c,&u1,4);
}
```

Teensy — Handling POS/VEL
RX path

FlexCAN RX callback → ring push → main loop parse → pos/vel snapshot (per ESC) with timestamp/seq.

```
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

Place this in control_step(), copy latest snapshot; optional small extrapolation:
pos_est = pos + vel * dt (dt = time since RX).

## Commands vs Telemetry (Priority)

- Commands: highest priority, periodic/event-driven; send right after compute.
Coalesce (latest-wins) if back-pressured; never block.
- Telemetry: lower priority, best-effort; okay to drop when bus busy.
- Fast POS/VEL: own tiny task on ESC; independent of slow telemetry.

## Bus Load Reality Check (CAN 2.0 @ 1 Mb/s)

- 2 ESCs × 1 kHz commands (~128 b per frame) ≈ 256 kb/s.
- 2 ESCs × 1 kHz POS/VEL ≈ 256 kb/s.
- Total ≈ ~50% bus → still comfortable; at 500 Hz status, ≈ ~25%.

## Implementation

- Teensy: superloop + IntervalTimer; FlexCAN ring buffer; non-blocking serial pump; PWM RC capture via CHANGE-edge ISRs; LED state machine (no delay()).
- ESC: keep ISR short; add xTaskCreate(TASK_CAN_posvel, ...); publish via TASK_CAN_add_float(..., timeout=0); leave TX centralized.
- Open Items / Decisions
  - Confirm MESC ID layout and numeric CAN_ID_* constants (pull from ESC headers).
  - Choose POSVEL units: wrapped angle vs cumulative ticks; document clearly.
  - Decide publish rate (500 vs 1000 Hz) based on ESC tick and CAN headroom.
  - Map ESC node IDs and your MY_NODE for addressing.

## TL;DR

- Perform 1 kHz control on Teensy
- Provision for deterministic control with modest CAN usage
- Do not rely on slow ESC telemetry for stabilization.
- Add a low overhead MESC FreeRTOS task that publishes pos/vel at 500–1000 Hz.
- On Teensy, decode POS/VEL, run control
- Prioritize these commands over telemetry

