# STM32F405 CAN Testing Plan

This workspace is a standalone CAN test effort for:
- Teensy 4.0 (test peer / traffic generator / logger)
- STM32F405 pill (bare-minimum PCB, fresh firmware)

This work is intentionally separate from `MESC_Firmware/can_testing`.

## Goal

Develop fresh STM32F405 CAN firmware with an architecture similar to the ESC-side structure used in MESC, so that proven high-bitrate CAN logic can be transferred back to ESC firmware with minimal refactoring.

## To run
Set compile time directive in MESC_Firmware
- `#define CAN_DIAG_COMPAT_V1 1`
- (it would be cool to have a can command set this state
- or a CLI command)

Open the teensy serial and type:
- `run`, `run 60` for a sixty second run
- `tx hz 250` to set the timing of IQREQ commands to 250 hz

default timing is now 500 hz. 

## Architecture Direction (Portability First)

Use ESC-like layering:
- `can_driver`: bxCAN/HAL init, filters, TX mailboxes, RX FIFO service, error-state access.
- `can_protocol`: frame IDs, sender/receiver/node addressing, payload pack/unpack, sequence counters.
- `can_scheduler`: periodic TX scheduling and timing ownership.
- `app_test_modes`: test orchestration (load profiles, burst modes, soak, fault-recovery checks).
- `diag_metrics`: counters/histograms/latency stats emitted to Teensy.

Design rule: protocol and scheduler interfaces should be written so files can be moved into ESC code with minimal API changes.

## Core Measurement Strategy

Teensy will receive sequenced telemetry/counter frames from STM32 and compute dropout and timing quality.

Primary outcomes:
- Determine stable operating bitrate and load envelope.
- Identify where failures start (dropouts, overruns, error-state escalation, latency tails).
- Produce a transport profile that is safe for balancing robot control.

## Metrics To Log Every Run

### Reliability and Ordering
- `seq_rx_total`
- `seq_missed_total`
- `seq_duplicate_total`
- `seq_out_of_order_total`
- `burst_miss_max`

### Timing Quality
- `gap_us_min`, `gap_us_p50`, `gap_us_p95`, `gap_us_p99`, `gap_us_max`
- `payload_age_us` (time since last valid message)
- end-to-end latency (when timestamped messages are enabled)

### Queue / Mailbox Health
- `tx_attempts`, `tx_ok`, `tx_fail`
- `tx_mailbox_full`
- `tx_enqueue_drop`
- `rx_fifo_overrun`
- `rx_queue_drop`

### Controller Error-State Health
- `can_tec`, `can_rec`
- `err_warn_count`
- `err_passive_count`
- `bus_off_count`
- protocol error class counters where available (ACK/bit/stuff/form/CRC)

### Runtime Correlation
- `loop_dt_us` stats and scheduler overruns on STM32
- any watchdog/reset reason indicators

## Phased Test Regime

## Phase 0: Electrical + Silent Bring-up
Duration: ~5 minutes
- Verify transceiver supply, standby pin behavior, common ground, termination.
- Start STM32 in listen-only then normal mode.

Pass:
- Valid frame detection.
- No immediate error-state escalation.

## What We Learned (Recent A/B + Soak)

### Summary
- The communication issue was not caused by CAN wire-level corruption.
- The command path from Teensy to STM32 was healthy during testing.
- The main reliability gain came from how the Teensy services receive traffic.

### Key Findings
- In dual-channel tests, CAN controller error indicators stayed clean (no persistent ACK/CRC/stuff/form/bit failures and no bus-off trend).
- STM32 command-receive sequence tracking showed no command-sequence loss during the validated runs.
- Configuring the ESC to transmit telemetry immediately (target <50 us) after the Teensy command ID significantly reduced dropouts and improved dual-channel reliability.
- With Teensy receive handled by polling in the main loop, message loss appeared in 30-second dual-channel tests.
- With Teensy receive handled by FIFO interrupt callbacks plus `events()` dispatch, the same test conditions showed zero lost telemetry sequence events.
- The Teensy RX path change is the primary improvement, but not the only requirement for stable performance.
- Stable runs also require correct command-to-bus mapping and node routing (node 11 on CAN2, node 12 on CAN1 in current wiring).
- Stable runs also require matching CAN bitrate on both devices (1 Mbps in current baseline tests).
- Stable runs also require hardware CAN filtering configured correctly on Teensy and STM32F405.
- Stable runs also depend on physical-layer health (powered transceivers, correct termination, and clean wiring/grounding).

### Practical Guidance
- Keep Teensy receive in interrupt/callback mode for this test framework.
- Keep node-to-bus routing aligned with physical wiring (node 11 on CAN2, node 12 on CAN1 in the current bench setup).
- Re-run the A/B check after major firmware or scheduler changes to confirm behavior remains stable.

## Variables Added and What They Proved

### 1) `CAN_RX_USE_ISR` (Teensy compile-time switch)
- Location: `teensy40/can_testing/src/main.h`
- Values:
  - `0` = receive by polling in `loop()`
  - `1` = receive by FIFO interrupt callback + `events()` dispatch
- What it proved:
  - This enabled a direct A/B comparison under the same wiring and test conditions.
  - Polling mode showed telemetry loss in dual-channel tests.
  - Interrupt/callback mode removed that loss in matched runs and 10-minute soak.

### 2) Callback-vs-main sequence counters (`CAN_INGRESS_SEQ` output)
- Location: `teensy40/can_testing/src/CAN_helper.cpp`, `teensy40/can_testing/src/test_can_transmit_mode.cpp`
- Added fields:
  - `left_cb_valid`, `left_cb_missed`, `left_cb_dup`, `left_cb_ooo`
  - `left_main_valid`, `left_main_missed`, `left_main_dup`, `left_main_ooo`
  - same for right side
- What it proved:
  - In polling mode, callback-level and main-level misses were both nonzero.
  - In interrupt/callback mode, both callback-level and main-level misses were zero.
  - This isolated the improvement to receive servicing behavior on Teensy.

### 3) STM32 command-sequence receive counters (`CAN_F405_IQREQ` output)
- Location: `MESC_Firmware/MESC_Interface/MESC/MESCinterface.c`, `teensy40/can_testing/src/CAN_helper.cpp`, `teensy40/can_testing/src/test_can_transmit_mode.cpp`
- Added fields:
  - `left_valid`, `left_missed`, `left_age_us`
  - `right_valid`, `right_missed`, `right_age_us`
- What it proved:
  - During validated runs, command-sequence misses on STM32 stayed at zero.
  - Therefore, command reception on STM32 was not the primary source of loss.

### 4) STM32 FIFO-overrun per-run deltas (`CAN_F405_OVR` output)
- Location: `MESC_Firmware/MESC_Interface/MESC/MESCinterface.c`, `teensy40/can_testing/src/test_can_transmit_mode.cpp`
- Added fields:
  - `left_fov0`, `left_fov1`, `left_fov_age_us`
  - `right_fov0`, `right_fov1`, `right_fov_age_us`
- What it proved:
  - In stable runs (including soak), these remained zero.
  - This aligned with zero observed sequence loss in the final validated configuration.

### 5) Node-to-bus transmit mapping constants
- Location: `teensy40/can_testing/src/main.h`
- Mapping used in validated setup:
  - node `11` -> CAN2
  - node `12` -> CAN1
- What it proved:
  - Keeping transmit routing aligned to physical wiring removed a source of avoidable timing and delivery instability.

### 6) Current command TX defaults
- `user_tx_period_us = 2000` (500 Hz) in `teensy40/can_testing/src/supervisor.h`
- Startup default in `teensy40/can_testing/src/main.cpp` also sets `user_tx_period_us = 2000`
- Balance mode uses `BALANCE_TX_PERIOD_US = 2000` (500 Hz) in `teensy40/can_testing/src/balance_TWR_mode.cpp`

## Impact on Robot Controller Architecture

For the intended system architecture (Teensy as the brain board, STM32F405 nodes as motor controllers), this testing outcome is significant:

- Dual-channel command and telemetry communication is now stable at 1 Mbps in the validated setup.
- A 10-minute soak test completed with zero observed telemetry sequence loss and zero command-sequence loss.
- No CAN controller error-state escalation or receive-overrun growth was observed in the stable runs.

## Practical implication:
- The communication layer is now suitable for closed-loop balancing and motion-control workloads where timing consistency matters.
- This lowers integration risk for using the Teensy as central supervisor/planner and F405 boards as motor-control endpoints.
- The same architecture remains portable to ESC-target firmware with less risk of reintroducing prior CAN reliability problems.

## CAN Comment Marker Legend

The source now includes inline markers to highlight reliability-focused edits:

- `CAN reliability improvement:` marks code paths that were changed to improve CAN robustness, timing, or observability.
- `CAN reliability improvement baseline:` marks default configuration values used in validated stable runs.

Primary files using these markers:

- `src/main.h`
- `src/main.cpp`
- `src/CAN_helper.h`
- `src/CAN_helper.cpp`
- `src/test_can_transmit_mode.cpp`
- `src/supervisor.h`
- `src/balance_TWR_mode.cpp`
