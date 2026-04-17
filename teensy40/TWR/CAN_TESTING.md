# CAN Testing Notes

## User-Reported Issues and Investigation History

1. We are debugging intermittent CAN telemetry loss/jitter between Teensy 4.0 receiver and STM32F405 ESC nodes (ID 11 and 12), mainly POS/VEL at about 500 Hz per node.
2. Early "missed" metrics were partly misleading because sequence checks were done in the 1 kHz control loop instead of per-CAN-frame receive path.
3. We added per-frame sequence accounting in Teensy CAN RX path (`left_seq_*`, `right_seq_*`, burst counters). This is now the trusted drop metric.
4. Teensy-side TX queue/fail diagnostics on ESC side were mostly clean (`enqueue_fail=0`, `hal_tx_fail=0`), so many losses were not from obvious HAL enqueue failures.
5. Enabling torque command traffic increased mailbox pressure and worsened telemetry fidelity; disabling torque improved results.
6. Serial1 activity on Teensy clearly hurt CAN receive quality. Disabling Serial1 reduced misses and max gaps substantially.
7. Physical layer/cabling has a strong effect. Twisted/shielded CAN pair improved average runs, but intermittent burst failures still appear.
8. Asymmetry between nodes was observed repeatedly (sometimes node 12 worse, other times node 11), suggesting mixed causes (electrical + timing/scheduling), not one single deterministic firmware bug.
9. Some tests showed severe periodic stalls/bursts (counts dropping to about 1200 to 1900 over windows where about 2400 expected, and very large max gaps). This happened even with motors disconnected in some cases.
10. Startup/warmup polluted max-gap metrics initially; we added warmup gating and per-button-run reset logic to reduce false interpretation.
11. We also built a clean no-RTOS STM32 CAN smoke-test project (PB8 = CAN1_RX AF9, PB9 = CAN1_TX AF9) to isolate ESC-stack effects from full MESC/RTOS behavior.
12. Current status: still intermittent reliability issues; likely contributors are electrical integrity plus occasional scheduling/servicing stalls under certain runtime conditions. No single conclusive root cause yet.

## Current Code Concerns (This `can_testing` Branch)

### Core concerns in `CAN_helper` + `main`

1. Trusted per-frame sequence-drop metrics are not present in this branch's `CAN_helper` stats structure; only timing-gap estimates (`est_missed`) are currently tracked.
2. Software RX ring buffer is small (`CAN_BUF_SIZE=32`) and overflows are not emitted in diagnostics, which can hide drops during brief service stalls.
3. Serial1 telemetry output is active in test mode and can perturb CAN RX performance, which risks contaminating transport test results.
4. Test start paths currently do not reset CAN RX stats/overflow counters, so prior-run and startup transients can pollute measurements.
5. RX parsing does not validate frame shape (`extended` flag and payload length) before decoding floats.

### Concerns in `test_can_transmit_mode`

1. ESC-alive guard checks only the left ESC; right ESC loss can be missed.
2. Torque TX is sent every 1 kHz control tick (two commands per tick), which increases CAN load and may worsen telemetry loss during diagnostics.
3. The mode currently reports timing/new-sample telemetry but does not directly report trusted per-frame sequence continuity counters.
4. `SUP_MODE_BALANCE_TWR` is also routed to `test_can_transmit_mode`, which can cause unintended behavior if balance mode is expected to be separate.
5. `dt_pos_us` telemetry averages left/right timing even when only one side is fresh, which can mask one-sided stalls.
6. The startup log message says "Balance mode started" even when this test transmit mode is running.

## Practical Test Guidance (Short)

1. For pure transport characterization, keep torque TX and Serial1 diagnostics independently gateable at compile time.
2. Always report per-frame sequence continuity counters and software overflow counters in summaries.
3. Reset all RX/drop metrics at run start (after warmup gate if used) to keep windows comparable.
