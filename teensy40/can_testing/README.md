## Editing markdown:
- Open `README.md.`
- Open Command Palette: `Cmd+Shift+P`.
- Run `Markdown Preview Enhanced`: Open Preview to the Side.

## Position/Velocity Filtering Debug Status
- controlLoop() is running at ~1 kHz (loop_dt_us ~ 1000), while ESC POSVEL is expected around ~500 Hz and asynchronous.
- Original unwrap gate used loop dt and over-rejected moving-wheel samples; this was patched to use fresh POS sample timing and to update prev_L/prev_R even on reject (prevents reject lockout).
- Current telemetry indicates asymmetric POS updates: right side updates regularly, left side often stale; this causes dL/dR mismatch and poor accept behavior on the stale side.
- Left channel in code maps to node 11 (esc_ids = {11,12}), so current evidence points to node 11 path being the one with intermittent POSVEL updates.

## Serial1 Packet (Teensy -> ESP32)
- Data sent over Serial1 is a binary unwrap dump stream, not JSON.
- Packet format: Header (32 bytes) + Payload (sample_count * 29 bytes) + Trailer CRC32 (4 bytes).
- Header magic is "TWR1" and includes version, sample rate, sample count, payload size, and payload CRC.
- Each payload sample includes fixed-point unwrap debug values (raw pos/vel, wrapped deltas dL/dR, accept flags, unwrapped positions, wheel position/velocity).
- main.cpp sends this only when supervisor.mode == SUP_MODE_IDLE and telemetry_unwrap_dump_active() is true.

## Purpose of SEND_TELEMETRY
- SEND_TELEMETRY gates live JSON debug output to Serial for rapid tuning/inspection during runs.
- It exposes IMU values, raw POSVEL values, unwrap gate internals (dL/dR, max step, accept flags), timing (loop_dt_us, loop_hz, POS sample dt), and derived state (x_wheel, x_dot).
- It is diagnostic/observability data for debugging filter behavior; it is separate from the binary unwrap dump sent to Serial1.
- Enabling it helps validate assumptions (rate mismatch, stale samples, per-wheel update behavior) without decoding the binary dump first.

## CAN Physical Layer Failure Modes
- One ESC updates consistently while the other goes stale for tens/hundreds of ms.
- Possible issues. 
- Bad termination is the top issue: CAN needs ~60 ohms across CANH/CANL (two 120-ohm terminators at bus ends). Missing/extra terminators cause reflections and frame errors.
- Wiring polarity swapped (CANH/CANL) can prevent reliable communication or create intermittent behavior.
- No common ground between nodes can shift transceiver common-mode voltage and cause random drops.
- Long stubs/poor topology (star wiring, long branch leads) degrade signal integrity, especially at 500 kbps.
- Bit rate mismatch between nodes/transceivers causes frequent CRC/ACK errors even if packets occasionally appear.
- Weak/noisy power to transceivers can create temperature/load-dependent dropouts.
- EMI coupling from motor phases/ESC switching into CAN lines can corrupt frames if twisted pair, routing, and shielding are poor.
- Faulty transceiver or damaged connector/crimp can create one-sided intermittent receive (looks like one node “drops frames”).

## telemetry

**ESP32/wifi_repeater**
access with: 
`screen /dev/tty.usbserial-024YH236 115200` 

also launch:
./ESP32/wifi_repeater/connect3.py

```
Header: Header(magic=b'TWR1', version=1, msg_type=1, sample_rate_hz=500, sample_bytes=29, sample_count=2500, start_index=1582, payload_bytes=72500, payload_crc32=1716258398, header_crc32=0)
CRC header=0x664C065E trailer=0x664C065E calc=0x664C065E
Unpacked 2500 samples.

[CAN POSVEL RX] Latest live stats (from Serial1 JSON)
  Left  id=11  count=18337  age_us=607  avg_gap_us=2027  min_gap_us=6  max_gap_us=33265  est_missed=8785
  Right id=12 count=18474 age_us=333 avg_gap_us=2012 min_gap_us=0 max_gap_us=30994 est_missed=8921

[Test 1] Raw wrapped position freshness (exact repeats)
  Left: repeats=1334/2499 (53.38%), max_run=11, avg_run=2.14, runs=1166
  Right: repeats=1161/2499 (46.46%), max_run=11, avg_run=1.87, runs=1339
  Longest constant runs (Left): [(277, 11, 548), (864, 11, 3208), (884, 11, 4998), (904, 11, 479), (1040, 11, 3277)]
  Longest constant runs (Right): [(874, 11, 3636), (894, 11, 1841), (1030, 11, 5199), (1050, 11, 3392), (1070, 11, 1591)]

[Test 2] LEFT unwrap/gate consistency (integer mrad)
  Accept tolerance: +/-1 mrad
  Samples (excluding i=0): 2499
  Accept: 194  Reject: 2305
  Reject mismatches (acc=0 but Δunwrap!=0): 0
  Accept mismatches (acc=1 and |Δunwrap-d_wrapped|>1): 0

[Test 2] RIGHT unwrap/gate consistency (integer mrad)
  Accept tolerance: +/-1 mrad
  Samples (excluding i=0): 2499
  Accept: 349  Reject: 2150
  Reject mismatches (acc=0 but Δunwrap!=0): 0
  Accept mismatches (acc=1 and |Δunwrap-d_wrapped|>1): 0

```

`Longest constant runs` means the longest stretches of consecutive samples where the raw wrapped position value stayed exactly the same.

Each tuple is:
- (start_index, length, value)
- So for example (165, 11, 925) means:
  -  starting at sample index 165,
  -  for 11 samples in a row,
  -  the value was constantly 925 mrad.
It’s a freshness/staleness indicator: longer runs imply more repeated (unchanged) samples.

`est_missed` is an approximate count of likely missed POSVEL frames per ESC, computed from large inter-arrival gaps.

How it’s computed in firmware:
- Expected POSVEL period is assumed 2000 us (500 Hz).
- For each received frame, compute gap_us = now - last_rx.
- If gap_us > 3000 us (1.5x expected), estimate how many periods fit in the gap:
- periods ≈ round(gap_us / 2000)
- `est_missed` += max(0, periods - 1)
- So `est_missed` is cumulative and heuristic, not exact packet-loss truth.

Why yours is large:
- Stats are long-running cumulative (count ~33k), not per-test window.
- Very large historical pauses (max_gap_us up to ~0.6–0.98 s) add lots of estimated misses at once.
- min_gap_us=0 and bursty timing also show queueing/jitter effects, which can inflate this estimator.

