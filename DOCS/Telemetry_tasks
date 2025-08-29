# Balancing Robot Telemetry Checklist

This checklist outlines staged plotting and logging goals for developing a Teensy + ESP32 balancing robot.  
Stages progress from low-level comms timing to full closed-loop balancing performance.

---

## Stage 0 — Comms & Timing (no control yet)
**Goal:** prove there’s no jitter and rates are correct.

- Loop period (`dt` in ms) vs time  
  - Histogram of dt  
  - Min/mean/max
- CAN message rate (frames/s) per ID  
  - Rolling count by ID  
  - Utilization % (frames/s × avg bytes / bus bitrate)
- UDP/Wi-Fi inter-arrival time (ms) & loss  
  - Sequence number for packet loss detection  
  - Latency estimate (TX vs RX timestamps)
- Queue depths / buffers (TX, RX)

---

## Stage 1 — Sensors alive & sane
**Goal:** confirm IMU/encoders are clean, oriented, and scaled.

- IMU accel & gyro (3 axes) vs time  
- Wheel/encoder position & velocity  
- Battery voltage & current  
- Noise snapshot (PSD / spectrum of gyro Z or tilt rate)

---

## Stage 2 — State estimation (if using a filter)
**Goal:** validate your state vector before closing the loop.

- Tilt angle (measured vs estimated) & tilt rate  
- Innovation/residuals (measured − predicted)  
- Confidence metrics (variance/covariance if available)

---

## Stage 3 — Open-loop actuator tests (wheels off-ground)
**Goal:** verify commands map to motion.

- Commanded motor effort vs measured wheel speed  
- Step response plots (speed vs time)  
- Current vs effort (torque constant check)

---

## Stage 4 — Closed-loop balancing at fixed height
**Goal:** bring it up gently and verify stability margins.

- Strip chart: tilt angle, tilt rate, wheel velocity, (optionally) position drift  
- Control inputs `u` (per motor) vs time  
- Optional: Kx terms breakdown (e.g., `k_theta*theta`, `k_dtheta*theta_dot`, etc.)  
- Settle time & overshoot for small disturbances

---

## Stage 5 — Robustness & performance
**Goal:** quantify comfort margins.

- Disturbance response (gentle pushes): angle, u, recovery time  
- Position creep/drift vs time at standstill  
- Motor/driver temperatures vs effort  
- Battery voltage sag vs thrust bursts

---

## Nice quality-of-life plots (any stage)

- Rolling stats panel: dt mean/std, packet loss %, CAN util %, CPU %  
- Event markers: log/annotate events (enable control, step, push)  
- Alarm flags: dt > threshold, loss > threshold, queue > N

---

## Suggested targets (ballpark)

- Control loop dt: mean ~1.0–2.0 ms (500–1000 Hz), std dev < 0.1–0.2 ms  
- CAN command/telemetry: 250–500 Hz per critical topic, bus util < ~40%  
- UDP loss: < 0.5%, inter-arrival jitter < 5–10 ms typical  
- Tilt at standstill: < 0.5–1.0° RMS; recovery < 0.3–0.5 s after a small bump  
- Actuator clipping: rare in steady state; visible only on disturbances

---

## Minimal telemetry packet schema (JSON-style)

```json
{
  "t": 12345678,
  "seq": 42,
  "dt": 0.002,
  "can_rx_counts": { "0x101": 500, "0x102": 250 },
  "imu": { "ax":0.0,"ay":0.0,"az":9.8,"gx":0.01,"gy":0.0,"gz":0.0 },
  "enc": { "pos":1234, "vel":12.3 },
  "state": { "theta":0.05,"theta_dot":-0.01,"v":0.1 },
  "u": { "left":0.2, "right":0.2 },
  "batt": { "V":23.8,"I":1.2 },
  "flags": { "clip":false, "overtemp":false, "loss":0 }
}
