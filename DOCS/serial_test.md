# Serial Transmission Performance Plots

## 1. Inter-arrival Time Plots ⏱️
- **What:** time between received packets vs time  
- **Why:** shows jitter and gaps. Ideally flat (e.g. 10 ms if 100 Hz)  
- **Variants:**  
  - Histogram of inter-arrival times → see spread  
  - CDF plot → see “99% of packets arrive within X ms”

## 2. Throughput vs Time 📊
- **What:** bytes per second (or packets per second), computed in rolling windows  
- **Why:** confirms you’re sustaining expected rates (e.g. 100 packets/s)  
- **Variant:** per-message ID breakdown (if multiple IDs are logged)

## 3. Sequence Number Gaps 🚧
- **What:** received `seq` vs expected sequence; plot missing/gap events  
- **Why:** any non-zero gaps = packet loss  
- **Display:** raster of loss events or cumulative loss counter

## 4. Queue Depth / Buffer Occupancy 📉
- **What:** TX and RX queue depth over time  
- **Why:** visualize back-pressure; rising depth means consumer can’t keep up  
- **Add:** plot high-water marks reset each interval

## 5. Latency (if you stamp) 📡
- **What:** plot `t_rx – t_tx` per packet (end-to-end delay)  
- **Why:** shows true transmission + buffering delay  
- **Variants:**  
  - Histogram (distribution of latency)  
  - Jitter plot (std dev or min/max bands)

## 6. Drop/Error Events ❌
- **What:** time-series of `drops` counters or CRC failures  
- **Why:** pinpoints bursts that cause overruns or framing errors

## 7. Aggregate Stats Panel 📋
- **Mean / Std of inter-arrival (ms)**  
- **% loss**  
- **Max observed queue depth**  
- **CPU % (optional)**
