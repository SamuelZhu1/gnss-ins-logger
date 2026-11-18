# Bring-up on the bench

Everything below exists in code. What's left is wiring, flashing and running it.

## 1. Wiring (defaults; change in `idf.py menuconfig`)

| Signal | ESP32-S3 GPIO | Notes |
|---|---|---|
| IMU SCLK / MOSI / MISO / CS | 12 / 11 / 13 / 10 | SPI2, 10 MHz, mode 0 |
| IMU INT1 | 14 | data ready, push-pull, active high |
| SD SCLK / MOSI / MISO / CS | 39 / 40 / 41 / 42 | SPI3; avoids GPIO 35-37 (octal PSRAM on N8R8 boards) |
| F9P RX1 ← ESP TX | 17 | UART1 |
| F9P TX1 → ESP RX | 18 | |
| F9P PPS (timepulse) | 7 | 1 Hz, aligned to GPS time by the firmware |
| RTCM corrections | F9P UART2 | from the phone's NTRIP client over a Bluetooth serial module |

Strapping pins (0, 3, 45, 46) are left alone.

## 2. First boot

1. `idf.py flash monitor`. Expect `icm42688: ready`, `gnss: receiver configured` and `logger: logging to /sdcard/R0001.BIN`.
2. In the shell, `status` should show `imu_hz` close to 200, `missed_irq` 0 and `overruns` 0.
3. Frame check: lay the board flat and `az` should read about -9.8 in the log. Tilt the nose up and pitch should go positive. If not, fix the mounting rotation in menuconfig.
4. Outside with sky view: wait for `fix` 3 and `pps_locked` true. `fix_latency_ms` should settle around 20 to 60. Write the number down; it's the arrival lag the PPS path removes.

## 3. Rides

1. Hold the bike still for 20 s after the fix (mode goes `ALIGNING` → `WAIT_YAW`), then ride. Mode should reach `NAV` within a few seconds of riding.
2. Start the NTRIP client on the phone so the F9P reaches RTK fixed (`carr` 2). Those epochs become the reference track.
3. Optional live outage: `mask gnss 20` in the shell while riding.
4. After the ride: `log stop`, copy `R####.BIN` off the card, then run `gins convert`.

## 4. Numbers to record (fill in docs/results.md)

| Measurement | How |
|---|---|
| Drift during 20 s outages, with and without the bike motion constraint | `gins bench --dataset dataset --lengths 10 20 30` |
| Dropped IMU samples, before vs after the ring buffer | flash the `experiments` build (`CONFIG_GINS_LOG_SYNC_WRITES=y`) for one 30-minute ride and the normal build for another, then compare `imu_dropped_pct` in the manifest |
| Filter CPU load | `status` → `filter_cpu_pct`, `filter_max_us` while riding |
| GNSS arrival lag | `status` → `fix_latency_ms`, or `fix_latency_ms` in the manifest |
| Effect of arrival stamping | `gins convert --stamp-on-arrival` on the same log, then compare the bench |
| Dataset size | `gins stats` |

## 5. Things likely to bite

- The handlebars rotate, so mount the IMU on the frame. Foam under the box helps with vibration.
- Cheap SD cards stall longer. A high-endurance card helps, and `sd_stall_max_ms` shows what you got.
- No NTRIP base nearby: log RAWX (already on at 1 Hz) and post-process with NRCan CSRS-PPP. Then import the result with `gins convert --reference ppp.csv` (columns `t_gps_us,lat,lon,h`).
