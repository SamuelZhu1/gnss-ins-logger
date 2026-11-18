# Design notes

## Frames and units

- Body frame: x forward, y right, z down (the bike's frame after the mounting rotation).
- Navigation frame: local north, east, down from the first good fix. Earth rotation and transport rate are ignored. At bike speeds the earth rate (about 15 deg/h) is far below the gyro noise.
- Accelerometer: specific force in m/s^2. At rest, level, it reads (0, 0, -9.81).
- Time: GPS time in microseconds (week rollovers counted). The board's local `esp_timer` microseconds are mapped to GPS time by `TimeBase`.

## Error-state Kalman filter (`core/src/ins_filter.cpp`)

The error state is `dp, dv, dtheta, dba, dbg` (15). `dtheta` is a small rotation in the body frame: `R_true = R (I + [dtheta]x)`.

Propagation for each IMU sample (dt about 5 ms):
- Nominal: `a = R (f - ba) + g`, `p += v dt + a dt^2/2`, `v += a dt`, `q = q * exp((w - bg) dt)`.
- Error transition: `dv' = -R [f]x dtheta - R dba` and `dtheta' = -[w]x dtheta - dbg`. It's discretized to first order, with the attitude block taken as the exact rotation `exp(-[w]x dt)`.
- Process noise: white accel and gyro noise, plus random-walk biases. The datasheet densities are inflated for frame vibration.

Updates use the Joseph form, so P stays symmetric positive definite in float. A floor on the diagonal guards against rounding. After each injection the attitude covariance gets the reset Jacobian `G = I - [dtheta/2]x`.

| Update | Measurement | H (non-zero blocks) | R |
|---|---|---|---|
| GNSS | p, v in NED | I on dp, I on dv | receiver hAcc, vAcc, sAcc squared |
| Bike motion constraint | lateral and vertical body velocity = 0 | rows 2-3 of R^T on dv, rows 2-3 of [v_b]x on dtheta | (0.3 m/s)^2 |
| ZUPT + ZARU | v = 0, gyro reading = bias | I on dv, I on dbg | (0.02 m/s)^2, (0.005 rad/s)^2 |

GNSS updates go through a chi-square gate (6 DOF, 99.9%, 22.46). The constraint and ZUPT are not gated.

## Navigator (`core/src/navigator.cpp`)

Modes: `WAIT_FIX` → `ALIGNING` → `WAIT_YAW` → `NAV` (and `DEGRADED` after 60 s with no accepted fix).

- **Static alignment:** 20 s still. Roll and pitch come from mean specific force, and gyro bias from the mean rate. Movement (accel norm spread or gyro rate) restarts it.
- **Yaw:** taken from GNSS course once speed stays above 3 m/s for 3 s. If the rider sets off before alignment finishes, the filter starts level with zero gyro bias and wider attitude uncertainty ("coarse" alignment, counted in diagnostics).
- **Time ordering:** IMU samples wait 100 ms in a queue, and fixes are merged by epoch time. A fix whose epoch falls between two samples is applied after propagating exactly to the epoch with the next sample's reading. A fix older than the filter state is dropped and counted as late.
- **Outage recovery:** after 10 rejected fixes in a row, the covariance is inflated by the innovation and the next fix is forced through.
- **Stationary detector:** a 40-sample window (0.2 s) of accel-norm spread and mean gyro rate.

## Time base (`core/src/time_base.cpp`)

The PPS ISR stamps each rising edge in local time. The next NAV-PVT with a whole-second epoch names that edge. A least-squares line through the last 8 pairs gives offset and crystal drift. Lock is lost after 3 s without a pair; the last fit carries on (holdover). Before the first pair, fixes are mapped by arrival time minus a nominal 40 ms. That fallback is the timing bug the PPS path removes. `CONFIG_GINS_STAMP_FIX_ON_ARRIVAL` and `gins convert --stamp-on-arrival` reproduce it for measurement.

At 6 m/s, a 40 ms timing error puts each GNSS update 0.24 m behind the bike. The innovation spikes show up on every turn. `Navigator.StampingFixesOnArrivalHurtsAccuracy` checks this on a synthetic ride.

## Real-time budget (firmware)

| Task / ISR | Core | Priority | Trigger | Budget |
|---|---|---|---|---|
| IMU data-ready ISR | - | ISR | 200 Hz | stamp time, notify (a few us) |
| PPS ISR | - | ISR | 1 Hz | stamp time, queue |
| nav | 1 | 20 | IMU notify | 5 ms per sample; overruns counted |
| gnss | 0 | 15 | UART bytes | parse, queue NAV-PVT, log |
| logger | 0 | 5 | ring buffer, 4 KB or 100 ms | none: the buffer absorbs stalls |
| shell | 0 | 3 | USB line | none |

The nav task measures its own busy time. `status` reports `filter_cpu_pct` and `filter_max_us`.

Ring buffer sizing: the log rate is about 6 KB/s (IMU 200 x 32 B, plus NAV-PVT 10 x 62 B, plus about 1.5 KB/s of RAWX). 64 KB covers about 10 s of SD stall, against observed stalls of 100 to 250 ms on cheap cards. Above 90% full, RAWX records are dropped first. IMU records are only dropped if the buffer is completely full, and every drop shows as a sequence gap.

## Log format

See `core/include/gins/log_format.hpp`. Each record has sync, type, length and a per-type sequence number, then the payload and a CRC-16. The reader resyncs after a bad CRC, so one corrupt record costs only itself. Payloads are fixed-length per type (except RAWX), which lets the Python side parse them with numpy structured dtypes.

## Why these choices

- **Error-state over full-state EKF:** the attitude error is small and three-dimensional; the quaternion has four numbers and a unit-norm constraint.
- **One filter source for firmware and replay:** a bug found on a ride replays bit-for-bit on the laptop (in double), and the float build runs the same GoogleTest suite.
- **Fixed lag over measurement back-dating:** 100 ms of buffered IMU costs 4 KB, and the filter never has to rewind. The live display lags 100 ms, which is fine for a logger.
- **SPI for the IMU:** it keeps the bus to itself, so the 200 Hz reads never wait behind anything else.
