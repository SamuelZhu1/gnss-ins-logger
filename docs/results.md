# Results

Two kinds of numbers live here, and they never mix:

- **Simulated:** synthetic rides from `gins synth` (exact kinematics plus modelled noise). These check that the pipeline works and that the bike motion constraint helps. They are optimistic: the synthetic IMU has constant biases, white vibration, a perfectly rigid mount and no lever arm.
- **Rides:** measured on the bike. Empty until the hardware runs.

## Outage drift, simulated (3 synthetic rides, 20 min each, seed 0)

| Outage | Config | Windows | Median (m) | 95th pct (m) | Max (m) |
|---|---|---|---|---|---|
| 10 s | bike_constraint | 20 | 1.21 | 2.84 | 2.97 |
| 10 s | no_constraint | 20 | 2.07 | 4.10 | 4.30 |
| 20 s | bike_constraint | 21 | 2.40 | 7.19 | 7.30 |
| 20 s | no_constraint | 21 | 6.08 | 12.11 | 14.23 |
| 30 s | bike_constraint | 16 | 4.44 | 21.90 | 23.55 |
| 30 s | no_constraint | 16 | 11.38 | 24.02 | 24.65 |

Reproduce:

```sh
for s in 11 12 13; do
  gins synth --out sim/s$s.bin --reference sim/s${s}_ref.csv --minutes 20 --seed $s
  gins convert sim/s$s.bin --dataset sim/ds --reference sim/s${s}_ref.csv --ride-id synth_s$s
done
gins bench --dataset sim/ds --per-ride 20 --seed 0
```

## Rides (to fill in)

| Metric | Simulated | Rides |
|---|---|---|
| Median drift, 20 s outage, bike constraint | 2.40 m | |
| Median drift, 20 s outage, no constraint | 6.08 m | |
| Dropped IMU samples, sync writes vs ring buffer | n/a | |
| Filter CPU load on the ESP32-S3 | n/a | |
| GNSS arrival lag (NAV-PVT after epoch) | 42 ms (modelled) | |
| Rides / hours in the dataset | 3 / 1.0 (synthetic) | |
