"""Synthetic rides in the on-board log format.

Used for tests, CI and trying the tools before real rides exist. A synthetic
ride is never a result: the README and results table keep simulated and
measured numbers apart.

Model: a bike starts still (for alignment), then rides a random route of
straight segments and corners with stops, leaning into turns. The IMU sees
the exact specific force and angular rate plus bias, sensor noise, road
vibration and quantization. GNSS fixes carry slowly varying position error
(first-order Gauss-Markov), worse in "downtown" stretches, and a short
tunnel with no fix. The board's clock runs at a small drift; PPS edges and
NAV-PVT arrivals (about 42 ms late) are stamped on it, as on the real board.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from . import logfmt

G = 9.80665
ACCEL_SCALE = 9.80665 / 4096.0              # +-8 g
GYRO_SCALE = (500.0 / 32768.0) * np.pi / 180  # +-500 dps
TEMP_SCALE = 1 / 132.48
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass
class SynthConfig:
    minutes: float = 5.0
    static_s: float = 25.0
    seed: int = 1
    origin: tuple[float, float, float] = (49.2606, -123.2460, 80.0)
    cruise: float = 6.0                  # m/s
    imu_hz: float = 200.0
    gnss_hz: float = 10.0
    accel_bias_sigma: float = 0.05       # m/s^2
    gyro_bias_sigma: float = 0.003       # rad/s
    accel_noise: float = 0.002           # m/s^2/sqrt(Hz)
    gyro_noise: float = 2e-4             # rad/s/sqrt(Hz)
    vibration_accel: float = 0.5         # m/s^2 per sample while moving
    vibration_gyro: float = 0.01         # rad/s per sample while moving
    gnss_sigma: float = 0.6              # m, Gauss-Markov horizontal
    gnss_tau: float = 60.0               # s
    downtown_sigma: float = 3.0          # m, in degraded stretches
    tunnel_s: float = 8.0                # one stretch with no fix
    latency_ms: float = 42.0             # NAV-PVT arrival after its epoch
    clock_drift_ppm: float = 18.0
    drop_imu_pct: float = 0.0            # simulate records lost before the card
    mount_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gps_t0_us: int = 345_600_000_000      # Wednesday 00:00 in GPS time of week


@dataclass
class Ride:
    raw: bytes
    truth: pd.DataFrame            # at every GNSS epoch: t_gps_us, lat, lon, h, vn, ve, yaw
    imu_truth_t: np.ndarray        # true GPS time of each IMU sample written
    info: dict = field(default_factory=dict)


def _route(cfg: SynthConfig, rng: np.random.Generator, t: np.ndarray):
    """Speed and yaw rate profiles on the fine time grid."""
    n = t.size
    speed = np.zeros(n)
    yaw_rate = np.zeros(n)
    downtown = np.zeros(n, dtype=bool)
    tt = cfg.static_s
    end = t[-1]
    while tt < end:
        straight = rng.uniform(20, 70)
        target = cfg.cruise * rng.uniform(0.8, 1.15)
        seg = (t >= tt) & (t < tt + straight)
        speed[seg] = target
        if rng.random() < 0.35:
            downtown[seg] = True
        tt += straight
        # Corner: optionally stop first (a red light), then turn about 90 degrees.
        if rng.random() < 0.3:
            stop = rng.uniform(5, 15)
            seg = (t >= tt) & (t < tt + stop)
            speed[seg] = 0.0
            tt += stop
        radius = rng.uniform(8, 18)
        v_turn = min(target, 4.5)
        angle = np.deg2rad(rng.uniform(70, 110)) * rng.choice([-1, 1])
        turn_s = abs(angle) * radius / v_turn
        seg = (t >= tt) & (t < tt + turn_s)
        speed[seg] = v_turn
        yaw_rate[seg] = np.sign(angle) * v_turn / radius
        tt += turn_s
    # Smooth both profiles so acceleration and lean rate stay finite.
    k = int(2.0 / (t[1] - t[0]))
    win = np.hanning(k)
    win /= win.sum()
    speed = np.convolve(speed, win, mode="same")
    yaw_rate = np.convolve(yaw_rate, win, mode="same")
    speed[t < cfg.static_s] = 0.0
    yaw_rate[t < cfg.static_s] = 0.0
    return speed, yaw_rate, downtown


def _rot(roll, pitch, yaw):
    """Body to NED rotation matrices for arrays of angles (ZYX)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    r = np.empty(np.broadcast(roll, pitch, yaw).shape + (3, 3))
    r[..., 0, 0] = cy * cp
    r[..., 0, 1] = cy * sp * sr - sy * cr
    r[..., 0, 2] = cy * sp * cr + sy * sr
    r[..., 1, 0] = sy * cp
    r[..., 1, 1] = sy * sp * sr + cy * cr
    r[..., 1, 2] = sy * sp * cr - cy * sr
    r[..., 2, 0] = -sp
    r[..., 2, 1] = cp * sr
    r[..., 2, 2] = cp * cr
    return r


def _ned_to_lla(origin, ned):
    lat0, lon0, h0 = np.deg2rad(origin[0]), np.deg2rad(origin[1]), origin[2]
    rn = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * np.sin(lat0) ** 2) ** 1.5
    re = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat0) ** 2)
    lat = lat0 + ned[:, 0] / (rn + h0)
    lon = lon0 + ned[:, 1] / ((re + h0) * np.cos(lat0))
    return np.rad2deg(lat), np.rad2deg(lon), h0 - ned[:, 2]


def _gauss_markov(rng, n, dt, sigma, tau):
    a = np.exp(-dt / tau)
    w = rng.normal(0, sigma * np.sqrt(1 - a * a), n)
    x = np.empty(n)
    x[0] = rng.normal(0, sigma)
    for i in range(1, n):
        x[i] = a * x[i - 1] + w[i]
    return x


def make_ride(cfg: SynthConfig | None = None) -> Ride:
    cfg = cfg or SynthConfig()
    rng = np.random.default_rng(cfg.seed)
    total = cfg.static_s + cfg.minutes * 60
    h = 1e-3
    t = np.arange(0, total + h / 2, h)
    speed, yaw_rate, downtown = _route(cfg, rng, t)

    yaw = np.cumsum(yaw_rate) * h
    vn = speed * np.cos(yaw)
    ve = speed * np.sin(yaw)
    pn = np.cumsum(vn) * h
    pe = np.cumsum(ve) * h
    lean = np.arctan(speed * yaw_rate / G)
    an = np.gradient(vn, h)
    ae = np.gradient(ve, h)
    lean_rate = np.gradient(lean, h)

    # IMU samples: the sensor's own clock is slightly off 200 Hz.
    imu_dt = 1.0 / (cfg.imu_hz * (1 + 35e-6))
    t_imu = np.arange(imu_dt, total, imu_dt)
    tm = t_imu - imu_dt / 2  # interval midpoint stands in for the interval average

    def at(x):
        return np.interp(tm, t, x)

    yaw_m, lean_m = at(yaw), at(lean)
    a_n = np.stack([at(an), at(ae), np.zeros_like(tm)], axis=1)
    r_nb = _rot(lean_m, np.zeros_like(tm), yaw_m)
    f_b = np.einsum("nji,nj->ni", r_nb, a_n - np.array([0.0, 0.0, G]))
    yr = at(yaw_rate)
    w_b = np.stack([at(lean_rate), yr * np.sin(lean_m), yr * np.cos(lean_m)], axis=1)

    moving = at(speed) > 0.3
    n_imu = t_imu.size
    ba = rng.normal(0, cfg.accel_bias_sigma, 3)
    bg = rng.normal(0, cfg.gyro_bias_sigma, 3)
    sa = cfg.accel_noise * np.sqrt(cfg.imu_hz)
    sg = cfg.gyro_noise * np.sqrt(cfg.imu_hz)
    acc = f_b + ba + rng.normal(0, sa, (n_imu, 3)) + moving[:, None] * rng.normal(0, cfg.vibration_accel, (n_imu, 3))
    gyr = w_b + bg + rng.normal(0, sg, (n_imu, 3)) + moving[:, None] * rng.normal(0, cfg.vibration_gyro, (n_imu, 3))

    # Body to sensor frame (inverse of the mounting rotation the tools apply).
    r_bs = _rot(*np.deg2rad(cfg.mount_rpy_deg))
    acc_s = acc @ r_bs   # (R_bs^T x) for each row
    gyr_s = gyr @ r_bs
    acc_c = np.clip(np.round(acc_s / ACCEL_SCALE), -32768, 32767).astype(np.int16)
    gyr_c = np.clip(np.round(gyr_s / GYRO_SCALE), -32768, 32767).astype(np.int16)
    temp_c = np.full(n_imu, int(round(6.0 / TEMP_SCALE)), dtype=np.int16)  # 31 C

    # GNSS epochs.
    t_gnss = np.arange(0.1, total, 1.0 / cfg.gnss_hz)
    t_gnss = np.round(t_gnss * 10) / 10
    dtg = 1.0 / cfg.gnss_hz
    ng = t_gnss.size
    ex = _gauss_markov(rng, ng, dtg, cfg.gnss_sigma, cfg.gnss_tau)
    ey = _gauss_markov(rng, ng, dtg, cfg.gnss_sigma, cfg.gnss_tau)
    ez = _gauss_markov(rng, ng, dtg, 1.6 * cfg.gnss_sigma, cfg.gnss_tau)
    dt_mask = np.interp(t_gnss, t, downtown.astype(float)) > 0.5
    ex[dt_mask] += rng.normal(0, cfg.downtown_sigma, dt_mask.sum())
    ey[dt_mask] += rng.normal(0, cfg.downtown_sigma, dt_mask.sum())
    truth_ned = np.stack([np.interp(t_gnss, t, pn), np.interp(t_gnss, t, pe), np.zeros(ng)], axis=1)
    meas_ned = truth_ned + np.stack([ex, ey, ez], axis=1)
    lat_t, lon_t, h_t = _ned_to_lla(cfg.origin, truth_ned)
    lat_m, lon_m, h_m = _ned_to_lla(cfg.origin, meas_ned)
    vng = np.interp(t_gnss, t, vn) + rng.normal(0, 0.05, ng)
    veg = np.interp(t_gnss, t, ve) + rng.normal(0, 0.05, ng)
    vdg = rng.normal(0, 0.05, ng)
    hacc = np.where(dt_mask, cfg.downtown_sigma * 1.2, 1.0) + rng.uniform(0, 0.3, ng)
    fix_type = np.full(ng, 3, dtype=np.uint8)
    # One tunnel after alignment, while riding.
    ride_t = t_gnss[(t_gnss > cfg.static_s + 60) & (np.interp(t_gnss, t, speed) > 3)]
    tunnel = (0.0, 0.0)
    if cfg.tunnel_s > 0 and ride_t.size:
        t0 = float(rng.choice(ride_t[: max(1, ride_t.size // 2)]))
        tunnel = (t0, t0 + cfg.tunnel_s)
        fix_type[(t_gnss >= tunnel[0]) & (t_gnss < tunnel[1])] = 0

    # Clocks: GPS time and the board's local microsecond counter.
    def gps_us(x):
        return cfg.gps_t0_us + np.round(np.asarray(x) * 1e6).astype(np.int64)

    def local_us(x):
        return (5_000_000 + np.round(np.asarray(x) * 1e6 * (1 + cfg.clock_drift_ppm * 1e-6))).astype(np.int64)

    imu_local = local_us(t_imu) + rng.integers(-3, 4, n_imu)
    pps_t = np.arange(1.0, total, 1.0)
    pps_local = local_us(pps_t) + rng.integers(-1, 2, pps_t.size)
    lat_s = np.clip(rng.normal(cfg.latency_ms, 4.0, ng), 20, 80) * 1e-3
    pvt_local = local_us(t_gnss + lat_s)
    itow_ms = (gps_us(t_gnss) // 1000).astype(np.uint32)

    # Records, payloads built as structured arrays then encoded in bulk.
    keep = np.ones(n_imu, dtype=bool)
    if cfg.drop_imu_pct > 0:
        keep = rng.random(n_imu) >= cfg.drop_imu_pct / 100
    imu = np.zeros(n_imu, dtype=logfmt.DTYPES[logfmt.IMU])
    imu["t_local_us"] = imu_local
    imu["ax"], imu["ay"], imu["az"] = acc_c.T
    imu["gx"], imu["gy"], imu["gz"] = gyr_c.T
    imu["temp"] = temp_c
    imu_seq = np.arange(n_imu, dtype=np.uint32)

    pvt = np.zeros(ng, dtype=logfmt.DTYPES[logfmt.GNSS_PVT])
    pvt["t_local_us"] = pvt_local
    pvt["itow_ms"] = itow_ms
    pvt["lat_e7"] = np.round(lat_m * 1e7)
    pvt["lon_e7"] = np.round(lon_m * 1e7)
    pvt["h_mm"] = np.round(h_m * 1e3)
    pvt["vn"], pvt["ve"], pvt["vd"] = (np.round(vng * 1e3), np.round(veg * 1e3), np.round(vdg * 1e3))
    pvt["hacc"] = np.round(hacc * 1e3)
    pvt["vacc"] = np.round(hacc * 1.6e3)
    pvt["sacc"] = 60
    pvt["fix_type"] = fix_type
    pvt["num_sv"] = np.where(fix_type > 0, rng.integers(14, 28, ng), 0)

    pps = np.zeros(pps_t.size, dtype=logfmt.DTYPES[logfmt.PPS])
    pps["t_local_us"] = pps_local

    header = np.zeros(1, dtype=logfmt.DTYPES[logfmt.HEADER])
    header["version"] = b"synthetic"
    header["accel_scale"] = ACCEL_SCALE
    header["gyro_scale"] = GYRO_SCALE
    header["temp_scale"] = TEMP_SCALE
    header["temp_offset"] = 25.0
    header["mount_roll"], header["mount_pitch"], header["mount_yaw"] = cfg.mount_rpy_deg
    header["imu_rate_hz"] = int(cfg.imu_hz)

    def enc(rtype, arr, seq):
        u8 = arr.view(np.uint8).reshape(arr.size, arr.dtype.itemsize)
        return logfmt.encode_many(rtype, seq, u8)

    blocks = [
        (np.array([0]), enc(logfmt.HEADER, header, np.array([0]))),
        (imu_local[keep], enc(logfmt.IMU, imu[keep], imu_seq[keep])),
        (pvt_local, enc(logfmt.GNSS_PVT, pvt, np.arange(ng))),
        (pps_local, enc(logfmt.PPS, pps, np.arange(pps_t.size))),
    ]
    # Write in local-time order, as the board's buffer would.
    times = np.concatenate([b[0] for b in blocks])
    recs = [r for b in blocks for r in b[1]]
    order = np.argsort(times, kind="stable")
    raw = b"".join(recs[i].tobytes() for i in order)

    truth = pd.DataFrame({
        "t_gps_us": gps_us(t_gnss), "lat": lat_t, "lon": lon_t, "h": h_t,
        "vn": np.interp(t_gnss, t, vn), "ve": np.interp(t_gnss, t, ve), "yaw": np.interp(t_gnss, t, yaw),
        "source": "truth",
    })
    info = {
        "duration_s": total, "tunnel": tunnel, "accel_bias": ba.tolist(), "gyro_bias": bg.tolist(),
        "imu_dropped": int((~keep).sum()), "imu_written": int(keep.sum()),
    }
    return Ride(raw=raw, truth=truth, imu_truth_t=gps_us(t_imu[keep]), info=info)


def write(ride: Ride, raw_path: Path, reference_csv: Path | None = None) -> None:
    raw_path = Path(raw_path)
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    raw_path.write_bytes(ride.raw)
    if reference_csv is not None:
        ride.truth[["t_gps_us", "lat", "lon", "h", "source"]].to_csv(reference_csv, index=False)
