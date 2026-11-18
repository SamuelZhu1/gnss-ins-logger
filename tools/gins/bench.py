"""GNSS outage benchmark.

For each ride, pick random windows (seeded) where the bike is moving and the
reference is good, mask GNSS over each window, and measure the horizontal
distance between the navigator's position at the end of the window and the
reference. Windows are spaced so the filter re-converges between them, which
lets one replay carry many windows.
"""
from __future__ import annotations

import zlib

import numpy as np
import pandas as pd

from . import _core
from .dataset import Dataset, RideData
from .replay import replay

CONFIGS = {
    "bike_constraint": {},
    "no_constraint": {"use_nhc": False},
}


def pick_windows(gnss: pd.DataFrame, reference: pd.DataFrame, length_s: float, n: int, rng: np.random.Generator,
                 min_speed: float = 3.0, start_after_s: float = 90.0, spacing_s: float = 120.0) -> list[tuple[int, int]]:
    t = gnss["t_gps_us"].to_numpy(np.int64)
    speed = np.hypot(gnss["vn"].to_numpy(), gnss["ve"].to_numpy())
    valid = gnss["fix_type"].to_numpy() >= 3
    good = valid & (speed >= min_speed)
    ref_t = reference["t_gps_us"].to_numpy(np.int64)
    if t.size == 0 or ref_t.size == 0:
        return []
    L = int(length_s * 1e6)
    t0 = t[0] + int(start_after_s * 1e6)

    # A start is usable if every epoch in the window is good and the reference has
    # a point within 150 ms of the window end.
    starts = []
    cum = np.concatenate([[0], np.cumsum(good)])
    for i in np.flatnonzero(good & (t >= t0)):
        j = np.searchsorted(t, t[i] + L)
        if j >= t.size:
            break
        if cum[j + 1] - cum[i] != j - i + 1:
            continue
        k = np.searchsorted(ref_t, t[i] + L)
        near = [abs(ref_t[x] - (t[i] + L)) for x in (k - 1, k) if 0 <= x < ref_t.size]
        if near and min(near) <= 150_000:
            starts.append(t[i])
    starts = np.array(starts, dtype=np.int64)
    rng.shuffle(starts)
    chosen: list[int] = []
    gap = int((length_s + spacing_s) * 1e6)
    for s in starts:
        if all(abs(int(s) - c) >= gap for c in chosen):
            chosen.append(int(s))
            if len(chosen) == n:
                break
    return sorted((c, c + L) for c in chosen)


def window_errors(out: pd.DataFrame, reference: pd.DataFrame, origin: tuple[float, float, float],
                  windows: list[tuple[int, int]]) -> list[float]:
    """Horizontal error at the last output inside each window."""
    t = out["t_gps_us"].to_numpy(np.int64)
    ref_t = reference["t_gps_us"].to_numpy(np.int64)
    ref_ned = _core.lla_to_ned(*origin, reference["lat"].to_numpy(), reference["lon"].to_numpy(),
                               reference["h"].to_numpy())
    est_ned = _core.lla_to_ned(*origin, out["lat"].to_numpy(), out["lon"].to_numpy(), out["h"].to_numpy())
    errs = []
    for start, end in windows:
        i = np.searchsorted(t, end) - 1
        if i < 0 or t[i] < start:
            errs.append(np.nan)
            continue
        rn = np.interp(t[i], ref_t, ref_ned[:, 0])
        re = np.interp(t[i], ref_t, ref_ned[:, 1])
        errs.append(float(np.hypot(est_ned[i, 0] - rn, est_ned[i, 1] - re)))
    return errs


def run_ride(ride: RideData, lengths=(10, 20, 30), per_ride: int = 20, seed: int = 0,
             configs: dict | None = None) -> pd.DataFrame:
    configs = configs or CONFIGS
    imu, gnss, ref = ride.imu, ride.gnss, ride.reference
    rows = []
    rng = np.random.default_rng([seed, zlib.crc32(ride.ride_id.encode())])
    origin = (float(ref["lat"].iloc[0]), float(ref["lon"].iloc[0]), float(ref["h"].iloc[0]))
    for length in lengths:
        windows = pick_windows(gnss, ref, length, per_ride, rng)
        if not windows:
            continue
        for name, cfg in configs.items():
            errs = []
            for k in range(0, len(windows), _core.MAX_MASKS):
                batch = windows[k:k + _core.MAX_MASKS]
                out, _ = replay(ride, batch, cfg, imu=imu, gnss=gnss)
                errs += window_errors(out, ref, origin, batch)
            rows += [{"ride_id": ride.ride_id, "length_s": length, "config": name, "start_gps_us": w[0],
                      "error_m": e} for w, e in zip(windows, errs)]
    return pd.DataFrame(rows)


def run(dataset: Dataset, lengths=(10, 20, 30), per_ride: int = 20, seed: int = 0,
        configs: dict | None = None) -> pd.DataFrame:
    frames = [run_ride(dataset.ride(rid), lengths, per_ride, seed, configs) for rid in dataset.ride_ids()]
    frames = [f for f in frames if not f.empty]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def summarize(results: pd.DataFrame) -> pd.DataFrame:
    if results.empty:
        return results
    g = results.dropna().groupby(["length_s", "config"])["error_m"]
    return pd.DataFrame({
        "windows": g.size(),
        "median_m": g.median().round(2),
        "p95_m": g.quantile(0.95).round(2),
        "max_m": g.max().round(2),
    }).reset_index()


def markdown(summary: pd.DataFrame, note: str = "") -> str:
    lines = ["| Outage | Config | Windows | Median (m) | 95th pct (m) | Max (m) |",
             "|---|---|---|---|---|---|"]
    for _, r in summary.iterrows():
        lines.append(f"| {r.length_s:g} s | {r.config} | {r.windows} | {r.median_m:.2f} | {r.p95_m:.2f} | {r.max_m:.2f} |")
    if note:
        lines += ["", note]
    return "\n".join(lines) + "\n"
