"""Convert a raw board log into a ride in the dataset."""
from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import pandas as pd

from . import _core, logfmt
from .dataset import Dataset

NOMINAL_LATENCY_US = 40_000


def _rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _seq_stats(seq: np.ndarray) -> tuple[int, int]:
    """(records expected from the sequence numbers, records present)."""
    if seq.size == 0:
        return 0, 0
    s = np.sort(seq.astype(np.int64))
    return int(s[-1] - s[0] + 1), int(np.unique(s).size)


def _outages(t_us: np.ndarray, valid: np.ndarray, min_gap_s: float = 1.0) -> list[dict]:
    tv = t_us[valid]
    if tv.size < 2:
        return []
    gaps = np.diff(tv) * 1e-6
    out = []
    for i in np.flatnonzero(gaps > min_gap_s):
        out.append({"start_gps_us": int(tv[i]), "end_gps_us": int(tv[i + 1]), "seconds": round(float(gaps[i]), 1)})
    return out


def convert(raw_path: str | Path, dataset_root: str | Path, ride_id: str | None = None,
            reference_csv: str | Path | None = None, tags: list[str] | None = None,
            stamp_on_arrival: bool = False) -> dict:
    raw_path = Path(raw_path)
    data = raw_path.read_bytes()
    split = _core.split_records(data)

    _, header = logfmt.parse(split, logfmt.HEADER)
    if header.size == 0:
        raise ValueError(f"{raw_path}: no header record (not a logger file?)")
    hd = header[0]
    imu_seq, imu = logfmt.parse(split, logfmt.IMU)
    _, pvt = logfmt.parse(split, logfmt.GNSS_PVT)
    _, pps = logfmt.parse(split, logfmt.PPS)
    _, filt = logfmt.parse(split, logfmt.FILTER)
    _, ev = logfmt.parse(split, logfmt.EVENT)
    if imu.size == 0 or pvt.size == 0:
        raise ValueError(f"{raw_path}: no IMU or no GNSS records")

    # Local clock to GPS time with the firmware's own time base.
    order = np.argsort(pvt["t_local_us"], kind="stable")
    pvt = pvt[order]
    queries = np.concatenate([imu["t_local_us"], pvt["t_local_us"], ev["t_local_us"]]).astype(np.int64)
    gps, drift_ppm, latency_ms, pps_count = _core.map_local_to_gps(
        np.sort(pps["t_local_us"]).astype(np.int64), pvt["t_local_us"].astype(np.int64),
        pvt["itow_ms"].astype(np.uint32), queries, NOMINAL_LATENCY_US)
    n_imu, n_pvt = imu.size, pvt.size
    imu_t = gps[:n_imu]
    arrival_t = gps[n_imu:n_imu + n_pvt]
    ev_t = gps[n_imu + n_pvt:]

    # Epoch times from time of week, with week rollover.
    itow = pvt["itow_ms"].astype(np.int64)
    week = np.concatenate([[0], np.cumsum(np.diff(itow) < -302_400_000)])
    epoch_t = (itow + week * 604_800_000) * 1000
    fix_t = arrival_t if stamp_on_arrival else epoch_t

    r_bs = _rot(*np.deg2rad([hd["mount_roll"], hd["mount_pitch"], hd["mount_yaw"]]))
    acc = np.stack([imu["ax"], imu["ay"], imu["az"]], axis=1) * float(hd["accel_scale"]) @ r_bs.T
    gyr = np.stack([imu["gx"], imu["gy"], imu["gz"]], axis=1) * float(hd["gyro_scale"]) @ r_bs.T
    imu_df = pd.DataFrame({
        "t_gps_us": imu_t, "ax": acc[:, 0], "ay": acc[:, 1], "az": acc[:, 2],
        "gx": gyr[:, 0], "gy": gyr[:, 1], "gz": gyr[:, 2],
        "temp_c": (imu["temp"] * float(hd["temp_scale"]) + float(hd["temp_offset"])).astype(np.float32),
        "seq": imu_seq,
    }).sort_values("t_gps_us", kind="stable")
    imu_df = imu_df[imu_df["t_gps_us"].diff().fillna(1) > 0]  # drop duplicate stamps if any

    gnss_df = pd.DataFrame({
        "t_gps_us": fix_t, "t_arrival_gps_us": arrival_t,
        "lat": pvt["lat_e7"] * 1e-7, "lon": pvt["lon_e7"] * 1e-7, "h": pvt["h_mm"] * 1e-3,
        "vn": pvt["vn"] * 1e-3, "ve": pvt["ve"] * 1e-3, "vd": pvt["vd"] * 1e-3,
        "hacc": pvt["hacc"] * 1e-3, "vacc": pvt["vacc"] * 1e-3, "sacc": pvt["sacc"] * 1e-3,
        "fix_type": pvt["fix_type"], "carr_soln": pvt["carr_soln"], "num_sv": pvt["num_sv"],
    })

    if reference_csv is not None:
        ref = pd.read_csv(reference_csv)
        if "source" not in ref:
            ref["source"] = "csv"
        ref = ref[["t_gps_us", "lat", "lon", "h", "source"]]
    else:
        fixed = gnss_df[(gnss_df["carr_soln"] == 2) & (gnss_df["fix_type"] >= 3)]
        ref = fixed[["t_gps_us", "lat", "lon", "h"]].assign(source="rtk_fixed")

    ride_id = ride_id or raw_path.stem
    ds = Dataset(dataset_root)
    out = ds.rides_dir / ride_id
    out.mkdir(parents=True, exist_ok=True)
    imu_df.to_parquet(out / "imu.parquet", index=False)
    gnss_df.to_parquet(out / "gnss.parquet", index=False)
    ref.to_parquet(out / "reference.parquet", index=False)
    if filt.size:
        pd.DataFrame({k: filt[k] for k in filt.dtype.names if k != "pad"}).to_parquet(
            out / "filter_onboard.parquet", index=False)
    pd.DataFrame({
        "t_gps_us": ev_t, "code": ev["code"], "value": ev["value"],
        "name": [logfmt.EVENT_NAMES.get(int(c), "unknown") for c in ev["code"]],
    }).to_parquet(out / "events.parquet", index=False)
    if raw_path.resolve() != (out / "raw.bin").resolve():
        shutil.copyfile(raw_path, out / "raw.bin")

    valid = gnss_df["fix_type"].to_numpy() >= 3
    g = gnss_df[valid]
    # Distance from GNSS speed: summing noisy position steps would overcount.
    distance = 0.0
    if len(g) > 1:
        dt = np.diff(g["t_gps_us"].to_numpy()) * 1e-6
        sp = np.hypot(g["vn"].to_numpy()[1:], g["ve"].to_numpy()[1:])
        distance = float((sp * np.clip(dt, 0, 1.0)).sum())
    expected, present = _seq_stats(imu_seq)
    entry = {
        "ride_id": ride_id,
        "firmware": hd["version"].split(b"\0")[0].decode(errors="replace"),
        "duration_s": round(float((imu_t.max() - imu_t.min()) * 1e-6), 1),
        "distance_km": round(distance / 1000, 2),
        "imu_samples": present,
        "imu_dropped": expected - present,
        "imu_dropped_pct": round(100.0 * (expected - present) / max(expected, 1), 3),
        "crc_errors": int(split["crc_errors"]),
        "gnss_epochs": int(n_pvt),
        "rtk_fixed_pct": round(100.0 * float((gnss_df["carr_soln"] == 2).mean()), 1),
        "reference": str(ref["source"].iloc[0]) if len(ref) else "none",
        "pps_edges": int(pps_count),
        "clock_drift_ppm": round(float(drift_ppm), 2),
        "fix_latency_ms": round(float(latency_ms), 1),
        "natural_outages": _outages(gnss_df["t_gps_us"].to_numpy(), valid),
        "sd_stall_max_ms": int(ev["value"][ev["code"] == 1].max()) if (ev["code"] == 1).any() else 0,
        "stamp_on_arrival": stamp_on_arrival,
        "tags": tags or [],
    }
    ds.save_entry(entry)
    return entry
