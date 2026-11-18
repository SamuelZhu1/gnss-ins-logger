"""Ride dataset on disk.

dataset/
  manifest.json                 one entry per ride
  rides/<ride_id>/
    imu.parquet                 t_gps_us, ax ay az (m/s^2), gx gy gz (rad/s) in the bike frame, temp_c, seq
    gnss.parquet                t_gps_us (epoch), t_arrival_gps_us, lat lon h, vn ve vd, hacc vacc sacc,
                                fix_type carr_soln num_sv
    reference.parquet           t_gps_us, lat lon h, source (rtk_fixed | truth | csv)
    filter_onboard.parquet      what the board's filter output live (if present)
    events.parquet              t_gps_us, code, name, value
    raw.bin                     the original log, kept
"""
from __future__ import annotations

import json
from pathlib import Path

import pandas as pd


class Dataset:
    def __init__(self, root: str | Path):
        self.root = Path(root)
        self.rides_dir = self.root / "rides"
        self.manifest_path = self.root / "manifest.json"

    def manifest(self) -> list[dict]:
        if not self.manifest_path.exists():
            return []
        return json.loads(self.manifest_path.read_text())["rides"]

    def save_entry(self, entry: dict) -> None:
        rides = [r for r in self.manifest() if r["ride_id"] != entry["ride_id"]]
        rides.append(entry)
        rides.sort(key=lambda r: r["ride_id"])
        self.root.mkdir(parents=True, exist_ok=True)
        self.manifest_path.write_text(json.dumps({"version": 1, "rides": rides}, indent=2) + "\n")

    def ride_ids(self) -> list[str]:
        return [r["ride_id"] for r in self.manifest()]

    def ride(self, ride_id: str) -> "RideData":
        return RideData(self.rides_dir / ride_id)

    def totals(self) -> dict:
        rides = self.manifest()
        return {
            "rides": len(rides),
            "hours": round(sum(r["duration_s"] for r in rides) / 3600, 2),
            "distance_km": round(sum(r["distance_km"] for r in rides), 1),
            "imu_samples": int(sum(r["imu_samples"] for r in rides)),
        }


class RideData:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        if not (self.path / "imu.parquet").exists():
            raise FileNotFoundError(f"no ride at {self.path}")
        self.ride_id = self.path.name

    def table(self, name: str) -> pd.DataFrame:
        p = self.path / f"{name}.parquet"
        return pd.read_parquet(p) if p.exists() else pd.DataFrame()

    @property
    def imu(self) -> pd.DataFrame:
        return self.table("imu")

    @property
    def gnss(self) -> pd.DataFrame:
        return self.table("gnss")

    @property
    def reference(self) -> pd.DataFrame:
        return self.table("reference")
