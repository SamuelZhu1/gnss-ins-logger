"""GeoJSON export for the map viewer (viewer/index.html)."""
from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

from .dataset import RideData
from .replay import replay


def _line(df: pd.DataFrame, name: str, every: int = 1) -> dict:
    d = df.iloc[::every]
    coords = [[round(float(lo), 7), round(float(la), 7)] for la, lo in zip(d["lat"], d["lon"])]
    return {"type": "Feature", "properties": {"track": name, "points": len(coords)},
            "geometry": {"type": "LineString", "coordinates": coords}}


def export(ride: RideData, out_path: str | Path, masks: list[tuple[int, int]] | None = None,
           config: dict | None = None) -> Path:
    gnss = ride.gnss
    gnss = gnss[gnss["fix_type"] >= 3]
    ref = ride.reference
    fused, _ = replay(ride, masks or [], config)
    fused = fused[fused["mode"] >= 3]
    feats = [
        _line(gnss, "gnss", every=2),
        _line(fused, "fused", every=2),
    ]
    if len(ref):
        feats.append(_line(ref, "reference", every=2))
    for start, end in masks or []:
        seg = fused[(fused["t_gps_us"] >= start) & (fused["t_gps_us"] < end)]
        if len(seg) > 1:
            f = _line(seg, "outage")
            f["properties"]["seconds"] = round((end - start) * 1e-6, 1)
            feats.append(f)
    fc = {"type": "FeatureCollection", "properties": {"ride_id": ride.ride_id}, "features": feats}
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(fc))
    return out_path


def parse_masks(specs: list[str], t0_us: int) -> list[tuple[int, int]]:
    """'START_S:LEN_S' relative to the first IMU sample."""
    out = []
    for s in specs:
        a, b = s.split(":")
        start = t0_us + int(float(a) * 1e6)
        out.append((start, start + int(float(b) * 1e6)))
    return out

