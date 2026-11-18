"""Replay a ride through the C++ navigator (the same code the board runs)."""
from __future__ import annotations

from typing import Iterable

import numpy as np
import pandas as pd

from . import _core
from .dataset import RideData


def replay(ride: RideData, masks: Iterable[tuple[int, int]] = (), config: dict | None = None,
           imu: pd.DataFrame | None = None, gnss: pd.DataFrame | None = None) -> tuple[pd.DataFrame, dict]:
    """Run one ride with GNSS masked over [start, end) windows (GPS us).

    Returns the navigator output (one row per output period) and its diagnostics.
    Pass imu/gnss to reuse tables already loaded (the benchmark replays a ride many times).
    """
    imu = ride.imu if imu is None else imu
    gnss = ride.gnss if gnss is None else gnss
    masks = list(masks)
    if len(masks) > _core.MAX_MASKS:
        raise ValueError(f"at most {_core.MAX_MASKS} masks per replay")
    ms = np.array([m[0] for m in masks], dtype=np.int64)
    me = np.array([m[1] for m in masks], dtype=np.int64)
    out = _core.replay(
        imu["t_gps_us"].to_numpy(np.int64),
        imu[["ax", "ay", "az"]].to_numpy(np.float64),
        imu[["gx", "gy", "gz"]].to_numpy(np.float64),
        gnss["t_gps_us"].to_numpy(np.int64),
        gnss["lat"].to_numpy(np.float64), gnss["lon"].to_numpy(np.float64), gnss["h"].to_numpy(np.float64),
        gnss[["vn", "ve", "vd"]].to_numpy(np.float64),
        gnss["hacc"].to_numpy(np.float64), gnss["vacc"].to_numpy(np.float64), gnss["sacc"].to_numpy(np.float64),
        gnss["fix_type"].to_numpy(np.uint8), gnss["carr_soln"].to_numpy(np.uint8),
        ms, me, config or {})
    diag = out.pop("diag")
    return pd.DataFrame(out), diag
