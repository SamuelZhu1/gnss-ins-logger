"""Replay regression on a synthetic ride. These thresholds guard the filter
against regressions; they are not ride results."""
import numpy as np
import pandas as pd

from gins import _core
from gins.replay import replay


def _errors(out: pd.DataFrame, ref: pd.DataFrame) -> np.ndarray:
    o = (float(ref.lat.iloc[0]), float(ref.lon.iloc[0]), float(ref.h.iloc[0]))
    est = _core.lla_to_ned(*o, out.lat.to_numpy(), out.lon.to_numpy(), out.h.to_numpy())
    rr = _core.lla_to_ned(*o, ref.lat.to_numpy(), ref.lon.to_numpy(), ref.h.to_numpy())
    rn = np.interp(out.t_gps_us, ref.t_gps_us, rr[:, 0])
    re = np.interp(out.t_gps_us, ref.t_gps_us, rr[:, 1])
    return np.hypot(est[:, 0] - rn, est[:, 1] - re)


def test_fused_track_beats_raw_gnss(dataset):
    ride = dataset.ride("synth_s7")
    out, diag = replay(ride)
    nav = out["mode"].to_numpy() >= 3
    assert nav.mean() > 0.8
    assert diag["coarse_alignments"] == 0
    err = _errors(out[nav], ride.reference)
    assert np.sqrt(np.mean(err**2)) < 1.5


def test_ten_second_outage_regression(dataset):
    ride = dataset.ride("synth_s7")
    t0 = int(ride.imu.t_gps_us.iloc[0])
    start = t0 + 150_000_000
    end = start + 10_000_000
    out, diag = replay(ride, [(start, end)])
    assert diag["fixes_masked"] >= 99
    inside = out[(out.t_gps_us >= start) & (out.t_gps_us < end)]
    assert inside["masked"].all()
    assert _errors(inside.tail(1), ride.reference)[0] < 5.0


def test_replay_is_deterministic(dataset):
    ride = dataset.ride("synth_s7")
    a, _ = replay(ride)
    b, _ = replay(ride)
    pd.testing.assert_frame_equal(a, b)
