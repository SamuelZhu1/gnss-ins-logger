import numpy as np

from gins import convert, synth
from gins.dataset import Dataset


def test_convert_counts_and_manifest(dataset, synth_ride):
    entry = dataset.manifest()[0]
    assert entry["ride_id"] == "synth_s7"
    assert entry["imu_samples"] == synth_ride.info["imu_written"]
    assert entry["imu_dropped"] == 0
    assert entry["crc_errors"] == 0
    assert entry["reference"] == "truth"
    assert abs(entry["clock_drift_ppm"] - 18.0) < 1.0
    assert abs(entry["fix_latency_ms"] - 42.0) < 2.0
    assert len(entry["natural_outages"]) == 1  # the tunnel
    assert dataset.totals()["rides"] == 1


def test_imu_times_map_to_gps_time_within_microseconds(dataset, synth_ride):
    imu = dataset.ride("synth_s7").imu
    truth = np.sort(synth_ride.imu_truth_t)
    # After the first PPS pair the time base is locked; allow for the start.
    err = np.abs(imu["t_gps_us"].to_numpy() - truth)[1000:]
    assert err.max() <= 10


def test_imu_in_bike_frame_reads_gravity_when_still(dataset):
    imu = dataset.ride("synth_s7").imu
    still = imu.iloc[200:2000]
    assert abs(still["az"].mean() + 9.81) < 0.2
    assert abs(still["ax"].mean()) < 0.3


def test_dropped_records_show_up_as_sequence_gaps(tmp_path):
    ride = synth.make_ride(synth.SynthConfig(minutes=1, seed=3, drop_imu_pct=2.0))
    raw = tmp_path / "d.bin"
    synth.write(ride, raw)
    entry = convert.convert(raw, tmp_path / "ds", "drops")
    assert entry["imu_dropped"] == ride.info["imu_dropped"]
    assert 1.5 < entry["imu_dropped_pct"] < 2.5
    assert entry["reference"] == "none"  # no RTK fixed epochs and no CSV


def test_stamp_on_arrival_shifts_fixes_by_the_latency(tmp_path, synth_ride):
    raw = tmp_path / "a.bin"
    synth.write(synth_ride, raw)
    convert.convert(raw, tmp_path / "ds", "epoch")
    convert.convert(raw, tmp_path / "ds", "arrival", stamp_on_arrival=True)
    ds = Dataset(tmp_path / "ds")
    shift = (ds.ride("arrival").gnss["t_gps_us"] - ds.ride("epoch").gnss["t_gps_us"]).to_numpy() * 1e-3
    assert 35 < np.median(shift) < 50
