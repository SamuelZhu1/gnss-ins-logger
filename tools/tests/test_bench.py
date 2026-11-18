import numpy as np

from gins import bench


def test_windows_are_seeded_spaced_and_moving(dataset):
    ride = dataset.ride("synth_s7")
    g, ref = ride.gnss, ride.reference
    w1 = bench.pick_windows(g, ref, 20, 5, np.random.default_rng(0), spacing_s=30)
    w2 = bench.pick_windows(g, ref, 20, 5, np.random.default_rng(0), spacing_s=30)
    assert w1 == w2 and len(w1) >= 2
    starts = [s for s, _ in w1]
    assert min(np.diff(starts)) >= 50_000_000  # 20 s window + 30 s spacing
    speed = np.hypot(g.vn, g.ve).to_numpy()
    for s, e in w1:
        sel = (g.t_gps_us >= s) & (g.t_gps_us <= e)
        assert speed[sel.to_numpy()].min() >= 3.0


def test_benchmark_summary_and_markdown(dataset):
    res = bench.run(dataset, lengths=(10,), per_ride=3, seed=1)
    assert set(res["config"]) == {"bike_constraint", "no_constraint"}
    s = bench.summarize(res)
    md = bench.markdown(s, "synthetic")
    assert md.startswith("| Outage |") and "synthetic" in md
    assert (s["windows"] > 0).all()
