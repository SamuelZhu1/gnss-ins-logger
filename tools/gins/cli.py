"""Command line: gins synth | convert | replay | bench | export | stats"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import bench, convert, export, synth
from .dataset import Dataset, RideData
from .replay import replay


def _cmd_synth(a):
    cfg = synth.SynthConfig(minutes=a.minutes, seed=a.seed, drop_imu_pct=a.drop_imu_pct)
    ride = synth.make_ride(cfg)
    synth.write(ride, Path(a.out), Path(a.reference) if a.reference else None)
    print(json.dumps({"out": a.out, **{k: v for k, v in ride.info.items() if k != "tunnel"}}))


def _cmd_convert(a):
    entry = convert.convert(a.raw, a.dataset, a.ride_id, a.reference, a.tags, a.stamp_on_arrival)
    print(json.dumps(entry, indent=2))


def _cmd_replay(a):
    ride = RideData(a.ride)
    t0 = int(ride.imu["t_gps_us"].iloc[0])
    masks = export.parse_masks(a.mask or [], t0)
    cfg = {"use_nhc": not a.no_nhc}
    out, diag = replay(ride, masks, cfg)
    if a.out:
        out.to_parquet(a.out, index=False)
    print(json.dumps({"outputs": len(out), **{k: int(v) for k, v in diag.items()}}))


def _cmd_bench(a):
    ds = Dataset(a.dataset)
    res = bench.run(ds, tuple(a.lengths), a.per_ride, a.seed)
    if res.empty:
        print("no usable windows")
        return
    summary = bench.summarize(res)
    print(summary.to_string(index=False))
    if a.csv:
        res.to_csv(a.csv, index=False)
    if a.markdown:
        Path(a.markdown).write_text(bench.markdown(summary, a.note or ""))


def _cmd_export(a):
    ride = RideData(a.ride)
    t0 = int(ride.imu["t_gps_us"].iloc[0])
    path = export.export(ride, a.out, export.parse_masks(a.mask or [], t0))
    print(path)


def _cmd_stats(a):
    ds = Dataset(a.dataset)
    print(json.dumps(ds.totals(), indent=2))


def main(argv=None):
    p = argparse.ArgumentParser(prog="gins", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("synth", help="write a synthetic ride log (for testing the tools)")
    s.add_argument("--out", required=True)
    s.add_argument("--reference", help="also write the true track as CSV")
    s.add_argument("--minutes", type=float, default=5)
    s.add_argument("--seed", type=int, default=1)
    s.add_argument("--drop-imu-pct", type=float, default=0.0)
    s.set_defaults(fn=_cmd_synth)

    s = sub.add_parser("convert", help="add a raw log to the dataset")
    s.add_argument("raw")
    s.add_argument("--dataset", default="dataset")
    s.add_argument("--ride-id")
    s.add_argument("--reference", help="CSV with t_gps_us,lat,lon,h (default: the receiver's RTK fixed epochs)")
    s.add_argument("--tags", nargs="*")
    s.add_argument("--stamp-on-arrival", action="store_true", help="experiment: time fixes by UART arrival")
    s.set_defaults(fn=_cmd_convert)

    s = sub.add_parser("replay", help="replay one ride through the filter")
    s.add_argument("ride", help="path to dataset/rides/<id>")
    s.add_argument("--mask", action="append", help="START_S:LEN_S from the ride start, repeatable")
    s.add_argument("--no-nhc", action="store_true")
    s.add_argument("--out")
    s.set_defaults(fn=_cmd_replay)

    s = sub.add_parser("bench", help="GNSS outage benchmark over the dataset")
    s.add_argument("--dataset", default="dataset")
    s.add_argument("--lengths", type=float, nargs="+", default=[10, 20, 30])
    s.add_argument("--per-ride", type=int, default=20)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--csv")
    s.add_argument("--markdown")
    s.add_argument("--note")
    s.set_defaults(fn=_cmd_bench)

    s = sub.add_parser("export", help="GeoJSON for the map viewer")
    s.add_argument("ride")
    s.add_argument("--out", required=True)
    s.add_argument("--mask", action="append")
    s.set_defaults(fn=_cmd_export)

    s = sub.add_parser("stats", help="dataset totals")
    s.add_argument("--dataset", default="dataset")
    s.set_defaults(fn=_cmd_stats)

    a = p.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
