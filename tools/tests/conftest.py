import pytest

from gins import convert, synth
from gins.dataset import Dataset


@pytest.fixture(scope="session")
def synth_ride():
    return synth.make_ride(synth.SynthConfig(minutes=4, seed=7))


@pytest.fixture(scope="session")
def dataset(tmp_path_factory, synth_ride):
    root = tmp_path_factory.mktemp("ds")
    raw = root / "s7.bin"
    ref = root / "s7_ref.csv"
    synth.write(synth_ride, raw, ref)
    convert.convert(raw, root / "dataset", "synth_s7", ref, ["synthetic"])
    return Dataset(root / "dataset")
