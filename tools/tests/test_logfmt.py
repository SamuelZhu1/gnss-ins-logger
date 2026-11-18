import struct

import numpy as np

from gins import _core, logfmt


def test_crc_check_value_matches_ccitt_false():
    assert logfmt.crc16(b"123456789") == 0x29B1
    assert _core.crc16(b"123456789") == 0x29B1


def test_python_and_cpp_encoders_are_byte_identical():
    rng = np.random.default_rng(0)
    for rtype, size in [(logfmt.IMU, 22), (logfmt.GNSS_PVT, 52), (logfmt.PPS, 8), (logfmt.EVENT, 16)]:
        for seq in [0, 1, 70000, 2**32 - 1]:
            payload = rng.integers(0, 256, size, dtype=np.uint8).tobytes()
            assert logfmt.encode(rtype, seq, payload) == _core.encode_record(rtype, seq, payload)


def test_bulk_encoder_matches_single_records():
    rng = np.random.default_rng(1)
    payloads = rng.integers(0, 256, (50, 22), dtype=np.uint8)
    seq = np.arange(50) * 3
    bulk = logfmt.encode_many(logfmt.IMU, seq, payloads)
    for i in range(50):
        assert bulk[i].tobytes() == logfmt.encode(logfmt.IMU, int(seq[i]), payloads[i].tobytes())


def test_split_skips_a_corrupt_record_and_keeps_the_rest():
    recs = [logfmt.encode(logfmt.PPS, i, struct.pack("<q", 1000 * i)) for i in range(5)]
    data = bytearray(b"\x00\xa5\x07" + b"".join(recs))
    data[3 + len(recs[0]) + 9] ^= 0xFF  # inside record 1's payload
    split = _core.split_records(bytes(data))
    seq, arr = logfmt.parse(split, logfmt.PPS)
    assert list(seq) == [0, 2, 3, 4]
    assert list(arr["t_local_us"]) == [0, 2000, 3000, 4000]
    assert split["crc_errors"] >= 1


def test_rejects_payload_too_long_for_type():
    try:
        _core.encode_record(logfmt.IMU, 0, bytes(30))
    except ValueError:
        return
    raise AssertionError("expected ValueError")
