"""On-board log format (mirrors core/include/gins/log_format.hpp).

Record: sync 0xA5 | type u8 | len u16 | seq u32 | payload | crc16 (CCITT-FALSE
over type..payload), little-endian. Fixed-length payloads are parsed into
numpy structured arrays; splitting a log into records is done in C++.
"""
from __future__ import annotations

import struct

import numpy as np

SYNC = 0xA5
IMU, GNSS_PVT, PPS, FILTER, RAWX, EVENT, HEADER = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10

EVENT_NAMES = {
    1: "sd_stall_ms",
    2: "ring_high_water",
    3: "dropped_imu",
    4: "dropped_rawx",
    5: "filter_overrun",
    6: "missed_imu_irq",
    7: "mode_change",
    8: "gnss_mask_start",
    9: "log_start",
}

# Payload layouts as numpy dtypes (little-endian, packed).
DTYPES = {
    IMU: np.dtype([("t_local_us", "<i8"), ("ax", "<i2"), ("ay", "<i2"), ("az", "<i2"),
                   ("gx", "<i2"), ("gy", "<i2"), ("gz", "<i2"), ("temp", "<i2")]),
    GNSS_PVT: np.dtype([("t_local_us", "<i8"), ("itow_ms", "<u4"), ("lat_e7", "<i4"), ("lon_e7", "<i4"),
                        ("h_mm", "<i4"), ("vn", "<i4"), ("ve", "<i4"), ("vd", "<i4"), ("hacc", "<u4"),
                        ("vacc", "<u4"), ("sacc", "<u4"), ("fix_type", "u1"), ("carr_soln", "u1"),
                        ("num_sv", "u1"), ("reserved", "u1")]),
    PPS: np.dtype([("t_local_us", "<i8")]),
    FILTER: np.dtype([("t_gps_us", "<i8"), ("lat", "<f8"), ("lon", "<f8"), ("h", "<f4"), ("vn", "<f4"),
                      ("ve", "<f4"), ("vd", "<f4"), ("roll", "<f4"), ("pitch", "<f4"), ("yaw", "<f4"),
                      ("sig_n", "<f4"), ("sig_e", "<f4"), ("sig_d", "<f4"), ("mode", "u1"), ("pad", "V3")]),
    EVENT: np.dtype([("t_local_us", "<i8"), ("code", "<u2"), ("reserved", "<u2"), ("value", "<u4")]),
    HEADER: np.dtype([("version", "S16"), ("accel_scale", "<f4"), ("gyro_scale", "<f4"), ("temp_scale", "<f4"),
                      ("temp_offset", "<f4"), ("mount_roll", "<f4"), ("mount_pitch", "<f4"),
                      ("mount_yaw", "<f4"), ("imu_rate_hz", "<u4"), ("config_hash", "<u4")]),
}


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """Pure-Python CRC-16/CCITT-FALSE (reference for tests; C++ does the real work)."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(rtype: int, seq: int, payload: bytes) -> bytes:
    """Pure-Python encoder, kept byte-identical to the C++ one (checked in tests)."""
    head = struct.pack("<BHI", rtype, len(payload), seq)
    crc = crc16(head + payload)
    return bytes([SYNC]) + head + payload + struct.pack("<H", crc)


_CRC_TABLE = np.array([crc16(bytes([i]), 0) for i in range(256)], dtype=np.uint16)


def encode_many(rtype: int, seq: np.ndarray, payloads: np.ndarray) -> np.ndarray:
    """Encode N records of one fixed-length type at once.

    payloads is an (N, L) uint8 array. The CRC runs byte position by byte
    position across all records, so a few minutes of 200 Hz data encodes in
    milliseconds instead of seconds.
    """
    n, length = payloads.shape
    head = np.zeros((n, 7), dtype=np.uint8)
    head[:, 0] = rtype
    head[:, 1] = length & 0xFF
    head[:, 2] = length >> 8
    s = seq.astype(np.uint32)
    for i in range(4):
        head[:, 3 + i] = (s >> (8 * i)) & 0xFF
    body = np.concatenate([head, payloads], axis=1)
    crc = np.full(n, 0xFFFF, dtype=np.uint16)
    for j in range(body.shape[1]):
        idx = ((crc >> 8) ^ body[:, j]).astype(np.uint8)
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[idx]
    out = np.empty((n, 1 + body.shape[1] + 2), dtype=np.uint8)
    out[:, 0] = SYNC
    out[:, 1:-2] = body
    out[:, -2] = crc & 0xFF
    out[:, -1] = crc >> 8
    return out


def parse(split: dict, rtype: int) -> tuple[np.ndarray, np.ndarray]:
    """Turn one type's records from _core.split_records into (seq, structured array)."""
    if rtype not in split:
        return np.zeros(0, np.uint32), np.zeros(0, DTYPES[rtype])
    rec = split[rtype]
    dt = DTYPES[rtype]
    good = rec["len"] == dt.itemsize
    if good.all():
        arr = np.frombuffer(rec["payload"], dtype=dt)
    else:
        # A different length means a different firmware version wrote it; keep what parses.
        offsets = np.concatenate([[0], np.cumsum(rec["len"].astype(np.int64))])
        raw = rec["payload"]
        arr = np.array([np.frombuffer(raw[offsets[i]:offsets[i] + dt.itemsize], dtype=dt)[0]
                        for i in np.flatnonzero(good)], dtype=dt)
    return rec["seq"][good], arr
