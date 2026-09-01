#!/usr/bin/env python3
"""Serial MCU motion + 0x7C autocharge + 0xFB BMS frame tests (no ROS)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xw_chassis.serial_mcu import (  # noqa: E402
    AUTOCHARGE_SIZE,
    BMS_SIZE,
    FrameParser,
    pack_speed,
    parse_bms_raw_frame,
    parse_charge_frame,
    xor_bcc,
)


def _charge_bytes(current_a: float = 0.42, red: int = 2, charging: int = 1, state: int = 1) -> bytes:
    raw = int(round(current_a * 1000.0))
    buf = bytearray(AUTOCHARGE_SIZE)
    buf[0] = 0x7C
    buf[1] = (raw >> 8) & 0xFF
    buf[2] = raw & 0xFF
    buf[3] = red & 0xFF
    buf[4] = charging & 0xFF
    buf[5] = state & 0xFF
    buf[6] = xor_bcc(buf[0:6])
    buf[7] = 0x7F
    return bytes(buf)


def _bms_bytes(
    *,
    byte_order: str = 'big',
    voltage_mv: int = 25000,
    current_ma: int = -1200,
    soc_permille: int = 500,
) -> bytes:
    frame = bytearray((0xFB, 0x01, 0x19))
    frame.extend(voltage_mv.to_bytes(4, byte_order, signed=False))
    frame.extend(current_ma.to_bytes(4, byte_order, signed=True))
    frame.extend(soc_permille.to_bytes(2, byte_order, signed=False))
    frame.append(98)  # SOH
    frame.append(2)  # discharging
    frame.extend((315).to_bytes(2, byte_order, signed=True))
    frame.extend((260).to_bytes(2, byte_order, signed=True))
    frame.extend((0).to_bytes(2, byte_order, signed=False))
    frame.extend((0).to_bytes(2, byte_order, signed=False))
    frame.extend((0).to_bytes(2, byte_order, signed=False))
    frame.extend((0).to_bytes(2, byte_order, signed=False))
    frame.append(1)  # comm ok
    frame.append(xor_bcc(frame))
    frame.append(0xFD)
    assert len(frame) == BMS_SIZE
    return bytes(frame)


def test_parse_charge_frame():
    f = parse_charge_frame(_charge_bytes(0.42, 2, 1, 1))
    assert f is not None
    assert abs(f.current - 0.42) < 1e-3
    assert f.red == 2
    assert f.charging is True
    assert f.charge_set_state == 1


def test_parse_bms_raw_frame():
    raw = _bms_bytes()
    assert parse_bms_raw_frame(raw) == raw
    bad = bytearray(raw)
    bad[28] ^= 0x01
    assert parse_bms_raw_frame(bytes(bad)) is None


def test_parser_interleaved():
    parser = FrameParser()
    speed = pack_speed(0.1, 0.0, 0.0, mode=1)
    assert speed[1] == 1
    rx = bytearray(24)
    rx[0] = 0x7B
    rx[1] = 1
    rx[23] = 0x7D
    rx[22] = xor_bcc(rx[0:22])
    bms = _bms_bytes()
    blob = bytes(rx) + _charge_bytes() + bms + bytes(rx)
    motion, charge, bms_frames = parser.feed(blob)
    assert len(motion) == 2
    assert len(charge) == 1
    assert len(bms_frames) == 1
    assert charge[0].charging is True
    assert bms_frames[0] == bms


if __name__ == '__main__':
    test_parse_charge_frame()
    test_parse_bms_raw_frame()
    test_parser_interleaved()
    print('serial_mcu tests passed')
