#!/usr/bin/env python3
"""Serial MCU motion + 0x7C autocharge frame tests (no ROS)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xw_chassis.serial_mcu import (  # noqa: E402
    AUTOCHARGE_SIZE,
    FrameParser,
    pack_speed,
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


def test_parse_charge_frame():
    f = parse_charge_frame(_charge_bytes(0.42, 2, 1, 1))
    assert f is not None
    assert abs(f.current - 0.42) < 1e-3
    assert f.red == 2
    assert f.charging is True
    assert f.charge_set_state == 1


def test_parser_interleaved():
    parser = FrameParser()
    speed = pack_speed(0.1, 0.0, 0.0, mode=1)
    assert speed[1] == 1
    # fake a valid 24-byte motion: reuse header/tail/bcc pattern from pack is TX not RX
    rx = bytearray(24)
    rx[0] = 0x7B
    rx[1] = 1
    rx[23] = 0x7D
    rx[22] = xor_bcc(rx[0:22])
    blob = bytes(rx) + _charge_bytes() + bytes(rx)
    motion, charge = parser.feed(blob)
    assert len(motion) == 2
    assert len(charge) == 1
    assert charge[0].charging is True


if __name__ == '__main__':
    test_parse_charge_frame()
    test_parser_interleaved()
    print('serial_mcu tests passed')
