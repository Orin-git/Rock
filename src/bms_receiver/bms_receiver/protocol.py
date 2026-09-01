"""Decode the fixed controller-to-host battery telemetry frame."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Sequence


FRAME_SIZE = 30


class ProtocolError(ValueError):
    """The telemetry frame is corrupt or contains an invalid value."""


class ChargeState(IntEnum):
    """Charge state values defined by the lower-controller frame."""

    STANDBY = 0
    CHARGING = 1
    DISCHARGING = 2


@dataclass(frozen=True)
class BatterySample:
    """One validated battery telemetry sample in engineering units."""

    voltage: float
    current: float
    soc_percent: float
    soh_percent: float
    state: ChargeState
    mos_temperature: float
    env_temperature: float
    warning_bits: int
    protection_bits: int
    comm_status: int


def xor_bcc(data: Sequence[int]) -> int:
    """Return the XOR checksum of all supplied bytes."""

    result = 0
    for value in data:
        result ^= int(value)
    return result


def _decode(frame: bytes, start: int, size: int, byte_order: str, signed=False):
    return int.from_bytes(
        frame[start:start + size], byteorder=byte_order, signed=signed
    )


def parse_battery_frame(frame, byte_order="little") -> BatterySample:
    """Validate and decode one 30-byte ``FB 01 19 ... BCC FD`` frame."""

    if byte_order not in ("little", "big"):
        raise ProtocolError(f"unsupported byte order: {byte_order}")

    try:
        raw = bytes(frame)
    except (TypeError, ValueError) as exc:
        raise ProtocolError("frame must contain byte values") from exc

    if len(raw) != FRAME_SIZE:
        raise ProtocolError(f"expected {FRAME_SIZE} bytes, got {len(raw)}")
    if raw[0] != 0xFB:
        raise ProtocolError("invalid frame header")
    if raw[1] != 0x01:
        raise ProtocolError("invalid battery frame type")
    if raw[2] != 0x19:
        raise ProtocolError("invalid payload length")
    if raw[29] != 0xFD:
        raise ProtocolError("invalid frame tail")
    if xor_bcc(raw[:28]) != raw[28]:
        raise ProtocolError("battery frame BCC mismatch")

    voltage = _decode(raw, 3, 4, byte_order) * 0.001
    current = _decode(raw, 7, 4, byte_order, signed=True) * 0.001
    soc_percent = _decode(raw, 11, 2, byte_order) * 0.1
    soh_percent = float(raw[13])
    try:
        state = ChargeState(raw[14])
    except ValueError as exc:
        raise ProtocolError(f"invalid charge state: {raw[14]}") from exc
    mos_temperature = _decode(raw, 15, 2, byte_order, signed=True) * 0.1
    env_temperature = _decode(raw, 17, 2, byte_order, signed=True) * 0.1
    warning_bits = (
        (_decode(raw, 19, 2, byte_order) << 16)
        | _decode(raw, 21, 2, byte_order)
    )
    protection_bits = (
        (_decode(raw, 23, 2, byte_order) << 16)
        | _decode(raw, 25, 2, byte_order)
    )

    if not 5.0 <= voltage <= 100.0:
        raise ProtocolError(f"implausible voltage: {voltage:.3f} V")
    if not -500.0 <= current <= 500.0:
        raise ProtocolError(f"implausible current: {current:.3f} A")
    if not 0.0 <= soc_percent <= 100.0:
        raise ProtocolError(f"implausible SOC: {soc_percent:.1f}%")
    if not 0.0 <= soh_percent <= 100.0:
        raise ProtocolError(f"implausible SOH: {soh_percent:.1f}%")
    if not -60.0 <= mos_temperature <= 150.0:
        raise ProtocolError(
            f"implausible MOS temperature: {mos_temperature:.1f} C"
        )
    if not -60.0 <= env_temperature <= 150.0:
        raise ProtocolError(
            f"implausible environment temperature: {env_temperature:.1f} C"
        )

    return BatterySample(
        voltage=voltage,
        current=current,
        soc_percent=soc_percent,
        soh_percent=soh_percent,
        state=state,
        mos_temperature=mos_temperature,
        env_temperature=env_temperature,
        warning_bits=warning_bits,
        protection_bits=protection_bits,
        comm_status=raw[27],
    )
