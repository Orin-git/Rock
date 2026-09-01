import pytest

from bms_receiver.protocol import (
    ChargeState,
    ProtocolError,
    parse_battery_frame,
    xor_bcc,
)


def make_battery_frame(
    *,
    byte_order="little",
    voltage_mv=25000,
    current_ma=-1200,
    soc_permille=500,
    soh_percent=98,
    charge_state=2,
    mos_temp_deci_c=315,
    env_temp_deci_c=260,
    warn_hi=0,
    warn_lo=1,
    protect_hi=0,
    protect_lo=0,
    comm_status=0,
):
    frame = bytearray((0xFB, 0x01, 0x19))
    frame.extend(voltage_mv.to_bytes(4, byte_order, signed=False))
    frame.extend(current_ma.to_bytes(4, byte_order, signed=True))
    frame.extend(soc_permille.to_bytes(2, byte_order, signed=False))
    frame.append(soh_percent)
    frame.append(charge_state)
    frame.extend(mos_temp_deci_c.to_bytes(2, byte_order, signed=True))
    frame.extend(env_temp_deci_c.to_bytes(2, byte_order, signed=True))
    frame.extend(warn_hi.to_bytes(2, byte_order, signed=False))
    frame.extend(warn_lo.to_bytes(2, byte_order, signed=False))
    frame.extend(protect_hi.to_bytes(2, byte_order, signed=False))
    frame.extend(protect_lo.to_bytes(2, byte_order, signed=False))
    frame.append(comm_status)
    frame.append(xor_bcc(frame))
    frame.append(0xFD)
    assert len(frame) == 30
    return bytes(frame)


def test_parse_little_endian_battery_frame():
    sample = parse_battery_frame(make_battery_frame(), byte_order="little")

    assert sample.voltage == 25.0
    assert sample.current == -1.2
    assert sample.soc_percent == 50.0
    assert sample.soh_percent == 98.0
    assert sample.state == ChargeState.DISCHARGING
    assert sample.mos_temperature == 31.5
    assert sample.env_temperature == 26.0
    assert sample.warning_bits == 1
    assert sample.protection_bits == 0
    assert sample.comm_status == 0


def test_parse_big_endian_and_signed_values():
    frame = make_battery_frame(
        byte_order="big",
        current_ma=1750,
        charge_state=1,
        mos_temp_deci_c=-55,
        env_temp_deci_c=-10,
    )
    sample = parse_battery_frame(frame, byte_order="big")

    assert sample.current == 1.75
    assert sample.state == ChargeState.CHARGING
    assert sample.mos_temperature == -5.5
    assert sample.env_temperature == -1.0


@pytest.mark.parametrize(
    ("offset", "value"),
    ((0, 0xFA), (1, 0x02), (2, 0x18), (29, 0xFC)),
)
def test_rejects_wrong_envelope(offset, value):
    frame = bytearray(make_battery_frame())
    frame[offset] = value
    if offset != 29:
        frame[28] = xor_bcc(frame[:28])
    with pytest.raises(ProtocolError):
        parse_battery_frame(frame, byte_order="little")


def test_rejects_bad_length_bcc_and_byte_order():
    frame = bytearray(make_battery_frame())
    frame[28] ^= 0x01

    with pytest.raises(ProtocolError):
        parse_battery_frame(frame[:-1], byte_order="little")
    with pytest.raises(ProtocolError):
        parse_battery_frame(frame, byte_order="little")
    with pytest.raises(ProtocolError):
        parse_battery_frame(make_battery_frame(), byte_order="middle")


@pytest.mark.parametrize(
    "overrides",
    (
        {"voltage_mv": 1000},
        {"soc_permille": 1001},
        {"soh_percent": 101},
        {"charge_state": 3},
        {"mos_temp_deci_c": 2000},
    ),
)
def test_rejects_implausible_payload(overrides):
    with pytest.raises(ProtocolError):
        parse_battery_frame(make_battery_frame(**overrides), byte_order="little")
