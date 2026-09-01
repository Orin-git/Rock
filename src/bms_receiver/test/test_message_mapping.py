import math

from sensor_msgs.msg import BatteryState

from bms_receiver.node import build_messages
from bms_receiver.protocol import BatterySample, ChargeState


def make_sample(**overrides):
    values = {
        "voltage": 25.0,
        "current": 1.75,
        "soc_percent": 50.0,
        "soh_percent": 98.0,
        "state": ChargeState.CHARGING,
        "mos_temperature": 31.5,
        "env_temperature": 26.0,
        "warning_bits": 0,
        "protection_bits": 0,
        "comm_status": 1,
    }
    values.update(overrides)
    return BatterySample(**values)


def test_maps_charging_sample_to_standard_and_compatibility_messages():
    messages = build_messages(make_sample())

    assert messages.battery_state.voltage == 25.0
    assert messages.battery_state.current == 1.75
    assert messages.battery_state.percentage == 0.5
    assert messages.battery_state.temperature == 26.0
    assert math.isnan(messages.battery_state.charge)
    assert math.isnan(messages.battery_state.capacity)
    assert math.isnan(messages.battery_state.design_capacity)
    assert (
        messages.battery_state.power_supply_status
        == BatteryState.POWER_SUPPLY_STATUS_CHARGING
    )
    assert messages.battery_state.present is True
    assert messages.power_voltage.data == 50.0
    assert messages.charger_status.voltage == 25.0
    assert messages.charger_status.percentage == 50.0
    assert messages.charger_status.charging is True
    assert messages.charging_flag.data is True
    assert messages.charging_current.data == 1.75


def test_maps_discharging_and_fault_status():
    messages = build_messages(
        make_sample(
            current=-2.0,
            state=ChargeState.DISCHARGING,
            warning_bits=1,
            comm_status=0,
        )
    )

    assert (
        messages.battery_state.power_supply_status
        == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    )
    assert (
        messages.battery_state.power_supply_health
        == BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
    )
    assert messages.battery_state.present is False
    assert messages.charging_flag.data is False
    assert messages.charging_current.data == 0.0
    assert "Communication fault (0)" in messages.charger_status.status
