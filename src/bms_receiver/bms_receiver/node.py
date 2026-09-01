"""ROS 2 node publishing controller-forwarded battery telemetry."""

import math
import time
from dataclasses import dataclass
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, ByteMultiArray, Float32

from bms_receiver.protocol import (
    BatterySample,
    ChargeState,
    ProtocolError,
    parse_battery_frame,
)

try:
    from ox_battery_util.msg import ChargerStatus
except ImportError:  # Gen2 workspace may not vendor ox_battery_util
    ChargerStatus = None  # type: ignore[misc, assignment]


@dataclass(frozen=True)
class MappedMessages:
    """Messages emitted for one battery sample."""

    battery_state: BatteryState
    power_voltage: Float32
    charger_status: object
    charging_flag: Bool
    charging_current: Float32


def _make_charger_status(
    voltage: float,
    percentage: float,
    charging: bool,
    charging_current: float,
    status: str,
) -> object:
    if ChargerStatus is not None:
        msg = ChargerStatus()
        msg.voltage = voltage
        msg.percentage = percentage
        msg.charging = charging
        msg.charging_current = charging_current
        msg.status = status
        return msg
    return SimpleNamespace(
        voltage=voltage,
        percentage=percentage,
        charging=charging,
        charging_current=charging_current,
        status=status,
    )


def build_messages(
    sample: BatterySample, comm_ok_value: int = 1
) -> MappedMessages:
    """Map a validated protocol sample to standard and compatibility messages."""

    charging = (
        sample.state == ChargeState.CHARGING
        and sample.comm_status == comm_ok_value
        and sample.protection_bits == 0
    )

    battery_state = BatteryState()
    battery_state.voltage = sample.voltage
    battery_state.temperature = sample.env_temperature
    battery_state.current = sample.current
    battery_state.charge = math.nan
    battery_state.capacity = math.nan
    battery_state.design_capacity = math.nan
    battery_state.percentage = sample.soc_percent / 100.0
    if sample.state == ChargeState.CHARGING:
        battery_state.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )
    elif sample.state == ChargeState.DISCHARGING:
        battery_state.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
    else:
        battery_state.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        )
    has_fault = bool(
        sample.comm_status != comm_ok_value
        or sample.warning_bits
        or sample.protection_bits
    )
    battery_state.power_supply_health = (
        BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        if has_fault
        else BatteryState.POWER_SUPPLY_HEALTH_GOOD
    )
    battery_state.power_supply_technology = (
        BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
    )
    battery_state.present = sample.comm_status == comm_ok_value

    state_text = {
        ChargeState.STANDBY: "Standby",
        ChargeState.CHARGING: "Charging",
        ChargeState.DISCHARGING: "Discharging",
    }[sample.state]
    if sample.comm_status != comm_ok_value:
        status_text = f"Communication fault ({sample.comm_status})"
    elif sample.protection_bits:
        status_text = f"{state_text}; protection=0x{sample.protection_bits:08X}"
    elif sample.warning_bits:
        status_text = f"{state_text}; warning=0x{sample.warning_bits:08X}"
    else:
        status_text = state_text

    charger_status = _make_charger_status(
        voltage=sample.voltage,
        percentage=sample.soc_percent,
        charging=charging,
        charging_current=abs(sample.current) if charging else 0.0,
        status=status_text,
    )

    return MappedMessages(
        battery_state=battery_state,
        power_voltage=Float32(data=sample.soc_percent),
        charger_status=charger_status,
        charging_flag=Bool(data=charging),
        charging_current=Float32(
            data=abs(sample.current) if charging else 0.0
        ),
    )


class BmsReceiverNode(Node):
    """Decode raw controller BMS frames and publish battery topics."""

    def __init__(self):
        super().__init__("bms_receiver_node")

        self.declare_parameter("byte_order", "big")
        self.declare_parameter("stale_timeout_sec", 3.0)
        self.declare_parameter("min_voltage_v", 15.0)
        self.declare_parameter("max_voltage_v", 35.0)
        self.declare_parameter("max_abs_current_a", 200.0)
        self.declare_parameter("min_temperature_c", -40.0)
        self.declare_parameter("max_temperature_c", 100.0)
        self.declare_parameter("comm_ok_value", 1)
        self.declare_parameter("raw_frame_topic", "/bms/raw_frame")
        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("power_voltage_topic", "/PowerVoltage")
        self.declare_parameter("charger_status_topic", "/charger_status")
        self.declare_parameter("charging_flag_topic", "/robot_charging_flag")
        self.declare_parameter(
            "charging_current_topic", "/robot_charging_current"
        )

        self.byte_order = self.get_parameter("byte_order").value
        if self.byte_order not in ("little", "big"):
            raise ValueError("byte_order must be 'little' or 'big'")
        self.stale_timeout_sec = float(
            self.get_parameter("stale_timeout_sec").value
        )
        self.min_voltage_v = float(self.get_parameter("min_voltage_v").value)
        self.max_voltage_v = float(self.get_parameter("max_voltage_v").value)
        self.max_abs_current_a = float(
            self.get_parameter("max_abs_current_a").value
        )
        self.min_temperature_c = float(
            self.get_parameter("min_temperature_c").value
        )
        self.max_temperature_c = float(
            self.get_parameter("max_temperature_c").value
        )
        self.comm_ok_value = int(self.get_parameter("comm_ok_value").value)

        self.battery_state_publisher = self.create_publisher(
            BatteryState, self.get_parameter("battery_state_topic").value, 10
        )
        self.power_voltage_publisher = self.create_publisher(
            Float32, self.get_parameter("power_voltage_topic").value, 10
        )
        self.charger_status_publisher = None
        if ChargerStatus is not None:
            self.charger_status_publisher = self.create_publisher(
                ChargerStatus,
                self.get_parameter("charger_status_topic").value,
                10,
            )
        self.charging_flag_publisher = self.create_publisher(
            Bool, self.get_parameter("charging_flag_topic").value, 10
        )
        self.charging_current_publisher = self.create_publisher(
            Float32, self.get_parameter("charging_current_topic").value, 10
        )
        self.raw_subscription = self.create_subscription(
            ByteMultiArray,
            self.get_parameter("raw_frame_topic").value,
            self._on_raw_frame,
            10,
        )

        now = time.monotonic()
        self._started_at = now
        self._last_valid_at = None
        self._last_invalid_log_at = 0.0
        self._last_stale_log_at = 0.0
        self._watchdog = self.create_timer(1.0, self._check_stale)
        self.get_logger().info(
            f"BMS receiver ready: byte_order={self.byte_order}, "
            f"stale_timeout={self.stale_timeout_sec:.1f}s"
        )

    def _validate_configured_limits(self, sample: BatterySample):
        if not self.min_voltage_v <= sample.voltage <= self.max_voltage_v:
            raise ProtocolError(
                f"voltage outside configured limits: {sample.voltage:.3f} V"
            )
        if abs(sample.current) > self.max_abs_current_a:
            raise ProtocolError(
                f"current outside configured limits: {sample.current:.3f} A"
            )
        for label, temperature in (
            ("MOS", sample.mos_temperature),
            ("environment", sample.env_temperature),
        ):
            if not self.min_temperature_c <= temperature <= self.max_temperature_c:
                raise ProtocolError(
                    f"{label} temperature outside configured limits: "
                    f"{temperature:.1f} C"
                )
        if sample.comm_status != self.comm_ok_value:
            raise ProtocolError(
                f"lower controller reports BMS communication status "
                f"{sample.comm_status}"
            )

    def _on_raw_frame(self, msg: ByteMultiArray):
        self.byte_order = self.get_parameter("byte_order").value
        self.comm_ok_value = int(self.get_parameter("comm_ok_value").value)
        try:
            sample = parse_battery_frame(msg.data, self.byte_order)
            self._validate_configured_limits(sample)
        except ProtocolError as exc:
            now = time.monotonic()
            if now - self._last_invalid_log_at >= 5.0:
                self.get_logger().warning(f"Dropped invalid BMS frame: {exc}")
                self._last_invalid_log_at = now
            return

        messages = build_messages(sample, self.comm_ok_value)
        messages.battery_state.header.stamp = self.get_clock().now().to_msg()
        messages.battery_state.header.frame_id = "battery"
        self.battery_state_publisher.publish(messages.battery_state)
        self.power_voltage_publisher.publish(messages.power_voltage)
        if self.charger_status_publisher is not None:
            self.charger_status_publisher.publish(messages.charger_status)
        self.charging_flag_publisher.publish(messages.charging_flag)
        self.charging_current_publisher.publish(messages.charging_current)
        self._last_valid_at = time.monotonic()

    def _check_stale(self):
        now = time.monotonic()
        reference = self._last_valid_at or self._started_at
        if now - reference < self.stale_timeout_sec:
            return
        if now - self._last_stale_log_at >= self.stale_timeout_sec:
            self.get_logger().warning(
                f"No valid BMS telemetry for {now - reference:.1f}s"
            )
            self._last_stale_log_at = now


def main(args=None):
    """Run the BMS receiver node."""

    rclpy.init(args=args)
    node = BmsReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
