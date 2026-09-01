#!/usr/bin/env python3
"""Chassis driver: mock odom or real STM32 serial (/cmd_vel → MCU)."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, UInt8MultiArray
from tf2_ros import TransformBroadcaster

from bms_receiver.protocol import ChargeState, ProtocolError, parse_battery_frame
from xw_chassis.serial_mcu import ChassisSerial, pack_speed, wait_for_port
from xw_interfaces.msg import PowerState


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ChassisNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_chassis')
        self.declare_parameter('use_sim_hw', True)
        # When EKF owns odom→base_link, set publish_odom_tf:=false and odom_topic:=odom/wheel.
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_odom_tf', True)  # alias; AND with publish_tf
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('serial_port', '/dev/chassis')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('serial_fallback', '/dev/ttyACM0')
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('serial_no_frame_reopen_sec', 2.0)
        self.declare_parameter('charge_current_dock_min', 0.05)
        self.declare_parameter('lock_motion_when_docked', True)
        self.declare_parameter('bms_byte_order', 'big')
        self.declare_parameter('bms_comm_ok_value', 1)
        self.declare_parameter('bms_raw_frame_topic', '/bms/raw_frame')

        self._use_sim = bool(self.get_parameter('use_sim_hw').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value) and bool(
            self.get_parameter('publish_odom_tf').value
        )
        self._odom_topic = str(self.get_parameter('odom_topic').value or 'odom').lstrip('/')
        self._base = str(self.get_parameter('base_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self._no_frame_reopen = float(self.get_parameter('serial_no_frame_reopen_sec').value)
        self._bms_byte_order = str(self.get_parameter('bms_byte_order').value or 'big')
        self._bms_comm_ok = int(self.get_parameter('bms_comm_ok_value').value)
        if self._bms_byte_order not in ('little', 'big'):
            raise ValueError("bms_byte_order must be 'little' or 'big'")

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._meas_vx = 0.0
        self._meas_vy = 0.0
        self._meas_wz = 0.0
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._flag_stop = 0  # MCU rx[1]; on this HW: !=0 = ready/enabled, 0 = cannot drive
        self._last_cmd_wall = 0.0
        self._last_frame_wall = 0.0
        self._last_motor_pub: Optional[bool] = None
        self._last_motor_pub_wall = 0.0
        self._serial: Optional[ChassisSerial] = None
        self._last_odom_wall = time.monotonic()
        self._tx_count = 0
        self._rx_count = 0
        self._last_tx_hex = ''
        self._write_errors = 0
        self._charge_mode = 0
        self._charging = False
        self._charging_current = 0.0
        self._ir_red = 0
        self._charge_set_state = 0
        self._docked = False
        self._saw_charge_frame = False
        self._battery_percent = 0.0
        self._battery_voltage = 0.0
        self._bms_current = 0.0
        self._bms_charging = False
        self._saw_bms_frame = False
        self._bms_rx_count = 0
        self._last_bms_invalid_log = 0.0

        self._cmd_sub = self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(Int8, '/xw/chassis/charge_mode', self._on_charge_mode, 10)
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._power_pub = self.create_publisher(PowerState, '/xw/power', 10)
        # true = cannot drive; false = enabled (ready). Maps MCU Flag_Stop inverted vs gen1 comments.
        self._motor_disabled_pub = self.create_publisher(Bool, '/xw/chassis/motor_disabled', 10)
        self._bms_raw_pub = self.create_publisher(
            UInt8MultiArray,
            str(self.get_parameter('bms_raw_frame_topic').value),
            10,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        if self._use_sim:
            self._timer = self.create_timer(0.05, self._tick_sim)
            self.get_logger().info('chassis started (use_sim_hw=True)')
        else:
            port = str(self.get_parameter('serial_port').value)
            baud = int(self.get_parameter('serial_baud_rate').value)
            fallback = str(self.get_parameter('serial_fallback').value)
            self._serial = ChassisSerial(
                port=port,
                baudrate=baud,
                fallback_ports=[fallback] if fallback else [],
            )
            self._timer = self.create_timer(1.0 / 30.0, self._tick_serial)
            self._try_open_serial(initial=True)
            self.get_logger().info(
                f'chassis started (use_sim_hw=False port={port} baud={baud} '
                f'bms_byte_order={self._bms_byte_order})'
            )

        self._power_timer = self.create_timer(1.0, self._publish_power)

    def _clamp_cmd(self, msg: Twist) -> tuple[float, float, float]:
        max_lin = float(self.get_parameter('max_linear').value)
        max_ang = float(self.get_parameter('max_angular').value)
        vx = max(-max_lin, min(max_lin, float(msg.linear.x)))
        vy = max(-max_lin, min(max_lin, float(msg.linear.y)))
        wz = max(-max_ang, min(max_ang, float(msg.angular.z)))
        return vx, vy, wz

    def _on_charge_mode(self, msg: Int8) -> None:
        self._charge_mode = int(msg.data) & 0xFF

    def _effective_cmd(self) -> tuple[float, float, float]:
        vx, vy, wz = self._cmd_vx, self._cmd_vy, self._cmd_wz
        lock = bool(self.get_parameter('lock_motion_when_docked').value)
        charging = self._bms_charging if self._saw_bms_frame else self._charging
        if lock and charging and self._charge_mode == 0:
            return 0.0, 0.0, 0.0
        return vx, vy, wz

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_vx, self._cmd_vy, self._cmd_wz = self._clamp_cmd(msg)
        self._last_cmd_wall = time.monotonic()
        vx, vy, wz = self._effective_cmd()
        if self._use_sim:
            self._meas_vx = vx
            self._meas_wz = wz
        else:
            self._send_speed(vx, vy, wz)

    def _try_open_serial(self, initial: bool = False) -> bool:
        if self._serial is None:
            return False
        paths = [self._serial.port, *self._serial.fallback_ports]
        ready = wait_for_port(paths, timeout_sec=5.0 if initial else 0.5)
        if ready is None:
            self.get_logger().error(
                f'chassis serial not ready ({", ".join(paths)})'
            )
            return False
        try:
            ok = self._serial.open()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'chassis serial open failed: {exc}')
            return False
        if ok:
            self._last_frame_wall = time.monotonic()
            self.get_logger().info(
                f'chassis serial open {self._serial.active_port} @ {self._serial.baudrate}'
            )
        return ok

    def _send_speed(self, vx: float, vy: float, wz: float) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        try:
            payload = pack_speed(vx, vy, wz, mode=self._charge_mode)
            self._serial.write(payload)
            self._tx_count += 1
            self._last_tx_hex = payload.hex()
        except Exception as exc:  # noqa: BLE001
            self._write_errors += 1
            self.get_logger().error(f'chassis serial write failed: {exc}')
            self._serial.close()

    def _publish_motor_disabled(self, disabled: bool, force: bool = False) -> None:
        now = time.monotonic()
        changed = self._last_motor_pub is None or disabled != self._last_motor_pub
        period = (now - self._last_motor_pub_wall) >= 1.0
        if not force and not changed and not period:
            return
        msg = Bool()
        msg.data = bool(disabled)
        self._motor_disabled_pub.publish(msg)
        self._last_motor_pub = bool(disabled)
        self._last_motor_pub_wall = now

    def _publish_odom(self) -> None:
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = yaw_to_quat(self._yaw)
        odom.twist.twist.linear.x = self._meas_vx
        odom.twist.twist.linear.y = self._meas_vy
        odom.twist.twist.angular.z = self._meas_wz
        # Non-zero diagonals required by robot_localization (EKF rejects all-zero cov).
        # Pose unused when odom0 only fuses vx; twist vx trusted, vy/wz de-weighted.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.15
        self._odom_pub.publish(odom)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = yaw_to_quat(self._yaw)
            self._tf_broadcaster.sendTransform(t)

    def _handle_bms_frames(self, bms_frames: list) -> None:
        if not bms_frames:
            return
        for raw in bms_frames:
            msg = UInt8MultiArray()
            msg.data = list(raw)
            self._bms_raw_pub.publish(msg)
            try:
                sample = parse_battery_frame(raw, self._bms_byte_order)
            except ProtocolError as exc:
                now = time.monotonic()
                if now - self._last_bms_invalid_log >= 5.0:
                    self.get_logger().warning(f'Dropped invalid BMS payload: {exc}')
                    self._last_bms_invalid_log = now
                continue
            if sample.comm_status != self._bms_comm_ok:
                now = time.monotonic()
                if now - self._last_bms_invalid_log >= 5.0:
                    self.get_logger().warning(
                        f'BMS communication status={sample.comm_status} '
                        f'(expected {self._bms_comm_ok})'
                    )
                    self._last_bms_invalid_log = now
                continue
            self._battery_percent = float(sample.soc_percent)
            self._battery_voltage = float(sample.voltage)
            self._bms_current = float(sample.current)
            self._bms_charging = (
                sample.state == ChargeState.CHARGING and sample.protection_bits == 0
            )
            self._saw_bms_frame = True
            self._bms_rx_count += 1
            if self._saw_charge_frame:
                imin = float(self.get_parameter('charge_current_dock_min').value)
                self._docked = self._bms_charging and self._charging_current >= imin

    def _tick_sim(self) -> None:
        dt = 0.05
        vx, vy, wz = self._effective_cmd()
        self._yaw += wz * dt
        self._x += vx * math.cos(self._yaw) * dt
        self._y += vx * math.sin(self._yaw) * dt
        self._meas_vx = vx
        self._meas_wz = wz
        self._publish_odom()

    def _tick_serial(self) -> None:
        assert self._serial is not None
        now = time.monotonic()

        if not self._serial.is_open:
            self._try_open_serial(initial=False)
            return

        if self._last_cmd_wall > 0.0 and (now - self._last_cmd_wall) > self._cmd_timeout:
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
            self._cmd_wz = 0.0

        # Always forward (gen1 does not gate TX on Flag_Stop); TX[1] carries charge mode
        evx, evy, ewz = self._effective_cmd()
        self._send_speed(evx, evy, ewz)

        try:
            frames, charge_frames, bms_frames = self._serial.drain()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'chassis serial read failed: {exc}')
            self._serial.close()
            return

        self._handle_bms_frames(bms_frames)

        if charge_frames:
            ch = charge_frames[-1]
            self._saw_charge_frame = True
            self._charging_current = float(ch.current)
            self._ir_red = int(ch.red)
            self._charging = bool(ch.charging)
            self._charge_set_state = int(ch.charge_set_state)
            imin = float(self.get_parameter('charge_current_dock_min').value)
            # Prefer BMS charge flag for "on charge" when available (gen1 behavior)
            charging_now = self._bms_charging if self._saw_bms_frame else self._charging
            self._docked = charging_now and self._charging_current >= imin

        if frames:
            self._last_frame_wall = now
            self._rx_count += len(frames)
            dt = now - self._last_odom_wall
            if dt < 1e-4:
                dt = 1e-4
            if dt > 0.2:
                dt = 0.2
            frame = frames[-1]
            self._meas_vx = frame.vx
            self._meas_vy = frame.vy
            self._meas_wz = frame.wz
            # MCU may report 0 twist while motors actually run — fall back to cmd for pose.
            meas_mag = abs(self._meas_vx) + abs(self._meas_vy) + abs(self._meas_wz)
            if meas_mag > 1e-3:
                ivx, ivy, iwz = self._meas_vx, self._meas_vy, self._meas_wz
            else:
                ivx, ivy, iwz = self._cmd_vx, self._cmd_vy, self._cmd_wz
            self._x += (ivx * math.cos(self._yaw) - ivy * math.sin(self._yaw)) * dt
            self._y += (ivx * math.sin(self._yaw) + ivy * math.cos(self._yaw)) * dt
            self._yaw += iwz * dt
            self._last_odom_wall = now
            # Expose integrated twist so jog / UI see motion even without encoder feedback
            if meas_mag <= 1e-3:
                self._meas_vx, self._meas_vy, self._meas_wz = ivx, ivy, iwz
            self._flag_stop = int(frame.flag_stop)
            # Live HW: Flag_Stop!=0 → 可控制；0 → 不可控制. Topic name is motor_disabled.
            disabled = self._flag_stop == 0
            if disabled and self._tx_count % 90 == 1:
                self.get_logger().warn(
                    f'MCU Flag_Stop={self._flag_stop} → motor_disabled=true (cannot drive)'
                )
            self._publish_motor_disabled(disabled)
            self._publish_odom()
        else:
            # No new frame: still dead-reckon on cmd so jog can complete
            dt = now - self._last_odom_wall
            if dt >= 0.02 and (
                abs(self._cmd_vx) + abs(self._cmd_vy) + abs(self._cmd_wz) > 1e-3
            ):
                if dt > 0.2:
                    dt = 0.2
                self._x += (
                    self._cmd_vx * math.cos(self._yaw) - self._cmd_vy * math.sin(self._yaw)
                ) * dt
                self._y += (
                    self._cmd_vx * math.sin(self._yaw) + self._cmd_vy * math.cos(self._yaw)
                ) * dt
                self._yaw += self._cmd_wz * dt
                self._meas_vx, self._meas_vy, self._meas_wz = (
                    self._cmd_vx,
                    self._cmd_vy,
                    self._cmd_wz,
                )
                self._last_odom_wall = now
            self._publish_odom()
            if self._last_frame_wall > 0.0 and (now - self._last_frame_wall) > self._no_frame_reopen:
                self.get_logger().warn('chassis serial no frame, reopening')
                self._serial.close()

    def _publish_power(self) -> None:
        p = PowerState()
        p.stamp = self.get_clock().now().to_msg()
        p.ir_red = int(self._ir_red) & 0xFF
        p.charging_current = float(self._charging_current)
        if self._use_sim:
            p.battery_percent = 88.0
            p.voltage = 24.5
            p.charging = bool(self._charging)
            p.docked = bool(self._docked)
            p.detail = f'mock charge_mode={self._charge_mode}'
        else:
            p.battery_percent = float(self._battery_percent)
            p.voltage = float(self._battery_voltage)
            # Prefer BMS charging flag when BMS frames are present
            p.charging = (
                bool(self._bms_charging) if self._saw_bms_frame else bool(self._charging)
            )
            p.docked = bool(self._docked)
            port = self._serial.active_port if self._serial else ''
            seen_7c = '0x7c' if self._saw_charge_frame else 'no-0x7c'
            seen_bms = 'bms' if self._saw_bms_frame else 'no-bms'
            p.detail = (
                f'serial:{port or "closed"} tx={self._tx_count} rx={self._rx_count} '
                f'bms_rx={self._bms_rx_count} err={self._write_errors} '
                f'flag_stop={self._flag_stop} {seen_7c} {seen_bms} '
                f'mode={self._charge_mode} ir={self._ir_red} '
                f'I={self._charging_current:.3f}A Ibms={self._bms_current:.3f}A '
                f'SOC={self._battery_percent:.1f}% V={self._battery_voltage:.2f} '
                f'last={self._last_tx_hex} '
                f'cmd=({self._cmd_vx:.2f},{self._cmd_wz:.2f}) '
                f'meas=({self._meas_vx:.2f},{self._meas_wz:.2f})'
            )
        self._power_pub.publish(p)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._send_speed(0.0, 0.0, 0.0)
            except Exception:  # noqa: BLE001
                pass
            self._serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
