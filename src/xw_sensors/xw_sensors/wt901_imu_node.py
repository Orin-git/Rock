#!/usr/bin/env python3
"""WT901C485 (RS-485 Modbus RTU) → /imu/data (frame_id=imu_link).

Does not stream UART; polls holding registers starting at 0x34.
Default slave 0x50, baud 9600 — verified on Rock 5T + CH340 (/dev/imu).
"""

from __future__ import annotations

import math
import struct
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None  # type: ignore


def _crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class Wt901ImuNode(Node):
    """Poll WT901C485 and publish sensor_msgs/Imu."""

    # AX AY AZ GX GY GZ HX HY HZ Roll Pitch Yaw
    REG_START = 0x34
    REG_COUNT = 12

    def __init__(self) -> None:
        super().__init__('xw_wt901_imu')
        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter('port_fallback', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('slave_id', 0x50)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 15.0)
        self.declare_parameter('timeout_s', 0.12)

        if serial is None:
            raise RuntimeError('pyserial not installed')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._slave = int(self.get_parameter('slave_id').value) & 0xFF
        self._baud = int(self.get_parameter('baud').value)
        self._timeout = float(self.get_parameter('timeout_s').value)
        rate = max(1.0, float(self.get_parameter('rate').value))

        port = str(self.get_parameter('port').value)
        fallback = str(self.get_parameter('port_fallback').value)
        self._port_candidates = [port, fallback]
        self._ser: Optional['serial.Serial'] = None
        self._fail_streak = 0
        self._ok_count = 0

        self._pub = self.create_publisher(Imu, '/imu/data', 10)
        self._open_serial()
        # Keep timer period > worst-case Modbus RTT (~40–80 ms @ 9600).
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'WT901C485 Modbus IMU port={self._ser.port if self._ser else "?"} '
            f'baud={self._baud} slave=0x{self._slave:02x} rate={rate:.1f}Hz → /imu/data'
        )

    def _open_serial(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        last_err: Optional[Exception] = None
        for p in self._port_candidates:
            try:
                # exclusive avoids silent multi-open failures on CH340
                self._ser = serial.Serial(
                    p,
                    self._baud,
                    timeout=0.02,
                    write_timeout=0.2,
                    exclusive=True,
                )
                self.get_logger().info(f'opened {p}')
                return
            except Exception as exc:  # noqa: BLE001
                last_err = exc
        self.get_logger().error(f'IMU serial open failed: {last_err}')

    def _build_read(self, start: int, count: int) -> bytes:
        req = struct.pack('>BBHH', self._slave, 0x03, start & 0xFFFF, count & 0xFFFF)
        crc = _crc16_modbus(req)
        return req + struct.pack('<H', crc)

    def _read_exact(self, n: int, deadline: float) -> bytes:
        buf = bytearray()
        assert self._ser is not None
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.002)
        return bytes(buf)

    def _read_regs(self) -> Optional[List[int]]:
        if self._ser is None:
            self._open_serial()
            if self._ser is None:
                return None
        expect = 5 + 2 * self.REG_COUNT  # addr func len data crc
        req = self._build_read(self.REG_START, self.REG_COUNT)
        try:
            self._ser.reset_input_buffer()
            self._ser.write(req)
            # RS-485 DE/RE turnaround
            time.sleep(0.005)
            resp = self._read_exact(expect, time.monotonic() + self._timeout)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'IMU IO error: {exc}')
            self._open_serial()
            return None

        if len(resp) < expect:
            return None
        if resp[0] != self._slave or resp[1] != 0x03 or resp[2] != 2 * self.REG_COUNT:
            return None
        crc_calc = _crc16_modbus(resp[:-2])
        crc_rx = resp[-2] | (resp[-1] << 8)
        if crc_calc != crc_rx:
            return None
        payload = resp[3:-2]
        return [struct.unpack('>h', payload[i : i + 2])[0] for i in range(0, len(payload), 2)]

    def _tick(self) -> None:
        regs = self._read_regs()
        if regs is None or len(regs) < 12:
            self._fail_streak += 1
            if self._fail_streak in (1, 10, 50, 200):
                self.get_logger().warn(f'IMU Modbus read fail streak={self._fail_streak}')
            if self._fail_streak % 30 == 0:
                self._open_serial()
            return
        if self._fail_streak:
            self.get_logger().info(f'IMU Modbus recovered after {self._fail_streak} fails')
        self._fail_streak = 0
        self._ok_count += 1

        ax, ay, az = [v / 32768.0 * 16.0 * 9.80665 for v in regs[0:3]]
        gx, gy, gz = [math.radians(v / 32768.0 * 2000.0) for v in regs[3:6]]
        roll, pitch, yaw = [math.radians(v / 32768.0 * 180.0) for v in regs[9:12]]

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation = _euler_to_quat(roll, pitch, yaw)
        msg.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.1,
        ]
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02,
        ]
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.08,
        ]
        self._pub.publish(msg)
        if self._ok_count == 1 or self._ok_count % 150 == 0:
            self.get_logger().info(
                f'IMU ok#{self._ok_count} az={az:.2f} yaw_deg={math.degrees(yaw):.1f}'
            )

    def destroy_node(self) -> bool:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Wt901ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
