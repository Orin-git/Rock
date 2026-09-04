#!/usr/bin/env python3
"""4-probe ultrasonic module (RS485 via CH340 USB) driver for Gen2.

Protocol (datasheet + verified on unit 2026-09-04):
  13-byte frames @ 9600 8N1:
    0xAA | A B C D E F G H | R1 R2 | chk | 0x55
    A..H = distances cm (30..255 valid, 0x00 probe lost, 0x01 blind zone)
    chk  = low 8 bits of (A+B+C+D+E+F+G+H+R1+R2)
  Power-on (activate) cmd: 0xAA 0x01 0xAB 0x55  (also sent on open + 30 s)
  Mount (Gen2): front probes 42 cm height, +-10 cm;
                rear probes parked (hardware standoffs).
"""
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from xw_interfaces.msg import UltrasonicArray

BAUD = 9600
FRAME_LEN = 13
FLAG = 0xAA
TRAIL = 0x55
POWER_ON_CMD = bytes([0xAA, 0x01, 0xAB, 0x55])


def _open_serial(port: str):
    import serial
    return serial.Serial(
        port=port, baudrate=BAUD, bytesize=8, parity='N', stopbits=1,
        timeout=0.4, xonxoff=0,
    )


def _frame_ok(f):
    total = (sum(f[1:11])) & 0xFF
    return f[0] == FLAG and f[11] == total and f[12] == TRAIL


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('xw_ultrasonic')
        self.declare_parameter('port', '/dev/ultrasonic')
        self._port = str(self.get_parameter('port').value)

        self._arr_pub = self.create_publisher(UltrasonicArray, '/ultrasonic_array', 10)
        self._ranges = []
        for i in range(4):
            self._ranges.append(
                self.create_publisher(Range, f'/ultrasonic_{i + 1}', 10))

        self._ser = None
        self._buf = b''
        self._last_frame = 0.0
        self._last_keepalive = 0.0
        self._dists = [0] * 4

        Thread(target=self._loop, daemon=True).start()
        self.get_logger().info(f'ultrasonic node up ({self._port})')

    def _open(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None
        try:
            self._ser = _open_serial(self._port)
            self._buf = b''
            try:
                self._ser.write(POWER_ON_CMD)
            except Exception:  # noqa: BLE001
                pass
            self.get_logger().info(f'ultrasonic serial opened {self._port}')
        except Exception as exc:  # noqa: BLE001
            self._ser = None
            self.get_logger().warn(f'ultrasonic open failed: {exc}')

    def _send_power_on(self):
        if self._ser is None:
            return
        try:
            self._ser.write(POWER_ON_CMD)
        except Exception:  # noqa: BLE001
            pass

    def _loop(self):
        while rclpy.ok():
            if self._ser is None:
                self._open()
                if self._ser is None:
                    time.sleep(2.0)
                    continue
            try:
                data = self._ser.read(512)
            except Exception:  # noqa: BLE001
                self._ser = None
                continue
            if not data:
                now = time.time()
                if self._last_frame and now - self._last_frame > 3.0:
                    self.get_logger().warn('ultrasonic frame silent, reopening')
                    self._open()
                elif now - self._last_keepalive > 30.0:
                    self._last_keepalive = now
                    self._send_power_on()
                continue
            self._buf += data
            self._parse()

    def _parse(self):
        while True:
            i = self._buf.find(bytes([FLAG]))
            if i < 0:
                self._buf = b''
                return
            if i > 0:
                self._buf = self._buf[i:]
            if len(self._buf) < FRAME_LEN:
                return
            f = self._buf[:FRAME_LEN]
            if not _frame_ok(f):
                # misaligned: drop one byte and resync
                self._buf = self._buf[1:]
                continue
            self._buf = self._buf[FRAME_LEN:]
            self._on_frame(f)

    def _on_frame(self, f):
        self._last_frame = time.time()
        for i in range(4):
            self._dists[i] = f[1 + i]  # A..D = probes 1..4

        arr = UltrasonicArray()
        arr.stamp = self.get_clock().now().to_msg()
        arr.labels = ['front_left', 'front_right', 'rear_left', 'rear_right']
        arr.ranges = [float(v) / 100.0 for v in self._dists]
        self._arr_pub.publish(arr)

        for i, v in enumerate(self._dists):
            cm = float('nan') if v == 0 else (2.5 if v == 1 else float(v))
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 1.0
            msg.min_range = 0.15
            msg.max_range = 2.55
            msg.range = cm
            self._ranges[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
