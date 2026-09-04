#!/usr/bin/env python3
"""4-probe ultrasonic module (RS485 via CH340 USB) driver for Gen2.

Protocol (measured 2026-09-04, one-to-four firmware):
  29-byte active frame @ 9600 8N1, ~7.3 Hz:
    [0..2]  = 50 03 18 frame header
    [18]    = probe 1 distance (cm, valid 15..255; 0x01 blind; 0x00 lost)
    [20]    = probe 2 distance (cm)
    [22],[24]= probes 3/4 raw (uncalibrated for now; rear sensors unused)
    [27..28]= checksum/status (not needed for values above)
Mount (Gen2): front probes 42 cm height, +-10 cm axis; rear probes parked.
Publishes /ultrasonic_array (UltrasonicArray, meters) for the web
sensor matrix, plus Range topics (ultrasonic_1..4) for conventions parity.
"""
import os
import time
from threading import Lock, Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray
from xw_interfaces.msg import UltrasonicArray

HEADER = bytes([0x50, 0x03, 0x18])
POWER_ON_CMD = bytes([0xAA, 0x01, 0xAB, 0x55])  # module activate (spec)
FRAME_LEN = 29
BAUD = 9600

# probe1 -> [18], probe2 -> [20], probe3 -> [22], probe4 -> [24]
PROBE_OFFSETS = [18, 20, 22, 24]
VALID_MIN = 15  # cm (module spec: 15..255), 0x00 lost, 0x01 blind


def _open_serial(port: str):
    import serial
    return serial.Serial(
        port=port, baudrate=BAUD, bytesize=8, parity='N', stopbits=1,
        timeout=0.4, xonxoff=0,
    )


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('xw_ultrasonic')
        self.declare_parameter('port', '/dev/ultrasonic')
        self.declare_parameter('poll_interval', 0.05)
        self.declare_parameter('frame_expected_hz', 7.0)
        self._port = str(self.get_parameter('port').value)
        self._interval = float(self.get_parameter('poll_interval').value)
        self._expected_hz = float(self.get_parameter('frame_expected_hz').value)

        self._arr_pub = self.create_publisher(UltrasonicArray, '/ultrasonic_array', 10)
        self._ranges = []
        for i in range(4):
            pub = self.create_publisher(Range, f'/ultrasonic_{i + 1}', 10)
            self._ranges.append(pub)

        self._ser = None
        self._lock = Lock()
        self._buf = b''
        self._last_frame = 0.0
        self._dists = [0] * 4

        # Reader thread takes care of serial; ROS publishes from the same
        # thread for simplicity (7 Hz is fine).
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()
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

    def _loop(self):
        self._last_keepalive = time.time()
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
                # silence watchdog: module must frame ~7 Hz
                if self._last_frame and time.time() - self._last_frame > 3.0:
                    self.get_logger().warn('ultrasonic frame silent, reopening')
                    self._open()
                elif time.time() - self._last_keepalive > 30.0:
                    # reactivate module (spec: needs AA 01 AB 55 after boot)
                    self._last_keepalive = time.time()
                    try:
                        self._ser.write(POWER_ON_CMD)
                    except Exception:  # noqa: BLE001
                        pass
                continue
            self._buf += data
            self._parse()

    def _parse(self):
        while True:
            idx = self._buf.find(HEADER)
            if idx < 0:
                self._buf = self._buf[-2:]
                return
            if idx > 0:
                self._buf = self._buf[idx:]
            if len(self._buf) < FRAME_LEN:
                return
            frame = self._buf[:FRAME_LEN]
            self._buf = self._buf[FRAME_LEN:]
            self._on_frame(frame)

    def _on_frame(self, frame):
        self._last_frame = time.time()
        arr = UltrasonicArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.labels = ['front_left', 'front_right', 'rear_left', 'rear_right']
        for i, off in enumerate(PROBE_OFFSETS):
            v = frame[off]
            self._dists[i] = v
        arr.ranges = [float(v) / 100.0 for v in self._dists]
        self._arr_pub.publish(arr)
        for i, v in enumerate(self._dists):
            if v < VALID_MIN:
                cm = float('nan') if v == 0 else 2.5  # blind zone
            else:
                cm = float(v)
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 1.0  # ~60 deg per module spec
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
