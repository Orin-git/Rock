"""STM32 chassis serial protocol (gen1 turn_on_ox_robot compatible).

Frames:
  ROS → MCU  11 bytes: 0x7B | mode | 0 | vx_be | vy_be | wz_be | bcc | 0x7D
               mode = TX[1] AutoRecharge latch (0 idle / 1 dock assist)
  MCU → ROS  24 bytes: 0x7B | Flag_Stop | vx | vy | vz | imu... | bcc | 0x7D
  MCU → ROS   8 bytes: 0x7C | I_hi | I_lo | Red | Charging | set_state | bcc | 0x7F
  MCU → ROS  30 bytes: 0xFB | 0x01 | 0x19 | battery... | bcc | 0xFD
Speeds are int16 big-endian in mm/s (angular ×1000); BCC = XOR of preceding bytes.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
SEND_SIZE = 11
RECV_SIZE = 24
AUTOCHARGE_HEADER = 0x7C
AUTOCHARGE_TAIL = 0x7F
AUTOCHARGE_SIZE = 8
BMS_HEADER = 0xFB
BMS_TAIL = 0xFD
BMS_SIZE = 30
BMS_TYPE = 0x01
BMS_PAYLOAD_LEN = 0x19


def xor_bcc(data: Sequence[int]) -> int:
    check = 0
    for b in data:
        check ^= int(b) & 0xFF
    return check & 0xFF


def pack_speed(vx: float, vy: float, wz: float, mode: int = 0) -> bytes:
    """Build 11-byte speed command (m/s, rad/s → mm/s scaled int16)."""
    tx = bytearray(SEND_SIZE)
    tx[0] = FRAME_HEADER
    tx[1] = int(mode) & 0xFF
    tx[2] = 0
    for i, val in enumerate((vx, vy, wz)):
        scaled = int(round(float(val) * 1000.0))
        if scaled > 32767:
            scaled = 32767
        elif scaled < -32768:
            scaled = -32768
        hi = (scaled >> 8) & 0xFF
        lo = scaled & 0xFF
        base = 3 + i * 2
        tx[base] = hi
        tx[base + 1] = lo
    tx[9] = xor_bcc(tx[0:9])
    tx[10] = FRAME_TAIL
    return bytes(tx)


def odom_trans(hi: int, lo: int) -> float:
    """mm/s int16 BE → m/s (gen1 Odom_Trans intent; use float divide for Python)."""
    raw = struct.unpack('>h', bytes((hi & 0xFF, lo & 0xFF)))[0]
    return float(raw) / 1000.0


@dataclass
class MotionFrame:
    flag_stop: int
    vx: float
    vy: float
    wz: float


@dataclass
class ChargeFrame:
    current: float
    red: int
    charging: bool
    charge_set_state: int


def parse_charge_frame(buf: bytes) -> Optional[ChargeFrame]:
    if len(buf) != AUTOCHARGE_SIZE:
        return None
    if buf[0] != AUTOCHARGE_HEADER or buf[7] != AUTOCHARGE_TAIL:
        return None
    if xor_bcc(buf[0:6]) != buf[6]:
        return None
    current = ((buf[1] << 8) | buf[2]) / 1000.0
    return ChargeFrame(
        current=float(current),
        red=int(buf[3]),
        charging=bool(buf[4]),
        charge_set_state=int(buf[5]),
    )


def parse_motion_frame(buf: bytes) -> Optional[MotionFrame]:
    if len(buf) != RECV_SIZE:
        return None
    if buf[0] != FRAME_HEADER or buf[23] != FRAME_TAIL:
        return None
    if xor_bcc(buf[0:22]) != buf[22]:
        return None
    vx = odom_trans(buf[2], buf[3])
    vy = odom_trans(buf[4], buf[5])
    # MCU Z sign opposite ROS CCW+
    wz = -odom_trans(buf[6], buf[7])
    return MotionFrame(flag_stop=int(buf[1]), vx=vx, vy=vy, wz=wz)


def parse_bms_raw_frame(buf: bytes) -> Optional[bytes]:
    """Validate BMS envelope and return the 30-byte frame, or None."""
    if len(buf) != BMS_SIZE:
        return None
    if (
        buf[0] != BMS_HEADER
        or buf[1] != BMS_TYPE
        or buf[2] != BMS_PAYLOAD_LEN
        or buf[29] != BMS_TAIL
    ):
        return None
    if xor_bcc(buf[0:28]) != buf[28]:
        return None
    return bytes(buf)


class FrameParser:
    """Byte-stream reassembler for motion / charge / BMS frames."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def reset(self) -> None:
        self._buf.clear()

    @staticmethod
    def _next_header(buf: bytearray) -> int:
        headers = (FRAME_HEADER, AUTOCHARGE_HEADER, BMS_HEADER)
        positions = [buf.find(bytes((h,))) for h in headers]
        valid = [p for p in positions if p >= 0]
        return min(valid) if valid else -1

    def feed(
        self, data: bytes
    ) -> Tuple[List[MotionFrame], List[ChargeFrame], List[bytes]]:
        motion: List[MotionFrame] = []
        charge: List[ChargeFrame] = []
        bms: List[bytes] = []
        if data:
            self._buf.extend(data)
        while True:
            if not self._buf:
                break
            start = self._next_header(self._buf)
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            kind = self._buf[0]
            if kind == AUTOCHARGE_HEADER:
                if len(self._buf) < AUTOCHARGE_SIZE:
                    break
                frame = parse_charge_frame(bytes(self._buf[:AUTOCHARGE_SIZE]))
                if frame is not None:
                    del self._buf[:AUTOCHARGE_SIZE]
                    charge.append(frame)
                    continue
                del self._buf[0]
                continue
            if kind == BMS_HEADER:
                if len(self._buf) < BMS_SIZE:
                    break
                frame_b = parse_bms_raw_frame(bytes(self._buf[:BMS_SIZE]))
                if frame_b is not None:
                    del self._buf[:BMS_SIZE]
                    bms.append(frame_b)
                    continue
                del self._buf[0]
                continue
            if len(self._buf) < RECV_SIZE:
                break
            frame_m = parse_motion_frame(bytes(self._buf[:RECV_SIZE]))
            if frame_m is not None:
                del self._buf[:RECV_SIZE]
                motion.append(frame_m)
                continue
            del self._buf[0]
        return motion, charge, bms


class ChassisSerial:
    """Thin pyserial wrapper with open/reopen helpers."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        fallback_ports: Optional[Sequence[str]] = None,
        timeout: float = 0.02,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.fallback_ports = list(fallback_ports or [])
        self.timeout = timeout
        self._ser = None
        self._active_port: Optional[str] = None
        self.parser = FrameParser()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and bool(getattr(self._ser, 'is_open', False))

    @property
    def active_port(self) -> Optional[str]:
        return self._active_port

    def _candidates(self) -> List[str]:
        seen = set()
        out: List[str] = []
        for p in [self.port, *self.fallback_ports]:
            if p and p not in seen:
                seen.add(p)
                out.append(p)
        return out

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None
        self._active_port = None
        self.parser.reset()

    def open(self) -> bool:
        try:
            from serial import Serial
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                'pyserial unavailable (from serial import Serial failed). '
                'Reinstall with: pip3 install --force-reinstall pyserial'
            ) from exc

        self.close()
        last_err: Optional[BaseException] = None
        for path in self._candidates():
            try:
                ser = Serial(
                    port=path,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                    dsrdtr=False,
                    rtscts=False,
                )
                # Avoid USB-CDC reset on open (DTR/RTS toggle)
                try:
                    ser.dtr = False
                    ser.rts = False
                except Exception:  # noqa: BLE001
                    pass
                try:
                    ser.reset_input_buffer()
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(0.05)
                self._ser = ser
                self._active_port = path
                return True
            except Exception as exc:  # noqa: BLE001
                last_err = exc
                continue
        if last_err is not None:
            raise last_err
        return False

    def write(self, payload: bytes) -> None:
        if not self.is_open:
            raise OSError('serial not open')
        self._ser.write(payload)

    def read_available(self) -> bytes:
        if not self.is_open:
            return b''
        n = getattr(self._ser, 'in_waiting', 0) or 0
        if n <= 0:
            return b''
        return bytes(self._ser.read(n))

    def drain(self) -> Tuple[List[MotionFrame], List[ChargeFrame], List[bytes]]:
        return self.parser.feed(self.read_available())

    def drain_frames(self) -> List[MotionFrame]:
        motion, _charge, _bms = self.drain()
        return motion


def wait_for_port(paths: Sequence[str], timeout_sec: float = 5.0) -> Optional[str]:
    import os

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for p in paths:
            if p and os.path.exists(p) and os.access(p, os.R_OK | os.W_OK):
                return p
        time.sleep(0.1)
    return None
