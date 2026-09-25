"""VESC binary protocol (FW 5.x–7.x): framing, CRC and the few commands the drive needs.

Pure Python, no ROS — unit-tested on any machine.
Frame: [0x02, len, payload..., crc_hi, crc_lo, 0x03] (len ≤ 255) or [0x03, len_hi, len_lo, ...].
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34

_GET_VALUES_MIN_LEN = 59  # up to and including controller_id
MAX_PAYLOAD = 512         # we only exchange short packets; longer "frames" are noise


def crc16(data: bytes) -> int:
    """CRC16-CCITT (XMODEM, init 0), as used by the VESC firmware."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(payload: bytes) -> bytes:
    n = len(payload)
    if n == 0:
        raise ValueError("empty payload")
    head = bytes([2, n]) if n <= 255 else bytes([3, (n >> 8) & 0xFF, n & 0xFF])
    crc = crc16(payload)
    return head + payload + bytes([crc >> 8, crc & 0xFF, 3])


class FrameDecoder:
    """Incremental decoder: feed raw serial bytes, get complete CRC-valid payloads."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[bytes]:
        self._buf.extend(data)
        out: list[bytes] = []
        buf = self._buf
        while buf:
            start = buf[0]
            if start == 2:
                if len(buf) < 2:
                    break
                hdr, plen = 2, buf[1]
            elif start == 3:
                if len(buf) < 3:
                    break
                hdr, plen = 3, (buf[1] << 8) | buf[2]
            else:
                del buf[0]
                continue
            # the 3-byte header is only used for payloads > 255 bytes
            if plen == 0 or plen > MAX_PAYLOAD or (hdr == 3 and plen <= 255):
                del buf[0]
                continue
            total = hdr + plen + 3
            if len(buf) < total:
                break
            payload = bytes(buf[hdr:hdr + plen])
            crc_rx = (buf[hdr + plen] << 8) | buf[hdr + plen + 1]
            if buf[hdr + plen + 2] == 3 and crc16(payload) == crc_rx:
                out.append(payload)
                del buf[:total]
            else:
                # not a valid frame at this position: resync one byte later
                self.crc_errors += 1
                del buf[0]
        return out


@dataclass(frozen=True)
class VescValues:
    controller_id: int
    temp_fet: float        # °C
    temp_motor: float      # °C
    current_motor: float   # A
    current_in: float      # A
    duty: float            # -1..1
    erpm: float
    v_in: float            # V
    tachometer: int        # 6 counts per electrical revolution
    fault: int             # mc_fault_code, 0 = none


def parse_values(payload: bytes) -> VescValues | None:
    if len(payload) < _GET_VALUES_MIN_LEN or payload[0] != COMM_GET_VALUES:
        return None
    return VescValues(
        controller_id=payload[58],
        temp_fet=struct.unpack_from(">h", payload, 1)[0] / 10.0,
        temp_motor=struct.unpack_from(">h", payload, 3)[0] / 10.0,
        current_motor=struct.unpack_from(">i", payload, 5)[0] / 100.0,
        current_in=struct.unpack_from(">i", payload, 9)[0] / 100.0,
        duty=struct.unpack_from(">h", payload, 21)[0] / 1000.0,
        erpm=float(struct.unpack_from(">i", payload, 23)[0]),
        v_in=struct.unpack_from(">h", payload, 27)[0] / 10.0,
        tachometer=struct.unpack_from(">i", payload, 45)[0],
        fault=payload[53],
    )


def _wrap(inner: bytes, can_id: int | None) -> bytes:
    return inner if can_id is None else bytes([COMM_FORWARD_CAN, can_id]) + inner


def cmd_get_values(can_id: int | None = None) -> bytes:
    return _wrap(bytes([COMM_GET_VALUES]), can_id)


def cmd_set_rpm(erpm: float, can_id: int | None = None) -> bytes:
    return _wrap(bytes([COMM_SET_RPM]) + struct.pack(">i", int(round(erpm))), can_id)


def cmd_set_current(amps: float, can_id: int | None = None) -> bytes:
    return _wrap(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(round(amps * 1000.0))), can_id)


def cmd_set_current_brake(amps: float, can_id: int | None = None) -> bytes:
    return _wrap(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", int(round(abs(amps) * 1000.0))), can_id)
