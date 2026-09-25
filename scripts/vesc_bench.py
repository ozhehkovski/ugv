#!/usr/bin/env python3
"""Bench test for the VESC Duet (robot on stand): FW info, telemetry, each wheel jog at low ERPM."""
from __future__ import annotations

import struct
import sys
import time

import serial

PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
FW, GET_VALUES, SET_CURRENT, SET_RPM, FORWARD_CAN = 0, 4, 6, 8, 34
WHEEL_R, POLE_PAIRS = 0.0825, 20


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(p: bytes) -> bytes:
    c = crc16(p)
    return bytes([2, len(p)]) + p + bytes([c >> 8, c & 0xFF, 3])


class Bus:
    def __init__(self, port: str) -> None:
        self.s = serial.Serial(port, 115200, timeout=0.02)
        self.buf = bytearray()

    def send(self, p: bytes, can: int | None) -> None:
        self.s.write(frame(p if can is None else bytes([FORWARD_CAN, can]) + p))

    def read_packets(self, t: float) -> list[bytes]:
        out: list[bytes] = []
        end = time.monotonic() + t
        while time.monotonic() < end:
            self.buf += self.s.read(512)
            while len(self.buf) >= 5:
                if self.buf[0] != 2:
                    self.buf.pop(0)
                    continue
                n = self.buf[1]
                if len(self.buf) < n + 5:
                    break
                p = bytes(self.buf[2:2 + n])
                ok = (self.buf[2 + n] << 8 | self.buf[3 + n]) == crc16(p) and self.buf[4 + n] == 3
                del self.buf[:n + 5]
                if ok:
                    out.append(p)
        return out

    def values(self, can: int | None) -> dict[str, float] | None:
        self.send(bytes([GET_VALUES]), can)
        for p in self.read_packets(0.15):
            if p[0] == GET_VALUES and len(p) >= 59:
                return {
                    "id": p[58],
                    "t_fet": struct.unpack_from(">h", p, 1)[0] / 10,
                    "t_mot": struct.unpack_from(">h", p, 3)[0] / 10,
                    "i_mot": struct.unpack_from(">i", p, 5)[0] / 100,
                    "i_in": struct.unpack_from(">i", p, 9)[0] / 100,
                    "duty": struct.unpack_from(">h", p, 21)[0] / 1000,
                    "erpm": struct.unpack_from(">i", p, 23)[0],
                    "v_in": struct.unpack_from(">h", p, 27)[0] / 10,
                    "tach": struct.unpack_from(">i", p, 45)[0],
                    "fault": p[53],
                }
        return None

    def fw(self, can: int | None) -> str | None:
        self.send(bytes([FW]), can)
        for p in self.read_packets(0.2):
            if p[0] == FW:
                name = p[3:].split(b"\0")[0].decode(errors="replace")
                return f"{p[1]}.{p[2]:02d} hw='{name}'"
        return None


def jog(bus: Bus, can: int | None, erpm: int, secs: float) -> None:
    v0 = bus.values(can)
    if v0 is None:
        print("  no telemetry"); return
    t_end = time.monotonic() + secs
    peak = 0
    while time.monotonic() < t_end:
        bus.send(bytes([SET_RPM]) + struct.pack(">i", erpm), can)
        v = bus.values(can)
        if v:
            peak = max(peak, abs(v["erpm"]))
            last = v
    for _ in range(5):
        bus.send(bytes([SET_CURRENT]) + struct.pack(">i", 0), can)
        time.sleep(0.02)
    time.sleep(0.8)
    v1 = bus.values(can)
    d_tach = v1["tach"] - v0["tach"]
    turns = d_tach / (6 * POLE_PAIRS)
    print(f"  cmd={erpm:+5d} ERPM  meas={last['erpm']:+5d} (peak {peak})  I_mot={last['i_mot']:.2f}A "
          f"duty={last['duty']:.3f}  tach Δ={d_tach:+d} → {turns:+.2f} rev = {turns * 6.2832 * WHEEL_R:+.3f} m  fault={v1['fault']}")


def main() -> None:
    bus = Bus(PORT)
    for name, can in (("local", None), ("can66", 66)):
        print(name, "FW", bus.fw(can), "VALUES", bus.values(can))
    if "--jog" not in sys.argv:
        return
    erpm = 300  # ≈0.13 m/s wheel speed
    for name, can in (("local(id65)", None), ("can(id66)", 66)):
        for sgn in (1, -1):
            print(f"{name} jog {sgn * erpm:+d} ERPM 3s")
            jog(bus, can, sgn * erpm, 3.0)


def one() -> None:
    """Usage: vesc_test.py one <65|66> <erpm> <secs>"""
    _, _, cid, erpm, secs = sys.argv
    bus = Bus(PORT)
    jog(bus, None if cid == "65" else int(cid), int(erpm), float(secs))


if __name__ == "__main__":
    one() if len(sys.argv) > 1 and sys.argv[1] == "one" else main()
