import struct

from ugv_drivers import vesc_protocol as vp


def _values_payload(controller_id: int, erpm: int, tach: int, v_in: float = 29.0, fault: int = 0) -> bytes:
    p = bytearray(60)
    p[0] = vp.COMM_GET_VALUES
    struct.pack_into(">h", p, 1, 243)            # temp_fet 24.3
    struct.pack_into(">i", p, 5, 27)             # current_motor 0.27
    struct.pack_into(">h", p, 21, 34)            # duty 0.034
    struct.pack_into(">i", p, 23, erpm)
    struct.pack_into(">h", p, 27, int(v_in * 10))
    struct.pack_into(">i", p, 45, tach)
    p[53] = fault
    p[58] = controller_id
    return bytes(p)


def test_crc16_known_vector() -> None:
    # CRC-16/XMODEM of "123456789"
    assert vp.crc16(b"123456789") == 0x31C3


def test_encode_frame_short() -> None:
    frame = vp.encode_frame(bytes([vp.COMM_GET_VALUES]))
    assert frame[0] == 2 and frame[1] == 1 and frame[2] == 4 and frame[-1] == 3
    assert (frame[3] << 8 | frame[4]) == vp.crc16(b"\x04")


def test_encode_frame_long_uses_16bit_length() -> None:
    frame = vp.encode_frame(bytes(300))
    assert frame[0] == 3 and (frame[1] << 8 | frame[2]) == 300


def test_decoder_roundtrip_split_and_garbage() -> None:
    a = vp.encode_frame(b"\x04abc")
    b = vp.encode_frame(bytes(300))
    stream = b"\xff\x00" + a + b"\x55" + b
    dec = vp.FrameDecoder()
    out = []
    for i in range(0, len(stream), 7):          # arbitrary chunking
        out += dec.feed(stream[i:i + 7])
    assert out == [b"\x04abc", bytes(300)]


def test_decoder_rejects_bad_crc_and_resyncs() -> None:
    bad = bytearray(vp.encode_frame(b"\x04xyz"))
    bad[-2] ^= 0xFF
    good = vp.encode_frame(b"\x04ok")
    dec = vp.FrameDecoder()
    assert dec.feed(bytes(bad) + good) == [b"\x04ok"]
    assert dec.crc_errors >= 1


def test_decoder_recovers_from_false_long_header_once_more_data_arrives() -> None:
    dec = vp.FrameDecoder()
    good = vp.encode_frame(b"\x04ok")
    # 0x03 0x01 0x10 looks like a 272-byte frame header; the stream continues afterwards
    out = dec.feed(b"\x03\x01\x10" + good)
    out += dec.feed(good * 60)
    assert out and all(p == b"\x04ok" for p in out)
    assert len(out) >= 60


def test_parse_values() -> None:
    v = vp.parse_values(_values_payload(66, erpm=-300, tach=1234))
    assert v is not None
    assert v.controller_id == 66 and v.erpm == -300 and v.tachometer == 1234
    assert abs(v.v_in - 29.0) < 1e-9 and abs(v.temp_fet - 24.3) < 1e-9 and abs(v.current_motor - 0.27) < 1e-9


def test_parse_values_rejects_short_or_other() -> None:
    assert vp.parse_values(b"\x04" + bytes(10)) is None
    assert vp.parse_values(b"\x00" + bytes(70)) is None


def test_commands_local_and_forward_can() -> None:
    assert vp.cmd_set_rpm(300) == bytes([vp.COMM_SET_RPM]) + struct.pack(">i", 300)
    assert vp.cmd_set_rpm(-300.4, can_id=66) == bytes([vp.COMM_FORWARD_CAN, 66, vp.COMM_SET_RPM]) + struct.pack(">i", -300)
    assert vp.cmd_set_current(1.5) == bytes([vp.COMM_SET_CURRENT]) + struct.pack(">i", 1500)
    assert vp.cmd_set_current_brake(-2.0) == bytes([vp.COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", 2000)
    assert vp.cmd_get_values(66) == bytes([vp.COMM_FORWARD_CAN, 66, vp.COMM_GET_VALUES])
