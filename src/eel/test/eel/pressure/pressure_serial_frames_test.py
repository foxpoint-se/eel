import struct

from eel.pressure.pressure_serial_frames import take_float32_le


def test__when_fewer_than_four_bytes__should_keep_all_as_leftover() -> None:
    values, leftover = take_float32_le(b"\x01\x02\x03")
    assert values == []
    assert leftover == b"\x01\x02\x03"


def test__when_exactly_one_float__should_decode_and_leave_empty() -> None:
    payload = struct.pack("<f", 1.5)
    values, leftover = take_float32_le(payload)
    assert values == [1.5]
    assert leftover == b""


def test__when_partial_trailing_bytes__should_keep_them_for_next_read() -> None:
    payload = struct.pack("<f", 2.0) + b"\xaa\xbb"
    values, leftover = take_float32_le(payload)
    assert values == [2.0]
    assert leftover == b"\xaa\xbb"


def test__when_two_floats__should_decode_both() -> None:
    payload = struct.pack("<f", 1.0) + struct.pack("<f", -3.25)
    values, leftover = take_float32_le(payload)
    assert values == [1.0, -3.25]
    assert leftover == b""
