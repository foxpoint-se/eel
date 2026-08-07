import struct

FLOAT32_SIZE = 4


def take_float32_le(buffer: bytes) -> tuple[list[float], bytes]:
    """Pull complete little-endian float32 frames; return values and leftover bytes."""
    values: list[float] = []
    offset = 0
    while offset + FLOAT32_SIZE <= len(buffer):
        chunk = buffer[offset : offset + FLOAT32_SIZE]
        (value,) = struct.unpack("<f", chunk)
        values.append(value)
        offset += FLOAT32_SIZE
    return values, buffer[offset:]
