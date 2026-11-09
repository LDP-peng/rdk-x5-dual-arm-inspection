"""Binary protocol helpers for the WHEELTEC C30D base controller."""

import struct
from dataclasses import dataclass


HEADER = 0x7B
TAIL = 0x7D
FEEDBACK_LEN = 24
COMMAND_LEN = 11


class ProtocolError(ValueError):
    pass


def bcc(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


@dataclass(frozen=True)
class C30dFeedback:
    flag: int
    vx_mm_s: int
    vy_mm_s: int
    wz_mrad_s: int
    ax_raw: int
    ay_raw: int
    az_raw: int
    gx_raw: int
    gy_raw: int
    gz_raw: int
    voltage_mv: int


def decode_feedback(frame):
    if len(frame) != FEEDBACK_LEN:
        raise ProtocolError(f'expected {FEEDBACK_LEN} bytes, got {len(frame)}')
    if frame[0] != HEADER or frame[-1] != TAIL:
        raise ProtocolError('invalid feedback frame markers')
    if bcc(frame[:22]) != frame[22]:
        raise ProtocolError('invalid feedback checksum')

    values = struct.unpack('>hhhhhhhhhh', frame[2:22])
    return C30dFeedback(frame[1], *values)


def build_command(vx_mm_s, wz_mrad_s, vy_mm_s=0):
    values = [int(round(vx_mm_s)), int(round(vy_mm_s)), int(round(wz_mrad_s))]
    values = [max(-32768, min(32767, value)) for value in values]

    frame = bytearray((HEADER, 0, 0))
    frame.extend(struct.pack('>hhh', *values))
    frame.append(bcc(frame))
    frame.append(TAIL)

    if len(frame) != COMMAND_LEN:
        raise ProtocolError(f'invalid command length: {len(frame)}')
    return bytes(frame)


def extract_frames(buffer):
    frames = []

    while len(buffer) >= FEEDBACK_LEN:
        start = buffer.find(bytes((HEADER,)))
        if start < 0:
            buffer.clear()
            break
        if start:
            del buffer[:start]
        if len(buffer) < FEEDBACK_LEN:
            break
        if buffer[FEEDBACK_LEN - 1] != TAIL:
            del buffer[0]
            continue

        frame = bytes(buffer[:FEEDBACK_LEN])
        if bcc(frame[:22]) != frame[22]:
            del buffer[0]
            continue

        frames.append(frame)
        del buffer[:FEEDBACK_LEN]

    return frames
