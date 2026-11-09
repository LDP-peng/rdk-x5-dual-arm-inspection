#!/usr/bin/env python3
"""Read and decode C30D 24-byte serial feedback frames without pyserial."""

import argparse
import os
import select
import struct
import sys
import termios
import time


HEADER = 0x7B
TAIL = 0x7D
FRAME_LEN = 24


def open_serial(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def extract_frames(buffer):
    frames = []
    while len(buffer) >= FRAME_LEN:
        start = buffer.find(bytes((HEADER,)))
        if start < 0:
            buffer.clear()
            break
        if start:
            del buffer[:start]
        if len(buffer) < FRAME_LEN:
            break
        if buffer[FRAME_LEN - 1] != TAIL:
            del buffer[0]
            continue

        frame = bytes(buffer[:FRAME_LEN])
        checksum = 0
        for byte in frame[:22]:
            checksum ^= byte
        if checksum != frame[22]:
            del buffer[0]
            continue

        frames.append(frame)
        del buffer[:FRAME_LEN]
    return frames


def decode(frame):
    fields = struct.unpack('>hhhhhhhhhh', frame[2:22])
    return {
        'flag': frame[1],
        'vx': fields[0],
        'vy': fields[1],
        'wz': fields[2],
        'ax': fields[3],
        'ay': fields[4],
        'az': fields[5],
        'gx': fields[6],
        'gy': fields[7],
        'gz': fields[8],
        'voltage_mv': fields[9],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--count', type=int, default=10)
    args = parser.parse_args()

    try:
        fd = open_serial(args.port)
    except OSError as exc:
        print(f'open {args.port} failed: {exc}', file=sys.stderr)
        return 1

    buffer = bytearray()
    deadline = time.time() + args.duration
    received = 0
    try:
        while time.time() < deadline and received < args.count:
            ready, _, _ = select.select([fd], [], [], 0.1)
            if not ready:
                continue
            try:
                buffer.extend(os.read(fd, 256))
            except BlockingIOError:
                continue

            for frame in extract_frames(buffer):
                values = decode(frame)
                pretty = ' '.join(f'{key}={value}' for key, value in values.items())
                print(f'{frame.hex(" ")} | {pretty}')
                received += 1
                if received >= args.count:
                    break
    finally:
        os.close(fd)

    if received == 0:
        print('no valid 0x7B ... 0x7D frames received', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
