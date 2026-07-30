#!/usr/bin/env python3
"""Finite Pi/Windows UART diagnostic: no heartbeat and no repeated request."""
from __future__ import annotations

import argparse
import struct
import time

SOF = b"\xA5\x5A"
VERSION = 1
MSG_CONTROL_COMMAND = 0x10
MSG_COMMAND_ACK = 0x84
PI_CMD_HELLO = 1


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, sequence: int, timestamp_ms: int, payload: bytes) -> bytes:
    body = struct.pack("<BBHII", VERSION, msg_type, len(payload), sequence, timestamp_ms) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


def build_hello(token: int = 0x12345678) -> bytes:
    payload = struct.pack("<BBhI", PI_CMD_HELLO, 0, 0, token)
    return encode_frame(MSG_CONTROL_COMMAND, 0, 0, payload)


def parse_frames(data: bytes) -> list[tuple[int, bytes]]:
    frames: list[tuple[int, bytes]] = []
    offset = 0
    while True:
        start = data.find(SOF, offset)
        if start < 0 or len(data) - start < 16:
            break
        version, msg_type, payload_len, _seq, _ts = struct.unpack_from("<BBHII", data, start + 2)
        total = 16 + payload_len
        if len(data) - start < total:
            break
        raw = data[start:start + total]
        body = raw[2:-2]
        received_crc = struct.unpack_from("<H", raw, total - 2)[0]
        if version == VERSION and crc16_ccitt_false(body) == received_crc:
            frames.append((msg_type, raw[14:-2]))
        offset = start + total
    return frames


def read_for(port, duration: float) -> bytes:
    deadline = time.monotonic() + duration
    chunks: list[bytes] = []
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            chunks.append(chunk)
    return b"".join(chunks)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="COM5 or /dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="action", required=True)
    capture = sub.add_parser("capture")
    capture.add_argument("--duration", type=float, default=3.0)
    hello = sub.add_parser("hello-once")
    hello.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    import serial  # type: ignore

    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        port.reset_input_buffer()
        if args.action == "capture":
            print("Port open. Reset the TI board now; capturing once.", flush=True)
            data = read_for(port, args.duration)
        else:
            request = build_hello()
            print("TX one HELLO:", request.hex(" "), flush=True)
            port.write(request)
            port.flush()
            data = read_for(port, args.duration)

    print("RX bytes:", len(data))
    print("RX hex  :", data.hex(" "))
    for msg_type, payload in parse_frames(data):
        print(f"FRAME type=0x{msg_type:02X} payload={payload.hex(' ')}")
        if msg_type == MSG_COMMAND_ACK and len(payload) == 12:
            command, result, profile, token, detail = struct.unpack("<BBHIi", payload)
            print(
                f"ACK command={command} result={result} profile={profile} "
                f"token=0x{token:08X} detail={detail}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
