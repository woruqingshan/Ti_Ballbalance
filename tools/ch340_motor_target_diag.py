#!/usr/bin/env python3
"""Finite CH340 test for the TI 8-byte motor-target protocol.

The script opens the COM port, waits for TARGET_DIAG_READY after TI reset,
then sends a finite number of target frames. No heartbeat, retry loop or
periodic request is used.

Protocol:
    A5 5A | sequence:u8 | flags:u8 | target_mdeg:i16_le | crc16_le
"""
from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass

SOF = b"\xA5\x5A"
FLAG_VALID = 0x01
FLAG_DISABLE = 0x02
READY_MARKER = b"TARGET_DIAG_READY"


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def encode_target(sequence: int, target_mdeg: int, flags: int = FLAG_VALID) -> bytes:
    if not -32768 <= target_mdeg <= 32767:
        raise ValueError("target_mdeg must fit int16")
    body = struct.pack("<BBh", sequence & 0xFF, flags & 0xFF, target_mdeg)
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


def read_for(port, duration_s: float) -> bytes:
    deadline = time.monotonic() + duration_s
    data = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            data.extend(chunk)
            print(chunk.decode("ascii", errors="replace"), end="", flush=True)
    return bytes(data)


def wait_ready(port, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    print(
        "COM port opened. Reset TI now and wait for automatic move to -28.1 deg.",
        flush=True,
    )
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            data.extend(chunk)
            print(chunk.decode("ascii", errors="replace"), end="", flush=True)
            if READY_MARKER in data:
                return bytes(data)
    raise TimeoutError("TARGET_DIAG_READY was not received")


def send_bytes(port, data: bytes, inter_byte_ms: float) -> None:
    if inter_byte_ms <= 0:
        port.write(data)
        port.flush()
        return

    delay_s = inter_byte_ms / 1000.0
    for byte in data:
        port.write(bytes([byte]))
        port.flush()
        time.sleep(delay_s)


@dataclass(frozen=True)
class Command:
    name: str
    target_mdeg: int
    wait_s: float


def send_command(
    port,
    command: Command,
    sequence: int,
    inter_byte_ms: float,
    reply_s: float,
) -> int:
    frame = encode_target(sequence, command.target_mdeg)
    print(
        f"\nTX {command.name}: target={command.target_mdeg:+d} mdeg "
        f"seq={sequence} frame={frame.hex(' ')}",
        flush=True,
    )
    send_bytes(port, frame, inter_byte_ms)
    read_for(port, reply_s)
    if command.wait_s > reply_s:
        time.sleep(command.wait_s - reply_s)
    return (sequence + 1) & 0xFF


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="CH340 COM port, e.g. COM9")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--inter-byte-ms",
        type=float,
        default=30.0,
        help="Delay between bytes. Start with 30 ms to test the known-good IRQ path.",
    )
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--reply-timeout", type=float, default=1.0)
    parser.add_argument(
        "--skip-ready",
        action="store_true",
        help="Do not wait for TARGET_DIAG_READY; use only when TI is already ready.",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    send = sub.add_parser("send", help="Send one target frame")
    send.add_argument("--target", type=int, required=True)
    send.add_argument("--sequence", type=int, default=1)

    sub.add_parser("sweep", help="Send 0, -500, +500, 0 once each")
    sub.add_parser("full-sweep", help="Send 0, -3000, +3000, 0 once each")

    probe = sub.add_parser("probe-byte", help="Send one raw byte only")
    probe.add_argument("--value", type=lambda value: int(value, 0), default=0x55)

    args = parser.parse_args()

    import serial  # type: ignore

    with serial.Serial(
        args.port,
        baudrate=args.baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.05,
        write_timeout=1.0,
    ) as port:
        try:
            port.dtr = False
            port.rts = False
        except Exception:
            pass

        port.reset_input_buffer()
        port.reset_output_buffer()

        if not args.skip_ready:
            wait_ready(port, args.ready_timeout)

        if args.command == "probe-byte":
            value = args.value & 0xFF
            print(f"\nTX one raw byte: 0x{value:02X}", flush=True)
            send_bytes(port, bytes([value]), args.inter_byte_ms)
            read_for(port, args.reply_timeout)
            return 0

        if args.command == "send":
            if not -3000 <= args.target <= 3000:
                parser.error("target must be inside [-3000, 3000] mdeg")
            send_command(
                port,
                Command("single", args.target, args.reply_timeout),
                args.sequence & 0xFF,
                args.inter_byte_ms,
                args.reply_timeout,
            )
            return 0

        targets = (
            [0, -500, 500, 0]
            if args.command == "sweep"
            else [0, -3000, 3000, 0]
        )
        sequence = 1
        for index, target in enumerate(targets):
            sequence = send_command(
                port,
                Command(f"stage-{index}", target, 2.0),
                sequence,
                args.inter_byte_ms,
                args.reply_timeout,
            )
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
