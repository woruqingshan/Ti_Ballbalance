#!/usr/bin/env python3
"""Finite host tests for TI formal latest-target stream mode 20.

No ACK, heartbeat, retransmission queue, or TI response is used.
Works with Raspberry Pi /dev/serial0 or a Windows CH340 COM port.
"""
from __future__ import annotations

import argparse
import struct
import time

SOF = b"\xA5\x5A"
FLAG_VALID = 0x01
FLAG_DISABLE = 0x02


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) & 0xFFFF) if crc & 0x8000 else ((crc << 1) & 0xFFFF)
    return crc


def encode_target(sequence: int, target_mdeg: int, *, valid: bool = True, disable: bool = False) -> bytes:
    if not -32768 <= target_mdeg <= 32767:
        raise ValueError("target_mdeg must fit int16")
    flags = (FLAG_VALID if valid else 0) | (FLAG_DISABLE if disable else 0)
    body = struct.pack("<BBh", sequence & 0xFF, flags, target_mdeg)
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


def publish(port, sequence: int, target: int, duration: float, rate: float, *, valid: bool = True, disable: bool = False) -> int:
    deadline = time.monotonic() + duration
    period = 1.0 / rate
    next_send = time.monotonic()
    count = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now < next_send:
            time.sleep(next_send - now)
        frame = encode_target(sequence, target, valid=valid, disable=disable)
        port.write(frame)
        sequence = (sequence + 1) & 0xFF
        count += 1
        next_send += period
    port.flush()
    print(f"target={target:+d} valid={valid} disable={disable} sent={count}")
    return sequence


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("self-test", "sweep", "latest", "timeout", "disable"))
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--dwell", type=float, default=2.0)
    parser.add_argument("--minimum", type=int, default=-500)
    parser.add_argument("--maximum", type=int, default=500)
    args = parser.parse_args()

    if args.command == "self-test":
        frame = encode_target(0, 0)
        assert frame.hex(" ") == "a5 5a 00 01 00 00 f0 b3"
        print("self-test: PASS")
        print(frame.hex(" "))
        return 0

    if args.rate <= 0 or args.dwell <= 0:
        parser.error("rate and dwell must be positive")
    if not -3000 <= args.minimum <= 3000 or not -3000 <= args.maximum <= 3000:
        parser.error("targets must stay inside [-3000, 3000] mdeg")

    import serial  # type: ignore

    with serial.Serial(
        args.port,
        args.baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0,
        write_timeout=0.5,
    ) as port:
        sequence = 0
        if args.command == "sweep":
            for target, duration in (
                (0, 1.0),
                (args.minimum, args.dwell),
                (args.maximum, args.dwell),
                (0, args.dwell),
            ):
                sequence = publish(port, sequence, target, duration, args.rate)
            print("Expected: horizontal -> minimum -> maximum -> horizontal")
        elif args.command == "latest":
            sequence = publish(port, sequence, args.minimum, 0.06, args.rate)
            sequence = publish(port, sequence, args.maximum, 0.06, args.rate)
            publish(port, sequence, 0, 0.20, args.rate)
            print("Expected: pending targets are overwritten; final target is horizontal.")
            time.sleep(1.2)
        elif args.command == "timeout":
            sequence = publish(port, sequence, args.minimum, 1.0, args.rate)
            print("TX stopped. TI should return to horizontal after about 500 ms.")
            time.sleep(1.5)
        else:
            sequence = publish(port, sequence, 0, 0.5, args.rate)
            publish(port, sequence, 0, 0.2, args.rate, valid=False, disable=True)
            print("Expected: motor disabled. TI reset is required before the next run.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
