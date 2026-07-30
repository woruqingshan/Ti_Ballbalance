#!/usr/bin/env python3
"""Send a finite latest-only motor target sweep to TI.

Protocol (8 bytes):
    A5 5A | sequence:u8 | flags:u8 | target_offset_mdeg:i16_le | crc16_le
CRC16-CCITT-FALSE covers bytes 2..5 only.

The script never reads from TI, never waits for ACK, never sends heartbeat and
never retries an old command. Each stage repeatedly publishes the current
absolute target so TI can apply latest-value semantics.
"""
from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass

SOF = b"\xA5\x5A"
FLAG_VALID = 0x01
FLAG_DISABLE = 0x02


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_target(sequence: int, target_offset_mdeg: int, flags: int = FLAG_VALID) -> bytes:
    if not -32768 <= target_offset_mdeg <= 32767:
        raise ValueError("target_offset_mdeg must fit int16")
    body = struct.pack("<BBh", sequence & 0xFF, flags & 0xFF, target_offset_mdeg)
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


@dataclass(frozen=True)
class Stage:
    name: str
    target_mdeg: int
    duration_s: float


def publish_stage(port, stage: Stage, rate_hz: float, sequence: int) -> int:
    period_s = 1.0 / rate_hz
    deadline = time.monotonic() + stage.duration_s
    next_send = time.monotonic()
    sent = 0
    print(
        f"stage={stage.name} target={stage.target_mdeg:+d} mdeg "
        f"duration={stage.duration_s:.2f}s rate={rate_hz:.1f}Hz",
        flush=True,
    )

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now < next_send:
            time.sleep(next_send - now)
        frame = encode_target(sequence, stage.target_mdeg)
        port.write(frame)
        sequence = (sequence + 1) & 0xFF
        sent += 1
        next_send += period_s

    port.flush()
    print(f"  sent={sent}", flush=True)
    return sequence


def self_test() -> None:
    frame = encode_target(0, 0)
    assert len(frame) == 8
    assert frame[:2] == SOF
    assert frame[2:6] == bytes.fromhex("00 01 00 00")
    assert crc16_ccitt_false(frame[2:6]) == int.from_bytes(frame[6:8], "little")
    print("self-test: PASS")
    print("zero frame:", frame.hex(" "))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--dwell", type=float, default=2.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--minimum-offset", type=int, default=-500)
    parser.add_argument("--maximum-offset", type=int, default=500)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0
    if args.rate <= 0 or args.dwell <= 0 or args.settle <= 0:
        parser.error("rate, dwell and settle must be positive")
    for value in (args.minimum_offset, args.maximum_offset):
        if not -3000 <= value <= 3000:
            parser.error("offsets must stay inside TI safety range [-3000, 3000] mdeg")

    import serial  # type: ignore

    stages = [
        Stage("origin", 0, args.settle),
        Stage("minimum", args.minimum_offset, args.dwell),
        Stage("maximum", args.maximum_offset, args.dwell),
        Stage("origin", 0, args.dwell),
    ]

    print(
        "No ACK/heartbeat is used. Start TI first and wait for its automatic "
        "move to the -28.1 degree horizontal point.",
        flush=True,
    )
    with serial.Serial(
        args.port,
        baudrate=args.baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0,
        write_timeout=0.5,
    ) as port:
        sequence = 0
        for stage in stages:
            sequence = publish_stage(port, stage, args.rate, sequence)

    print("sweep complete; final target is 0 mdeg (horizontal origin)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
