#!/usr/bin/env python3
"""Finite and soak-test sender for TI BallObservation T1 receive mode.

Frame (14 bytes):
    A5 5A | sequence:u8 | flags:u8 | position:i16_le | velocity:i16_le |
    confidence:u16_le | vision_age_ms:u16_le | crc16:u16_le

CRC16/CCITT-FALSE covers bytes 2..11. This tool never waits for ACK and never
reads from TI. It is intended for APP_MODE_BALL_OBSERVATION_RX_DIAG (Mode 18).
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass

SOF = b"\xA5\x5A"
FLAG_FOUND = 0x01
FLAG_VALID = 0x02
FLAG_PREDICTED = 0x04
FLAG_EMERGENCY = 0x08
KNOWN_FLAGS = FLAG_FOUND | FLAG_VALID | FLAG_PREDICTED | FLAG_EMERGENCY
FRAME_SIZE = 14


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_observation(
    sequence: int,
    *,
    flags: int,
    position_centi_cm: int,
    velocity_centi_cm_s: int,
    confidence_milli: int,
    vision_age_ms: int,
) -> bytes:
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must be in 0..255")
    if not 0 <= flags <= 0xFF:
        raise ValueError("flags must fit uint8")
    if not -32768 <= position_centi_cm <= 32767:
        raise ValueError("position must fit int16")
    if not -32768 <= velocity_centi_cm_s <= 32767:
        raise ValueError("velocity must fit int16")
    if not 0 <= confidence_milli <= 0xFFFF:
        raise ValueError("confidence must fit uint16")
    if not 0 <= vision_age_ms <= 0xFFFF:
        raise ValueError("vision_age_ms must fit uint16")

    body = struct.pack(
        "<BBhhHH",
        sequence,
        flags,
        position_centi_cm,
        velocity_centi_cm_s,
        confidence_milli,
        vision_age_ms,
    )
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


@dataclass(frozen=True, slots=True)
class DemoFrame:
    name: str
    frame: bytes


def self_test() -> None:
    frame = encode_observation(
        42,
        flags=FLAG_FOUND | FLAG_VALID,
        position_centi_cm=325,
        velocity_centi_cm_s=-840,
        confidence_milli=950,
        vision_age_ms=27,
    )
    assert len(frame) == FRAME_SIZE
    assert frame[:2] == SOF
    assert frame[2:12] == struct.pack("<BBhhHH", 42, 3, 325, -840, 950, 27)
    assert int.from_bytes(frame[12:14], "little") == crc16_ccitt_false(frame[2:12])
    print("BallObservation Python self-test: PASS")
    print("golden frame:", frame.hex(" "))


def build_demo_frames() -> list[DemoFrame]:
    frames: list[DemoFrame] = []

    def add(
        name: str,
        seq: int,
        flags: int,
        pos: int,
        vel: int,
        conf: int = 950,
        age: int = 25,
    ) -> None:
        frames.append(
            DemoFrame(
                name,
                encode_observation(
                    seq,
                    flags=flags,
                    position_centi_cm=pos,
                    velocity_centi_cm_s=vel,
                    confidence_milli=conf,
                    vision_age_ms=age,
                ),
            )
        )

    add("zero_valid", 0, FLAG_FOUND | FLAG_VALID, 0, 0)
    add("right_positive", 1, FLAG_FOUND | FLAG_VALID, 500, 120)
    add("left_negative", 2, FLAG_FOUND | FLAG_VALID, -500, -120)
    add("invalid_observation", 3, 0, 0, 0, 0, 35)
    add("predicted", 4, FLAG_VALID | FLAG_PREDICTED, 120, 40, 700, 55)
    add("duplicate_seq_ignored", 4, FLAG_FOUND | FLAG_VALID, 999, 999)
    add("sequence_gap_to_7", 7, FLAG_FOUND | FLAG_VALID, 250, -30)

    bad_crc = bytearray(
        encode_observation(
            8,
            flags=FLAG_FOUND | FLAG_VALID,
            position_centi_cm=100,
            velocity_centi_cm_s=0,
            confidence_milli=900,
            vision_age_ms=20,
        )
    )
    bad_crc[12] ^= 0x01
    frames.append(DemoFrame("bad_crc_rejected", bytes(bad_crc)))

    add("unknown_flags_rejected", 8, 0x80, 100, 0)
    add("confidence_1001_rejected", 8, FLAG_FOUND | FLAG_VALID, 100, 0, 1001, 20)
    add("emergency_recorded", 8, FLAG_EMERGENCY, 0, 0, 0, 15)
    return frames


def open_serial(port_name: str, baudrate: int):
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required: python -m pip install pyserial"
        ) from exc

    return serial.Serial(
        port=port_name,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0,
        write_timeout=0.5,
    )


def run_demo(port_name: str, baudrate: int, gap_ms: float) -> None:
    frames = build_demo_frames()
    print(f"opening {port_name} at {baudrate} 8N1", flush=True)
    with open_serial(port_name, baudrate) as port:
        for item in frames:
            port.write(item.frame)
            port.flush()
            print(f"{item.name:28s} {item.frame.hex(' ')}", flush=True)
            time.sleep(max(gap_ms, 0.0) / 1000.0)

    print("\nDemo complete. Expected Keil Watch totals:")
    print("  rx_bytes=154, completed_frames=11, frames_ok=8")
    print("  crc_errors=1, flag_errors=1, confidence_errors=1")
    print("  duplicate_frames=1, sequence_gaps=2, latest_version=7")
    print("  latest_sequence=8, latest_flags=0x08, emergency_frames=1")
    print("  motor must remain completely stationary")


def run_soak(
    port_name: str,
    baudrate: int,
    duration_s: float,
    rate_hz: float,
) -> None:
    if duration_s <= 0 or rate_hz <= 0:
        raise ValueError("duration and rate must be positive")

    period_s = 1.0 / rate_hz
    deadline = time.monotonic() + duration_s
    next_send = time.monotonic()
    sequence = 0
    sent = 0

    print(
        f"soak port={port_name} baud={baudrate} duration={duration_s:.1f}s "
        f"rate={rate_hz:.1f}Hz",
        flush=True,
    )
    with open_serial(port_name, baudrate) as port:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_send:
                time.sleep(next_send - now)
            phase = sent / max(rate_hz, 1.0)
            position = int(round(500.0 * math.sin(2.0 * math.pi * phase / 4.0)))
            velocity = int(round(500.0 * (2.0 * math.pi / 4.0) * math.cos(2.0 * math.pi * phase / 4.0)))
            frame = encode_observation(
                sequence,
                flags=FLAG_FOUND | FLAG_VALID,
                position_centi_cm=max(-32768, min(32767, position)),
                velocity_centi_cm_s=max(-32768, min(32767, velocity)),
                confidence_milli=950,
                vision_age_ms=25,
            )
            port.write(frame)
            sequence = (sequence + 1) & 0xFF
            sent += 1
            next_send += period_s
        port.flush()

    print(f"soak complete: sent={sent}")
    print("Expected: frames_ok and latest_version increase with sent; errors/gaps/overflow stay 0")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("self-test")

    demo = subparsers.add_parser("demo")
    demo.add_argument("--port", required=True)
    demo.add_argument("--baudrate", type=int, default=115200)
    demo.add_argument("--gap-ms", type=float, default=50.0)

    soak = subparsers.add_parser("soak")
    soak.add_argument("--port", required=True)
    soak.add_argument("--baudrate", type=int, default=115200)
    soak.add_argument("--duration", type=float, default=600.0)
    soak.add_argument("--rate", type=float, default=40.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "self-test":
            self_test()
        elif args.command == "demo":
            run_demo(args.port, args.baudrate, args.gap_ms)
        elif args.command == "soak":
            run_soak(args.port, args.baudrate, args.duration, args.rate)
        else:
            raise AssertionError(f"unsupported command: {args.command}")
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
