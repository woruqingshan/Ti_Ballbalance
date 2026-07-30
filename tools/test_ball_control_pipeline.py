#!/usr/bin/env python3
"""Finite Fixed-14 tests for TI Mode 18 observer and Mode 19 PD dry-run."""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

from fixed14_protocol import (
    REASON_LOW_CONFIDENCE,
    REASON_NOT_FOUND,
    REASON_OUT_OF_RANGE,
    REASON_REACQUIRING,
    REASON_STALE,
    REASON_VELOCITY_INVALID,
    encode_observation,
    make_flags,
)


@dataclass(frozen=True)
class Stage:
    name: str
    frames: int
    position: int
    velocity: int
    confidence: int
    age_ms: int
    flags: int


def send_stages(port_name: str, baudrate: int, rate_hz: float, stages: list[Stage]) -> None:
    import serial  # type: ignore

    period = 1.0 / rate_hz
    sequence = 0
    print(f"opening {port_name} at {baudrate} 8N1 rate={rate_hz:.1f}Hz")
    with serial.Serial(
        port=port_name,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0,
        write_timeout=0.5,
    ) as port:
        for stage in stages:
            print(
                f"stage={stage.name:24s} frames={stage.frames:3d} "
                f"pos={stage.position:+6d} vel={stage.velocity:+6d} "
                f"flags=0x{stage.flags:02X} conf={stage.confidence:4d} "
                f"age={stage.age_ms:3d}",
                flush=True,
            )
            next_send = time.monotonic()
            for _ in range(stage.frames):
                frame = encode_observation(
                    sequence,
                    stage.flags,
                    stage.position,
                    stage.velocity,
                    stage.confidence,
                    stage.age_ms,
                )
                port.write(frame)
                sequence = (sequence + 1) & 0xFF
                next_send += period
                delay = next_send - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            port.flush()
    print("Finite send complete. Inspect Keil Watch; TI sends no ACK.")


def mode18_stages() -> list[Stage]:
    valid = make_flags(found=True, valid=True)
    return [
        Stage("center_valid", 8, 0, 0, 950, 25, valid),
        Stage("right_positive", 8, 500, 120, 950, 25, valid),
        Stage("left_negative", 8, -500, -120, 950, 25, valid),
        Stage(
            "not_found_reason_1",
            5,
            0,
            0,
            0,
            25,
            make_flags(reason=REASON_NOT_FOUND),
        ),
        Stage(
            "low_conf_reason_2",
            5,
            0,
            0,
            349,
            25,
            make_flags(found=True, reason=REASON_LOW_CONFIDENCE),
        ),
        Stage(
            "range_reason_6",
            5,
            1301,
            0,
            950,
            25,
            make_flags(found=True, reason=REASON_OUT_OF_RANGE),
        ),
        Stage(
            "velocity_reason_9",
            5,
            0,
            5001,
            950,
            25,
            make_flags(found=True, reason=REASON_VELOCITY_INVALID),
        ),
        Stage(
            "stale_reason_5",
            5,
            0,
            0,
            950,
            121,
            make_flags(found=True, reason=REASON_STALE),
        ),
        Stage(
            "reacquiring_reason_10",
            5,
            0,
            0,
            900,
            25,
            make_flags(found=True, reason=REASON_REACQUIRING),
        ),
        Stage("valid_recovery", 8, 0, 0, 950, 25, valid),
    ]


def mode19_stages() -> list[Stage]:
    valid = make_flags(found=True, valid=True)
    return [
        Stage("center_acquire", 5, 0, 0, 950, 25, valid),
        Stage("right_positive", 10, 500, 0, 950, 25, valid),
        Stage("right_positive_velocity", 8, 500, 120, 950, 25, valid),
        Stage("center_return", 8, 0, 0, 950, 25, valid),
        Stage("left_negative", 10, -500, 0, 950, 25, valid),
        Stage("left_negative_velocity", 8, -500, -120, 950, 25, valid),
        Stage(
            "not_found_reason_1",
            5,
            0,
            0,
            0,
            25,
            make_flags(reason=REASON_NOT_FOUND),
        ),
        Stage(
            "reacquiring_reason_10",
            5,
            0,
            0,
            900,
            25,
            make_flags(found=True, reason=REASON_REACQUIRING),
        ),
        Stage("valid_reacquire", 6, 0, 0, 950, 25, valid),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["mode18", "mode19"])
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=40.0)
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")
    stages = mode18_stages() if args.command == "mode18" else mode19_stages()
    send_stages(args.port, args.baudrate, args.rate, stages)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
