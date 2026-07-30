#!/usr/bin/env python3
"""Finite and safe Fixed-14 tests for TI Mode 21 live motor output."""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

from fixed14_protocol import (
    REASON_NOT_FOUND,
    REASON_SHUTDOWN,
    encode_observation,
    make_flags,
)


@dataclass(frozen=True)
class Stage:
    name: str
    duration_s: float
    position: int
    velocity: int
    confidence: int
    age_ms: int
    flags: int


def run_stages(port_name: str, baudrate: int, rate_hz: float, stages: list[Stage]) -> None:
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
                f"stage={stage.name:22s} duration={stage.duration_s:.2f}s "
                f"pos={stage.position:+5d} vel={stage.velocity:+5d} "
                f"flags=0x{stage.flags:02X}",
                flush=True,
            )
            deadline = time.monotonic() + stage.duration_s
            next_send = time.monotonic()
            while time.monotonic() < deadline:
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
    print("Finite live test complete; TI sends no ACK.")


def motion_demo() -> list[Stage]:
    valid = make_flags(found=True, valid=True)
    return [
        Stage("center_acquire", 1.0, 0, 0, 950, 25, valid),
        Stage("right_plus_2cm", 2.0, 200, 0, 950, 25, valid),
        Stage("center", 1.5, 0, 0, 950, 25, valid),
        Stage("left_minus_2cm", 2.0, -200, 0, 950, 25, valid),
        Stage("final_center", 2.0, 0, 0, 950, 25, valid),
    ]


def invalid_demo() -> list[Stage]:
    valid = make_flags(found=True, valid=True)
    return [
        Stage("center_acquire", 1.0, 0, 0, 950, 25, valid),
        Stage("right_plus_2cm", 1.5, 200, 0, 950, 25, valid),
        Stage(
            "not_found_invalid",
            1.0,
            0,
            0,
            0,
            25,
            make_flags(reason=REASON_NOT_FOUND),
        ),
        Stage("valid_recovery", 1.5, 0, 0, 950, 25, valid),
    ]


def emergency_demo() -> list[Stage]:
    valid = make_flags(found=True, valid=True)
    return [
        Stage("center_acquire", 1.0, 0, 0, 950, 25, valid),
        Stage(
            "emergency_shutdown",
            0.20,
            0,
            0,
            0,
            65535,
            make_flags(emergency=True, reason=REASON_SHUTDOWN),
        ),
    ]


def timeout_demo(port_name: str, baudrate: int, rate_hz: float) -> None:
    import serial  # type: ignore

    valid = make_flags(found=True, valid=True)
    sequence = 0
    period = 1.0 / rate_hz
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
        def publish(seconds: float, position: int = 0) -> None:
            nonlocal sequence
            deadline = time.monotonic() + seconds
            next_send = time.monotonic()
            while time.monotonic() < deadline:
                port.write(
                    encode_observation(
                        sequence, valid, position, 0, 950, 25
                    )
                )
                sequence = (sequence + 1) & 0xFF
                next_send += period
                delay = next_send - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            port.flush()

        print("stage=center_acquire 1.0s")
        publish(1.0)
        print("pause=0.20s -> expect STALE and target 0")
        time.sleep(0.20)
        print("stage=recover 1.0s -> expect ACTIVE")
        publish(1.0)
        print("pause=0.70s -> expect LINK_TIMEOUT and motor disabled")
        time.sleep(0.70)
        print("stage=recover_after_timeout 1.5s -> expect re-enable after 3 valid frames")
        publish(1.5)
    print("Timeout demo complete.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command",
        choices=["motion-demo", "invalid-demo", "timeout-demo", "emergency-once"],
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=40.0)
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")

    if args.command == "timeout-demo":
        timeout_demo(args.port, args.baudrate, args.rate)
    else:
        stages = {
            "motion-demo": motion_demo,
            "invalid-demo": invalid_demo,
            "emergency-once": emergency_demo,
        }[args.command]()
        run_stages(args.port, args.baudrate, args.rate, stages)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
