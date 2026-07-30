#!/usr/bin/env python3
"""T2 observer and T3 controller-dry-run sender.

Uses the T1 14-byte BallObservation encoder. TI never sends ACK/heartbeat in
Modes 18/19, and this tool opens only the host TX direction.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

from ball_observation_t1_sender import (
    FLAG_EMERGENCY,
    FLAG_FOUND,
    FLAG_PREDICTED,
    FLAG_VALID,
    encode_observation,
    open_serial,
)


@dataclass(frozen=True, slots=True)
class Stage:
    name: str
    frames: int
    position: int
    velocity: int
    flags: int = FLAG_FOUND | FLAG_VALID
    confidence: int = 950
    age_ms: int = 25


def send_stage(port, sequence: int, stage: Stage, rate_hz: float) -> int:
    period_s = 1.0 / rate_hz
    next_send = time.monotonic()
    print(
        f"stage={stage.name:22s} frames={stage.frames:3d} "
        f"pos={stage.position:+6d} vel={stage.velocity:+6d} "
        f"flags=0x{stage.flags:02X} conf={stage.confidence:4d} "
        f"age={stage.age_ms:3d}",
        flush=True,
    )
    for _ in range(stage.frames):
        now = time.monotonic()
        if now < next_send:
            time.sleep(next_send - now)
        frame = encode_observation(
            sequence,
            flags=stage.flags,
            position_centi_cm=stage.position,
            velocity_centi_cm_s=stage.velocity,
            confidence_milli=stage.confidence,
            vision_age_ms=stage.age_ms,
        )
        port.write(frame)
        sequence = (sequence + 1) & 0xFF
        next_send += period_s
    port.flush()
    return sequence


def run_observer_demo(port_name: str, baudrate: int, rate_hz: float) -> None:
    stages = [
        Stage("center_valid", 8, 0, 0),
        Stage("right_tail_positive", 8, 500, 120),
        Stage("left_negative", 8, -500, -120),
        Stage("low_confidence", 5, 0, 0, confidence=349),
        Stage(
            "predicted_rejected",
            5,
            100,
            20,
            flags=FLAG_VALID | FLAG_PREDICTED,
            confidence=800,
        ),
        Stage("position_out_of_range", 5, 1301, 0),
        Stage("velocity_out_of_range", 5, 0, 5001),
        Stage("source_age_stale", 5, 0, 0, age_ms=121),
        Stage("valid_recovery", 8, 0, 0),
    ]

    print(f"observer-demo port={port_name} baud={baudrate} rate={rate_hz:.1f}Hz")
    with open_serial(port_name, baudrate) as port:
        sequence = 0
        for stage in stages:
            sequence = send_stage(port, sequence, stage, rate_hz)

    print("observer-demo complete")
    print("Expected Mode 18: motor stationary; final observer control_valid=1")
    print("During stages invalid_reason should cover low confidence, predicted, range and stale")


def run_dry_run_demo(port_name: str, baudrate: int, rate_hz: float) -> None:
    stages = [
        Stage("center_acquire", 5, 0, 0),
        Stage("right_positive", 10, 500, 0),
        Stage("right_with_positive_v", 8, 500, 120),
        Stage("center_return", 8, 0, 0),
        Stage("left_negative", 10, -500, 0),
        Stage("left_with_negative_v", 8, -500, -120),
        Stage("invalid_reset", 5, 0, 0, flags=0, confidence=0),
        Stage("valid_reacquire", 6, 0, 0),
    ]

    print(f"dry-run-demo port={port_name} baud={baudrate} rate={rate_hz:.1f}Hz")
    with open_serial(port_name, baudrate) as port:
        sequence = 0
        for stage in stages:
            sequence = send_stage(port, sequence, stage, rate_hz)

    print("dry-run-demo complete")
    print("Expected Mode 19 with motor_sign=+1:")
    print("  right/vehicle-tail positive position -> negative error/output")
    print("  left negative position -> positive error/output")
    print("  target_offset_mdeg is clamped to +/-200 and slew-limited")
    print("  invalid_reset forces target_offset_mdeg=0")
    print("  motor must remain completely stationary")


def run_timeout_demo(port_name: str, baudrate: int, rate_hz: float) -> None:
    print(f"timeout-demo port={port_name} baud={baudrate} rate={rate_hz:.1f}Hz")
    with open_serial(port_name, baudrate) as port:
        sequence = 0
        sequence = send_stage(port, sequence, Stage("initial_active", 6, 300, 0), rate_hz)
        print("pause 0.20 s: expect STALE and output zero", flush=True)
        time.sleep(0.20)
        sequence = send_stage(port, sequence, Stage("reacquire", 6, -300, 0), rate_hz)
        print("pause 0.70 s: expect LINK_TIMEOUT and output zero", flush=True)
        time.sleep(0.70)
    print("timeout-demo complete; final Mode 19 state should be LINK_TIMEOUT")
    print("motor must remain completely stationary")


def run_emergency_once(port_name: str, baudrate: int) -> None:
    frame = encode_observation(
        0,
        flags=FLAG_EMERGENCY,
        position_centi_cm=0,
        velocity_centi_cm_s=0,
        confidence_milli=0,
        vision_age_ms=10,
    )
    with open_serial(port_name, baudrate) as port:
        port.write(frame)
        port.flush()
    print("emergency frame sent")
    print("Mode 19 should latch EMERGENCY until TI Reset; motor remains stationary")


def run_stream(
    port_name: str,
    baudrate: int,
    rate_hz: float,
    duration_s: float,
    amplitude_centi_cm: int,
    period_s: float,
) -> None:
    if rate_hz <= 0 or duration_s <= 0 or period_s <= 0:
        raise ValueError("rate, duration and period must be positive")
    if not 0 <= amplitude_centi_cm <= 1300:
        raise ValueError("amplitude must be in 0..1300 centi-cm")

    period_tx = 1.0 / rate_hz
    angular = 2.0 * math.pi / period_s
    start = time.monotonic()
    deadline = start + duration_s
    next_send = start
    sequence = 0
    sent = 0

    print(
        f"stream port={port_name} rate={rate_hz:.1f}Hz duration={duration_s:.1f}s "
        f"amplitude={amplitude_centi_cm / 100:.2f}cm period={period_s:.2f}s",
        flush=True,
    )
    with open_serial(port_name, baudrate) as port:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_send:
                time.sleep(next_send - now)
                now = time.monotonic()
            elapsed = now - start
            position = int(round(amplitude_centi_cm * math.sin(angular * elapsed)))
            velocity = int(round(amplitude_centi_cm * angular * math.cos(angular * elapsed)))
            velocity = max(-32768, min(32767, velocity))
            port.write(
                encode_observation(
                    sequence,
                    flags=FLAG_FOUND | FLAG_VALID,
                    position_centi_cm=position,
                    velocity_centi_cm_s=velocity,
                    confidence_milli=950,
                    vision_age_ms=25,
                )
            )
            sequence = (sequence + 1) & 0xFF
            sent += 1
            next_send += period_tx
        port.flush()
    print(f"stream complete: sent={sent}")
    print("Expected: protocol errors/gaps/overflow remain zero; motor stationary")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    for name in ("observer-demo", "dry-run-demo", "timeout-demo"):
        item = sub.add_parser(name)
        item.add_argument("--port", required=True)
        item.add_argument("--baudrate", type=int, default=115200)
        item.add_argument("--rate", type=float, default=40.0)

    emergency = sub.add_parser("emergency-once")
    emergency.add_argument("--port", required=True)
    emergency.add_argument("--baudrate", type=int, default=115200)

    stream = sub.add_parser("stream")
    stream.add_argument("--port", required=True)
    stream.add_argument("--baudrate", type=int, default=115200)
    stream.add_argument("--rate", type=float, default=40.0)
    stream.add_argument("--duration", type=float, default=60.0)
    stream.add_argument("--amplitude", type=int, default=500)
    stream.add_argument("--period", type=float, default=4.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "observer-demo":
            run_observer_demo(args.port, args.baudrate, args.rate)
        elif args.command == "dry-run-demo":
            run_dry_run_demo(args.port, args.baudrate, args.rate)
        elif args.command == "timeout-demo":
            run_timeout_demo(args.port, args.baudrate, args.rate)
        elif args.command == "emergency-once":
            run_emergency_once(args.port, args.baudrate)
        elif args.command == "stream":
            run_stream(
                args.port,
                args.baudrate,
                args.rate,
                args.duration,
                args.amplitude,
                args.period,
            )
        else:
            raise AssertionError(args.command)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
