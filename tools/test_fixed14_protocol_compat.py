#!/usr/bin/env python3
"""Test Frozen Fixed-14 golden vectors locally or send them to TI Mode 18."""
from __future__ import annotations

import argparse
import time

from fixed14_protocol import (
    FLAG_EMERGENCY,
    FLAG_FOUND,
    FLAG_PREDICTED,
    FLAG_VALID,
    GOLDEN_EMERGENCY,
    GOLDEN_INVALID_PREDICTED,
    GOLDEN_VALID,
    REASON_NONE,
    REASON_NOT_FOUND,
    REASON_SHUTDOWN,
    decode_observation,
    encode_observation,
    make_flags,
)

CASES = {
    "valid": GOLDEN_VALID,
    "invalid-predicted": GOLDEN_INVALID_PREDICTED,
    "emergency": GOLDEN_EMERGENCY,
}


def self_test() -> None:
    expected_valid = encode_observation(
        42,
        make_flags(found=True, valid=True, reason=REASON_NONE),
        500,
        -250,
        900,
        45,
    )
    expected_invalid = encode_observation(
        255,
        make_flags(predicted=True, reason=REASON_NOT_FOUND),
        -500,
        1234,
        0,
        120,
    )
    expected_emergency = encode_observation(
        0,
        make_flags(emergency=True, reason=REASON_SHUTDOWN),
        0,
        0,
        0,
        65535,
    )
    assert expected_valid == GOLDEN_VALID
    assert expected_invalid == GOLDEN_INVALID_PREDICTED
    assert expected_emergency == GOLDEN_EMERGENCY

    decoded = [decode_observation(frame) for frame in CASES.values()]
    assert decoded[0].flags == FLAG_FOUND | FLAG_VALID
    assert decoded[0].invalid_reason == REASON_NONE
    assert decoded[1].flags == FLAG_PREDICTED | (REASON_NOT_FOUND << 4)
    assert decoded[1].invalid_reason == REASON_NOT_FOUND
    assert decoded[2].flags == FLAG_EMERGENCY | (REASON_SHUTDOWN << 4)
    assert decoded[2].invalid_reason == REASON_SHUTDOWN

    print("Fixed-14 frozen protocol self-test: PASS")
    for name, frame in CASES.items():
        print(f"{name:20s} {frame.hex(' ')}")


def send_case(port_name: str, baudrate: int, case: str, repeat: int) -> None:
    import serial  # type: ignore

    frame = CASES[case]
    decoded = decode_observation(frame)
    print(
        f"opening {port_name} at {baudrate} 8N1; case={case}; repeat={repeat}",
        flush=True,
    )
    with serial.Serial(
        port=port_name,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0,
        write_timeout=0.5,
    ) as port:
        for index in range(repeat):
            port.write(frame)
            port.flush()
            print(f"send[{index}] {frame.hex(' ')}", flush=True)
            time.sleep(0.05)
    print(
        "decoded: "
        f"seq={decoded.sequence} flags=0x{decoded.flags:02X} "
        f"reason={decoded.invalid_reason} pos={decoded.position_centi_cm} "
        f"vel={decoded.velocity_centi_cm_s} conf={decoded.confidence_milli} "
        f"age={decoded.vision_age_ms}"
    )
    print("Send complete. Inspect Keil Watch; TI sends no ACK.")


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-test")
    send = sub.add_parser("send-case")
    send.add_argument("--port", required=True)
    send.add_argument("--baudrate", type=int, default=115200)
    send.add_argument("--case", choices=sorted(CASES), required=True)
    send.add_argument("--repeat", type=int, default=1)
    args = parser.parse_args()

    if args.command == "self-test":
        self_test()
    else:
        send_case(args.port, args.baudrate, args.case, args.repeat)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
