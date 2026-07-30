#!/usr/bin/env python3
"""Frozen BallObservation Fixed-14 protocol helpers."""
from __future__ import annotations

import struct
from dataclasses import dataclass

SOF = b"\xA5\x5A"
FRAME_SIZE = 14

FLAG_FOUND = 0x01
FLAG_VALID = 0x02
FLAG_PREDICTED = 0x04
FLAG_EMERGENCY = 0x08

REASON_NONE = 0
REASON_NOT_FOUND = 1
REASON_LOW_CONFIDENCE = 2
REASON_OUT_OF_ROI = 3
REASON_POSITION_JUMP = 4
REASON_STALE = 5
REASON_OUT_OF_RANGE = 6
REASON_SOURCE_ERROR = 7
REASON_PIPE_AXIS_INVALID = 8
REASON_VELOCITY_INVALID = 9
REASON_REACQUIRING = 10
REASON_SHUTDOWN = 11


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


def make_flags(
    *,
    found: bool = False,
    valid: bool = False,
    predicted: bool = False,
    emergency: bool = False,
    reason: int = REASON_NONE,
) -> int:
    if not 0 <= reason <= 15:
        raise ValueError("reason must be in [0, 15]")
    return (
        (FLAG_FOUND if found else 0)
        | (FLAG_VALID if valid else 0)
        | (FLAG_PREDICTED if predicted else 0)
        | (FLAG_EMERGENCY if emergency else 0)
        | ((reason & 0x0F) << 4)
    )


def encode_observation(
    sequence: int,
    flags: int,
    position_centi_cm: int,
    velocity_centi_cm_s: int,
    confidence_milli: int,
    vision_age_ms: int,
) -> bytes:
    if not -32768 <= position_centi_cm <= 32767:
        raise ValueError("position must fit int16")
    if not -32768 <= velocity_centi_cm_s <= 32767:
        raise ValueError("velocity must fit int16")
    if not 0 <= confidence_milli <= 1000:
        raise ValueError("confidence must be in [0, 1000]")
    if not 0 <= vision_age_ms <= 65535:
        raise ValueError("vision_age_ms must fit uint16")
    body = struct.pack(
        "<BBhhHH",
        sequence & 0xFF,
        flags & 0xFF,
        position_centi_cm,
        velocity_centi_cm_s,
        confidence_milli,
        vision_age_ms,
    )
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


@dataclass(frozen=True)
class DecodedObservation:
    sequence: int
    flags: int
    position_centi_cm: int
    velocity_centi_cm_s: int
    confidence_milli: int
    vision_age_ms: int
    invalid_reason: int


def decode_observation(frame: bytes) -> DecodedObservation:
    if len(frame) != FRAME_SIZE:
        raise ValueError(f"frame length must be {FRAME_SIZE}")
    if frame[:2] != SOF:
        raise ValueError("bad SOF")
    body = frame[2:12]
    received_crc = int.from_bytes(frame[12:14], "little")
    calculated_crc = crc16_ccitt_false(body)
    if received_crc != calculated_crc:
        raise ValueError(
            f"bad CRC received=0x{received_crc:04X} calculated=0x{calculated_crc:04X}"
        )
    sequence, flags, position, velocity, confidence, age = struct.unpack(
        "<BBhhHH", body
    )
    if confidence > 1000:
        raise ValueError("confidence exceeds 1000")
    return DecodedObservation(
        sequence=sequence,
        flags=flags,
        position_centi_cm=position,
        velocity_centi_cm_s=velocity,
        confidence_milli=confidence,
        vision_age_ms=age,
        invalid_reason=(flags >> 4) & 0x0F,
    )


GOLDEN_VALID = bytes.fromhex(
    "A5 5A 2A 03 F4 01 06 FF 84 03 2D 00 5D 72"
)
GOLDEN_INVALID_PREDICTED = bytes.fromhex(
    "A5 5A FF 14 0C FE D2 04 00 00 78 00 89 68"
)
GOLDEN_EMERGENCY = bytes.fromhex(
    "A5 5A 00 B8 00 00 00 00 00 00 FF FF 79 9E"
)
