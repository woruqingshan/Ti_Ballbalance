#!/usr/bin/env python3
"""Phase 5/6/7 Pi-TI protocol exerciser.

Works on Windows (COMx) and Raspberry Pi (/dev/serial0, /dev/ttyAMA0, ...).
Requires pyserial for real serial use. `self-test` requires only Python.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

SOF = b"\xA5\x5A"
VERSION = 1
MAX_PAYLOAD = 64

MSG_BALL_STATE = 0x01
MSG_PI_STATUS = 0x02
MSG_CONTROL_COMMAND = 0x10
MSG_HEARTBEAT = 0x83
MSG_COMMAND_ACK = 0x84
MSG_MOTOR_STATUS = 0x85
MSG_LINK_STATS = 0x86
MSG_BALL_PREVIEW = 0x87

PI_CMD_HELLO = 1
PI_CMD_GET_STATUS = 2
PI_CMD_STARTUP_HORIZONTAL = 3
PI_CMD_SET_OFFSET_MDEG = 4
PI_CMD_RETURN_HORIZONTAL = 5
PI_CMD_ENABLE_MOTOR = 6
PI_CMD_DISABLE_MOTOR = 7
PI_CMD_EMERGENCY_DISABLE = 8
PI_CMD_ARM_PREVIEW = 9
PI_CMD_DISARM_PREVIEW = 10
PI_CMD_SET_BALL_TARGET_CENTI_CM = 11
PI_CMD_CLEAR_LINK_STATS = 12

ACK_NAMES = {
    0: "OK",
    1: "ACCEPTED",
    2: "BAD_COMMAND",
    3: "BAD_STATE",
    4: "OUT_OF_RANGE",
    5: "BUSY",
    6: "DISABLED",
    7: "UNSUPPORTED",
    8: "DUPLICATE",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, sequence: int, timestamp_ms: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = struct.pack("<BBHII", VERSION, msg_type, len(payload), sequence & 0xFFFFFFFF, timestamp_ms & 0xFFFFFFFF) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


@dataclass
class Frame:
    msg_type: int
    sequence: int
    timestamp_ms: int
    payload: bytes


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.crc_errors = 0
        self.length_errors = 0
        self.version_errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buf.extend(data)
        frames: list[Frame] = []
        while True:
            start = self.buf.find(SOF)
            if start < 0:
                self.buf[:] = self.buf[-1:] if self.buf[-1:] == SOF[:1] else b""
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 16:
                break
            version, msg_type, payload_len, sequence, timestamp_ms = struct.unpack_from("<BBHII", self.buf, 2)
            if payload_len > MAX_PAYLOAD:
                self.length_errors += 1
                del self.buf[0]
                continue
            total = 16 + payload_len
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            body = raw[2:-2]
            received_crc = struct.unpack_from("<H", raw, total - 2)[0]
            if version != VERSION:
                self.version_errors += 1
                continue
            if crc16_ccitt_false(body) != received_crc:
                self.crc_errors += 1
                continue
            frames.append(Frame(msg_type, sequence, timestamp_ms, raw[14:-2]))
        return frames


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, value))


def build_command(command: int, token: int, value: int = 0, argument: int = 0) -> bytes:
    return struct.pack("<BBhI", command, argument, clamp_i16(value), token & 0xFFFFFFFF)


def build_ball_state(frame_id: int, position_cm: float, velocity_cm_s: float,
                     confidence: float = 0.95, latency_ms: int = 25,
                     found: bool = True, predicted: bool = False,
                     valid: bool = True, invalid_reason: int = 0) -> bytes:
    flags = (1 if found else 0) | (2 if predicted else 0) | (4 if valid else 0)
    return struct.pack(
        "<IhhHHBBH",
        frame_id & 0xFFFFFFFF,
        clamp_i16(round(position_cm * 100.0)),
        clamp_i16(round(velocity_cm_s * 100.0)),
        max(0, min(1000, round(confidence * 1000.0))),
        max(0, min(65535, latency_ms)),
        flags,
        invalid_reason & 0xFF,
        0,
    )


def decode_frame(frame: Frame) -> str:
    p = frame.payload
    if frame.msg_type == MSG_COMMAND_ACK and len(p) == 12:
        command, result, profile, token, detail = struct.unpack("<BBHIi", p)
        return f"ACK cmd={command} result={ACK_NAMES.get(result, result)} profile={profile} token={token} detail={detail}"
    if frame.msg_type == MSG_MOTOR_STATUS and len(p) == 32:
        mode, flags, control_state, last_ack = struct.unpack_from("<BBBB", p, 0)
        uptime, age = struct.unpack_from("<II", p, 4)
        target_mdeg, commanded_mdeg, target_pulse, commanded_pulse, faults = struct.unpack_from("<iiiiI", p, 12)
        return (f"MOTOR mode={mode} flags=0x{flags:02X} state={control_state} ack={ACK_NAMES.get(last_ack,last_ack)} "
                f"uptime={uptime}ms rx_age={age}ms target={target_mdeg/1000:.3f}deg "
                f"cmd={commanded_mdeg/1000:.3f}deg pulse={target_pulse}/{commanded_pulse} faults=0x{faults:08X}")
    if frame.msg_type == MSG_LINK_STATS and len(p) == 32:
        vals = struct.unpack("<8I", p)
        return ("LINK rx_frames=%d rx_gaps=%d ball=%d ball_gaps=%d crc=%d len=%d ver=%d overflow=%d" % vals)
    if frame.msg_type == MSG_BALL_PREVIEW and len(p) == 32:
        flags, invalid_reason, profile = struct.unpack_from("<BBH", p, 0)
        frame_id, age = struct.unpack_from("<II", p, 4)
        position, velocity, raw, limited = struct.unpack_from("<hhhh", p, 12)
        offset, physical, gaps = struct.unpack_from("<iiI", p, 20)
        return (f"PREVIEW flags=0x{flags:02X} reason={invalid_reason} profile={profile} frame={frame_id} age={age}ms "
                f"pos={position/100:.2f}cm vel={velocity/100:.2f}cm/s raw={raw/1000:.3f} "
                f"limited={limited/1000:.3f} offset={offset/1000:.3f}deg physical={physical/1000:.3f}deg gaps={gaps}")
    if frame.msg_type == MSG_HEARTBEAT and len(p) == 8:
        mode, uptime = struct.unpack("<II", p)
        return f"HEARTBEAT mode={mode} uptime={uptime}ms"
    return f"FRAME type=0x{frame.msg_type:02X} seq={frame.sequence} len={len(p)} payload={p.hex(' ')}"


class PiTiClient:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.02) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.parser = FrameParser()
        self.tx_sequence = 0
        self.token = (int(time.time() * 1000.0) ^ (id(self) & 0xFFFFFFFF)) & 0xFFFFFFFF
        if self.token == 0:
            self.token = 1
        self.start = time.monotonic()
        self.next_heartbeat = 0.0

    def close(self) -> None:
        self.serial.close()

    def now_ms(self) -> int:
        return int((time.monotonic() - self.start) * 1000.0) & 0xFFFFFFFF

    def send(self, msg_type: int, payload: bytes = b"") -> None:
        self.serial.write(encode_frame(msg_type, self.tx_sequence, self.now_ms(), payload))
        self.tx_sequence = (self.tx_sequence + 1) & 0xFFFFFFFF

    def heartbeat(self) -> None:
        self.send(MSG_PI_STATUS, struct.pack("<I", self.now_ms()))

    def poll(self, duration: float = 0.0, print_frames: bool = True,
             send_heartbeat: bool = True) -> list[Frame]:
        deadline = time.monotonic() + duration
        frames: list[Frame] = []
        first = True
        while first or time.monotonic() < deadline:
            first = False
            now = time.monotonic()
            if send_heartbeat and now >= self.next_heartbeat:
                self.heartbeat()
                self.next_heartbeat = now + 0.20
            chunk = self.serial.read(256)
            if chunk:
                parsed = self.parser.feed(chunk)
                frames.extend(parsed)
                if print_frames:
                    for frame in parsed:
                        print(decode_frame(frame))
            if duration <= 0:
                break
        return frames

    def command(self, command: int, value: int = 0, argument: int = 0,
                wait: float = 1.5) -> int:
        token = self.token
        self.token += 1
        self.send(MSG_CONTROL_COMMAND, build_command(command, token, value, argument))
        deadline = time.monotonic() + wait
        final_result: Optional[int] = None
        suppress_heartbeat = command == PI_CMD_STARTUP_HORIZONTAL
        while time.monotonic() < deadline:
            for frame in self.poll(0.08, print_frames=True,
                                   send_heartbeat=not suppress_heartbeat):
                if frame.msg_type == MSG_COMMAND_ACK and len(frame.payload) == 12:
                    cmd, result, _profile, ack_token, _detail = struct.unpack("<BBHIi", frame.payload)
                    if cmd == command and ack_token == token and result != 1:
                        final_result = result
                        return result
        return 255 if final_result is None else final_result


def run_simulation(client: PiTiClient, duration: float, rate: float,
                   pattern: str, amplitude: float, target_cm: float,
                   invalid_after: float | None, startup_motor: bool,
                   enable_motor: bool) -> None:
    client.command(PI_CMD_HELLO)
    if startup_motor:
        result = client.command(PI_CMD_STARTUP_HORIZONTAL, wait=6.0)
        if result not in (0, 1):
            raise RuntimeError(f"startup failed with ACK result {result}")
    if enable_motor:
        result = client.command(PI_CMD_ENABLE_MOTOR)
        if result not in (0, 1):
            raise RuntimeError(f"enable failed with ACK result {result}")
    client.command(PI_CMD_SET_BALL_TARGET_CENTI_CM, round(target_cm * 100.0))
    client.command(PI_CMD_ARM_PREVIEW)
    period = 1.0 / rate
    started = time.monotonic()
    next_send = started
    last_pos = 0.0
    frame_id = 0
    while time.monotonic() - started < duration:
        now = time.monotonic()
        elapsed = now - started
        if now >= next_send:
            if pattern == "sine":
                pos = amplitude * math.sin(2.0 * math.pi * 0.25 * elapsed)
            elif pattern == "ramp":
                phase = (elapsed % 4.0) / 4.0
                pos = -amplitude + 2.0 * amplitude * phase
            else:
                pos = amplitude if int(elapsed) % 2 == 0 else -amplitude
            vel = (pos - last_pos) / period if frame_id else 0.0
            valid = invalid_after is None or elapsed < invalid_after
            payload = build_ball_state(frame_id, pos, vel, valid=valid,
                                       found=valid, invalid_reason=0 if valid else 7)
            client.send(MSG_BALL_STATE, payload)
            last_pos = pos
            frame_id += 1
            next_send += period
        client.poll(0.01, print_frames=True)
    client.command(PI_CMD_DISARM_PREVIEW)
    if enable_motor:
        client.command(PI_CMD_DISABLE_MOTOR)


def run_manual_demo(client: PiTiClient, offsets_mdeg: list[int],
                    dwell: float) -> None:
    client.command(PI_CMD_HELLO)
    result = client.command(PI_CMD_STARTUP_HORIZONTAL, wait=6.0)
    if result not in (0, 1):
        raise RuntimeError(f"startup failed with ACK result {result}")
    for offset in offsets_mdeg:
        result = client.command(PI_CMD_SET_OFFSET_MDEG, offset)
        if result not in (0, 1):
            raise RuntimeError(f"offset {offset} failed with ACK result {result}")
        client.poll(dwell, print_frames=True)
    client.command(PI_CMD_RETURN_HORIZONTAL)
    client.poll(dwell, print_frames=True)
    client.command(PI_CMD_DISABLE_MOTOR)


def self_test() -> None:
    parser = FrameParser()
    payload = build_command(PI_CMD_SET_OFFSET_MDEG, 123, 500)
    raw = encode_frame(MSG_CONTROL_COMMAND, 7, 42, payload)
    frames = parser.feed(raw[:5]) + parser.feed(raw[5:])
    assert len(frames) == 1
    assert frames[0].msg_type == MSG_CONTROL_COMMAND
    assert frames[0].payload == payload
    ball = build_ball_state(9, 1.25, -2.5)
    assert len(ball) == 16
    corrupted = bytearray(raw)
    corrupted[-1] ^= 0x01
    assert not FrameParser().feed(bytes(corrupted))
    print("protocol self-test: PASS")


def command_id(name: str) -> int:
    return {
        "hello": PI_CMD_HELLO,
        "status": PI_CMD_GET_STATUS,
        "startup": PI_CMD_STARTUP_HORIZONTAL,
        "horizontal": PI_CMD_RETURN_HORIZONTAL,
        "enable": PI_CMD_ENABLE_MOTOR,
        "disable": PI_CMD_DISABLE_MOTOR,
        "estop": PI_CMD_EMERGENCY_DISABLE,
        "arm": PI_CMD_ARM_PREVIEW,
        "disarm": PI_CMD_DISARM_PREVIEW,
        "clear-stats": PI_CMD_CLEAR_LINK_STATS,
    }[name]


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="COM5, /dev/serial0, /dev/ttyAMA0, ...")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("self-test")
    for name in ["hello", "status", "startup", "horizontal", "enable", "disable", "estop", "arm", "disarm", "clear-stats"]:
        sub.add_parser(name)
    offset = sub.add_parser("offset")
    offset.add_argument("mdeg", type=int, help="offset from -28.1deg horizontal, e.g. 500")
    target = sub.add_parser("target")
    target.add_argument("cm", type=float)
    listen = sub.add_parser("listen")
    listen.add_argument("--duration", type=float, default=10.0)
    manual_demo = sub.add_parser("manual-demo")
    manual_demo.add_argument("--offsets", type=int, nargs="+",
                             default=[200, 0, -500, 0])
    manual_demo.add_argument("--dwell", type=float, default=1.0)
    simulate = sub.add_parser("simulate")
    simulate.add_argument("--duration", type=float, default=10.0)
    simulate.add_argument("--rate", type=float, default=40.0)
    simulate.add_argument("--pattern", choices=["step", "sine", "ramp"], default="step")
    simulate.add_argument("--amplitude", type=float, default=2.0)
    simulate.add_argument("--target-cm", type=float, default=0.0)
    simulate.add_argument("--invalid-after", type=float)
    simulate.add_argument("--startup-motor", action="store_true",
                          help="physical 0deg must be established before reset")
    simulate.add_argument("--enable-motor", action="store_true",
                          help="phase 6 only; firmware output must be compiled in")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.action == "self-test":
        self_test()
        return 0
    if not args.port:
        parser.error("--port is required for serial actions")

    client = PiTiClient(args.port, args.baud)
    try:
        if args.action == "offset":
            return 0 if client.command(PI_CMD_SET_OFFSET_MDEG, args.mdeg) in (0, 1) else 2
        if args.action == "target":
            return 0 if client.command(PI_CMD_SET_BALL_TARGET_CENTI_CM, round(args.cm * 100.0)) in (0, 1) else 2
        if args.action == "listen":
            client.poll(args.duration, print_frames=True)
            return 0
        if args.action == "manual-demo":
            run_manual_demo(client, args.offsets, args.dwell)
            return 0
        if args.action == "simulate":
            run_simulation(client, args.duration, args.rate, args.pattern,
                           args.amplitude, args.target_cm, args.invalid_after,
                           args.startup_motor, args.enable_motor)
            return 0
        wait = 6.0 if args.action == "startup" else 1.5
        result = client.command(command_id(args.action), wait=wait)
        client.poll(0.5, print_frames=True)
        return 0 if result in (0, 1) else 2
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
