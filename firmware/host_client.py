#!/usr/bin/env python3
"""Host-side client for the labrobot binary protocol.

    firmware/host_client.py --status                 watch status for 5 s
    firmware/host_client.py --ping
    firmware/host_client.py --drive 10 10 --seconds 4    wheel-rpm per side
    firmware/host_client.py --watchdog-test          prove the robot self-stops
"""

import argparse
import math
import os
import select
import signal
import struct
import sys
import termios
import time

SYNC0, SYNC1 = 0xA5, 0x5A
MAX_PAYLOAD = 32

CMD_VELOCITY, CMD_STOP, CMD_ESTOP, CMD_RESET_ODOM, CMD_PING = 1, 2, 3, 4, 5
MSG_STATUS, MSG_PONG = 0x81, 0x85

FLAGS = [
    (1 << 0, "L_OK"),
    (1 << 1, "R_OK"),
    (1 << 2, "INIT"),
    (1 << 3, "ESTOP"),
    (1 << 4, "WATCHDOG"),
]

COUNTS_PER_WHEEL_REV = 52 * 241  # 12532
MAX_WHEEL_RPM = 60
MAX_MRAD_S = 6283  # 60 rpm = 2*pi rad/s


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload), msg_type]) + payload
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc16(body))


def encode_velocity(left_mrad_s: int, right_mrad_s: int) -> bytes:
    return encode(CMD_VELOCITY, struct.pack("<hh", left_mrad_s, right_mrad_s))


def wheel_rpm_to_mrad_s(rpm: float) -> int:
    return int(round(rpm * 2.0 * math.pi / 60.0 * 1000.0))


class Parser:
    """Mirrors the firmware's FrameParser, including its length checks."""

    REQUIRED = {CMD_VELOCITY: 4, CMD_STOP: 0, CMD_ESTOP: 0, CMD_RESET_ODOM: 0,
                CMD_PING: 0, MSG_STATUS: 22, MSG_PONG: 0}

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes):
        """Yields (type, payload) for every valid frame in the stream."""
        self.buf.extend(data)
        while True:
            start = self.buf.find(bytes([SYNC0, SYNC1]))
            if start < 0:
                # Keep one byte in case a sync pair straddles a read boundary.
                del self.buf[: max(0, len(self.buf) - 1)]
                return
            del self.buf[:start]
            if len(self.buf) < 4:
                return
            length = self.buf[2]
            if length > MAX_PAYLOAD:
                self.errors += 1
                del self.buf[:2]
                continue
            total = length + 6
            if len(self.buf) < total:
                return
            msg_type = self.buf[3]
            body = bytes(self.buf[2 : 4 + length])
            crc_rx = struct.unpack("<H", bytes(self.buf[4 + length : total]))[0]
            payload = bytes(self.buf[4 : 4 + length])
            del self.buf[:total]
            if crc16(body) != crc_rx or self.REQUIRED.get(msg_type, -1) != length:
                self.errors += 1
                continue
            yield msg_type, payload


def decode_status(p: bytes) -> dict:
    seq, flags, lc, rc, lv, rv, lf, rf, hf, he = struct.unpack("<BBiihhHHHH", p)
    return dict(seq=seq, flags=flags, left_counts=lc, right_counts=rc,
                left_mrad_s=lv, right_mrad_s=rv, left_faults=lf, right_faults=rf,
                host_frames=hf, host_errors=he)


def flags_str(flags: int) -> str:
    return ",".join(name for bit, name in FLAGS if flags & bit) or "-"


def mrad_to_rpm(mrad: int) -> float:
    return mrad / 1000.0 * 60.0 / (2.0 * math.pi)


def open_port(path: str, baud: int = 500000) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}")
    cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
    cc = list(attrs[6])
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [0, 0, cflag, 0, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/labrobot_mcu")
    ap.add_argument("--baud", type=int, default=500000)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--status", action="store_true", help="print status frames")
    ap.add_argument("--ping", action="store_true")
    ap.add_argument("--reset-odom", action="store_true")
    ap.add_argument("--drive", nargs=2, type=float, metavar=("L_RPM", "R_RPM"),
                    help="stream a wheel-rpm command per side at 20 Hz")
    ap.add_argument("--watchdog-test", action="store_true",
                    help="drive briefly, then go silent and prove it self-stops")
    ap.add_argument("--settle", type=float, default=2.5,
                    help="seconds to wait after opening, for the bootloader")
    args = ap.parse_args()

    fd = open_port(args.port, args.baud)
    parser = Parser()
    stop_sent = False

    def send_stop():
        nonlocal stop_sent
        if not stop_sent:
            try:
                os.write(fd, encode(CMD_STOP))
            except OSError:
                pass
            stop_sent = True

    def on_signal(_sig, _frm):
        send_stop()
        os.close(fd)
        sys.exit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        time.sleep(args.settle)
        termios.tcflush(fd, termios.TCIFLUSH)

        if args.ping:
            os.write(fd, encode(CMD_PING))
        if args.reset_odom:
            os.write(fd, encode(CMD_RESET_ODOM))

        left_mrad = right_mrad = 0
        streaming = False
        if args.drive:
            left_mrad = wheel_rpm_to_mrad_s(args.drive[0])
            right_mrad = wheel_rpm_to_mrad_s(args.drive[1])
            streaming = True
            print(f"driving L={args.drive[0]} R={args.drive[1]} wheel-rpm "
                  f"({left_mrad}/{right_mrad} mrad/s) for {args.seconds}s")

        wd_phase = "drive" if args.watchdog_test else None
        if wd_phase:
            left_mrad = right_mrad = wheel_rpm_to_mrad_s(10)
            streaming = True
            print("watchdog test: driving 1.0 s, then going silent")

        t0 = time.time()
        next_cmd = 0.0
        n_status = 0
        while time.time() - t0 < args.seconds:
            now = time.time() - t0

            if wd_phase == "drive" and now >= 1.0:
                wd_phase = "silent"
                streaming = False
                print(f"[{now:5.2f}s] --- going silent, expect WATCHDOG ---")

            if streaming and now >= next_cmd:
                os.write(fd, encode_velocity(left_mrad, right_mrad))
                next_cmd = now + 0.05  # 20 Hz

            r, _, _ = select.select([fd], [], [], 0.02)
            if not r:
                continue
            for mtype, payload in parser.feed(os.read(fd, 4096)):
                if mtype == MSG_PONG:
                    print(f"[{now:5.2f}s] PONG")
                elif mtype == MSG_STATUS:
                    n_status += 1
                    s = decode_status(payload)
                    if args.status or args.drive or args.watchdog_test:
                        if n_status % 5 == 0 or (s["flags"] & (1 << 4)):
                            print(f"[{now:5.2f}s] seq={s['seq']:3d} "
                                  f"L={s['left_counts']:8d} ({mrad_to_rpm(s['left_mrad_s']):6.2f} rpm)  "
                                  f"R={s['right_counts']:8d} ({mrad_to_rpm(s['right_mrad_s']):6.2f} rpm)  "
                                  f"rx={s['host_frames']}/{s['host_errors']}err  "
                                  f"[{flags_str(s['flags'])}]")

        send_stop()
        time.sleep(0.2)
        print(f"\n{n_status} status frames, {parser.errors} frame errors "
              f"({n_status / args.seconds:.0f} Hz)")
    finally:
        send_stop()
        os.close(fd)


if __name__ == "__main__":
    main()
