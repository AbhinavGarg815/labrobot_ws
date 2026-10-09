#!/usr/bin/env python3
"""Non-interactive serial console for the labrobot Mega bench firmware.

Uses termios directly so it works over SSH and from non-interactive shells,
where `stty` on a tty raises SIGTTOU. No pyserial dependency.

    firmware/serial_console.py --seconds 8
    firmware/serial_console.py --send f --seconds 5
    firmware/serial_console.py --send 'h' --send 'f' --delay 3 --send 's'
"""

import argparse
import os
import select
import sys
import termios
import time

BAUD = {9600: termios.B9600, 115200: termios.B115200, 500000: termios.B500000}


def open_port(path, baud, reset):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

    # Raw 8N1: no translation, no echo, no flow control, ignore modem lines.
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
    if not reset:
        # HUPCL drops DTR on close, which resets the Mega. Clearing it lets us
        # attach to already-running firmware without restarting it.
        cflag &= ~termios.HUPCL
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0

    speed = BAUD[baud]
    termios.tcsetattr(
        fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc]
    )
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/labrobot_mcu")
    ap.add_argument("--baud", type=int, default=115200, choices=sorted(BAUD))
    ap.add_argument("--seconds", type=float, default=8.0,
                    help="how long to keep reading after the last command")
    ap.add_argument("--send", action="append", default=[],
                    help="command to send; repeatable, in order. Use 'cmd:secs' "
                         "to override how long to wait before the next one.")
    ap.add_argument("--delay", type=float, default=2.0,
                    help="default seconds to wait between --send commands")
    ap.add_argument("--settle", type=float, default=2.5,
                    help="seconds to wait after opening, for the bootloader")
    ap.add_argument("--no-reset", action="store_true",
                    help="attach without resetting the board")
    args = ap.parse_args()

    # Split each --send into (command, delay-after-it).
    pending = []
    for item in args.send:
        cmd, sep, secs = item.partition(":")
        pending.append((cmd, float(secs) if sep and secs else args.delay))

    fd = open_port(args.port, args.baud, reset=not args.no_reset)
    total_delays = sum(d for _, d in pending)
    deadline = time.time() + args.settle + total_delays + args.seconds
    next_send = time.time() + args.settle

    try:
        while time.time() < deadline:
            if pending and time.time() >= next_send:
                cmd, hold = pending.pop(0)
                os.write(fd, (cmd + "\n").encode())
                sys.stdout.write(f"\n>>> sent {cmd!r} (hold {hold}s)\n")
                sys.stdout.flush()
                next_send = time.time() + hold

            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                data = os.read(fd, 4096)
                if data:
                    sys.stdout.write(data.decode("utf-8", errors="replace"))
                    sys.stdout.flush()
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
