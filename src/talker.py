#!/usr/bin/env python3

import argparse
import signal
import sys
import time
from datetime import datetime, timezone

import serial


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send plain ASCII test packets over a serial port."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial device path")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate")
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between packets",
    )
    parser.add_argument(
        "--label",
        default="robot",
        help="Short label included in each packet",
    )
    parser.add_argument(
        "--message",
        default="serial-test",
        help="Payload text included in each packet",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of packets to send. 0 means forever.",
    )
    return parser


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def main() -> int:
    args = build_parser().parse_args()
    running = True

    def handle_signal(signum, frame):
        del signum, frame
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        ser = serial.Serial(
            args.port,
            args.baud,
            timeout=0.2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except Exception as exc:
        print(f"[talker] serial open failed: {exc}", file=sys.stderr)
        return 1

    print(
        f"[talker] opened port={args.port} baud={args.baud} "
        f"interval={args.interval}s label={args.label}"
    )

    seq = 0
    try:
        while running:
            seq += 1
            packet = (
                f"PING label={args.label} seq={seq} ts={utc_now_iso()} "
                f"msg={args.message}\n"
            )
            payload = packet.encode("ascii")
            ser.write(payload)
            ser.flush()
            print(f"[talker] tx {seq}: {packet.strip()}")

            if args.count > 0 and seq >= args.count:
                break

            time.sleep(args.interval)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[talker] closed")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
