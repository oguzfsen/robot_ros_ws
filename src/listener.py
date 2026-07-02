#!/usr/bin/env python3

import argparse
import signal
import sys
import time

import serial


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read raw bytes from a serial port and print text plus hex."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial device path")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.2,
        help="Serial read timeout in seconds",
    )
    parser.add_argument(
        "--show-hex",
        action="store_true",
        help="Always print payload hex in addition to text",
    )
    return parser


def to_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def main() -> int:
    args = build_parser().parse_args()
    running = True
    buffer = bytearray()

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
            timeout=args.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except Exception as exc:
        print(f"[listener] serial open failed: {exc}", file=sys.stderr)
        return 1

    print(f"[listener] opened port={args.port} baud={args.baud}")

    try:
        while running:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue

            buffer.extend(chunk)

            while b"\n" in buffer:
                line, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)

                try:
                    text = line.decode("ascii")
                    print(f"[listener] rx text: {text}")
                except UnicodeDecodeError:
                    print(f"[listener] rx binary: {to_hex(line)}")
                    continue

                if args.show_hex:
                    print(f"[listener] rx hex:  {to_hex(line)}")

        if buffer:
            try:
                text = buffer.decode("ascii")
                print(f"[listener] trailing text: {text}")
            except UnicodeDecodeError:
                print(f"[listener] trailing binary: {to_hex(buffer)}")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[listener] closed")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
