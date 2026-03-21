#!/usr/bin/env python3
"""Stream and print everything received from the ESP32 TCP repeater."""

import argparse
import socket
import sys

DEFAULT_HOST = "twr-repeater.local"
DEFAULT_PORT = 9000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print raw TCP stream from ESP32 repeater")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"TCP host (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"TCP port (default: {DEFAULT_PORT})")
    parser.add_argument("--chunk", type=int, default=4096, help="Socket recv chunk size in bytes")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        with socket.create_connection((args.host, args.port)) as sock:
            while True:
                data = sock.recv(args.chunk)
                if not data:
                    return 0
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
    except KeyboardInterrupt:
        return 0
    except OSError as exc:
        print(f"connection error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
