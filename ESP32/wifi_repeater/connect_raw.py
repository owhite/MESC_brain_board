#!/usr/bin/env python3
"""Receive raw TCP telemetry, show curses status, and flush blocks to disk."""

from __future__ import annotations

import argparse
import curses
import os
import queue
import select
import socket
import sys
import threading
import time
from dataclasses import dataclass

DEFAULT_HOST = "twr-repeater.local"
DEFAULT_PORT = 9000
DEFAULT_CHUNK = 16384
DEFAULT_FLUSH_TIMEOUT = 0.50
DEFAULT_CONNECT_TIMEOUT = 10.0
DEFAULT_OUTPUT = "telem_dump.bin"


@dataclass
class UiState:
    host: str
    port: int
    output: str
    connected: bool = False
    status: str = "starting"
    started_at: float = 0.0
    total_rx: int = 0
    total_flushed: int = 0
    buffered: int = 0
    blocks_written: int = 0
    last_flush_msg: str = "none"
    last_rx_at: float | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture raw TCP stream from ESP32 repeater into block files"
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"TCP host (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"TCP port (default: {DEFAULT_PORT})")
    parser.add_argument("--chunk", type=int, default=DEFAULT_CHUNK, help="Socket recv chunk size in bytes")
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT,
        help=f"Output file path (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--flush-timeout",
        type=float,
        default=DEFAULT_FLUSH_TIMEOUT,
        help=f"Flush buffered block after this many idle seconds (default: {DEFAULT_FLUSH_TIMEOUT})",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=DEFAULT_CONNECT_TIMEOUT,
        help=f"TCP connect timeout in seconds (default: {DEFAULT_CONNECT_TIMEOUT})",
    )
    return parser.parse_args()


def fmt_bytes(n: int) -> str:
    units = ["B", "KB", "MB", "GB"]
    value = float(n)
    idx = 0
    while value >= 1024.0 and idx < len(units) - 1:
        value /= 1024.0
        idx += 1
    if idx == 0:
        return f"{int(value)} {units[idx]}"
    return f"{value:.2f} {units[idx]}"


def draw(stdscr: curses.window, st: UiState, flush_timeout: float) -> None:
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    now = time.monotonic()
    up_s = int(max(0.0, now - st.started_at))
    if st.last_rx_at is None:
        idle_s = "n/a"
    else:
        idle_s = f"{max(0.0, now - st.last_rx_at):.3f}s"

    lines = [
        "ESP32 Raw Telemetry Capture",
        "",
        f"TCP: {st.host}:{st.port}",
        f"Connection: {'CONNECTED' if st.connected else 'DISCONNECTED'}",
        f"Status: {st.status}",
        f"Output file: {st.output}",
        "",
        f"Uptime: {up_s}s",
        f"RX total: {st.total_rx} bytes ({fmt_bytes(st.total_rx)})",
        f"Buffered (current block): {st.buffered} bytes ({fmt_bytes(st.buffered)})",
        f"Flushed total: {st.total_flushed} bytes ({fmt_bytes(st.total_flushed)})",
        f"Blocks written: {st.blocks_written}",
        f"Idle since last RX: {idle_s}",
        f"Flush timeout: {flush_timeout:.3f}s",
        "",
        f"Last flush: {st.last_flush_msg}",
        "",
        "Press q to quit",
        "Press c to clear",
    ]

    for i, line in enumerate(lines):
        if i >= h:
            break
        stdscr.addnstr(i, 0, line, max(1, w - 1))
    stdscr.refresh()


def disk_writer_worker(output_path: str, disk_queue: "queue.Queue[tuple[str, bytes | None] | None]") -> None:
    out_f = open(output_path, "wb")
    try:
        while True:
            item = disk_queue.get()
            if item is None:
                break
            cmd, payload = item
            if cmd == "write" and payload:
                out_f.write(payload)
                out_f.flush()
            elif cmd == "truncate":
                out_f.close()
                out_f = open(output_path, "wb")
    finally:
        try:
            out_f.close()
        except OSError:
            pass


def flush_block(
    block: bytearray,
    disk_queue: "queue.Queue[tuple[str, bytes | None] | None]",
    st: UiState,
    reason: str,
) -> None:
    if not block:
        return
    payload = bytes(block)
    disk_queue.put(("write", payload))
    wrote = len(block)
    st.total_flushed += wrote
    st.blocks_written += 1
    st.last_flush_msg = f"block #{st.blocks_written}: {wrote} bytes ({reason})"
    block.clear()
    st.buffered = 0


def clear_capture_file(
    disk_queue: "queue.Queue[tuple[str, bytes | None] | None]",
    block: bytearray,
    st: UiState,
):
    block.clear()
    st.buffered = 0
    disk_queue.put(("truncate", None))

    st.total_rx = 0
    st.total_flushed = 0
    st.blocks_written = 0
    st.last_rx_at = None
    st.last_flush_msg = "capture file cleared"
    st.status = "capture cleared; waiting for data"


def run_capture(stdscr: curses.window, args: argparse.Namespace) -> int:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    state = UiState(host=args.host, port=args.port, output=args.output, started_at=time.monotonic())
    block = bytearray()

    state.status = "connecting"
    draw(stdscr, state, args.flush_timeout)

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    try:
        with socket.create_connection((args.host, args.port), timeout=args.connect_timeout) as sock:
            sock.setblocking(False)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            disk_queue: "queue.Queue[tuple[str, bytes | None] | None]" = queue.Queue()
            writer_thread = threading.Thread(
                target=disk_writer_worker,
                args=(args.output, disk_queue),
                daemon=True,
            )
            writer_thread.start()
            try:
                state.connected = True
                state.status = "connected; waiting for data"

                while True:
                    key = stdscr.getch()
                    if key in (ord("q"), ord("Q")):
                        state.status = "user requested exit"
                        flush_block(block, disk_queue, state, reason="quit")
                        draw(stdscr, state, args.flush_timeout)
                        return 0
                    if key in (ord("c"), ord("C")):
                        clear_capture_file(disk_queue, block, state)
                        draw(stdscr, state, args.flush_timeout)
                        continue

                    now = time.monotonic()

                    ready, _, _ = select.select([sock], [], [], 0.0)
                    if ready:
                        data = sock.recv(args.chunk)
                        if not data:
                            state.connected = False
                            state.status = "peer closed connection"
                            flush_block(block, disk_queue, state, reason="disconnect")
                            draw(stdscr, state, args.flush_timeout)
                            return 0
                        state.total_rx += len(data)
                        block.extend(data)
                        state.buffered = len(block)
                        state.last_rx_at = now
                        state.status = "receiving"

                    if block and state.last_rx_at is not None:
                        idle = now - state.last_rx_at
                        if idle >= args.flush_timeout:
                            flush_block(block, disk_queue, state, reason=f"idle {idle:.3f}s")
                            state.status = "block flushed"

                    draw(stdscr, state, args.flush_timeout)
            finally:
                disk_queue.put(None)
                writer_thread.join()

    except KeyboardInterrupt:
        return 0
    except OSError as exc:
        state.connected = False
        state.status = f"connection error: {exc}"
        draw(stdscr, state, args.flush_timeout)
        time.sleep(1.5)
        return 1


def main() -> int:
    args = parse_args()

    if args.chunk <= 0:
        print("--chunk must be > 0", file=sys.stderr)
        return 2
    if args.flush_timeout <= 0:
        print("--flush-timeout must be > 0", file=sys.stderr)
        return 2

    return curses.wrapper(lambda stdscr: run_capture(stdscr, args))


if __name__ == "__main__":
    raise SystemExit(main())
