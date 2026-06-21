#!/usr/bin/env python3
import argparse
import datetime as dt
import json
import os
import secrets
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path


TERMINAL_STATES = {"completed", "timed_out", "error", "size_limit"}


def _utc_compact_run_id() -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = secrets.token_hex(3)
    return f"{stamp}_{suffix}"


def _run(cmd, cwd=None, env=None, check=False, capture_output=False):
    return subprocess.run(
        cmd,
        cwd=cwd,
        env=env,
        check=check,
        text=True,
        capture_output=capture_output,
    )


def _ssh_base(user: str, host: str):
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", f"{user}@{host}"]


def _scp_base():
    return ["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10"]


def _remote_set_run_id(user: str, host: str, run_base: str, run_id: str) -> int:
    run_base_q = shlex.quote(run_base)
    run_id_q = shlex.quote(run_id)
    remote_cmd = (
        f"mkdir -p {run_base_q} "
        f"&& printf '%s\\n' {run_id_q} > {run_base_q}/current_run_id"
    )
    result = _run(_ssh_base(user, host) + [remote_cmd])
    return result.returncode


def _remote_read_status(user: str, host: str, run_base: str, run_id: str):
    status_path = f"{run_base}/{run_id}/status.json"
    status_q = shlex.quote(status_path)
    remote_cmd = f"if [ -f {status_q} ]; then cat {status_q}; fi"
    result = _run(_ssh_base(user, host) + [remote_cmd], capture_output=True)
    if result.returncode != 0:
        return None

    payload = (result.stdout or "").strip()
    if not payload:
        return None

    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return None


def _wait_for_terminal_status(user: str, host: str, run_base: str, run_id: str, timeout_sec: int):
    deadline = time.time() + timeout_sec
    last_state = ""
    while time.time() < deadline:
        status = _remote_read_status(user, host, run_base, run_id)
        if status:
            state = status.get("state", "")
            if state and state != last_state:
                print(f"[run-cycle] state={state}")
                last_state = state
            if state in TERMINAL_STATES:
                return status
        time.sleep(1.0)
    return None


def _fetch_run_artifacts(user: str, host: str, run_base: str, run_id: str, local_logs_dir: Path) -> int:
    local_run_dir = local_logs_dir / run_id
    local_run_dir.mkdir(parents=True, exist_ok=True)

    remote_src = f"{user}@{host}:{run_base}/{run_id}/."
    result = _run(_scp_base() + ["-r", remote_src, str(local_run_dir)])
    return result.returncode


def _trim_raw_log_at_run_start(local_run_dir: Path) -> int:
    raw_path = local_run_dir / "serial_raw.log"
    if not raw_path.exists():
        return 0

    lines = raw_path.read_text(encoding="utf-8", errors="replace").splitlines()
    start_idx = -1
    for i, line in enumerate(lines):
        if "RUN_START" in line.upper():
            start_idx = i
            break

    if start_idx <= 0:
        return 0

    trimmed = lines[start_idx:]
    raw_path.write_text("\n".join(trimmed) + "\n", encoding="utf-8")
    return start_idx


def _trim_parsed_log_at_run_start(local_run_dir: Path) -> int:
    parsed_path = local_run_dir / "serial_parsed.jsonl"
    if not parsed_path.exists():
        return 0

    lines = parsed_path.read_text(encoding="utf-8", errors="replace").splitlines()
    start_idx = -1

    for i, line in enumerate(lines):
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue

        payload = record.get("payload", {}) if isinstance(record, dict) else {}
        if isinstance(payload, dict):
            msg = str(payload.get("msg", ""))
            raw = str(payload.get("raw", ""))
            if msg == "RUN_START" or "RUN_START" in raw.upper():
                start_idx = i
                break

    if start_idx <= 0:
        return 0

    trimmed = lines[start_idx:]
    parsed_path.write_text("\n".join(trimmed) + "\n", encoding="utf-8")
    return start_idx


def _trim_artifacts_pre_start(local_run_dir: Path) -> tuple[int, int]:
    raw_trim = _trim_raw_log_at_run_start(local_run_dir)
    parsed_trim = _trim_parsed_log_at_run_start(local_run_dir)
    return raw_trim, parsed_trim


def _cleanup_local_logs(local_logs_dir: Path, keep_count: int, protect_run_id: str) -> int:
    if keep_count <= 0:
        return 0

    if not local_logs_dir.exists():
        return 0

    run_dirs = [p for p in local_logs_dir.iterdir() if p.is_dir()]
    if len(run_dirs) <= keep_count:
        return 0

    run_dirs.sort(key=lambda p: p.stat().st_mtime)
    to_delete = len(run_dirs) - keep_count
    removed = 0

    for path in run_dirs:
        if removed >= to_delete:
            break
        if path.name == protect_run_id:
            continue
        shutil.rmtree(path, ignore_errors=True)
        removed += 1

    return removed


def main() -> int:
    parser = argparse.ArgumentParser(description="Run full Teensy OTA + Pi log capture cycle")
    parser.add_argument("--firmware-dir", default=".", help="PlatformIO project directory")
    parser.add_argument("--logs-dir", default="logs", help="Local directory to store fetched run artifacts")
    parser.add_argument("--host", default=os.environ.get("TEENSY_BRIDGE_HOST", "teensybridge.local"))
    parser.add_argument("--user", default=os.environ.get("TEENSY_BRIDGE_USER", "pi"))
    parser.add_argument("--run-base", default=os.environ.get("TEENSY_RUN_BASE", ""))
    parser.add_argument("--pio-cmd", default=os.environ.get("PIO_CMD", "pio"))
    parser.add_argument("--pio-target", default=os.environ.get("PIO_TARGET", "bridge_send"))
    parser.add_argument("--wait-timeout", type=int, default=int(os.environ.get("RUN_WAIT_TIMEOUT_SEC", "240")))
    parser.add_argument("--run-id", default="", help="Optional explicit run_id")
    parser.add_argument(
        "--keep-local-runs",
        type=int,
        default=int(os.environ.get("KEEP_LOCAL_RUN_DIRS", "100")),
        help="Maximum number of local run directories to keep in logs-dir",
    )
    args = parser.parse_args()

    firmware_dir = Path(args.firmware_dir).resolve()
    if not firmware_dir.exists():
        print(f"[run-cycle] firmware dir does not exist: {firmware_dir}", file=sys.stderr)
        return 2

    run_base = args.run_base or f"/home/{args.user}/teensy_runs"
    run_id = args.run_id or _utc_compact_run_id()
    logs_dir = Path(args.logs_dir).resolve()

    print(f"[run-cycle] run_id={run_id}")
    print(f"[run-cycle] firmware_dir={firmware_dir}")
    print(f"[run-cycle] pi={args.user}@{args.host} run_base={run_base}")

    prep_rc = _remote_set_run_id(args.user, args.host, run_base, run_id)
    if prep_rc != 0:
        print("[run-cycle] failed to write run_id on Pi", file=sys.stderr)
        return prep_rc

    pio_cmd = [args.pio_cmd, "run", "-t", args.pio_target]
    print(f"[run-cycle] running: {' '.join(pio_cmd)}")
    pio_rc = _run(pio_cmd, cwd=str(firmware_dir)).returncode
    if pio_rc != 0:
        print(f"[run-cycle] pio command failed with code {pio_rc}", file=sys.stderr)
        return pio_rc

    status = _wait_for_terminal_status(args.user, args.host, run_base, run_id, args.wait_timeout)
    if not status:
        print("[run-cycle] timed out waiting for terminal run status", file=sys.stderr)
        return 124

    state = status.get("state", "")
    print(f"[run-cycle] terminal state={state}")

    fetch_rc = _fetch_run_artifacts(args.user, args.host, run_base, run_id, logs_dir)
    if fetch_rc != 0:
        print(f"[run-cycle] failed to fetch run artifacts, scp code={fetch_rc}", file=sys.stderr)
        return fetch_rc

    local_run_dir = logs_dir / run_id
    raw_trim, parsed_trim = _trim_artifacts_pre_start(local_run_dir)

    print(f"[run-cycle] artifacts saved to {local_run_dir}")
    if raw_trim or parsed_trim:
        print(
            f"[run-cycle] trimmed pre-start noise lines: raw={raw_trim}, parsed={parsed_trim}"
        )

    removed = _cleanup_local_logs(logs_dir, args.keep_local_runs, run_id)
    if removed > 0:
        print(f"[run-cycle] pruned {removed} old local run directories (keep={args.keep_local_runs})")

    if state == "completed":
        return 0
    if state == "timed_out":
        return 124
    return 10


if __name__ == "__main__":
    raise SystemExit(main())
