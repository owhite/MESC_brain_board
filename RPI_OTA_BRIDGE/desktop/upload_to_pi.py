# PlatformIO post upload hook: copy firmware .bin to Pi and stream flash log.
import os
import pathlib
import subprocess
import sys
import time

from SCons.Script import COMMAND_LINE_TARGETS, Import

Import("env")

PI_HOST = os.environ.get("TEENSY_BRIDGE_HOST", "teensybridge.local")
PI_USER = os.environ.get("TEENSY_BRIDGE_USER", "pi")
PI_DROP_DIR = os.environ.get("TEENSY_BRIDGE_DROP_DIR", "/home/pi/teensy_drops")
STATUS_LOG = f"{PI_DROP_DIR}/last_upload_status.log"
SSH_BASE = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", f"{PI_USER}@{PI_HOST}"]
UPLOAD_DONE = False


def _run(cmd):
    return subprocess.run(cmd, check=False)


def _upload_action(source, target, _env):
    global UPLOAD_DONE
    if UPLOAD_DONE:
        return

    firmware = pathlib.Path(str(source[0]))
    if not firmware.exists():
        print(f"[teensy-bridge] Firmware not found: {firmware}", file=sys.stderr)
        env.Exit(2)

    print(f"[teensy-bridge] Uploading {firmware.name} to {PI_USER}@{PI_HOST}:{PI_DROP_DIR}")

    prep_cmd = SSH_BASE + [f"mkdir -p {PI_DROP_DIR} {PI_DROP_DIR}/processed && : > {STATUS_LOG}"]
    prep = _run(prep_cmd)
    if prep.returncode != 0:
        print("[teensy-bridge] SSH prep failed.", file=sys.stderr)
        env.Exit(prep.returncode)

    tail_cmd = SSH_BASE + [f"tail -n +1 -f {STATUS_LOG}"]
    tail_proc = subprocess.Popen(
        tail_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    scp_cmd = ["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", str(firmware), f"{PI_USER}@{PI_HOST}:{PI_DROP_DIR}/"]
    scp = _run(scp_cmd)
    if scp.returncode != 0:
        tail_proc.terminate()
        print("[teensy-bridge] SCP failed.", file=sys.stderr)
        env.Exit(scp.returncode)

    exit_code = None
    deadline = time.time() + 180

    try:
        while time.time() < deadline:
            line = tail_proc.stdout.readline()
            if not line:
                if tail_proc.poll() is not None:
                    break
                time.sleep(0.1)
                continue

            print(line, end="")
            if line.startswith("EXIT_CODE="):
                try:
                    exit_code = int(line.split("=", 1)[1].strip())
                except ValueError:
                    exit_code = 99
                break
    finally:
        tail_proc.terminate()
        try:
            tail_proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            tail_proc.kill()

    if exit_code is None:
        print("[teensy-bridge] Timed out waiting for flash result.", file=sys.stderr)
        env.Exit(124)

    if exit_code != 0:
        print(f"[teensy-bridge] Flash failed with code {exit_code}.", file=sys.stderr)
        env.Exit(exit_code)

    UPLOAD_DONE = True
    print("[teensy-bridge] Flash completed successfully.")


if "upload" in COMMAND_LINE_TARGETS:
    env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", _upload_action)
