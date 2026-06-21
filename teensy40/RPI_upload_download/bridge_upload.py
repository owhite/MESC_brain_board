import os
import pathlib
import subprocess
import sys
import time

Import("env")

PI_HOST = os.environ.get("TEENSY_BRIDGE_HOST", "teensybridge.local")
PI_USER = os.environ.get("TEENSY_BRIDGE_USER", "owhite")
PI_DROP_DIR = os.environ.get("TEENSY_BRIDGE_DROP_DIR", f"/home/{PI_USER}/teensy_drops")
STATUS_LOG = f"{PI_DROP_DIR}/last_upload_status.log"
SSH_BASE = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", f"{PI_USER}@{PI_HOST}"]


def _run(cmd):
    return subprocess.run(cmd, check=False)


def _print_help(target=None, source=None, env=None):
    print("")
    print("Bridge upload help")
    print("Compile only:")
    print("  pio run")
    print("Compile and upload to Raspberry Pi bridge:")
    print("  pio run -t bridge_send")
    print("Optional overrides:")
    print("  TEENSY_BRIDGE_HOST=teensybridge.local \\")
    print("  TEENSY_BRIDGE_USER=owhite \\")
    print("  TEENSY_BRIDGE_DROP_DIR=/home/owhite/teensy_drops \\")
    print("  pio run -t bridge_send")
    print("Wrapper script shortcuts:")
    print("  ./pio_bridge.sh --bridge_help")
    print("  ./pio_bridge.sh --bridge_send")
    print("")
    return 0


def _upload_action(target=None, source=None, env=None):
    firmware = pathlib.Path(str(source[0]))
    if not firmware.exists():
        print(f"[teensy-bridge] Firmware not found: {firmware}", file=sys.stderr)
        return 2

    print(f"[teensy-bridge] Uploading {firmware.name} to {PI_USER}@{PI_HOST}:{PI_DROP_DIR}")

    prep_cmd = SSH_BASE + [f"mkdir -p {PI_DROP_DIR} {PI_DROP_DIR}/processed && : > {STATUS_LOG}"]
    prep = _run(prep_cmd)
    if prep.returncode != 0:
        print("[teensy-bridge] SSH prep failed.", file=sys.stderr)
        return prep.returncode

    tail_cmd = SSH_BASE + [f"tail -n +1 -f {STATUS_LOG}"]
    tail_proc = subprocess.Popen(
        tail_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    scp_cmd = [
        "scp",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=10",
        str(firmware),
        f"{PI_USER}@{PI_HOST}:{PI_DROP_DIR}/",
    ]
    scp = _run(scp_cmd)
    if scp.returncode != 0:
        tail_proc.terminate()
        print("[teensy-bridge] SCP failed.", file=sys.stderr)
        return scp.returncode

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
        return 124

    if exit_code != 0:
        print(f"[teensy-bridge] Flash failed with code {exit_code}.", file=sys.stderr)
        return exit_code

    print("[teensy-bridge] Flash completed successfully.")
    return 0


env.AddCustomTarget(
    name="bridge_help",
    dependencies=None,
    actions=[_print_help],
    title="Bridge Help",
    description="Prints commands to build and upload firmware to the Raspberry Pi bridge",
)

env.AddCustomTarget(
    name="bridge_send",
    dependencies="$BUILD_DIR/${PROGNAME}.hex",
    actions=[_upload_action],
    title="Bridge Send",
    description="Build firmware and SCP the HEX to Raspberry Pi bridge",
)
