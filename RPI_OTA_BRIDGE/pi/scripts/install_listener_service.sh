#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE_USER="${BRIDGE_USER:-${SUDO_USER:-$USER}}"

if command -v getent >/dev/null 2>&1; then
  BRIDGE_HOME="$(getent passwd "$BRIDGE_USER" | cut -d: -f6)"
else
  BRIDGE_HOME="$(BRIDGE_USER_VALUE="$BRIDGE_USER" python3 - <<'PY'
import pwd
import os
import sys
user = os.environ.get("BRIDGE_USER_VALUE", "")
try:
    print(pwd.getpwnam(user).pw_dir)
except KeyError:
    sys.exit(1)
PY
)"
fi

if [[ -z "$BRIDGE_HOME" ]]; then
  echo "Unable to resolve home directory for user: $BRIDGE_USER" >&2
  exit 1
fi

TEENSY_RUN_BASE="$BRIDGE_HOME/teensy_runs"

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  python3-serial

mkdir -p "$BRIDGE_HOME/teensy-listener"
mkdir -p "$TEENSY_RUN_BASE"
cp "$PROJECT_ROOT/listener/serial_listener.py" "$BRIDGE_HOME/teensy-listener/serial_listener.py"
chmod +x "$BRIDGE_HOME/teensy-listener/serial_listener.py"

SERVICE_TMP="$(mktemp)"
sed \
  -e "s|__BRIDGE_USER__|$BRIDGE_USER|g" \
  -e "s|__BRIDGE_HOME__|$BRIDGE_HOME|g" \
  -e "s|__TEENSY_RUN_BASE__|$TEENSY_RUN_BASE|g" \
  "$PROJECT_ROOT/systemd/teensy-listener.service" > "$SERVICE_TMP"

sudo cp "$SERVICE_TMP" /etc/systemd/system/teensy-listener.service
rm -f "$SERVICE_TMP"
sudo systemctl daemon-reload
sudo systemctl enable teensy-listener.service
sudo systemctl restart teensy-listener.service

echo "Teensy listener service installed and started."
systemctl --no-pager --full status teensy-listener.service | sed -n '1,20p'
