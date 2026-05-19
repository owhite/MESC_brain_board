#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE_USER="${BRIDGE_USER:-${SUDO_USER:-$USER}}"
BRIDGE_HOME="$(getent passwd "$BRIDGE_USER" | cut -d: -f6)"

if [[ -z "$BRIDGE_HOME" ]]; then
  echo "Unable to resolve home directory for user: $BRIDGE_USER" >&2
  exit 1
fi

TEENSY_DROP_DIR="$BRIDGE_HOME/teensy_drops"

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  python3-watchdog \
  openssh-client

mkdir -p "$BRIDGE_HOME/teensy-bridge"
mkdir -p "$TEENSY_DROP_DIR/processed"
cp "$PROJECT_ROOT/bridge/bridge.py" "$BRIDGE_HOME/teensy-bridge/bridge.py"
chmod +x "$BRIDGE_HOME/teensy-bridge/bridge.py"

SERVICE_TMP="$(mktemp)"
sed \
  -e "s|__BRIDGE_USER__|$BRIDGE_USER|g" \
  -e "s|__BRIDGE_HOME__|$BRIDGE_HOME|g" \
  -e "s|__TEENSY_DROP_DIR__|$TEENSY_DROP_DIR|g" \
  "$PROJECT_ROOT/systemd/teensy-bridge.service" > "$SERVICE_TMP"

sudo cp "$SERVICE_TMP" /etc/systemd/system/teensy-bridge.service
rm -f "$SERVICE_TMP"
sudo systemctl daemon-reload
sudo systemctl enable teensy-bridge.service
sudo systemctl restart teensy-bridge.service

echo "Bridge service installed and started."
systemctl --no-pager --full status teensy-bridge.service | sed -n '1,20p'
