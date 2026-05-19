#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

sudo cp "$PROJECT_ROOT/udev/49-teensy.rules" /etc/udev/rules.d/49-teensy.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed /etc/udev/rules.d/49-teensy.rules and reloaded udev rules."
