#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--bridge_help" ]]; then
  cat <<'EOF'
Bridge upload help

Compile only:
  pio run

Compile and upload to Raspberry Pi bridge:
  pio run -t bridge_send

Optional overrides:
  TEENSY_BRIDGE_HOST=teensybridge.local \
  TEENSY_BRIDGE_USER=owhite \
  TEENSY_BRIDGE_DROP_DIR=/home/owhite/teensy_drops \
  pio run -t bridge_send

Wrapper shortcuts:
  ./pio_bridge.sh --bridge_send
EOF
  exit 0
fi

if [[ "${1:-}" == "--bridge_send" ]]; then
  shift
  pio run "$@" -t bridge_send
  exit 0
fi

pio run "$@"
