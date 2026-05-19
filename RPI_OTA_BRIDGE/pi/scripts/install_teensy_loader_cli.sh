#!/usr/bin/env bash
set -euo pipefail

REPO_URL="https://github.com/PaulStoffregen/teensy_loader_cli.git"
BUILD_DIR="/tmp/teensy_loader_cli"

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential \
  libusb-dev \
  git

rm -rf "$BUILD_DIR"
git clone --depth 1 "$REPO_URL" "$BUILD_DIR"

pushd "$BUILD_DIR" >/dev/null
make
sudo install -m 0755 teensy_loader_cli /usr/local/bin/teensy_loader_cli
popd >/dev/null

echo "Installed teensy_loader_cli to /usr/local/bin/teensy_loader_cli"
/usr/local/bin/teensy_loader_cli --help | head -n 5
