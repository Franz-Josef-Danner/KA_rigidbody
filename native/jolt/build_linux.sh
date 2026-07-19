#!/usr/bin/env sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
BUILD_DIR="${1:-$ROOT/build/jolt-linux}"
cmake -S "$ROOT/native/jolt" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --config Release
mkdir -p "$ROOT/vendor/jolt_bridge/linux_x86_64"
cp "$BUILD_DIR/libka_jolt_bridge.so" "$ROOT/vendor/jolt_bridge/linux_x86_64/libka_jolt_bridge.so"
echo "Installed vendor/jolt_bridge/linux_x86_64/libka_jolt_bridge.so"
