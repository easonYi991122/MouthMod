#!/usr/bin/env bash
# Compiles a sketch and writes build/<Sketch>.uf2 (drag it onto XIAO-SENSE).
# Usage: scripts/build-firmware.sh [ImuBench|ImuLookMini]   (default ImuBench)
set -euo pipefail
cd "$(dirname "$0")/.."
. scripts/_arduino.sh
SKETCH=${1:-ImuBench}
CLI=$(find_cli) || { echo "ERROR: arduino-cli not found; run scripts/setup-macos.sh" >&2; exit 1; }
"$CLI" compile --fqbn "$FQBN" --warnings all --output-dir "build/$SKETCH" "firmware/$SKETCH"
/usr/bin/python3 "$CORE_DIR/tools/uf2conv/uf2conv.py" -f 0xADA52840 -c \
  -o "build/$SKETCH.uf2" "build/$SKETCH/$SKETCH.ino.hex" >/dev/null
echo "OK build/$SKETCH.uf2"
