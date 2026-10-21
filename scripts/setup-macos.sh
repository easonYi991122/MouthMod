#!/usr/bin/env bash
# One-shot macOS setup and self-check for MouthMod. Safe to run again.
#
#   scripts/setup-macos.sh          install what is missing, then build + test
#   scripts/setup-macos.sh --check  only report (still runs tests and builds into build/)
#
# Installs only user-level Arduino data (board core, library, the macOS
# platform.local.txt fix). Never installs system software: Xcode Command Line
# Tools, Rosetta 2 and arduino-cli itself are reported with the command a
# human should run. Every line starts with [ok], [fix] or [FAIL]; the exit
# code is the number of [FAIL] lines.
set -uo pipefail
cd "$(dirname "$0")/.."
. scripts/_arduino.sh

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1
FAILS=0
ok()   { echo "[ok]   $*"; }
fixed(){ echo "[fix]  $*"; }
fail() { echo "[FAIL] $*"; FAILS=$((FAILS + 1)); }

# 1. Platform and system tools.
[ "$(uname -s)" = "Darwin" ] || { fail "not macOS"; exit 1; }
ok "macOS $(sw_vers -productVersion) $(uname -m)"
if xcode-select -p >/dev/null 2>&1 && command -v c++ >/dev/null && [ -x /usr/bin/python3 ]; then
  ok "Command Line Tools: c++ and /usr/bin/python3 present"
else
  fail "Command Line Tools missing -> human runs: xcode-select --install"
fi
if [ "$(uname -m)" = "arm64" ]; then
  # The board core's gcc and adafruit-nrfutil are x86_64 binaries.
  if arch -x86_64 /usr/bin/true >/dev/null 2>&1; then
    ok "Rosetta 2 installed (needed for the x86_64 board toolchain)"
  else
    fail "Rosetta 2 missing -> human runs: softwareupdate --install-rosetta --agree-to-license"
  fi
fi

# 2. arduino-cli.
if CLI=$(find_cli); then
  ok "arduino-cli: $CLI ($("$CLI" version 2>/dev/null | awk '{print $3}'))"
else
  fail "arduino-cli not found -> install Arduino IDE 2 (bundles it) or: brew install arduino-cli"
  echo "SUMMARY fails=$FAILS"; exit "$FAILS"
fi

# 3. Board core Seeeduino:nrf52 1.1.13 (not the mbed one).
if "$CLI" core list 2>/dev/null | grep -Eq '^Seeeduino:nrf52 +1\.1\.13 '; then
  ok "core $CORE"
elif [ $CHECK = 1 ]; then
  fail "core $CORE not installed"
else
  "$CLI" core update-index --additional-urls "$BOARD_URL" >/dev/null &&
    "$CLI" core install "$CORE" --additional-urls "$BOARD_URL" >/dev/null &&
    fixed "installed core $CORE" || fail "core install failed (network? see output above)"
fi

# 4. Library: LSM6DS3 2.0.7; no separately installed TinyUSB.
if "$CLI" lib list 2>/dev/null | grep -Eq '^Seeed Arduino LSM6DS3 +2\.0\.7 '; then
  ok "library $LIB"
elif [ $CHECK = 1 ]; then
  fail "library $LIB not installed"
else
  "$CLI" lib install "$LIB" >/dev/null && fixed "installed library $LIB" || fail "library install failed"
fi
if "$CLI" lib list 2>/dev/null | grep -q '^Adafruit TinyUSB Library '; then
  fail "a separate 'Adafruit TinyUSB Library' is installed; it conflicts with the core's own 1.7.0 -> human removes it: $CLI lib uninstall \"Adafruit TinyUSB Library\""
else
  ok "no separate TinyUSB library (the core's bundled one is used)"
fi

# 5. macOS fix: platform.txt calls "python", which macOS lacks.
if [ -d "$CORE_DIR" ]; then
  if cmp -s tools/macos/platform.local.txt "$CORE_DIR/platform.local.txt"; then
    ok "platform.local.txt in place"
  elif [ $CHECK = 1 ]; then
    fail "platform.local.txt missing in $CORE_DIR"
  else
    cp tools/macos/platform.local.txt "$CORE_DIR/" && fixed "copied platform.local.txt (restart Arduino IDE if open)"
  fi
fi

# 6. Host tests and firmware build (skipped if anything above failed).
if [ $FAILS = 0 ]; then
  if ./scripts/test-native.sh >/dev/null 2>&1; then ok "host tests passed"; else fail "host tests: run ./scripts/test-native.sh"; fi
  if out=$(./scripts/build-firmware.sh ImuBench 2>&1); then
    ok "firmware built: build/ImuBench.uf2 ($(echo "$out" | grep -o 'Sketch uses [0-9]* bytes'))"
  else
    echo "$out" | tail -15; fail "firmware build failed"
  fi
fi

# 7. Board (information only).
PORTS=$(ls /dev/cu.usbmodem* 2>/dev/null | tr '\n' ' ')
if [ -n "$PORTS" ]; then ok "board serial port(s): $PORTS"; else echo "[info] no /dev/cu.usbmodem* (board not plugged in, or in bootloader)"; fi
[ -d /Volumes/XIAO-SENSE ] && echo "[info] XIAO-SENSE drive mounted: cp build/ImuBench.uf2 /Volumes/XIAO-SENSE/ flashes it"

echo "SUMMARY fails=$FAILS"
exit "$FAILS"
