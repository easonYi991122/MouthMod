# Shared by setup-macos.sh and build-firmware.sh. Sourced, not run.
FQBN=Seeeduino:nrf52:xiaonRF52840Sense
CORE=Seeeduino:nrf52@1.1.13
LIB="Seeed Arduino LSM6DS3@2.0.7"
BOARD_URL=https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
CORE_DIR="${ARDUINO_DIRECTORIES_DATA:-$HOME/Library/Arduino15}/packages/Seeeduino/hardware/nrf52/1.1.13"
IDE_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"

# Prints the arduino-cli to use: $ARDUINO_CLI, then PATH, then the copy
# bundled with Arduino IDE 2. Fails if there is none.
find_cli() {
  if [ -n "${ARDUINO_CLI:-}" ]; then
    echo "$ARDUINO_CLI"
  elif command -v arduino-cli >/dev/null 2>&1; then
    command -v arduino-cli
  elif [ -x "$IDE_CLI" ]; then
    echo "$IDE_CLI"
  else
    return 1
  fi
}
