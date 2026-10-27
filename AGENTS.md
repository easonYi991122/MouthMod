# AGENTS.md — MouthMod (for coding agents: Codex, Claude Code, …)

Hands-free Minecraft Education controller for a JHU PDBMI course project. One
**Seeed XIAO nRF52840 Sense** acts as a USB keyboard + mouse: tongue joystick
→ W/A/S/D, bite → Space, puff → left mouse button, sip → Space (long sip disarms/arms), head tilt
(on-board IMU) → mouse look. A Python bridge on the Mac shows status, tunes
parameters and lets output through only while Minecraft is the front app.

The team works in **Simplified Chinese**; human docs are in Chinese. Answer
the user in Chinese unless they write in another language.

## Facts (machine-readable)

```yaml
repo: https://github.com/easonYi991122/MouthMod   # branch main
firmware_version_file: VERSION                    # also cfg::kFirmwareVersion in firmware/ImuBench/src/Config.h
host: macOS (Apple Silicon or Intel). Windows/Linux: build may work, bridge focus gate is macOS-only
board:
  name: Seeed XIAO nRF52840 Sense          # not "Sense Plus"
  fqbn: Seeeduino:nrf52:xiaonRF52840Sense
  core: Seeeduino:nrf52@1.1.13             # "Seeed nRF52 Boards"; NOT the "mbed-enabled" core
  board_manager_url: https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
  libraries: ["Seeed Arduino LSM6DS3@2.0.7"]
  do_not_install: ["Adafruit TinyUSB Library"]   # core bundles TinyUSB 1.7.0; a 3.x copy breaks the build
  core_binaries_arch: x86_64               # gcc + adafruit-nrfutil -> Apple Silicon needs Rosetta 2
  macos_fix: copy tools/macos/platform.local.txt into ~/Library/Arduino15/packages/Seeeduino/hardware/nrf52/1.1.13/
sketches:
  ImuBench: firmware/ImuBench      # the full firmware; this is what gets released
  ImuLookMini: firmware/ImuLookMini # single-file IMU-only demo
outputs:
  uf2: build/ImuBench.uf2           # build/ is gitignored
serial:
  port_glob: /dev/cu.usbmodem*
  one_owner_at_a_time: [bridge.py, Arduino serial monitor, Chrome Web Serial, arduino-cli upload]
bridge:
  cmd: python3 tools/look_demo/bridge.py [--http PORT|0] [--serial DEV] [--focus-app SUBSTR] [--exit-with-parent]
  default_http: 8765               # pick another (e.g. 8766) if taken
  pages: ["/ (full test page)", "/float.html (floating debug window)"]
float_app:  # preferred UI (D32); browser + bridge is the backup
  source: tools/float_app/MouthModFloat.swift
  build: scripts/build-float-app.sh   # -> "build/MouthMod Float.app", needs only Command Line Tools
  runs: /usr/bin/python3 <app>/Contents/Resources/look_demo/bridge.py --http 0 --exit-with-parent
  extra_bridge_args_env: MOUTHMOD_BRIDGE_ARGS   # e.g. "--serial /dev/cu.usbmodem1101"
  note: bundles a copy of tools/look_demo; rebuild after changing bridge.py or the pages
  api: {GET: [/bridge-info, /events (SSE, one board line per event), /profile], POST: [/cmd (body = one command line), /profile (saved command lines; empty body clears)]}
  profile: ~/.mouthmod/profile.txt (--profile PATH); replayed 1.5 s after every serial connect (D36)
pins:  # frozen by the spec; do not change
  A0: joystick X wiper
  A1: joystick Y wiper
  D2/D3: HX710B #1 OUT/SCK (breath)
  D4/D5: HX710B #2 OUT/SCK (bite)
  D6: caregiver switch to GND
  power: 3V3 only, never 5 V
```

## Deploy on a new Mac (do these in order)

Each step has a check. Steps marked **HUMAN** need the user (GUI, admin
password, or physical action): ask them and wait. Do not run `sudo`.

1. **Repo.** `git clone https://github.com/easonYi991122/MouthMod.git && cd MouthMod`
   (or `git pull` in an existing clone).
2. **Self-check.** `./scripts/setup-macos.sh --check` — prints one `[ok]`/`[FAIL]`
   line per item and `SUMMARY fails=N`. Fix each `[FAIL]` with the step below.
3. **HUMAN — Command Line Tools** (if `[FAIL] Command Line Tools`): user runs
   `xcode-select --install` and clicks through the dialog.
4. **HUMAN — Rosetta 2** (Apple Silicon only, if `[FAIL] Rosetta 2`): user runs
   `softwareupdate --install-rosetta --agree-to-license`. Without it the build
   fails with `bad CPU type in executable`.
5. **arduino-cli** (if `[FAIL] arduino-cli not found`): either the user installs
   **Arduino IDE 2** (it bundles arduino-cli; the script finds it), or
   `brew install arduino-cli` (ask before installing). Arduino IDE is also
   what the humans use, so prefer it.
6. **Install and build.** `./scripts/setup-macos.sh` — installs the board core
   (≈200 MB download), the LSM6DS3 library and the macOS fix, then runs the
   host tests and builds. Success = `SUMMARY fails=0`,
   `[ok] firmware built: build/ImuBench.uf2` and `[ok] floating window app: …`.
7. **HUMAN — flash.** Plug in the board, user **double-taps RESET** quickly;
   a drive `XIAO-SENSE` appears. Then:
   `cp build/ImuBench.uf2 /Volumes/XIAO-SENSE/` (the drive ejects itself; a
   Finder warning about ejecting is harmless). Alternative without the
   double-tap: stop the bridge, then
   `arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn Seeeduino:nrf52:xiaonRF52840Sense --input-dir build/ImuBench`
   (`arduino-cli` = the path the self-check printed; the uploader needs Rosetta too).
   Or download `ImuBench.uf2` from the latest GitHub Release instead of building.
8. **Verify the board.** Start the bridge (step "Run") and read
   `curl -s http://127.0.0.1:PORT/bridge-info` → `"port": "/dev/cu.usbmodem…"`.
   Then `curl -s -N --max-time 3 http://127.0.0.1:PORT/events | grep '#I'` must show
   `fw=MouthMod-ImuBench,ver=<VERSION>` and `imu_ok=1`.

## Run (daily use)

Preferred: `open "build/MouthMod Float.app"` — an always-on-top panel that
never takes focus from Minecraft; it starts its own bridge on a free port
(find it with `lsof -nP -iTCP -sTCP:LISTEN | grep Python`). Backup:

```bash
python3 tools/look_demo/bridge.py --http 8766   # leave running; Ctrl+C stops it and disarms the board
```

then the human opens `http://localhost:8766/float.html` in Chrome/Edge and
clicks "弹出置顶悬浮窗". Never run both: one serial owner. Full usage and
command list: `docs/USAGE.md`.
Agents can drive the board through the bridge:
`curl -s -X POST --data 'get' http://127.0.0.1:8766/cmd`.
Commands that make the computer receive keys/mouse (`arm` when HID is on,
`act`, `tap`, `turn`) act on the user's real desktop: ask first.

## Build / test / release

```bash
./scripts/test-native.sh            # host tests of firmware/ImuBench/src/logic (needs c++), must print "all ... tests passed"
./scripts/build-firmware.sh         # -> build/ImuBench.uf2 ; ImuLookMini as argument for the mini sketch
```

Release = bump `VERSION` and `kFirmwareVersion` together, run both scripts,
commit, push, `gh release create vX.Y.Z build/ImuBench.uf2`. Only when the
user asks.

## Rules for changes

- `firmware/ImuBench/src/logic/` is plain C++ with no Arduino headers; every
  behaviour change there gets a host test in `tests/native/`.
- Hardware access lives in `src/hardware/`, wiring and defaults in `src/Config.h`.
- Spec-frozen items (pins, 3V3, input mapping, arm/disarm rules) change only
  by team decision; record any decision as a new row `D<n>` in
  `docs/imu-decisions.md` and update `docs/USAGE.md`.
- Keep the serial protocol and the two pages (`tools/look_demo/index.html`,
  `float.html`) in sync with `App.cpp` (`#P/#Q/#K/#E/#F/D/S` lines).
- Stop the bridge before uploading; only one program may hold the port.

## Known errors

| Symptom | Fix |
|---|---|
| `exec: "python": executable file not found` at the end of compile | platform.local.txt not copied (step 6) |
| `bad CPU type in executable` | Rosetta 2 missing (step 4) |
| TinyUSB redefinition / `Adafruit_USBD_*` errors | a separate Adafruit TinyUSB Library is installed; remove it |
| board not listed / wrong core | installed the mbed core; install `Seeeduino:nrf52@1.1.13` |
| bridge: `Address already in use` | another program owns the HTTP port: `--http 8766` |
| bridge: port busy / no lines | Arduino serial monitor or Chrome Web Serial holds the port |
| `#I ... imu_ok=0` | wrong board selected (needs the Sense) or IMU fault; power-cycle |
