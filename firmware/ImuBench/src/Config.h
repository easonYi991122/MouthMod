// ImuBench configuration. Pins follow [Z] section 5; values marked
// "provisional" are bench choices, not V5 frozen values (docs/imu-decisions.md).
#ifndef MOUTHMOD_CONFIG_H
#define MOUTHMOD_CONFIG_H

#include <stdint.h>

namespace cfg {

// --- Build / identity ---
static const char* const kFirmwareName = "MouthMod-ImuBench";
static const char* const kFirmwareVersion = "0.8.1";

// --- Loop ---
static const uint32_t kLoopPeriodUs = 10000;  // 100 Hz, non-blocking (frozen)

// --- IMU (on-board LSM6DS3TR-C, internal Wire1) ---
static const uint8_t kImuI2cAddress = 0x6A;   // frozen
static const uint16_t kImuOdrHz = 104;        // provisional (guide section f)
static const uint16_t kAccelRangeG = 2;       // provisional
static const uint16_t kGyroRangeDps = 245;    // provisional
static const uint32_t kImuTimeoutUs = 100000; // provisional: ~10 missed samples

// --- Mount: which sensor axis points up / forward at neutral ---
// Defaults assume the board lies flat on top of the head, chip side up,
// USB-C toward the back. Re-set over serial ("mount +Z +X") after checking.
static const char* const kDefaultMountUp = "+Z";
static const char* const kDefaultMountForward = "+X";

// --- Head look (spec starting points; k is per-person) ---
static const float kAlpha = 0.98f;
// Engage edge raised from the spec start point 3 deg to 6 deg after bench
// use (small head motions turned the camera); ramp velocity so speed starts
// near zero at the edge, and a bump must last kEngageConfirmMs (D27).
// D33: v0.7.0 panel flexibility; independent look sources and dead bands.
static const uint8_t kHorizontalSource = 0;  // roll
static const uint8_t kVerticalSource = 1;    // pitch
static const float kYawTauS = 20.0f;        // 0 disables yaw leak
static const float kDeadbandXDeg = 6.0f;
static const float kDeadbandYDeg = 6.0f;
static const uint8_t kVelMode = 1;            // 0 = spec k*e, 1 = ramp
static const float kEngageConfirmMs = 80.0f;
static const float kHysteresisDeg = 2.0f;
static const float kSaturationDeg = 20.0f;
static const float kKxCpsPerDeg = 40.0f;      // provisional, counts/s per deg
static const float kKyCpsPerDeg = 30.0f;      // provisional
static const float kMaxPendingCounts = 64.0f; // stale-motion cap per axis

// D37 fusion, D38 look law, D39 fine calibration: preserve v0.7.0 defaults
// (fine calibration starts off).
static const uint8_t kFusionMode = 0;
static const float kArmXMetres = 0.10f, kArmZMetres = -0.05f;
static const float kAccelRejectG = 0.15f;
static const uint8_t kLookMode = 0;
static const float kGyroGainX = 8.0f, kGyroGainY = 8.0f;
static const float kGyroDeadDps = 3.0f;

// --- Safety inputs ---
// D6 caregiver switch (INPUT_PULLUP, closed to GND = enable). Off on the
// bench until the switch is wired; then the serial "arm" also needs D6 closed.
static const bool kUseD6Enable = false;
static const uint8_t kPinEnable = 6;          // D6
// Bench-only: closing the serial port (DTR low) disarms, so closing the
// demo page always stops the mouse.
static const bool kDisarmOnSerialClose = true;
// Real HID output starts enabled ("hid 0" for a dry run). Output still
// needs ARMED, and with "fgate 1" Minecraft in front (D30).
static const bool kHidEnabledAtBoot = true;
// Watchdog is not started in the bench build (guide section f, step 1).
// Before turning it on: check that upload / double-tap DFU still works with
// it running (the WDT survives a soft reset into the bootloader).
static const bool kWatchdogStarted = false;
static const uint32_t kWatchdogTimeoutMs = 500;  // frozen start point

// --- Tongue joystick (pots on 3V3 only; A0 = X wiper, A1 = Y wiper) ---
// Runtime: "chan joy 0|1". Thresholds/spans: logic/JoystickMapper.cpp.
// D33: per-axis trigger/release thresholds (fraction of comfortable span).
static const float kJoyOnX = 0.12f, kJoyOnY = 0.12f;
static const float kJoyOffX = 0.08f, kJoyOffY = 0.08f;
static const bool kUseJoystick = true;
static const uint8_t kPinJoyX = 0;            // A0/D0 (frozen pin)
static const uint8_t kPinJoyY = 1;            // A1/D1 (frozen pin)
static const uint8_t kJoyAdcAverage = 4;      // samples per read (as the port)
static const uint32_t kJoySpanCaptureUs = 6000000;  // "jspan" window

// --- Pressure modules (HX710B, 27 pulses = 40 Hz) ---
// #1 breath (puff = left button, sip = jump/disarm), #2 bite (Space). Runtime:
// "chan breath 0|1", "chan bite 0|1". Thresholds: logic/PressureChannel.cpp.
static const bool kUseBreath = true;
static const bool kUseBite = false;           // no bite hardware code yet
static const uint8_t kPinBreathOut = 2;       // D2 (frozen pin)
static const uint8_t kPinBreathSck = 3;       // D3
static const uint8_t kPinBiteOut = 4;         // D4
static const uint8_t kPinBiteSck = 5;         // D5
static const uint32_t kPressureTimeoutUs = 250000;   // provisional (port: 350 ms)
static const uint32_t kPressureGraceUs = 1000000;    // after boot / enable
static const bool kSipDisarms = true;
// D33: puff mode 0 hold / 1 toggle / 2 repeat; period in milliseconds.
static const uint8_t kPuffMode = 0;
static const uint32_t kPuffRepeatMs = 600;
// D34: sip -> one jump tap; a longer sip can still disarm (0 disables it).
static const bool kSipJump = true;
static const uint32_t kSipDisarmMs = 1500;
// Long sip arms ([Z] section 6): DISARMED + D6 closed + sip >= 1 s. The
// serial "arm" stays as the bench fallback. With kDisarmOnSerialClose the
// enable input also needs the serial port open.
static const bool kSipArms = true;
static const uint32_t kSipArmUs = 1000000;    // frozen start point

// --- Focus gate (host tells the board when Minecraft is frontmost) ---
// "fgate 1" turns it on; the host then repeats "focus 1" while Minecraft is
// in front. No refresh within this time counts as "not in front", so a dead
// host program also stops the output. Applies to real HID output only.
static const uint32_t kFocusHoldUs = 700000;

// --- Logging ---
// USB CDC ignores baud, but never use 1200 (it triggers the bootloader).
static const uint32_t kSerialBaud = 115200;
static const uint16_t kDefaultTelemetryHz = 50;
static const uint16_t kSensorLineHz = 20;     // "S," channel lines

}  // namespace cfg

#endif
