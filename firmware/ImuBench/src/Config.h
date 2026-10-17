// ImuBench configuration. Pins follow [Z] section 5; values marked
// "provisional" are bench choices, not V5 frozen values (docs/imu-decisions.md).
#ifndef MOUTHMOD_CONFIG_H
#define MOUTHMOD_CONFIG_H

#include <stdint.h>

namespace cfg {

// --- Build / identity ---
static const char* const kFirmwareName = "MouthMod-ImuBench";
static const char* const kFirmwareVersion = "0.1.0";

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
static const float kDeadbandDeg = 3.0f;
static const float kHysteresisDeg = 2.0f;
static const float kSaturationDeg = 20.0f;
static const float kKxCpsPerDeg = 40.0f;      // provisional, counts/s per deg
static const float kKyCpsPerDeg = 30.0f;      // provisional
static const float kMaxPendingCounts = 64.0f; // stale-motion cap per axis

// --- Safety inputs ---
// D6 caregiver switch (INPUT_PULLUP, closed to GND = enable). Off on the
// bench until the switch is wired; then the serial "arm" also needs D6 closed.
static const bool kUseD6Enable = false;
static const uint8_t kPinEnable = 6;          // D6
// Bench-only: closing the serial port (DTR low) disarms, so closing the
// demo page always stops the mouse.
static const bool kDisarmOnSerialClose = true;
// Real HID mouse output starts disabled; enable with "hid 1".
static const bool kHidEnabledAtBoot = false;
// Watchdog is not started in the bench build (guide section f, step 1).
static const bool kWatchdogStarted = false;

// --- Logging ---
// USB CDC ignores baud, but never use 1200 (it triggers the bootloader).
static const uint32_t kSerialBaud = 115200;
static const uint16_t kDefaultTelemetryHz = 50;

}  // namespace cfg

#endif
