// nRF52840 hardware watchdog (500 ms, [Z] section 6) and reset reason.
//
// Once started the WDT cannot be stopped until the next reset; it keeps
// counting while the debugger halts the CPU (paused) and in sleep (runs).
// Fed only after the loop's sensor validation (guide §f step 4), never from
// a timer, so a hung loop really resets. Off in the bench build until the
// bootloader/DFU path has been checked with it running (cfg).
#ifndef MOUTHMOD_WATCHDOG_H
#define MOUTHMOD_WATCHDOG_H

#include <stdint.h>

class Watchdog {
 public:
  Watchdog();

  void begin(uint32_t timeoutMs);
  void feed();
  bool running() const { return running_; }

  // POWER->RESETREAS captured at boot (then cleared), e.g. 0x2 = watchdog.
  static uint32_t captureResetReason();
  static const char* resetReasonName(uint32_t reason);

 private:
  bool running_;
};

#endif
