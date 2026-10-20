#include "Watchdog.h"

#include <Arduino.h>

Watchdog::Watchdog() : running_(false) {}

void Watchdog::begin(uint32_t timeoutMs) {
  if (running_) return;
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
                    (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = (timeoutMs * 32768U) / 1000U;  // 32.768 kHz ticks
  NRF_WDT->RREN = WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
  running_ = true;
}

void Watchdog::feed() {
  if (running_) NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

uint32_t Watchdog::captureResetReason() {
  const uint32_t r = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = r;  // write-1-to-clear, so the next boot is fresh
  return r;
}

const char* Watchdog::resetReasonName(uint32_t r) {
  if (r & POWER_RESETREAS_DOG_Msk) return "watchdog";
  if (r & POWER_RESETREAS_LOCKUP_Msk) return "lockup";
  if (r & POWER_RESETREAS_SREQ_Msk) return "soft";
  if (r & POWER_RESETREAS_RESETPIN_Msk) return "pin";
  if (r == 0) return "power_on";
  return "other";
}
