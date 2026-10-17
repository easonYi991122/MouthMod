// Provenance: source path D:/1919/Arduino/nRF52840/MonotonicClock.cpp; repo commit f07fffe.
// copied, not adapted
#include "MonotonicClock.h"

MonotonicClock64::MonotonicClock64() {
  reset();
}

void MonotonicClock64::reset() {
  initialized_ = false;
  previous_ = 0U;
  epoch_ = 0U;
}

uint64_t MonotonicClock64::update(uint32_t rawMicros) {
  if (!initialized_) {
    initialized_ = true;
    previous_ = rawMicros;
    return rawMicros;
  }
  if (rawMicros < previous_) {
    epoch_ += (1ULL << 32);
  }
  previous_ = rawMicros;
  return epoch_ | rawMicros;
}
