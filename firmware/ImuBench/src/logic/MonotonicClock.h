// Provenance: source path D:/1919/Arduino/nRF52840/MonotonicClock.h; repo commit f07fffe.
// copied, not adapted
#ifndef MONOTONIC_CLOCK_H
#define MONOTONIC_CLOCK_H

#include <stdint.h>

class MonotonicClock64 {
 public:
  MonotonicClock64();

  void reset();
  uint64_t update(uint32_t rawMicros);

 private:
  bool initialized_;
  uint32_t previous_;
  uint64_t epoch_;
};

#endif
