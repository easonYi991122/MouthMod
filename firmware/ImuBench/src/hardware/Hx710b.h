// Non-blocking HX710B reader (pressure modules #1 D2/D3, #2 D4/D5).
//
// Ported from the team's readPressure(): 24 data bits MSB first plus 3 extra
// pulses = 27 total = differential input at 40 Hz for the next conversion.
// Changed per guide §HX710B: OUT high -> return at once (never wait); each
// SCK-high phase is its own short critical section that restores the prior
// interrupt state (a whole-read noInterrupts()/interrupts() would re-enable
// IRQs the caller had masked, and a stretched high phase > 60 us powers the
// chip down).
#ifndef MOUTHMOD_HX710B_H
#define MOUTHMOD_HX710B_H

#include <stdint.h>

class Hx710b {
 public:
  Hx710b(uint8_t outPin, uint8_t sckPin);

  void begin();  // SCK low, OUT input

  // True and *bits = 24-bit word if a conversion was ready.
  bool tryRead(uint32_t* bits);

  uint32_t reads() const { return reads_; }

 private:
  bool pulse();  // one SCK high/low; returns OUT sampled while high

  uint8_t out_;
  uint8_t sck_;
  uint32_t reads_;
};

#endif
