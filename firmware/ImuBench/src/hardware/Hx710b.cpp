#include "Hx710b.h"

#include <Arduino.h>

namespace {
// 25 = differential 10 Hz, 26 = not pressure, 27 = differential 40 Hz.
const int kTotalPulses = 27;
}  // namespace

Hx710b::Hx710b(uint8_t outPin, uint8_t sckPin)
    : out_(outPin), sck_(sckPin), reads_(0) {}

void Hx710b::begin() {
  // Pull-up: a missing module reads "not ready" and times out to FAULT
  // instead of floating low and returning garbage words.
  pinMode(out_, INPUT_PULLUP);
  pinMode(sck_, OUTPUT);
  digitalWrite(sck_, LOW);
}

bool Hx710b::pulse() {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  digitalWrite(sck_, HIGH);
  delayMicroseconds(1);  // T3 0.2-50 us
  const bool bit = digitalRead(out_) == HIGH;
  digitalWrite(sck_, LOW);
  __set_PRIMASK(primask);
  delayMicroseconds(1);  // T4 >= 0.2 us, interrupts allowed
  return bit;
}

bool Hx710b::tryRead(uint32_t* bits) {
  if (digitalRead(out_) == HIGH) return false;  // not ready
  uint32_t data = 0;
  for (int i = 0; i < 24; ++i) data = (data << 1) | (pulse() ? 1U : 0U);
  for (int i = 24; i < kTotalPulses; ++i) pulse();
  *bits = data;
  ++reads_;
  return true;
}
