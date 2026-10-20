// One HX710B pressure channel -> puff / sip (breath, module #1) or bite
// (module #2) levels.
//
// Ported from the team's puff sketch (HX710B, 120000 / 50000 counts,
// 250 ms release delay, slow baseline), changed to the guide's contract:
// - signed: p = sign * (raw - baseline); puff/bite > 0, sip < 0. The port
//   used |delta|, which made a sip click too; sign must be measured;
// - trigger needs 2 consecutive *fresh* samples (40 Hz -> 50 ms);
// - baseline seeded from rest samples without blocking, then tracked with
//   tau = 10 s only while |p| < 0.3 x trigger (frozen during actions);
// - sipHeldUs() lets the caller disarm after >= 150 ms below threshold;
// - no sip while a puff is held or within sipLockoutUs after it releases:
//   a hard puff rebounds below baseline and must not disarm (D31).
// Pure C++: the caller passes samples and their read time.
#ifndef MOUTHMOD_PRESSURE_CHANNEL_H
#define MOUTHMOD_PRESSURE_CHANNEL_H

#include <stdint.h>

// HX710B: 24-bit two's complement, MSB first (guide §HX710B).
int32_t hx710bToSigned(uint32_t bits);

struct PressureParams {
  int8_t sign;              // +1 or -1 so that puff/bite read positive
  int32_t onCounts;         // positive trigger
  int32_t offCounts;        // positive release (< onCounts)
  int32_t sipOnCounts;      // negative trigger magnitude; 0 = no sip side
  int32_t sipOffCounts;     // negative release magnitude
  uint8_t confirmSamples;   // consecutive fresh samples to trigger
  uint32_t releaseHoldUs;   // p must stay <= off this long to release
  uint32_t sipLockoutUs;    // no sip this long after a puff releases
  float baselineTauUs;      // slow tracking time constant
  float freezeFraction;     // track only while |p| < this x onCounts
  uint8_t seedDiscard;      // samples dropped after reset (mode settling)
  uint8_t seedSamples;      // rest samples averaged into the first baseline
};

PressureParams defaultBreathParams();
PressureParams defaultBiteParams();

class PressureChannel {
 public:
  PressureChannel();

  void setParams(const PressureParams& p) { params_ = p; }
  const PressureParams& params() const { return params_; }

  // Drops the baseline and levels; the next rest samples re-seed it.
  void reset();

  // Call only for a fresh conversion.
  void addSample(uint64_t sampleUs, int32_t raw);

  bool seeded() const { return seeded_; }
  bool active() const { return active_; }   // puff / bite
  bool sip() const { return sip_; }
  // How long p has been below -sipOnCounts (0 if not); for the disarm rule.
  uint64_t sipHeldUs(uint64_t nowUs) const;
  // No action and |p| inside the release bands (neutral gate).
  bool neutral() const;

  int32_t raw() const { return raw_; }
  int32_t baseline() const { return static_cast<int32_t>(baseline_); }
  int32_t pressure() const { return p_; }
  uint64_t lastSampleUs() const { return lastUs_; }
  bool hasSample() const { return hasSample_; }

 private:
  PressureParams params_;
  bool hasSample_;
  uint64_t lastUs_;
  int32_t raw_;
  int32_t p_;

  bool seeded_;
  uint8_t discarded_;
  uint8_t seedN_;
  int64_t seedSum_;
  float baseline_;

  bool active_;
  uint8_t onCount_;
  bool releasing_;
  uint64_t releaseSinceUs_;
  bool puffEnded_;
  uint64_t puffEndUs_;

  bool sip_;
  uint8_t sipCount_;
  bool sipBelow_;
  uint64_t sipSinceUs_;
};

#endif
