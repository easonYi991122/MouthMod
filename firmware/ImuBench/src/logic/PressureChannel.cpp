#include "PressureChannel.h"

int32_t hx710bToSigned(uint32_t bits) {
  int32_t raw = static_cast<int32_t>(bits & 0x7FFFFFU);
  if (bits & 0x800000U) raw -= 0x800000;
  return raw;
}

PressureParams defaultBreathParams() {
  PressureParams p;
  p.sign = 1;  // provisional: measure that a puff reads positive
  p.onCounts = 120000;   // team's tested PRESS_THRESHOLD
  p.offCounts = 50000;   // team's tested RELEASE_THRESHOLD
  p.sipOnCounts = 120000;  // provisional, mirror of the puff side
  p.sipOffCounts = 50000;
  p.confirmSamples = 2;
  p.releaseHoldUs = 250000;  // team's RELEASE_DELAY
  p.baselineTauUs = 10e6f;
  p.freezeFraction = 0.3f;
  p.seedDiscard = 5;
  p.seedSamples = 25;
  return p;
}

PressureParams defaultBiteParams() {
  PressureParams p = defaultBreathParams();
  p.sipOnCounts = 0;  // bite tube: positive side only
  p.sipOffCounts = 0;
  p.releaseHoldUs = 0;
  return p;
}

PressureChannel::PressureChannel() : params_(defaultBreathParams()) {
  hasSample_ = false;
  lastUs_ = 0;
  raw_ = 0;
  reset();
}

void PressureChannel::reset() {
  p_ = 0;
  seeded_ = false;
  discarded_ = 0;
  seedN_ = 0;
  seedSum_ = 0;
  baseline_ = 0.0f;
  active_ = false;
  onCount_ = 0;
  releasing_ = false;
  releaseSinceUs_ = 0;
  sip_ = false;
  sipCount_ = 0;
  sipBelow_ = false;
  sipSinceUs_ = 0;
}

void PressureChannel::addSample(uint64_t sampleUs, int32_t raw) {
  const uint64_t prevUs = lastUs_;
  const bool hadSample = hasSample_;
  hasSample_ = true;
  lastUs_ = sampleUs;
  raw_ = raw;

  if (!seeded_) {
    if (discarded_ < params_.seedDiscard) {
      ++discarded_;
      return;
    }
    if (raw >= 0x7FFFFF || raw <= -0x800000) {  // saturated rest: no baseline
      seedN_ = 0;
      seedSum_ = 0;
      return;
    }
    seedSum_ += raw;
    if (++seedN_ >= params_.seedSamples) {
      baseline_ = static_cast<float>(seedSum_ / seedN_);
      seeded_ = true;
    }
    return;
  }

  const float sign = params_.sign < 0 ? -1.0f : 1.0f;
  const float pf = sign * (static_cast<float>(raw) - baseline_);
  p_ = static_cast<int32_t>(pf);

  // Puff / bite: 2 fresh samples to trigger, held release band to release.
  if (!active_) {
    onCount_ = p_ >= params_.onCounts ? onCount_ + 1 : 0;
    if (onCount_ >= params_.confirmSamples) {
      active_ = true;
      onCount_ = 0;
      releasing_ = false;
    }
  } else if (p_ <= params_.offCounts) {
    if (!releasing_) {
      releasing_ = true;
      releaseSinceUs_ = sampleUs;
    }
    if (sampleUs - releaseSinceUs_ >= params_.releaseHoldUs) {
      active_ = false;
      releasing_ = false;
    }
  } else {
    releasing_ = false;
  }

  // Sip: below -sipOn; the hold time counts from the first such sample.
  if (params_.sipOnCounts > 0) {
    const bool below = p_ <= -params_.sipOnCounts;
    if (below && !sipBelow_) sipSinceUs_ = sampleUs;
    sipBelow_ = below;
    if (!sip_) {
      sipCount_ = below ? sipCount_ + 1 : 0;
      if (sipCount_ >= params_.confirmSamples) {
        sip_ = true;
        sipCount_ = 0;
      }
    } else if (p_ >= -params_.sipOffCounts) {
      sip_ = false;
    }
  }

  // Slow baseline tracking only at clear rest (never learns a held action).
  const float limit = params_.freezeFraction * static_cast<float>(params_.onCounts);
  if (!active_ && !sip_ && pf < limit && pf > -limit && hadSample &&
      sampleUs > prevUs) {
    float a = static_cast<float>(sampleUs - prevUs) / params_.baselineTauUs;
    if (a > 1.0f) a = 1.0f;
    baseline_ += a * (static_cast<float>(raw) - baseline_);
  }
}

uint64_t PressureChannel::sipHeldUs(uint64_t nowUs) const {
  if (!sip_ || !sipBelow_ || nowUs < sipSinceUs_) return 0;
  return nowUs - sipSinceUs_;
}

bool PressureChannel::neutral() const {
  if (!seeded_ || active_ || sip_) return false;
  if (p_ > params_.offCounts) return false;
  if (params_.sipOnCounts > 0 && p_ < -params_.sipOffCounts) return false;
  return true;
}
