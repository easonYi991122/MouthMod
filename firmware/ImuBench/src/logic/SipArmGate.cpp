#include "SipArmGate.h"

bool SipDisarmGate::update(bool armed, bool sipActive, uint64_t sipHeldUs) {
  if (!sipActive) fired_ = false;
  if (!armed || !sipActive || fired_ || holdMs_ == 0 ||
      sipHeldUs < static_cast<uint64_t>(holdMs_) * 1000U) return false;
  fired_ = true;
  return true;
}

// Starts not eligible: a sip already held at power-up must be released.
SipArmGate::SipArmGate(uint32_t holdUs) : holdUs_(holdUs), eligible_(false) {}

bool SipArmGate::update(bool disarmed, bool enable, bool sipActive,
                        uint64_t sipHeldUs) {
  if (!disarmed || !enable) {
    eligible_ = false;  // must see a release while DISARMED and enabled
    return false;
  }
  // Release = the sip level dropped (not a dip of the hold timer, which
  // restarts whenever pressure wobbles between the sip on/off bands).
  if (!sipActive) {
    eligible_ = true;
    return false;
  }
  if (eligible_ && sipHeldUs >= holdUs_) {
    eligible_ = false;  // one arm per sip
    return true;
  }
  return false;
}
