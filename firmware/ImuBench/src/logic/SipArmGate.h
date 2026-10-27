// Long-sip arm gesture ([Z] section 6): DISARMED + D6 closed + sip held
// >= 1 s -> start calibration.
//
// The short disarming sip and the long arming sip are the same gesture held
// for different times, so one continuous sip must never disarm and then
// re-arm. A sip only counts if it *started* while DISARMED: after any other
// state the gate waits to see the sip released first (guide §c, "连吸与重新
// 启用"). Pure C++: the caller passes the sip hold time from PressureChannel.
#ifndef MOUTHMOD_SIP_ARM_GATE_H
#define MOUTHMOD_SIP_ARM_GATE_H

#include <stdint.h>

// D34: runtime armed-sip delay; zero disables disarming. Time comes from
// PressureChannel so its existing sip threshold/hold semantics are retained.
class SipDisarmGate {
 public:
  explicit SipDisarmGate(uint32_t holdMs = 1500U)
      : holdMs_(holdMs), fired_(false) {}
  void setHoldMs(uint32_t ms) { holdMs_ = ms; }
  uint32_t holdMs() const { return holdMs_; }
  bool update(bool armed, bool sipActive, uint64_t sipHeldUs);
 private:
  uint32_t holdMs_;
  bool fired_;
};

class SipArmGate {
 public:
  explicit SipArmGate(uint32_t holdUs = 1000000U);

  void setHoldUs(uint32_t us) { holdUs_ = us; }
  uint32_t holdUs() const { return holdUs_; }

  // Once per loop. disarmed: state is DISARMED; enable: D6 closed (true when
  // not wired); sipActive: the sip level (its hysteresis decides release);
  // sipHeldUs: time below the trigger. Returns true once per gesture.
  bool update(bool disarmed, bool enable, bool sipActive, uint64_t sipHeldUs);

  // A sip that began elsewhere has not been released yet.
  bool waitingForRelease() const { return !eligible_; }

 private:
  uint32_t holdUs_;
  bool eligible_;
};

#endif
