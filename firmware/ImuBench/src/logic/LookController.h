// Bench controller for the head-look channel: owns mount mapping, filter,
// calibration, mapper and a reduced copy of the V5 safety states.
//
// Mapping to the V5 state machine ([Z] section 6):
//   DISARMED     power-up default, no output
//   CALIBRATING  3 s still window (+ the 300 ms neutral gate = WAIT_NEUTRAL)
//   ACTIVE       head tilt -> mouse velocity
//   FAULT        IMU timeout / init failure; latched until acknowledged
// On the bench the long-sip arm gesture is replaced by an explicit arm
// request (serial command) and the D6 toggle by acknowledgeFault().
// The pure logic never reads clocks or pins: time and samples are passed in.
#ifndef MOUTHMOD_LOOK_CONTROLLER_H
#define MOUTHMOD_LOOK_CONTROLLER_H

#include <stdint.h>

#include "ComplementaryFilter.h"
#include "MountFrame.h"
#include "MouseLookMapper.h"
#include "StillCalibration.h"
#include "Vec3.h"

enum LookState {
  LOOK_DISARMED = 0,
  LOOK_CALIBRATING,
  LOOK_WAIT_NEUTRAL,
  LOOK_ACTIVE,
  LOOK_FAULT
};

const char* lookStateName(LookState s);

struct ImuInput {
  bool fresh;        // a new sample was read this tick
  uint64_t sampleUs; // time the fresh sample was read
  Vec3 accelG;       // sensor frame
  Vec3 gyroDps;      // sensor frame, raw (bias not removed)
};

struct LookTick {
  LookState state;
  float rollDeg;
  float pitchDeg;
  float rollErrDeg;
  float pitchErrDeg;
  LookVelocity velocity;   // zero unless ACTIVE
  bool releaseAll;         // state left ACTIVE / entered a no-output state
  const char* event;       // static string or 0
};

struct LookTiming {
  uint32_t imuTimeoutUs;     // no fresh sample for this long -> FAULT
  uint32_t neutralHoldUs;    // 300 ms neutral before ACTIVE
};

class LookController {
 public:
  LookController();

  void setTiming(const LookTiming& t) { timing_ = t; }
  const LookTiming& timing() const { return timing_; }

  // Configuration. Changing the mount invalidates bias/neutral -> DISARMED.
  bool setMount(SignedAxis up, SignedAxis forward);
  const MountFrame& mount() const { return mount_; }
  void setAlpha(float alpha) { filter_.setAlpha(alpha); }
  float alpha() const { return filter_.alpha(); }
  void setLookParams(const MouseLookParams& p) { mapper_.setParams(p); }
  const MouseLookParams& lookParams() const { return mapper_.params(); }
  StillCalibration& calibration() { return calibration_; }

  // Requests; take effect on the next tick().
  void requestArm() { armRequested_ = true; }
  void requestDisarm() { disarmRequested_ = true; }
  void acknowledgeFault() { ackRequested_ = true; }
  void reportFault(const char* reason);
  // Joystick / pressure channels neutral; WAIT_NEUTRAL needs this too.
  void setOtherInputsNeutral(bool neutral) { otherNeutral_ = neutral; }

  // enableInput mirrors the caregiver switch (true when not wired).
  LookTick tick(uint64_t nowUs, const ImuInput& imu, bool enableInput);

  LookState state() const { return state_; }
  Vec3 gyroBiasDps() const { return bias_; }
  float roll0Deg() const { return roll0_; }
  float pitch0Deg() const { return pitch0_; }
  bool calibrated() const { return calibrated_; }
  uint64_t lastSampleUs() const { return lastSampleUs_; }
  bool hasSample() const { return hasSample_; }
  Vec3 lastAccelBody() const { return lastAccelBody_; }
  Vec3 lastGyroBody() const { return lastGyroBody_; }  // bias removed

 private:
  void enter(LookState s, const char* event, LookTick* out);

  LookTiming timing_;
  MountFrame mount_;
  ComplementaryFilter filter_;
  StillCalibration calibration_;
  MouseLookMapper mapper_;

  LookState state_;
  bool armRequested_;
  bool disarmRequested_;
  bool ackRequested_;
  bool mountChanged_;
  const char* pendingFault_;

  bool hasSample_;
  uint64_t lastSampleUs_;
  uint64_t bootUs_;
  bool bootSet_;
  uint64_t neutralSinceUs_;
  bool neutralTiming_;
  bool otherNeutral_;
  uint64_t lastMapUs_;

  bool calibrated_;
  Vec3 bias_;
  float roll0_;
  float pitch0_;
  Vec3 lastAccelBody_;
  Vec3 lastGyroBody_;
};

#endif
