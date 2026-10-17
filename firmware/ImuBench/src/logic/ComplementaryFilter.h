// Roll/pitch complementary filter in the head (body) frame.
//
// Conventions (body frame x fwd, y left, z up; ZYX Euler, yaw ignored):
//   roll  > 0 : right ear down          roll_acc  = atan2(ay, az)
//   pitch > 0 : nose down (look down)   pitch_acc = atan2(-ax, sqrt(ay^2+az^2))
// Gyro input must already have the bias removed. Yaw is never integrated.
//
// alpha is specified at the nominal 100 Hz step (spec: alpha ~= 0.98) and is
// converted to a time constant so irregular real dt gives the same blending.
#ifndef MOUTHMOD_COMPLEMENTARY_FILTER_H
#define MOUTHMOD_COMPLEMENTARY_FILTER_H

#include "Vec3.h"

class ComplementaryFilter {
 public:
  static const float kNominalDtS;   // 0.01 s, the step alpha refers to
  static const float kMaxDtS;       // larger gaps re-seed from the accelerometer
  static const float kAccelMinG;    // accel norm window for using gravity
  static const float kAccelMaxG;

  ComplementaryFilter();

  void reset();
  void setAlpha(float alpha);
  float alpha() const { return alpha_; }

  // Seeds the angles directly from one accelerometer sample.
  bool seed(const Vec3& accelG);

  // One fresh IMU sample. Returns false (state unchanged) on non-finite input
  // or non-positive dt. A gap > kMaxDtS re-seeds from the accelerometer.
  bool update(const Vec3& accelG, const Vec3& gyroDps, float dtS);

  bool initialized() const { return initialized_; }
  float rollDeg() const { return rollDeg_; }
  float pitchDeg() const { return pitchDeg_; }
  bool lastUsedAccel() const { return lastUsedAccel_; }

  static float accelRollDeg(const Vec3& a);
  static float accelPitchDeg(const Vec3& a);

 private:
  float alpha_;
  float tauS_;
  bool initialized_;
  bool lastUsedAccel_;
  float rollDeg_;
  float pitchDeg_;
};

#endif
