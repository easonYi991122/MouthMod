// D39 five-pose calibration. No hardware, clocks or serial dependencies.
//
// v0.8.1: the poses only define the head's own left/right and down/up turn
// axes (unit vectors in head-deviation space, see HeadDeviation.h). The look
// errors stay in degrees along those axes, so dead band, saturation and gain
// keep their meaning. The neutral pose is only the origin for the other four;
// every basic calibration replaces it with the new neutral (D40).
#ifndef MOUTHMOD_FINE_CALIBRATION_H
#define MOUTHMOD_FINE_CALIBRATION_H

#include <stdint.h>
#include "HeadDeviation.h"
#include "Vec3.h"

class FineCalibration {
 public:
  FineCalibration();
  static int poseIndex(char pose);  // n,l,r,d,u
  bool has(unsigned pose) const { return pose < 5 && have_[pose]; }
  bool complete() const;
  // n: absolute roll/pitch (yaw 0); l,r,d,u: head deviation from n, degrees.
  Vec3 pose(unsigned i) const { return poses_[i]; }
  bool on() const { return on_; }
  void off() { on_ = false; }
  void clear();
  bool setNeutral(float rollDeg, float pitchDeg);  // keeps l,r,d,u
  bool setRelative(char pose, const Vec3& deviation);
  // v0.8.0 profile lines: absolute roll/pitch, yaw relative to n.
  bool setAbsolute(char pose, const Vec3& angles);
  // Null on success; otherwise fcal_incomplete / fcal_bad_h / fcal_bad_v /
  // fcal_bad_axes. Stays off on failure.
  const char* enable();
  Vec3 axis(unsigned a) const { return axes_[a < 2 ? a : 1]; }
  // Deviation (or its rate) -> look x (+right) / y (+down) along the axes,
  // decoupled so a pure left/right pose gives no up/down error.
  void project(const Vec3& d, float* x, float* y) const;

  // Caller enforces controller state and feeds only fresh samples after start.
  const char* start(char pose, uint64_t nowUs, bool calibrated);
  void cancel() { capturing_ = -1; }
  bool capturing() const { return capturing_ >= 0; }
  char capturePose() const;
  // 0 collecting/idle, 1 captured, -1 fail_motion. Old pose survives failure.
  int addSample(uint64_t sampleUs, const Vec3& up, float yawDeg);
  float capturedNeutralYaw() const { return capturedNeutralYaw_; }

 private:
  HeadFrame neutralFrame() const;
  bool have_[5];
  Vec3 poses_[5];
  bool on_;
  Vec3 axes_[2];
  float cross_;
  int capturing_;
  uint64_t startUs_, lastUs_;
  uint32_t count_;
  HeadFrame frame_;
  Vec3 sum_, upSum_, min_, max_;
  bool motion_;
  float capturedNeutralYaw_;
};

#endif
