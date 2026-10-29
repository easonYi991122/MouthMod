// D39 five-pose calibration. No hardware, clocks or serial dependencies.
#ifndef MOUTHMOD_FINE_CALIBRATION_H
#define MOUTHMOD_FINE_CALIBRATION_H

#include <stdint.h>
#include "Vec3.h"

class FineCalibration {
 public:
  FineCalibration();
  static int poseIndex(char pose);  // n,l,r,d,u
  bool has(unsigned pose) const { return pose < 5 && have_[pose]; }
  Vec3 pose(unsigned i) const { return poses_[i]; }
  bool on() const { return on_; }
  void off() { on_ = false; }
  void clear();
  bool autoSource() const { return autoSource_; }
  void setAutoSource(bool on) { autoSource_ = on; }
  bool setPose(char pose, const Vec3& angles);
  // Null on success; otherwise fcal_incomplete / fcal_bad_h / fcal_bad_v.
  const char* enable(bool yawKnown, uint8_t* hsrc, uint8_t* vsrc);
  bool validSources(bool yawKnown, uint8_t hsrc, uint8_t vsrc) const;
  float map(unsigned axis, uint8_t source, float value, float sat) const;
  float gyroScale(unsigned axis, uint8_t source, float sat) const;

  // Caller enforces controller state and feeds only fresh samples after start.
  const char* start(char pose, uint64_t nowUs);
  void cancel() { capturing_ = -1; }
  bool capturing() const { return capturing_ >= 0; }
  char capturePose() const;
  // 0 collecting/idle, 1 captured, -1 fail_motion. Old pose survives failure.
  int addSample(uint64_t sampleUs, const Vec3& angles, bool yawKnown);
  float capturedNeutralYaw() const { return capturedNeutralYaw_; }

 private:
  float score(unsigned axis, uint8_t source, bool yawKnown) const;
  bool have_[5];
  Vec3 poses_[5];
  bool on_;
  bool autoSource_;
  int capturing_;
  uint64_t startUs_, lastUs_;
  uint32_t count_;
  Vec3 sum_, min_, max_;
  bool motion_;
  float capturedNeutralYaw_;
};

#endif
