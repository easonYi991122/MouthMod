// Stationary-window calibration for the head IMU (spec: hold still 3 s).
// Estimates gyro bias and the neutral roll0/pitch0 from body-frame samples.
// Only fresh IMU samples may be fed; the caller supplies monotonic time.
//
// The stillness limits are provisional project choices (spec gives none);
// see docs/imu-decisions.md.
#ifndef MOUTHMOD_STILL_CALIBRATION_H
#define MOUTHMOD_STILL_CALIBRATION_H

#include <stdint.h>

#include "Vec3.h"

struct StillCalibrationConfig {
  uint32_t windowUs;       // 3 000 000
  uint32_t minSamples;     // e.g. 80 % of ODR * window
  float maxGyroStdDps;     // per-axis standard deviation limit
  float minAccelG;         // mean accel norm window
  float maxAccelG;
};

StillCalibrationConfig defaultStillCalibrationConfig();

enum CalibrationStatus {
  CAL_IDLE = 0,
  CAL_COLLECTING,
  CAL_DONE,
  CAL_FAIL_MOTION,
  CAL_FAIL_SAMPLES,
  CAL_FAIL_ACCEL
};

const char* calibrationStatusName(CalibrationStatus status);

struct CalibrationResult {
  Vec3 gyroBiasDps;
  Vec3 meanAccelG;
  Vec3 gyroStdDps;
  float roll0Deg;
  float pitch0Deg;
  uint32_t samples;
};

class StillCalibration {
 public:
  StillCalibration();

  void setConfig(const StillCalibrationConfig& config) { config_ = config; }
  const StillCalibrationConfig& config() const { return config_; }

  void start(uint64_t nowUs);
  void cancel();

  // Feed one fresh body-frame sample. Returns the status after this sample.
  // The window is judged once nowUs - start >= windowUs.
  CalibrationStatus addSample(uint64_t nowUs, const Vec3& accelG,
                              const Vec3& gyroDps);

  CalibrationStatus status() const { return status_; }
  const CalibrationResult& result() const { return result_; }
  uint64_t elapsedUs(uint64_t nowUs) const;

 private:
  void accumulate(const Vec3& accel, const Vec3& gyro);
  CalibrationStatus finish();

  StillCalibrationConfig config_;
  CalibrationStatus status_;
  uint64_t startUs_;
  uint32_t n_;
  double accelMean_[3];
  double gyroMean_[3];
  double gyroM2_[3];
  CalibrationResult result_;
};

#endif
