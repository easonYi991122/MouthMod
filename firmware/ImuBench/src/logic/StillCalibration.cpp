#include "StillCalibration.h"

#include "ComplementaryFilter.h"

StillCalibrationConfig defaultStillCalibrationConfig() {
  StillCalibrationConfig c;
  c.windowUs = 3000000U;
  c.minSamples = 250U;  // ~80 % of 104 Hz * 3 s
  c.maxGyroStdDps = 1.0f;
  c.minAccelG = 0.9f;
  c.maxAccelG = 1.1f;
  return c;
}

const char* calibrationStatusName(CalibrationStatus status) {
  switch (status) {
    case CAL_IDLE:
      return "idle";
    case CAL_COLLECTING:
      return "collecting";
    case CAL_DONE:
      return "done";
    case CAL_FAIL_MOTION:
      return "fail_motion";
    case CAL_FAIL_SAMPLES:
      return "fail_samples";
    case CAL_FAIL_ACCEL:
      return "fail_accel";
  }
  return "?";
}

StillCalibration::StillCalibration()
    : config_(defaultStillCalibrationConfig()), status_(CAL_IDLE), startUs_(0U),
      n_(0U) {
  cancel();
}

void StillCalibration::cancel() {
  status_ = CAL_IDLE;
  n_ = 0U;
  for (int i = 0; i < 3; ++i) {
    accelMean_[i] = 0.0;
    gyroMean_[i] = 0.0;
    gyroM2_[i] = 0.0;
  }
  result_.gyroBiasDps = makeVec3(0.0f, 0.0f, 0.0f);
  result_.meanAccelG = makeVec3(0.0f, 0.0f, 0.0f);
  result_.gyroStdDps = makeVec3(0.0f, 0.0f, 0.0f);
  result_.roll0Deg = 0.0f;
  result_.pitch0Deg = 0.0f;
  result_.samples = 0U;
}

void StillCalibration::start(uint64_t nowUs) {
  cancel();
  startUs_ = nowUs;
  status_ = CAL_COLLECTING;
}

uint64_t StillCalibration::elapsedUs(uint64_t nowUs) const {
  return (status_ == CAL_IDLE || nowUs < startUs_) ? 0U : nowUs - startUs_;
}

void StillCalibration::accumulate(const Vec3& accel, const Vec3& gyro) {
  const double a[3] = {accel.x, accel.y, accel.z};
  const double g[3] = {gyro.x, gyro.y, gyro.z};
  ++n_;
  for (int i = 0; i < 3; ++i) {
    accelMean_[i] += (a[i] - accelMean_[i]) / n_;
    const double d = g[i] - gyroMean_[i];
    gyroMean_[i] += d / n_;
    gyroM2_[i] += d * (g[i] - gyroMean_[i]);
  }
}

CalibrationStatus StillCalibration::addSample(uint64_t nowUs,
                                              const Vec3& accelG,
                                              const Vec3& gyroDps) {
  if (status_ != CAL_COLLECTING) {
    return status_;
  }
  if (vecFinite(accelG) && vecFinite(gyroDps)) {
    accumulate(accelG, gyroDps);
  }
  if (elapsedUs(nowUs) >= config_.windowUs) {
    status_ = finish();
  }
  return status_;
}

CalibrationStatus StillCalibration::finish() {
  result_.samples = n_;
  if (n_ < config_.minSamples || n_ < 2U) {
    return CAL_FAIL_SAMPLES;
  }
  float stdv[3];
  for (int i = 0; i < 3; ++i) {
    stdv[i] = static_cast<float>(sqrt(gyroM2_[i] / (n_ - 1U)));
  }
  result_.gyroStdDps = makeVec3(stdv[0], stdv[1], stdv[2]);
  result_.gyroBiasDps =
      makeVec3(static_cast<float>(gyroMean_[0]), static_cast<float>(gyroMean_[1]),
               static_cast<float>(gyroMean_[2]));
  result_.meanAccelG =
      makeVec3(static_cast<float>(accelMean_[0]),
               static_cast<float>(accelMean_[1]),
               static_cast<float>(accelMean_[2]));
  for (int i = 0; i < 3; ++i) {
    if (stdv[i] > config_.maxGyroStdDps) {
      return CAL_FAIL_MOTION;
    }
  }
  const float norm = vecNorm(result_.meanAccelG);
  if (norm < config_.minAccelG || norm > config_.maxAccelG) {
    return CAL_FAIL_ACCEL;
  }
  result_.roll0Deg = ComplementaryFilter::accelRollDeg(result_.meanAccelG);
  result_.pitch0Deg = ComplementaryFilter::accelPitchDeg(result_.meanAccelG);
  return CAL_DONE;
}
