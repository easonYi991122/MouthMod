#include "ComplementaryFilter.h"

namespace {
const float kRadToDeg = 57.29577951f;
const float kDegToRad = 0.01745329252f;

float wrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
}  // namespace

const float ComplementaryFilter::kNominalDtS = 0.01f;
const float ComplementaryFilter::kMaxDtS = 0.1f;
const float ComplementaryFilter::kAccelMinG = 0.7f;
const float ComplementaryFilter::kAccelMaxG = 1.3f;

ComplementaryFilter::ComplementaryFilter() : alpha_(0.98f), tauS_(0.0f) {
  setAlpha(0.98f);
  reset();
}

void ComplementaryFilter::reset() {
  initialized_ = false;
  lastUsedAccel_ = false;
  rollDeg_ = 0.0f;
  pitchDeg_ = 0.0f;
}

void ComplementaryFilter::setAlpha(float alpha) {
  if (!(alpha > 0.0f) || !(alpha < 1.0f)) {
    return;
  }
  alpha_ = alpha;
  tauS_ = alpha * kNominalDtS / (1.0f - alpha);
}

float ComplementaryFilter::accelRollDeg(const Vec3& a) {
  return atan2f(a.y, a.z) * kRadToDeg;
}

float ComplementaryFilter::accelPitchDeg(const Vec3& a) {
  return atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z)) * kRadToDeg;
}

bool ComplementaryFilter::seed(const Vec3& accelG) {
  if (!vecFinite(accelG) || vecNorm(accelG) < 1e-3f) {
    return false;
  }
  rollDeg_ = accelRollDeg(accelG);
  pitchDeg_ = accelPitchDeg(accelG);
  initialized_ = true;
  lastUsedAccel_ = true;
  return true;
}

bool ComplementaryFilter::update(const Vec3& accelG, const Vec3& gyroDps,
                                 float dtS) {
  if (!vecFinite(accelG) || !vecFinite(gyroDps) || !isfinite(dtS) ||
      !(dtS > 0.0f)) {
    return false;
  }
  if (!initialized_ || dtS > kMaxDtS) {
    return seed(accelG);
  }

  // Euler-rate kinematics (ZYX); exact enough for head tilts of +/-45 deg.
  const float r = rollDeg_ * kDegToRad;
  const float p = pitchDeg_ * kDegToRad;
  const float cr = cosf(r);
  const float sr = sinf(r);
  const float cp = cosf(p);
  const float tp = (fabsf(cp) > 1e-3f) ? sinf(p) / cp : 0.0f;
  const float rollRate = gyroDps.x + (sr * gyroDps.y + cr * gyroDps.z) * tp;
  const float pitchRate = cr * gyroDps.y - sr * gyroDps.z;

  float roll = rollDeg_ + rollRate * dtS;
  float pitch = pitchDeg_ + pitchRate * dtS;

  const float norm = vecNorm(accelG);
  lastUsedAccel_ = (norm >= kAccelMinG && norm <= kAccelMaxG);
  if (lastUsedAccel_) {
    const float k = tauS_ / (tauS_ + dtS);
    roll += (1.0f - k) * wrapDeg(accelRollDeg(accelG) - roll);
    pitch += (1.0f - k) * (accelPitchDeg(accelG) - pitch);
  }
  rollDeg_ = wrapDeg(roll);
  pitchDeg_ = pitch;
  return true;
}
