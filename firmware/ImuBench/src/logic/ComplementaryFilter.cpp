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
  haveOmega_ = false;
  omega_ = omegaDot_ = makeVec3(0, 0, 0);
  accelTrust_ = linearG_ = rotationalG_ = 0.0f;
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

namespace {
Vec3 cross(const Vec3& a, const Vec3& b) {
  return makeVec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z,
                  a.x*b.y - a.y*b.x);
}
}  // namespace

Vec3 ComplementaryFilter::eulerRates(const Vec3& g) const {
  const float r = rollDeg_ * kDegToRad;
  const float p = fmaxf(-85.0f, fminf(85.0f, pitchDeg_)) * kDegToRad;
  const float crossRate = sinf(r)*g.y + cosf(r)*g.z;
  return makeVec3(g.x + crossRate*tanf(p), cosf(r)*g.y - sinf(r)*g.z,
                  -crossRate/cosf(p));
}

bool ComplementaryFilter::updateEnhanced(const Vec3& a, const Vec3& g,
                                         float dt, float armX, float armZ,
                                         float accelRejectG) {
  if (!vecFinite(a) || !vecFinite(g) || !isfinite(dt) || !(dt > 0) ||
      !isfinite(armX) || !isfinite(armZ) ||
      !isfinite(accelRejectG) || !(accelRejectG > 0)) return false;
  const Vec3 w = makeVec3(g.x*kDegToRad, g.y*kDegToRad, g.z*kDegToRad);
  if (!haveOmega_ || dt > kMaxDtS) {
    omegaDot_ = makeVec3(0, 0, 0);
  } else {
    const float gain = dt / (0.020f + dt);
    omegaDot_.x += gain*((w.x - omega_.x)/dt - omegaDot_.x);
    omegaDot_.y += gain*((w.y - omega_.y)/dt - omegaDot_.y);
    omegaDot_.z += gain*((w.z - omega_.z)/dt - omegaDot_.z);
  }
  omega_ = w;
  haveOmega_ = true;
  const Vec3 r = makeVec3(armX, 0, armZ);
  const Vec3 tangential = cross(omegaDot_, r);
  const Vec3 centripetal = cross(w, cross(w, r));
  const Vec3 rot = makeVec3((tangential.x + centripetal.x)/9.80665f,
                            (tangential.y + centripetal.y)/9.80665f,
                            (tangential.z + centripetal.z)/9.80665f);
  const Vec3 grav = vecSub(a, rot);
  rotationalG_ = vecNorm(rot);
  const float dev = fabsf(vecNorm(grav) - 1.0f);
  accelTrust_ = fmaxf(0.0f, fminf(1.0f, 2.0f*(1.0f - dev/accelRejectG)));
  if (!initialized_ || dt > kMaxDtS) {
    if (accelTrust_ > 0) seed(grav);
  } else {
    const Vec3 rates = eulerRates(g);
    float roll = rollDeg_ + rates.x*dt;
    float pitch = pitchDeg_ + rates.y*dt;
    const float gain = accelTrust_ * dt/(tauS_ + dt);
    if (gain > 0) {
      roll += gain*wrapDeg(accelRollDeg(grav) - roll);
      pitch += gain*(accelPitchDeg(grav) - pitch);
    }
    rollDeg_ = wrapDeg(roll);
    pitchDeg_ = pitch;
  }
  lastUsedAccel_ = accelTrust_ > 0;
  const float roll = rollDeg_*kDegToRad, pitch = pitchDeg_*kDegToRad;
  const Vec3 gravity = makeVec3(-sinf(pitch), sinf(roll)*cosf(pitch),
                                cosf(roll)*cosf(pitch));
  linearG_ = vecNorm(vecSub(grav, gravity));
  return true;
}

FusionParams defaultFusionParams() {
  FusionParams p = {0, 0.10f, -0.05f, 0.15f};
  return p;
}

void sanitizeFusionParams(FusionParams* p) {
  if (p->mode > 1) p->mode = 1;
  p->armX = isfinite(p->armX) ? fmaxf(-0.30f, fminf(0.30f, p->armX)) : 0.10f;
  p->armZ = isfinite(p->armZ) ? fmaxf(-0.30f, fminf(0.30f, p->armZ)) : -0.05f;
  p->accelRejectG = isfinite(p->accelRejectG)
      ? fmaxf(0.02f, fminf(1.0f, p->accelRejectG)) : 0.15f;
}
