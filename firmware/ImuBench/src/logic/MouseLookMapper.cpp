#include "MouseLookMapper.h"

#include <math.h>

MouseLookParams defaultMouseLookParams() {
  MouseLookParams p;
  p.lmode = 0;
  p.kgx = p.kgy = 8.0f;
  p.gyroDeadDps = 3.0f;
  p.kxCpsPerDeg = 40.0f;
  p.kyCpsPerDeg = 30.0f;
  p.deadbandDeg[0] = p.deadbandDeg[1] = 6.0f;
  p.hsrc = 0;
  p.vsrc = 1;
  p.yawTauS = 20.0f;
  p.hysteresisDeg = 2.0f;
  p.saturationDeg = 20.0f;
  p.hystMode = 0;
  p.velMode = 0;
  p.invertX = false;
  p.invertY = false;
  p.engageConfirmS = 0.0f;
  return p;
}

namespace {
bool clampField(float* v, float lo, float hi) {
  if (!isfinite(*v) || *v < lo) {
    *v = lo;
    return true;
  }
  if (*v > hi) {
    *v = hi;
    return true;
  }
  return false;
}
}  // namespace

bool sanitizeMouseLookParams(MouseLookParams* p) {
  bool changed = false;
  if (p->lmode > 2) { p->lmode = 2; changed = true; }
  changed |= clampField(&p->kgx, 0.0f, 100.0f);
  changed |= clampField(&p->kgy, 0.0f, 100.0f);
  changed |= clampField(&p->gyroDeadDps, 0.0f, 50.0f);
  changed |= clampField(&p->kxCpsPerDeg, 0.0f, 500.0f);
  changed |= clampField(&p->kyCpsPerDeg, 0.0f, 500.0f);
  for (unsigned i = 0; i < 2; ++i)
    changed |= clampField(&p->deadbandDeg[i], 0.0f, 15.0f);
  changed |= clampField(&p->yawTauS, 0.0f, 120.0f);
  if (p->hsrc > 2) { p->hsrc = 2; changed = true; }
  if (p->vsrc > 2) { p->vsrc = 2; changed = true; }
  changed |= clampField(&p->hysteresisDeg, 0.0f, 10.0f);
  changed |= clampField(&p->saturationDeg, 1.0f, 60.0f);
  changed |= clampField(&p->engageConfirmS, 0.0f, 0.5f);
  if (p->hystMode > 1) {
    p->hystMode = 0;
    changed = true;
  }
  if (p->velMode > 1) {
    p->velMode = 0;
    changed = true;
  }
  const float minSat = fmaxf(LookAxis::engageEdge(*p, 0),
                            LookAxis::engageEdge(*p, 1)) + 0.5f;
  if (p->saturationDeg < minSat) {
    p->saturationDeg = minSat;
    changed = true;
  }
  return changed;
}

float LookAxis::engageEdge(const MouseLookParams& p, unsigned axis) {
  return p.hystMode == 1 ? p.deadbandDeg[axis] + p.hysteresisDeg : p.deadbandDeg[axis];
}

float LookAxis::releaseEdge(const MouseLookParams& p, unsigned axis) {
  const float edge =
      p.hystMode == 1 ? p.deadbandDeg[axis] : p.deadbandDeg[axis] - p.hysteresisDeg;
  return edge > 0.0f ? edge : 0.0f;
}

float LookAxis::update(float errDeg, float k, const MouseLookParams& p,
                       float dtS, unsigned axis) {
  if (!isfinite(errDeg)) {
    reset();
    return 0.0f;
  }
  const float mag = fabsf(errDeg);
  if (engaged_) {
    if (mag < releaseEdge(p, axis)) {
      engaged_ = false;
      aboveS_ = 0.0f;
    }
  } else if (mag >= engageEdge(p, axis)) {
    if (isfinite(dtS) && dtS > 0.0f) aboveS_ += dtS;
    if (aboveS_ + 1e-6f >= p.engageConfirmS) engaged_ = true;
  } else {
    aboveS_ = 0.0f;  // the bump ended before the confirmation time
  }
  if (!engaged_) {
    return 0.0f;
  }
  const float sat = mag < p.saturationDeg ? mag : p.saturationDeg;
  const float sign = errDeg < 0.0f ? -1.0f : 1.0f;
  if (p.velMode == 1) {
    const float ramp = sat - releaseEdge(p, axis);
    return ramp > 0.0f ? sign * k * ramp : 0.0f;
  }
  return sign * k * sat;
}

MouseLookMapper::MouseLookMapper() : params_(defaultMouseLookParams()) {}

void MouseLookMapper::setParams(const MouseLookParams& p) {
  params_ = p;
  sanitizeMouseLookParams(&params_);
}

void MouseLookMapper::reset() {
  x_.reset();
  y_.reset();
}

LookVelocity MouseLookMapper::update(float xErrDeg, float yErrDeg,
                                    float dtS, float xRateDps, float yRateDps) {
  LookVelocity v;
  v.vxCps = x_.update(xErrDeg, params_.kxCpsPerDeg, params_, dtS, 0);
  v.vyCps = y_.update(yErrDeg, params_.kyCpsPerDeg, params_, dtS, 1);
  if (params_.lmode == 1) v.vxCps = v.vyCps = 0;
  if (params_.lmode != 0) {
    const float rates[2] = {xRateDps, yRateDps};
    const float gains[2] = {params_.kgx, params_.kgy};
    float gyro[2] = {0, 0};
    for (unsigned i = 0; i < 2; ++i) {
      if (isfinite(rates[i]) && isfinite(gains[i])) {
        const float mag = fmaxf(0, fabsf(rates[i]) - params_.gyroDeadDps);
        const float term = gains[i] * (rates[i] < 0 ? -mag : mag);
        gyro[i] = fmaxf(-4000.0f, fminf(4000.0f, term));
      }
    }
    v.vxCps += gyro[0];
    v.vyCps += gyro[1];
  }
  if (params_.invertX) v.vxCps = -v.vxCps;
  if (params_.invertY) v.vyCps = -v.vyCps;
  v.engagedX = x_.engaged();
  v.engagedY = y_.engaged();
  return v;
}

DeltaAccumulator::DeltaAccumulator(float maxPendingCounts)
    : pending_(0.0f), maxPending_(maxPendingCounts) {}

void DeltaAccumulator::add(float velocityCps, float dtS) {
  if (!isfinite(velocityCps) || !isfinite(dtS) || !(dtS > 0.0f)) {
    return;
  }
  if (velocityCps == 0.0f) {
    pending_ = 0.0f;  // drop sub-count residue when the axis stops
    return;
  }
  pending_ += velocityCps * dtS;
  if (pending_ > maxPending_) pending_ = maxPending_;
  if (pending_ < -maxPending_) pending_ = -maxPending_;
}

int8_t DeltaAccumulator::peek() const {
  float whole = truncf(pending_);
  if (whole > 127.0f) whole = 127.0f;
  if (whole < -127.0f) whole = -127.0f;
  return static_cast<int8_t>(whole);
}

void DeltaAccumulator::commit(int8_t sent) {
  pending_ -= static_cast<float>(sent);
}
