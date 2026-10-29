#include "HeadDeviation.h"

namespace {
const float kDegPerRad = 57.29577951f;
const float kRadPerDeg = 0.01745329252f;

Vec3 unit(const Vec3& v, const Vec3& fallback) {
  const float n = vecNorm(v);
  return n > 1e-6f ? vecScale(v, 1.0f / n) : fallback;
}
}  // namespace

Vec3 upFromRollPitch(float rollDeg, float pitchDeg) {
  const float r = rollDeg * kRadPerDeg, p = pitchDeg * kRadPerDeg;
  return makeVec3(-sinf(p), sinf(r) * cosf(p), cosf(r) * cosf(p));
}

void rollPitchFromUp(const Vec3& up, float* rollDeg, float* pitchDeg) {
  *rollDeg = atan2f(up.y, up.z) * kDegPerRad;
  *pitchDeg = atan2f(-up.x, sqrtf(up.y * up.y + up.z * up.z)) * kDegPerRad;
}

HeadFrame makeHeadFrame(const Vec3& up0) {
  HeadFrame f;
  f.up0 = unit(up0, makeVec3(0, 0, 1));
  // Forward axis made horizontal; if forward points (almost) straight up or
  // down, build the frame from the left axis instead.
  const Vec3 fwd = vecSub(makeVec3(1, 0, 0), vecScale(f.up0, f.up0.x));
  if (vecNorm(fwd) >= 0.3f) {
    f.rollAxis = unit(fwd, makeVec3(1, 0, 0));
    f.pitchAxis = vecCross(f.up0, f.rollAxis);
  } else {
    const Vec3 left = vecSub(makeVec3(0, 1, 0), vecScale(f.up0, f.up0.y));
    f.pitchAxis = unit(left, makeVec3(0, 1, 0));
    f.rollAxis = vecCross(f.pitchAxis, f.up0);
  }
  return f;
}

Vec3 headDeviation(const HeadFrame& f, const Vec3& up, float yawDeg) {
  // The head turned by the rotation that carries up back onto up0.
  const Vec3 u = unit(up, f.up0);
  const Vec3 c = vecCross(u, f.up0);
  const float s = vecNorm(c);
  const float k = (s > 1e-6f ? atan2f(s, vecDot(u, f.up0)) / s : 1.0f) * kDegPerRad;
  return makeVec3(vecDot(c, f.rollAxis) * k, vecDot(c, f.pitchAxis) * k, yawDeg);
}

Vec3 headRates(const HeadFrame& f, const Vec3& up, const Vec3& gyroDps) {
  return makeVec3(vecDot(gyroDps, f.rollAxis), vecDot(gyroDps, f.pitchAxis),
                  -vecDot(gyroDps, unit(up, f.up0)));
}
