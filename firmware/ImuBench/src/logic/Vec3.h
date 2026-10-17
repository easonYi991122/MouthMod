// Plain 3-vector used by the IMU logic. Standard C++ only.
#ifndef MOUTHMOD_VEC3_H
#define MOUTHMOD_VEC3_H

#include <math.h>

struct Vec3 {
  float x;
  float y;
  float z;
};

inline Vec3 makeVec3(float x, float y, float z) {
  Vec3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

inline Vec3 vecSub(const Vec3& a, const Vec3& b) {
  return makeVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline float vecNorm(const Vec3& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline bool vecFinite(const Vec3& v) {
  return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

#endif
