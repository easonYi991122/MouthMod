#include "MountFrame.h"

namespace {

Vec3 axisVector(SignedAxis axis) {
  const float s = (static_cast<int>(axis) % 2 == 0) ? 1.0f : -1.0f;
  switch (static_cast<int>(axis) / 2) {
    case 0:
      return makeVec3(s, 0.0f, 0.0f);
    case 1:
      return makeVec3(0.0f, s, 0.0f);
    default:
      return makeVec3(0.0f, 0.0f, s);
  }
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return makeVec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x);
}

float dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

}  // namespace

bool parseSignedAxis(const char* text, SignedAxis* out) {
  if (text == 0 || out == 0) {
    return false;
  }
  int sign = 0;
  if (*text == '+') {
    ++text;
  } else if (*text == '-') {
    sign = 1;
    ++text;
  }
  int index;
  switch (*text) {
    case 'x':
    case 'X':
      index = 0;
      break;
    case 'y':
    case 'Y':
      index = 1;
      break;
    case 'z':
    case 'Z':
      index = 2;
      break;
    default:
      return false;
  }
  if (text[1] != '\0') {
    return false;
  }
  *out = static_cast<SignedAxis>(index * 2 + sign);
  return true;
}

const char* signedAxisName(SignedAxis axis) {
  static const char* const kNames[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  const int i = static_cast<int>(axis);
  return (i >= 0 && i < 6) ? kNames[i] : "?";
}

MountFrame::MountFrame()
    : up_(AXIS_POS_Z), forward_(AXIS_POS_X) {
  configure(AXIS_POS_Z, AXIS_POS_X);
}

bool MountFrame::configure(SignedAxis up, SignedAxis forward) {
  if (static_cast<int>(up) / 2 == static_cast<int>(forward) / 2) {
    return false;
  }
  up_ = up;
  forward_ = forward;
  upRow_ = axisVector(up);
  fwdRow_ = axisVector(forward);
  leftRow_ = cross(upRow_, fwdRow_);
  return true;
}

Vec3 MountFrame::toBody(const Vec3& sensor) const {
  return makeVec3(dot(fwdRow_, sensor), dot(leftRow_, sensor),
                  dot(upRow_, sensor));
}
