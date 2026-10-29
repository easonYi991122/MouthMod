#include "FineCalibration.h"

namespace {
float component(const Vec3& a, uint8_t s) {
  return s == 0 ? a.x : (s == 1 ? a.y : a.z);
}
const char kPoses[] = "nlrdu";
}  // namespace

FineCalibration::FineCalibration() : autoSource_(true) { clear(); }

int FineCalibration::poseIndex(char pose) {
  for (int i = 0; i < 5; ++i) if (kPoses[i] == pose) return i;
  return -1;
}

void FineCalibration::clear() {
  on_ = false;
  capturing_ = -1;
  capturedNeutralYaw_ = 0;
  for (unsigned i = 0; i < 5; ++i) {
    have_[i] = false;
    poses_[i] = makeVec3(0, 0, 0);
  }
}

bool FineCalibration::setPose(char pose, const Vec3& angles) {
  const int i = poseIndex(pose);
  if (i < 0 || !vecFinite(angles)) return false;
  poses_[i] = angles;  // val yaw is already in N-relative coordinates
  if (i == 0) poses_[i].z = 0;
  have_[i] = true;
  return true;
}

float FineCalibration::score(unsigned axis, uint8_t s, bool yawKnown) const {
  if (s > 2 || (s == 2 && !yawKnown)) return 0;
  const unsigned positive = axis == 0 ? 2 : 3;
  const unsigned negative = axis == 0 ? 1 : 4;
  if (!have_[0] || !have_[positive] || !have_[negative]) return 0;
  const float dp = component(poses_[positive], s) - component(poses_[0], s);
  const float dm = component(poses_[negative], s) - component(poses_[0], s);
  if (fabsf(dp) < 3 || fabsf(dm) < 3 || (dp > 0) == (dm > 0)) return 0;
  return fminf(fabsf(dp), fabsf(dm));
}

bool FineCalibration::validSources(bool yawKnown, uint8_t h, uint8_t v) const {
  return score(0, h, yawKnown) > 0 && score(1, v, yawKnown) > 0;
}

const char* FineCalibration::enable(bool yawKnown, uint8_t* h, uint8_t* v) {
  on_ = false;
  for (unsigned i = 0; i < 5; ++i) if (!have_[i]) return "fcal_incomplete";
  uint8_t sources[2] = {*h, *v};
  for (unsigned axis = 0; axis < 2; ++axis) {
    if (autoSource_) {
      float best = 0;
      for (uint8_t s = 0; s < 3; ++s) {
        const float candidate = score(axis, s, yawKnown);
        if (candidate > best) { best = candidate; sources[axis] = s; }
      }
      if (best == 0) return axis == 0 ? "fcal_bad_h" : "fcal_bad_v";
    } else if (score(axis, sources[axis], yawKnown) == 0) {
      return axis == 0 ? "fcal_bad_h" : "fcal_bad_v";
    }
  }
  *h = sources[0]; *v = sources[1];
  on_ = true;
  return 0;
}

float FineCalibration::map(unsigned axis, uint8_t s, float x, float sat) const {
  const float n = component(poses_[0], s);
  const float dp = component(poses_[axis == 0 ? 2 : 3], s) - n;
  const float dm = component(poses_[axis == 0 ? 1 : 4], s) - n;
  const float delta = x - n;
  return delta*dp >= 0 ? sat*delta/dp : -sat*delta/dm;
}

float FineCalibration::gyroScale(unsigned axis, uint8_t s, float sat) const {
  const float n = component(poses_[0], s);
  const float dp = component(poses_[axis == 0 ? 2 : 3], s) - n;
  const float dm = component(poses_[axis == 0 ? 1 : 4], s) - n;
  return (dp < 0 ? -1 : 1)*sat*2/(fabsf(dp) + fabsf(dm));
}

const char* FineCalibration::start(char pose, uint64_t nowUs) {
  const int i = poseIndex(pose);
  if (i < 0) return "fcal_pose";
  if (i != 0 && !have_[0]) return "fcal_need_neutral";
  capturing_ = i;
  startUs_ = lastUs_ = nowUs;
  count_ = 0;
  sum_ = min_ = max_ = makeVec3(0, 0, 0);
  motion_ = false;
  return 0;
}

char FineCalibration::capturePose() const {
  return capturing_ >= 0 ? kPoses[capturing_] : '-';
}

int FineCalibration::addSample(uint64_t t, const Vec3& a, bool yawKnown) {
  if (capturing_ < 0 || t <= lastUs_ || !vecFinite(a)) return 0;
  lastUs_ = t;
  // A delayed sample closes the window but must not enter its average.
  if (t - startUs_ <= 500000U) {
    if (count_ == 0) min_ = max_ = a;
    min_.x = fminf(min_.x, a.x); max_.x = fmaxf(max_.x, a.x);
    min_.y = fminf(min_.y, a.y); max_.y = fmaxf(max_.y, a.y);
    min_.z = fminf(min_.z, a.z); max_.z = fmaxf(max_.z, a.z);
    motion_ |= max_.x - min_.x > 4 || max_.y - min_.y > 4 ||
               (yawKnown && max_.z - min_.z > 4);
    sum_.x += a.x; sum_.y += a.y; sum_.z += a.z;
    ++count_;
  }
  if (t - startUs_ < 500000U) return 0;
  const char pose = capturePose();
  capturing_ = -1;
  if (motion_ || count_ == 0) return -1;
  const Vec3 mean = makeVec3(sum_.x/count_, sum_.y/count_, sum_.z/count_);
  if (pose == 'n') capturedNeutralYaw_ = mean.z;
  setPose(pose, mean);
  return 1;
}
