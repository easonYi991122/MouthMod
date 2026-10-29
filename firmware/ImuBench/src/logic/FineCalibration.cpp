#include "FineCalibration.h"

namespace {
const char kPoses[] = "nlrdu";
}  // namespace

FineCalibration::FineCalibration() { clear(); }

int FineCalibration::poseIndex(char pose) {
  for (int i = 0; i < 5; ++i) if (kPoses[i] == pose) return i;
  return -1;
}

bool FineCalibration::complete() const {
  for (unsigned i = 0; i < 5; ++i) if (!have_[i]) return false;
  return true;
}

void FineCalibration::clear() {
  on_ = false;
  capturing_ = -1;
  capturedNeutralYaw_ = 0;
  cross_ = 0;
  axes_[0] = makeVec3(0, 0, 1);
  axes_[1] = makeVec3(0, 1, 0);
  for (unsigned i = 0; i < 5; ++i) {
    have_[i] = false;
    poses_[i] = makeVec3(0, 0, 0);
  }
}

HeadFrame FineCalibration::neutralFrame() const {
  return makeHeadFrame(upFromRollPitch(poses_[0].x, poses_[0].y));
}

bool FineCalibration::setNeutral(float rollDeg, float pitchDeg) {
  if (!isfinite(rollDeg) || !isfinite(pitchDeg)) return false;
  poses_[0] = makeVec3(rollDeg, pitchDeg, 0);
  have_[0] = true;
  return true;
}

bool FineCalibration::setRelative(char pose, const Vec3& d) {
  const int i = poseIndex(pose);
  if (i < 0 || !vecFinite(d)) return false;
  if (i == 0) return setNeutral(d.x, d.y);
  poses_[i] = d;
  have_[i] = true;
  return true;
}

bool FineCalibration::setAbsolute(char pose, const Vec3& a) {
  const int i = poseIndex(pose);
  if (i < 0 || !vecFinite(a)) return false;
  if (i == 0) return setNeutral(a.x, a.y);
  if (!have_[0]) return false;
  return setRelative(pose, headDeviation(neutralFrame(),
                                         upFromRollPitch(a.x, a.y), a.z));
}

const char* FineCalibration::enable() {
  on_ = false;
  if (!complete()) return "fcal_incomplete";
  for (unsigned a = 0; a < 2; ++a) {
    // Horizontal: + = right pose; vertical: + = down pose (look-Y down).
    const Vec3 plus = poses_[a == 0 ? 2 : 3], minus = poses_[a == 0 ? 1 : 4];
    const Vec3 span = vecSub(plus, minus);
    const float len = vecNorm(span);
    const char* bad = a == 0 ? "fcal_bad_h" : "fcal_bad_v";
    if (!(len >= 6)) return bad;
    const Vec3 u = vecScale(span, 1.0f / len);
    if (vecDot(plus, u) < 3 || vecDot(minus, u) > -3) return bad;
    axes_[a] = u;
  }
  cross_ = vecDot(axes_[0], axes_[1]);
  if (fabsf(cross_) > 0.8f) return "fcal_bad_axes";
  on_ = true;
  return 0;
}

void FineCalibration::project(const Vec3& d, float* x, float* y) const {
  const float a = vecDot(d, axes_[0]), b = vecDot(d, axes_[1]);
  const float det = 1 - cross_ * cross_;
  *x = (a - cross_ * b) / det;
  *y = (b - cross_ * a) / det;
}

const char* FineCalibration::start(char pose, uint64_t nowUs, bool calibrated) {
  const int i = poseIndex(pose);
  if (i < 0) return "fcal_pose";
  if (!calibrated) return "fcal_need_basic";  // yaw needs the gyro bias
  if (i != 0 && !have_[0]) return "fcal_need_neutral";
  capturing_ = i;
  startUs_ = lastUs_ = nowUs;
  count_ = 0;
  sum_ = upSum_ = min_ = max_ = makeVec3(0, 0, 0);
  motion_ = false;
  if (i != 0) frame_ = neutralFrame();
  return 0;
}

char FineCalibration::capturePose() const {
  return capturing_ >= 0 ? kPoses[capturing_] : '-';
}

int FineCalibration::addSample(uint64_t t, const Vec3& up, float yawDeg) {
  if (capturing_ < 0 || t <= lastUs_ || !vecFinite(up) || !isfinite(yawDeg))
    return 0;
  lastUs_ = t;
  // A delayed sample closes the window but must not enter its average.
  if (t - startUs_ <= 500000U) {
    // Neutral is measured around its own first sample, others around n.
    if (capturing_ == 0 && count_ == 0) frame_ = makeHeadFrame(up);
    const Vec3 d = headDeviation(frame_, up, yawDeg);
    if (count_ == 0) min_ = max_ = d;
    min_.x = fminf(min_.x, d.x); max_.x = fmaxf(max_.x, d.x);
    min_.y = fminf(min_.y, d.y); max_.y = fmaxf(max_.y, d.y);
    min_.z = fminf(min_.z, d.z); max_.z = fmaxf(max_.z, d.z);
    motion_ |= max_.x - min_.x > 4 || max_.y - min_.y > 4 ||
               max_.z - min_.z > 4;
    sum_ = vecAdd(sum_, d);
    upSum_ = vecAdd(upSum_, up);
    ++count_;
  }
  if (t - startUs_ < 500000U) return 0;
  const char pose = capturePose();
  capturing_ = -1;
  if (motion_ || count_ == 0) return -1;
  const Vec3 mean = vecScale(sum_, 1.0f / count_);
  if (pose == 'n') {
    float roll, pitch;
    rollPitchFromUp(upSum_, &roll, &pitch);
    capturedNeutralYaw_ = mean.z;
    setNeutral(roll, pitch);
  } else {
    setRelative(pose, mean);
  }
  return 1;
}
