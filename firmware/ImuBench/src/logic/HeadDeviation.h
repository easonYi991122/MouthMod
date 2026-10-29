// D40 head deviation from the neutral pose, independent of how the IMU sits.
//
// up is the unit vector the accelerometer reads at rest (body frame, from the
// filter's roll/pitch). The deviation is the head rotation since neutral, in
// degrees, split along a frame fixed at neutral:
//   x  about the forward axis made horizontal  (~ roll - roll0, right ear down +)
//   y  about the horizontal left axis           (~ pitch - pitch0, nose down +)
//   z  yaw, integrated about the current up     (right turn +)
// Unlike Euler-angle differences it never wraps at +-180 deg and stays
// defined when the board stands on its edge (pitch near +-90 deg).
#ifndef MOUTHMOD_HEAD_DEVIATION_H
#define MOUTHMOD_HEAD_DEVIATION_H

#include "Vec3.h"

struct HeadFrame {
  Vec3 up0;
  Vec3 rollAxis;
  Vec3 pitchAxis;
};

Vec3 upFromRollPitch(float rollDeg, float pitchDeg);
void rollPitchFromUp(const Vec3& up, float* rollDeg, float* pitchDeg);
HeadFrame makeHeadFrame(const Vec3& up0);
Vec3 headDeviation(const HeadFrame& f, const Vec3& up, float yawDeg);
// Body gyro (deg/s, bias removed) -> rates of the three deviation components.
Vec3 headRates(const HeadFrame& f, const Vec3& up, const Vec3& gyroDps);

#endif
