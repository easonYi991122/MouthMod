// Maps IMU sensor axes to the head (body) frame.
//
// Body frame: x = forward (nose), y = left ear, z = up (crown); right-handed.
// The mount is described by which signed sensor axis points up and which
// points forward when the wearer sits in the neutral pose. The left axis is
// derived as up x forward. Any of the 24 axis-aligned mountings is valid.
#ifndef MOUTHMOD_MOUNT_FRAME_H
#define MOUTHMOD_MOUNT_FRAME_H

#include "Vec3.h"

// Signed sensor axis: 0..5 = +X, -X, +Y, -Y, +Z, -Z.
enum SignedAxis {
  AXIS_POS_X = 0,
  AXIS_NEG_X = 1,
  AXIS_POS_Y = 2,
  AXIS_NEG_Y = 3,
  AXIS_POS_Z = 4,
  AXIS_NEG_Z = 5
};

// Parses "+X", "-y", "z" (implicit +). Returns false on anything else.
bool parseSignedAxis(const char* text, SignedAxis* out);
const char* signedAxisName(SignedAxis axis);

class MountFrame {
 public:
  MountFrame();

  // Returns false (and keeps the previous mount) if up and forward are
  // parallel or anti-parallel.
  bool configure(SignedAxis up, SignedAxis forward);

  Vec3 toBody(const Vec3& sensor) const;

  SignedAxis up() const { return up_; }
  SignedAxis forward() const { return forward_; }

 private:
  SignedAxis up_;
  SignedAxis forward_;
  Vec3 fwdRow_;
  Vec3 leftRow_;
  Vec3 upRow_;
};

#endif
