// Mapped head-angle errors -> mouse velocity, with per-axis dead bands.
//
// Units: k is HID counts per second per degree of tilt. Velocity is turned
// into integer HID deltas with the real elapsed time (DeltaAccumulator), so
// the camera speed does not depend on the loop or USB report rate.
//
// The spec does not define how dead-band and hysteresis combine, nor whether
// velocity starts at k*3deg or ramps from zero. Both are selectable here and
// listed as pending decisions in docs/imu-decisions.md:
//   hystMode 0 (inner): engage |e| >= dead,        release |e| < dead - hyst
//   hystMode 1 (outer): engage |e| >= dead + hyst, release |e| < dead
//   velMode  0 (spec):  v = k * clamp(e, +/-sat)
//   velMode  1 (ramp):  v = k * sign(e) * (min(|e|, sat) - releaseEdge)
// engageConfirmS: |e| must stay at or beyond the engage edge this long
// before an axis starts moving, so a brief bump does not turn the camera
// (0 = engage at once, the spec behaviour).
#ifndef MOUTHMOD_MOUSE_LOOK_MAPPER_H
#define MOUTHMOD_MOUSE_LOOK_MAPPER_H

#include <stdint.h>

struct MouseLookParams {
  uint8_t lmode;         // D38: 0 tilt, 1 gyro, 2 hybrid
  float kgx, kgy;        // counts per degree of rotation
  float gyroDeadDps;
  float kxCpsPerDeg;
  float kyCpsPerDeg;
  float deadbandDeg[2];  // horizontal, vertical
  uint8_t hsrc;          // 0 roll, 1 pitch, 2 yaw (selected by LookController)
  uint8_t vsrc;
  float yawTauS;         // leaky yaw integrator time constant; 0 = no leak
  float hysteresisDeg;
  float saturationDeg;
  uint8_t hystMode;
  uint8_t velMode;
  bool invertX;
  bool invertY;
  float engageConfirmS;
};

MouseLookParams defaultMouseLookParams();

// Clamps parameters into a usable range; returns false if anything changed.
bool sanitizeMouseLookParams(MouseLookParams* p);

class LookAxis {
 public:
  LookAxis() : engaged_(false), aboveS_(0.0f) {}
  void reset() {
    engaged_ = false;
    aboveS_ = 0.0f;
  }
  // Returns velocity in counts/s for a tilt error in degrees; dtS is the
  // time since the previous update (for the engage confirmation).
  float update(float errDeg, float k, const MouseLookParams& p, float dtS,
               unsigned axis = 0);
  bool engaged() const { return engaged_; }

  static float engageEdge(const MouseLookParams& p, unsigned axis = 0);
  static float releaseEdge(const MouseLookParams& p, unsigned axis = 0);

 private:
  bool engaged_;
  float aboveS_;
};

struct LookVelocity {
  float vxCps;  // + = turn right
  float vyCps;  // + = look down (HID y grows downward)
  bool engagedX;
  bool engagedY;
};

class MouseLookMapper {
 public:
  MouseLookMapper();
  void setParams(const MouseLookParams& p);
  const MouseLookParams& params() const { return params_; }
  void reset();

  // Source selection happens in LookController; inversion happens here.
  LookVelocity update(float xErrDeg, float yErrDeg, float dtS = 0.01f,
                      float xRateDps = 0, float yRateDps = 0,
                      float xFineScale = 1, float yFineScale = 1);

 private:
  MouseLookParams params_;
  LookAxis x_;
  LookAxis y_;
};

// Integrates velocity over real time into int8 HID deltas, keeping the
// fractional residual. Pending motion is capped so a stalled endpoint does
// not release a burst of stale camera movement later.
class DeltaAccumulator {
 public:
  explicit DeltaAccumulator(float maxPendingCounts = 64.0f);
  void clear() { pending_ = 0.0f; }
  void add(float velocityCps, float dtS);
  int8_t peek() const;           // what take() would return
  void commit(int8_t sent);      // remove counts that were actually sent
  float pending() const { return pending_; }

 private:
  float pending_;
  float maxPending_;
};

#endif
