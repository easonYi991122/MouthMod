// Tongue joystick (two potentiometers on A0/A1) -> W/A/S/D levels.
//
// Ported from the team's joystick_usb_keyboard.ino: same direction
// convention (y < center -> W, x < center -> A), same per-axis hysteresis
// and swap/invert options. Changed to the guide's contract (§c, §d):
// - deviation from the released-stick center, normalised per half-axis by
//   the comfortable span (not by the pot's full scale), 0.12 on / 0.08 off;
// - first-order low-pass with the real dt before the hysteresis;
// - no usable span -> that half never triggers (no divide by zero);
// - center from the wear calibration, span from an explicit capture.
// Pure C++: no Arduino headers, no clocks.
#ifndef MOUTHMOD_JOYSTICK_MAPPER_H
#define MOUTHMOD_JOYSTICK_MAPPER_H

#include <stdint.h>

struct JoystickParams {
  float onNorm;          // trigger, fraction of the half-axis span
  float offNorm;         // release (< onNorm)
  float filterTauUs;     // low-pass time constant
  float minSpanCounts;   // spans below this are unusable
  float spanPos[2];      // [x, y] counts from center to comfortable + extreme
  float spanNeg[2];      // [x, y] counts from center to comfortable - extreme
  bool swapXY;
  bool invertX;
  bool invertY;
};

JoystickParams defaultJoystickParams();

class JoystickMapper {
 public:
  JoystickMapper();

  void setParams(const JoystickParams& p) { params_ = p; }
  const JoystickParams& params() const { return params_; }

  // Center calibration (stick released, during the 3 s still window).
  void beginCenter();
  // Returns false if too few samples or the stick moved (spread > offNorm).
  bool finishCenter();
  bool centered() const { return centered_; }

  // Span capture: center = current filtered value, then push every direction
  // to its comfortable limit; finish keeps each half's largest excursion.
  void beginSpanCapture();
  bool finishSpanCapture();  // false if any half stayed below minSpanCounts
  bool capturingSpan() const { return capturingSpan_; }

  // Once per loop with the raw ADC counts and the real loop interval.
  void update(float rawX, float rawY, uint32_t dtUs);
  void clearOutput();  // directions -> 0 (state left ACTIVE)

  bool forward() const { return dir_[1] < 0; }
  bool back() const { return dir_[1] > 0; }
  bool left() const { return dir_[0] < 0; }
  bool right() const { return dir_[0] > 0; }
  // Both axes inside the release band (used by the 300 ms neutral gate).
  bool neutral() const;
  float norm(int axis) const { return norm_[axis]; }
  float filtered(int axis) const { return filt_[axis]; }
  float center(int axis) const { return center_[axis]; }

 private:
  float normalise(int axis, float dev) const;

  JoystickParams params_;
  bool primed_;
  float filt_[2];
  float center_[2];
  bool centered_;
  float norm_[2];
  int dir_[2];

  bool collecting_;
  uint32_t n_;
  float sum_[2];
  float min_[2];
  float max_[2];

  bool capturingSpan_;
  float capMin_[2];
  float capMax_[2];
};

#endif
