#include "JoystickMapper.h"

#include <math.h>

JoystickParams defaultJoystickParams() {
  JoystickParams p;
  p.onNorm = 0.12f;
  p.offNorm = 0.08f;
  p.filterTauUs = 20000.0f;
  p.minSpanCounts = 100.0f;
  // Nominal half travel of a 3.3 V pot on the 12-bit, 3.6 V ADC range.
  // Replace per person with "jspan" (comfortable range, guide §c).
  for (int i = 0; i < 2; ++i) {
    p.spanPos[i] = 1877.0f;
    p.spanNeg[i] = 1877.0f;
  }
  p.swapXY = false;
  p.invertX = false;
  p.invertY = false;
  return p;
}

JoystickMapper::JoystickMapper()
    : params_(defaultJoystickParams()),
      primed_(false),
      centered_(false),
      collecting_(false),
      n_(0),
      capturingSpan_(false) {
  for (int i = 0; i < 2; ++i) {
    filt_[i] = center_[i] = norm_[i] = 0.0f;
    dir_[i] = 0;
    sum_[i] = min_[i] = max_[i] = 0.0f;
    capMin_[i] = capMax_[i] = 0.0f;
  }
}

void JoystickMapper::beginCenter() {
  collecting_ = true;
  n_ = 0;
  for (int i = 0; i < 2; ++i) {
    sum_[i] = 0.0f;
    min_[i] = 1e9f;
    max_[i] = -1e9f;
  }
  clearOutput();
}

bool JoystickMapper::finishCenter() {
  collecting_ = false;
  if (n_ < 10) return false;
  for (int i = 0; i < 2; ++i) {
    const float spread = max_[i] - min_[i];
    const float smallest =
        params_.spanPos[i] < params_.spanNeg[i] ? params_.spanPos[i] : params_.spanNeg[i];
    if (spread > params_.offNorm * smallest) return false;
  }
  for (int i = 0; i < 2; ++i) {
    center_[i] = sum_[i] / static_cast<float>(n_);
    norm_[i] = normalise(i, filt_[i] - center_[i]);
  }
  centered_ = true;
  clearOutput();
  return true;
}

void JoystickMapper::beginSpanCapture() {
  capturingSpan_ = true;
  for (int i = 0; i < 2; ++i) {
    center_[i] = filt_[i];
    capMin_[i] = capMax_[i] = 0.0f;
  }
  centered_ = true;
  clearOutput();
}

bool JoystickMapper::finishSpanCapture() {
  capturingSpan_ = false;
  bool ok = true;
  for (int i = 0; i < 2; ++i) {
    if (capMax_[i] >= params_.minSpanCounts) params_.spanPos[i] = capMax_[i];
    else ok = false;
    if (-capMin_[i] >= params_.minSpanCounts) params_.spanNeg[i] = -capMin_[i];
    else ok = false;
  }
  return ok;
}

float JoystickMapper::normalise(int axis, float dev) const {
  const float span = dev >= 0.0f ? params_.spanPos[axis] : params_.spanNeg[axis];
  if (!(span >= params_.minSpanCounts)) return 0.0f;  // unusable half
  return dev / span;
}

void JoystickMapper::update(float rawX, float rawY, uint32_t dtUs) {
  const float raw[2] = {rawX, rawY};
  if (!primed_ || dtUs > 200000U) {
    filt_[0] = rawX;
    filt_[1] = rawY;
    primed_ = true;
  } else {
    const float a = static_cast<float>(dtUs) /
                    (params_.filterTauUs + static_cast<float>(dtUs));
    for (int i = 0; i < 2; ++i) filt_[i] += a * (raw[i] - filt_[i]);
  }

  if (collecting_) {
    for (int i = 0; i < 2; ++i) {
      sum_[i] += filt_[i];
      if (filt_[i] < min_[i]) min_[i] = filt_[i];
      if (filt_[i] > max_[i]) max_[i] = filt_[i];
    }
    ++n_;
  }

  // Deviation in stick axes: swap, then invert (same order as the port).
  float dev[2] = {filt_[0] - center_[0], filt_[1] - center_[1]};
  if (capturingSpan_) {
    for (int i = 0; i < 2; ++i) {
      if (dev[i] > capMax_[i]) capMax_[i] = dev[i];
      if (dev[i] < capMin_[i]) capMin_[i] = dev[i];
    }
  }
  // Spans are stored in raw ADC axes; normalise before swapping.
  float n[2] = {normalise(0, dev[0]), normalise(1, dev[1])};
  if (params_.swapXY) {
    const float t = n[0];
    n[0] = n[1];
    n[1] = t;
  }
  if (params_.invertX) n[0] = -n[0];
  if (params_.invertY) n[1] = -n[1];
  norm_[0] = n[0];
  norm_[1] = n[1];

  if (!centered_ || collecting_ || capturingSpan_) {
    clearOutput();
    return;
  }
  for (int i = 0; i < 2; ++i) {
    const float v = n[i];
    int d = dir_[i];
    if (v >= params_.onNorm) d = 1;
    else if (v <= -params_.onNorm) d = -1;
    else if (d == 1 && v > params_.offNorm) d = 1;
    else if (d == -1 && v < -params_.offNorm) d = -1;
    else d = 0;
    dir_[i] = d;
  }
}

void JoystickMapper::clearOutput() {
  dir_[0] = dir_[1] = 0;
}

bool JoystickMapper::neutral() const {
  return centered_ && fabsf(norm_[0]) < params_.offNorm &&
         fabsf(norm_[1]) < params_.offNorm;
}
