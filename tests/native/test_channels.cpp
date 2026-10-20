// Host tests for the ported joystick and pressure channels.
#include <cassert>

#include "JoystickMapper.h"
#include "PressureChannel.h"
#include "SipArmGate.h"

namespace {

const uint32_t kLoop = 10000;     // 100 Hz loop
const uint64_t kSample = 25000;   // HX710B 40 Hz

// Feeds a constant raw position long enough for the low-pass to settle.
void hold(JoystickMapper* j, float x, float y, int loops = 30) {
  for (int i = 0; i < loops; ++i) j->update(x, y, kLoop);
}

JoystickMapper centeredStick() {
  JoystickMapper j;
  JoystickParams p = j.params();
  for (int i = 0; i < 2; ++i) p.spanPos[i] = p.spanNeg[i] = 1000.0f;
  j.setParams(p);
  hold(&j, 2000, 2000);
  j.beginCenter();
  hold(&j, 2000, 2000, 50);
  assert(j.finishCenter());
  return j;
}

void test_hx710b_sign_extension() {
  assert(hx710bToSigned(0x000000U) == 0);
  assert(hx710bToSigned(0x7FFFFFU) == 8388607);
  assert(hx710bToSigned(0x800000U) == -8388608);
  assert(hx710bToSigned(0xFFFFFFU) == -1);
}

void test_joystick_directions_and_hysteresis() {
  JoystickMapper j = centeredStick();
  assert(j.neutral() && !j.forward() && !j.left());
  // Port convention: y below center -> W, x below center -> A.
  hold(&j, 2000, 2000 - 150);  // 0.15 >= 0.12
  assert(j.forward() && !j.back());
  hold(&j, 2000, 2000 - 100);  // 0.10: inside hysteresis, stays
  assert(j.forward());
  hold(&j, 2000, 2000 - 70);   // 0.07 < 0.08: released
  assert(!j.forward());
  hold(&j, 2000, 2000 - 100);  // 0.10 from rest: not enough to trigger
  assert(!j.forward());
  // Diagonal combines.
  hold(&j, 2000 + 200, 2000 + 200);
  assert(j.right() && j.back() && !j.neutral());
}

void test_joystick_half_axis_spans() {
  JoystickMapper j = centeredStick();
  JoystickParams p = j.params();
  p.spanPos[0] = 500.0f;   // short comfortable reach to the right
  p.spanNeg[0] = 2000.0f;
  j.setParams(p);
  hold(&j, 2000 + 70, 2000);   // 0.14 of the short half
  assert(j.right());
  hold(&j, 2000 - 200, 2000);  // 0.10 of the long half
  assert(!j.left());
  // Unusable span never triggers and never divides by zero.
  p.spanNeg[0] = 0.0f;
  j.setParams(p);
  hold(&j, 0, 2000);
  assert(!j.left() && j.norm(0) == 0.0f);
}

void test_joystick_swap_invert() {
  JoystickMapper j = centeredStick();
  JoystickParams p = j.params();
  p.swapXY = true;
  p.invertY = true;
  j.setParams(p);
  hold(&j, 2000 - 200, 2000);  // raw X low -> stick Y low -> inverted -> S
  assert(j.back() && !j.left());
}

void test_joystick_center_rejects_motion_and_span_capture() {
  JoystickMapper j;
  hold(&j, 2000, 2000);
  j.beginCenter();
  for (int i = 0; i < 60; ++i) j.update(2000 + (i % 20) * 30.0f, 2000, kLoop);
  assert(!j.finishCenter());
  assert(!j.forward() && !j.neutral());  // never centered: no output

  hold(&j, 2000, 2000);
  j.beginSpanCapture();
  hold(&j, 2000 + 800, 2000);
  hold(&j, 2000 - 600, 2000);
  hold(&j, 2000, 2000 + 700);
  hold(&j, 2000, 2000 - 900);
  hold(&j, 2000, 2000);
  assert(j.finishSpanCapture());
  assert(j.params().spanPos[0] > 750 && j.params().spanPos[0] <= 800);
  assert(j.params().spanNeg[1] > 850 && j.params().spanNeg[1] <= 900);
  // Capture with one direction missing keeps that half and reports it.
  j.beginSpanCapture();
  hold(&j, 2000 + 800, 2000);
  hold(&j, 2000, 2000);
  assert(!j.finishSpanCapture());
}

// Seeds a channel at rest = 1000 and returns the time of the last sample.
uint64_t seed(PressureChannel* c) {
  uint64_t t = 0;
  for (int i = 0; i < 40 && !c->seeded(); ++i) {
    t += kSample;
    c->addSample(t, 1000 + (i % 3) * 100);
  }
  assert(c->seeded());
  return t;
}

void test_pressure_seed_and_confirm() {
  PressureChannel c;
  uint64_t t = seed(&c);
  assert(c.neutral() && c.baseline() > 1000 && c.baseline() < 1200);
  // One sample over the threshold is noise; two fresh samples trigger.
  c.addSample(t += kSample, 1000 + 130000);
  assert(!c.active());
  c.addSample(t += kSample, 1000);
  c.addSample(t += kSample, 1000 + 130000);
  assert(!c.active());
  c.addSample(t += kSample, 1000 + 130000);
  assert(c.active() && !c.neutral());
}

void test_pressure_release_delay_and_freeze() {
  PressureChannel c;
  uint64_t t = seed(&c);
  const int32_t base = c.baseline();
  for (int i = 0; i < 2; ++i) c.addSample(t += kSample, base + 200000);
  assert(c.active());
  // A long puff does not drag the baseline up.
  for (int i = 0; i < 400; ++i) c.addSample(t += kSample, base + 200000);
  assert(c.baseline() == base);
  // Dips shorter than the 250 ms release hold keep the button down.
  for (int i = 0; i < 8; ++i) c.addSample(t += kSample, base);  // 175 ms
  assert(c.active());
  c.addSample(t += kSample, base + 200000);
  for (int i = 0; i < 11; ++i) c.addSample(t += kSample, base);  // 250 ms
  assert(!c.active());
}

void test_pressure_sip_is_not_a_puff() {
  PressureChannel c;
  uint64_t t = seed(&c);
  const int32_t base = c.baseline();
  // The port used |delta|: a sip clicked too. Signed: sip only.
  c.addSample(t += kSample, base - 150000);
  const uint64_t first = t;
  c.addSample(t += kSample, base - 150000);
  assert(c.sip() && !c.active());
  for (int i = 0; i < 5; ++i) c.addSample(t += kSample, base - 150000);
  assert(c.sipHeldUs(t) == t - first && c.sipHeldUs(t) >= 150000);
  c.addSample(t += kSample, base);
  assert(!c.sip() && c.sipHeldUs(t) == 0);

  // Flipped sign: the same sip now reads as a puff.
  PressureParams p = c.params();
  p.sign = -1;
  c.setParams(p);
  for (int i = 0; i < 2; ++i) c.addSample(t += kSample, base - 150000);
  assert(c.active() && !c.sip());
}

void test_pressure_baseline_tracks_drift_and_saturated_seed() {
  PressureChannel c;
  uint64_t t = seed(&c);
  const int32_t base = c.baseline();
  // 20 s of slow drift +20000 counts (< 0.3 x 120000): tracked with tau 10 s.
  for (int i = 0; i < 800; ++i) c.addSample(t += kSample, base + 20000);
  assert(c.baseline() > base + 15000 && !c.active());

  PressureChannel s;
  t = 0;
  for (int i = 0; i < 60; ++i) s.addSample(t += kSample, 0x7FFFFF);
  assert(!s.seeded());

  // Bite channel has no sip side.
  PressureChannel b;
  b.setParams(defaultBiteParams());
  t = seed(&b);
  for (int i = 0; i < 4; ++i) b.addSample(t += kSample, b.baseline() - 300000);
  assert(!b.sip() && b.sipHeldUs(t) == 0 && b.neutral());
}

// Runs the gate for ms milliseconds of a sip that started at `start` ms.
bool sipFor(SipArmGate* g, int fromMs, int toMs, bool disarmed = true,
            bool enable = true, int startMs = -1) {
  bool fired = false;
  const int start = startMs < 0 ? fromMs : startMs;
  for (int t = fromMs; t <= toMs; t += 10) {
    const uint64_t held = static_cast<uint64_t>(t - start) * 1000U;
    fired = g->update(disarmed, enable, true, held == 0 ? 1 : held) || fired;
  }
  return fired;
}

void test_sip_arm_gate() {
  SipArmGate g;
  // Sip already held at power-up: ignored until released.
  assert(!sipFor(&g, 0, 2000));
  assert(g.waitingForRelease());
  assert(!g.update(true, true, false, 0));
  // Short sip: no arm. Long sip: exactly one arm.
  assert(!sipFor(&g, 0, 500));
  g.update(true, true, false, 0);
  int fires = 0;
  for (int t = 0; t <= 3000; t += 10) {
    if (g.update(true, true, true, static_cast<uint64_t>(t) * 1000U + 1)) ++fires;
  }
  assert(fires == 1);
  g.update(true, true, false, 0);

  // Disarming sip that continues into DISARMED never re-arms.
  assert(!sipFor(&g, 0, 140, false));       // ACTIVE: the sip starts
  assert(!sipFor(&g, 150, 3000, true, true, 0));  // now DISARMED, same sip
  g.update(true, true, false, 0);                  // released
  assert(sipFor(&g, 0, 1000));              // a new long sip arms

  // Disarming sip whose hold timer restarts (pressure wobbles inside the
  // sip hysteresis band) but whose level never drops: still no re-arm.
  g.update(true, true, false, 0);
  assert(!sipFor(&g, 0, 140, false));
  g.update(true, true, true, 0);            // timer dip, sip still active
  assert(!sipFor(&g, 0, 2000));

  // D6 open: no arm, and closing D6 mid-sip needs a release first.
  g.update(true, true, false, 0);
  assert(!sipFor(&g, 0, 1500, true, false));
  assert(!sipFor(&g, 1500, 3000, true, true, 0));
}

}  // namespace

void run_channel_tests() {
  test_sip_arm_gate();
  test_hx710b_sign_extension();
  test_joystick_directions_and_hysteresis();
  test_joystick_half_axis_spans();
  test_joystick_swap_invert();
  test_joystick_center_rejects_motion_and_span_capture();
  test_pressure_seed_and_confirm();
  test_pressure_release_delay_and_freeze();
  test_pressure_sip_is_not_a_puff();
  test_pressure_baseline_tracks_drift_and_saturated_seed();
}
