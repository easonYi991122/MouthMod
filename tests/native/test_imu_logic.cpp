// Deterministic host tests for the head-look logic. Build: scripts/test-native.sh
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "ComplementaryFilter.h"
#include "LookController.h"
#include "MonotonicClock.h"
#include "MountFrame.h"
#include "MouseLookMapper.h"
#include "StillCalibration.h"

namespace {

const float kDegToRad = 0.01745329252f;

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// Gravity reaction (what a still accelerometer reads) for a body pose.
Vec3 accelFor(float rollDeg, float pitchDeg) {
  const float r = rollDeg * kDegToRad;
  const float p = pitchDeg * kDegToRad;
  return makeVec3(-std::sin(p), std::sin(r) * std::cos(p),
                  std::cos(r) * std::cos(p));
}

ImuInput sample(uint64_t t, const Vec3& a, const Vec3& g) {
  ImuInput in;
  in.fresh = true;
  in.sampleUs = t;
  in.accelG = a;
  in.gyroDps = g;
  return in;
}

ImuInput noSample() {
  ImuInput in;
  in.fresh = false;
  in.sampleUs = 0;
  in.accelG = makeVec3(0, 0, 0);
  in.gyroDps = makeVec3(0, 0, 0);
  return in;
}

void test_parse_axis() {
  SignedAxis a;
  assert(parseSignedAxis("+X", &a) && a == AXIS_POS_X);
  assert(parseSignedAxis("-y", &a) && a == AXIS_NEG_Y);
  assert(parseSignedAxis("z", &a) && a == AXIS_POS_Z);
  assert(!parseSignedAxis("+W", &a));
  assert(!parseSignedAxis("+XY", &a));
  assert(!parseSignedAxis("", &a));
  assert(std::strcmp(signedAxisName(AXIS_NEG_Z), "-Z") == 0);
}

void test_mount_frame() {
  MountFrame m;
  Vec3 b = m.toBody(makeVec3(1, 2, 3));
  assert(b.x == 1 && b.y == 2 && b.z == 3);

  // Board standing on the forehead: sensor -Y up, sensor +Z forward.
  assert(m.configure(AXIS_NEG_Y, AXIS_POS_Z));
  b = m.toBody(makeVec3(0, -1, 0));  // gravity reaction along sensor -Y
  assert(near(b.z, 1, 1e-6f) && near(b.x, 0, 1e-6f) && near(b.y, 0, 1e-6f));
  b = m.toBody(makeVec3(0, 0, 1));
  assert(near(b.x, 1, 1e-6f));
  b = m.toBody(makeVec3(-1, 0, 0));  // left = up x fwd = -X
  assert(near(b.y, 1, 1e-6f));

  assert(!m.configure(AXIS_POS_Z, AXIS_NEG_Z));
  assert(m.up() == AXIS_NEG_Y);  // unchanged after rejected config
}

void test_filter_static_and_gap() {
  ComplementaryFilter f;
  const Vec3 zero = makeVec3(0, 0, 0);
  assert(!f.update(accelFor(0, 0), zero, 0.0f));
  assert(f.update(accelFor(10, -5), zero, 0.01f));  // first sample seeds
  assert(near(f.rollDeg(), 10, 0.01f) && near(f.pitchDeg(), -5, 0.01f));

  // Step to a new static pose: converges via the accelerometer.
  for (int i = 0; i < 400; ++i) f.update(accelFor(-8, 12), zero, 0.01f);
  assert(near(f.rollDeg(), -8, 0.05f) && near(f.pitchDeg(), 12, 0.05f));

  // A long gap re-seeds instead of integrating a stale rate.
  f.update(accelFor(0, 0), makeVec3(500, 0, 0), 0.5f);
  assert(near(f.rollDeg(), 0, 0.01f));

  Vec3 bad = makeVec3(NAN, 0, 1);
  const float before = f.rollDeg();
  assert(!f.update(bad, zero, 0.01f));
  assert(f.rollDeg() == before);
}

void test_filter_tracks_gyro_and_rejects_linear_accel() {
  ComplementaryFilter f;
  f.seed(accelFor(0, 0));
  // Roll right at 20 dps for 1 s with a consistent accelerometer.
  for (int i = 1; i <= 100; ++i) {
    f.update(accelFor(0.2f * i, 0), makeVec3(20, 0, 0), 0.01f);
  }
  assert(near(f.rollDeg(), 20, 0.3f));

  // Pitch rate maps to pitch, not roll.
  ComplementaryFilter g;
  g.seed(accelFor(0, 0));
  for (int i = 1; i <= 50; ++i) {
    g.update(accelFor(0, 0.2f * i), makeVec3(0, 20, 0), 0.01f);
  }
  assert(near(g.pitchDeg(), 10, 0.3f) && near(g.rollDeg(), 0, 0.3f));

  // A 2 g bump is ignored by the gravity correction for that step.
  ComplementaryFilter h;
  h.seed(accelFor(0, 0));
  h.update(makeVec3(0, 2.0f, 0.5f), makeVec3(0, 0, 0), 0.01f);
  assert(!h.lastUsedAccel() && near(h.rollDeg(), 0, 1e-4f));
}

void test_filter_alpha_is_dt_robust() {
  // Same wall time, different step sizes -> similar convergence.
  ComplementaryFilter a, b;
  a.seed(accelFor(0, 0));
  b.seed(accelFor(0, 0));
  for (int i = 0; i < 50; ++i) a.update(accelFor(10, 0), makeVec3(0, 0, 0), 0.01f);
  for (int i = 0; i < 25; ++i) b.update(accelFor(10, 0), makeVec3(0, 0, 0), 0.02f);
  assert(near(a.rollDeg(), b.rollDeg(), 0.3f));
}

void test_still_calibration() {
  StillCalibration c;
  c.start(0);
  CalibrationStatus s = CAL_COLLECTING;
  uint64_t t = 0;
  for (int i = 0; i < 320 && s == CAL_COLLECTING; ++i) {
    t = static_cast<uint64_t>(i) * 9615U;  // 104 Hz
    const float n = (i % 2 ? 0.05f : -0.05f);
    s = c.addSample(t, accelFor(4, -2), makeVec3(1.5f + n, -0.7f, 0.3f));
  }
  assert(s == CAL_DONE);
  assert(near(c.result().gyroBiasDps.x, 1.5f, 0.01f));
  assert(near(c.result().gyroBiasDps.y, -0.7f, 1e-4f));
  assert(near(c.result().roll0Deg, 4, 0.01f) && near(c.result().pitch0Deg, -2, 0.01f));

  // Motion in the window fails.
  c.start(0);
  s = CAL_COLLECTING;
  for (int i = 0; i < 320 && s == CAL_COLLECTING; ++i) {
    const float g = (i / 30) % 2 ? 15.0f : -15.0f;
    s = c.addSample(static_cast<uint64_t>(i) * 9615U, accelFor(0, 0), makeVec3(g, 0, 0));
  }
  assert(s == CAL_FAIL_MOTION);

  // Too few fresh samples fails.
  c.start(0);
  s = c.addSample(1000, accelFor(0, 0), makeVec3(0, 0, 0));
  s = c.addSample(3000001, accelFor(0, 0), makeVec3(0, 0, 0));
  assert(s == CAL_FAIL_SAMPLES);

  // Free-fall / bad accel norm fails.
  c.start(0);
  s = CAL_COLLECTING;
  for (int i = 0; i < 320 && s == CAL_COLLECTING; ++i) {
    s = c.addSample(static_cast<uint64_t>(i) * 9615U, makeVec3(0, 0, 0.5f), makeVec3(0, 0, 0));
  }
  assert(s == CAL_FAIL_ACCEL);
}

void test_mapper_inner_hysteresis_spec_velocity() {
  MouseLookMapper m;
  MouseLookParams p = defaultMouseLookParams();
  p.deadbandDeg[0] = p.deadbandDeg[1] = 3.0f;
  p.kxCpsPerDeg = 10;
  p.kyCpsPerDeg = 20;
  m.setParams(p);
  assert(m.update(2.9f, 0).vxCps == 0);
  LookVelocity v = m.update(3.0f, 0);
  assert(v.engagedX && near(v.vxCps, 30, 1e-4f));
  v = m.update(1.5f, 0);  // inside hysteresis band: still engaged
  assert(v.engagedX && near(v.vxCps, 15, 1e-4f));
  v = m.update(0.9f, 0);  // below dead - hyst = 1 deg: released
  assert(!v.engagedX && v.vxCps == 0);
  v = m.update(-2.0f, 0);  // not re-engaged until |e| >= 3 again
  assert(v.vxCps == 0);
  v = m.update(-35.0f, 0);  // saturation at 20 deg
  assert(near(v.vxCps, -200, 1e-3f));
  v = m.update(0, 5.0f);
  assert(near(v.vyCps, 100, 1e-3f) && v.vxCps == 0);
}

void test_mapper_outer_and_ramp_modes_and_invert() {
  MouseLookMapper m;
  MouseLookParams p = defaultMouseLookParams();
  p.deadbandDeg[0] = p.deadbandDeg[1] = 3.0f;
  p.kxCpsPerDeg = 10;
  p.hystMode = 1;  // engage 5, release 3
  p.velMode = 1;   // ramp from release edge
  p.invertX = true;
  m.setParams(p);
  assert(m.update(4.9f, 0).vxCps == 0);
  LookVelocity v = m.update(5.0f, 0);
  assert(v.engagedX && near(v.vxCps, -20, 1e-4f));  // -(10*(5-3))
  v = m.update(3.1f, 0);
  assert(v.engagedX && near(v.vxCps, -1, 1e-3f));
  v = m.update(2.9f, 0);
  assert(!v.engagedX && v.vxCps == 0);
  v = m.update(50, 0);
  assert(near(v.vxCps, -170, 1e-3f));  // -(10*(20-3))
}

void test_mapper_engage_confirmation() {
  MouseLookMapper m;
  MouseLookParams p = defaultMouseLookParams();
  p.kxCpsPerDeg = 10;
  p.deadbandDeg[0] = p.deadbandDeg[1] = 6;   // engage 6, release 4
  p.velMode = 1;
  p.engageConfirmS = 0.08f;
  m.setParams(p);
  // A 50 ms bump past the edge: no motion.
  for (int i = 0; i < 5; ++i) assert(m.update(9.0f, 0, 0.01f).vxCps == 0);
  assert(m.update(2.0f, 0, 0.01f).vxCps == 0);
  // Held 80 ms: moves, ramp from the release edge (10 * (9 - 4)).
  LookVelocity v;
  for (int i = 0; i < 7; ++i) assert(m.update(9.0f, 0, 0.01f).vxCps == 0);
  v = m.update(9.0f, 0, 0.01f);
  assert(v.engagedX && near(v.vxCps, 50, 1e-3f));
  // Once moving, dropping inside the hysteresis band keeps moving.
  v = m.update(5.0f, 0, 0.01f);
  assert(v.engagedX && near(v.vxCps, 10, 1e-3f));
  v = m.update(3.9f, 0, 0.01f);
  assert(!v.engagedX);
}

void test_sanitize() {
  MouseLookParams p = defaultMouseLookParams();
  p.deadbandDeg[0] = p.deadbandDeg[1] = 3.0f;
  p.kxCpsPerDeg = -5;
  p.saturationDeg = 2;  // below engage edge
  p.velMode = 7;
  assert(sanitizeMouseLookParams(&p));
  assert(p.kxCpsPerDeg == 0 && p.velMode == 0 && p.saturationDeg >= 3.5f);
  MouseLookParams q = defaultMouseLookParams();
  assert(!sanitizeMouseLookParams(&q));
}

void test_delta_accumulator() {
  DeltaAccumulator acc(64);
  int total = 0;
  for (int i = 0; i < 100; ++i) {  // 37 counts/s for 1 s at 100 Hz
    acc.add(37.0f, 0.01f);
    const int8_t d = acc.peek();
    acc.commit(d);
    total += d;
  }
  assert(total == 36 || total == 37);  // float residue, never more than v*t
  // Stalled endpoint: pending is capped, not accumulated.
  DeltaAccumulator s(64);
  for (int i = 0; i < 1000; ++i) s.add(1000.0f, 0.01f);
  assert(near(s.pending(), 64, 1e-4f) && s.peek() == 64);
  // Zero velocity drops the residue.
  s.add(0.0f, 0.01f);
  assert(s.pending() == 0 && s.peek() == 0);
  // Negative direction truncates toward zero.
  s.add(-250.0f, 0.01f);
  assert(s.peek() == -2);
}

// --- LookController scenarios (100 Hz ticks with a fresh sample each) ---

struct Sim {
  LookController c;
  uint64_t t;
  Sim() : t(1000000) {}
  LookTick step(float roll, float pitch, bool enable = true,
                Vec3 gyro = makeVec3(0.4f, -0.2f, 0.1f)) {
    t += 10000;
    return c.tick(t, sample(t, accelFor(roll, pitch), gyro), enable);
  }
  LookTick stepNoImu(bool enable = true) {
    t += 10000;
    return c.tick(t, noSample(), enable);
  }
  // Runs until state changes or n ticks elapse.
  LookTick runStill(int n, float roll = 0, float pitch = 0) {
    LookTick k = step(roll, pitch);
    for (int i = 1; i < n; ++i) k = step(roll, pitch);
    return k;
  }
};

void arm_to_active(Sim* s) {
  s->c.requestArm();
  LookTick k = s->step(0, 0);
  assert(k.state == LOOK_CALIBRATING && k.releaseAll);
  k = s->runStill(305);
  assert(s->c.state() == LOOK_WAIT_NEUTRAL);
  k = s->runStill(31);
  assert(k.state == LOOK_ACTIVE);
}

void test_controller_powerup_disarmed_no_output() {
  Sim s;
  for (int i = 0; i < 50; ++i) {
    LookTick k = s.step(15, 10);
    assert(k.state == LOOK_DISARMED);
    assert(k.velocity.vxCps == 0 && k.velocity.vyCps == 0);
  }
}

void test_controller_full_arm_flow_and_mapping_sign() {
  Sim s;
  arm_to_active(&s);
  assert(s.c.calibrated());
  assert(near(s.c.gyroBiasDps().x, 0.4f, 1e-3f));
  // Tilt right / nose down -> turn right / look down after the filter settles.
  LookTick k = s.runStill(200, 10, 8);
  assert(k.state == LOOK_ACTIVE);
  assert(k.velocity.vxCps > 0 && k.velocity.vyCps > 0);
  assert(near(k.rollErrDeg, 10, 0.2f));
  k = s.runStill(200, -10, -8);
  assert(k.velocity.vxCps < 0 && k.velocity.vyCps < 0);
  // Back to neutral stops the camera.
  k = s.runStill(200, 0, 0);
  assert(k.velocity.vxCps == 0 && k.velocity.vyCps == 0);
}

void test_controller_neutral_gate() {
  Sim s;
  MouseLookParams p = s.c.lookParams();
  p.deadbandDeg[0] = p.deadbandDeg[1] = 3;  // original gate scenario
  s.c.setLookParams(p);
  s.c.requestArm();
  s.step(0, 0);
  s.runStill(305);
  assert(s.c.state() == LOOK_WAIT_NEUTRAL);
  // Hold a tilt: never becomes ACTIVE.
  LookTick k = s.runStill(300, 8, 0);
  assert(k.state == LOOK_WAIT_NEUTRAL && k.velocity.vxCps == 0);
  // Brief neutral shorter than 300 ms does not count.
  s.runStill(20, 0, 0);
  k = s.runStill(10, 8, 0);
  assert(k.state == LOOK_WAIT_NEUTRAL);
  k = s.runStill(100, 0, 0);
  assert(k.state == LOOK_ACTIVE);
}

void test_controller_neutral_gate_waits_for_other_inputs() {
  Sim s;
  s.c.requestArm();
  s.step(0, 0);
  s.runStill(305);
  assert(s.c.state() == LOOK_WAIT_NEUTRAL);
  // Head neutral but the joystick / a pressure channel is not: wait.
  s.c.setOtherInputsNeutral(false);
  LookTick k = s.runStill(100, 0, 0);
  assert(k.state == LOOK_WAIT_NEUTRAL);
  s.c.setOtherInputsNeutral(true);
  k = s.runStill(25, 0, 0);
  assert(k.state == LOOK_WAIT_NEUTRAL);  // the 300 ms restarts
  k = s.runStill(10, 0, 0);
  assert(k.state == LOOK_ACTIVE);
}

void test_controller_disarm_sources_release_all() {
  Sim s;
  arm_to_active(&s);
  s.c.requestDisarm();
  LookTick k = s.step(10, 0);
  assert(k.state == LOOK_DISARMED && k.releaseAll && k.velocity.vxCps == 0);
  assert(std::strcmp(k.event, "disarm_cmd") == 0);

  arm_to_active(&s);
  k = s.step(10, 0, false);
  assert(k.state == LOOK_DISARMED && k.releaseAll);
  // Arm is refused while the enable input is open.
  s.c.requestArm();
  k = s.step(0, 0, false);
  assert(k.state == LOOK_DISARMED);

  arm_to_active(&s);
  assert(s.c.setMount(AXIS_NEG_Y, AXIS_POS_Z));
  k = s.step(0, 0);
  assert(k.state == LOOK_DISARMED && k.releaseAll && !s.c.calibrated());
}

void test_controller_fault_latch() {
  Sim s;
  arm_to_active(&s);
  LookTick k = s.stepNoImu();
  for (int i = 0; i < 11 && k.state != LOOK_FAULT; ++i) k = s.stepNoImu();
  assert(k.state == LOOK_FAULT && k.releaseAll);
  // Samples resume: still latched; arm ignored.
  s.c.requestArm();
  k = s.runStill(50);
  assert(k.state == LOOK_FAULT && k.velocity.vxCps == 0);
  // Acknowledge -> DISARMED only, not ACTIVE.
  s.c.acknowledgeFault();
  k = s.step(0, 0);
  assert(k.state == LOOK_DISARMED);
  // Acknowledge while the IMU is still dead keeps FAULT.
  s.c.reportFault("imu_read_error");
  k = s.step(0, 0);
  assert(k.state == LOOK_FAULT);
  for (int i = 0; i < 15; ++i) k = s.stepNoImu();
  s.c.acknowledgeFault();
  k = s.stepNoImu();
  assert(k.state == LOOK_FAULT);
}

void test_controller_startup_without_imu_faults() {
  Sim s;
  LookTick k = s.stepNoImu();
  for (int i = 0; i < 20; ++i) k = s.stepNoImu();
  assert(k.state == LOOK_FAULT);
}

void test_controller_cal_motion_returns_disarmed() {
  Sim s;
  s.c.requestArm();
  s.step(0, 0);
  LookTick k;
  for (int i = 0; i < 305; ++i) {
    const float g = (i / 20) % 2 ? 25.0f : -25.0f;
    k = s.step(0, 0, true, makeVec3(g, 0, 0));
  }
  assert(k.state == LOOK_DISARMED);
  assert(!s.c.calibrated());
}

void test_look_sources_swap_and_same_source() {
  Sim s;
  arm_to_active(&s);
  MouseLookParams p = s.c.lookParams();
  p.hsrc = 1; p.vsrc = 0;
  p.invertX = true;
  s.c.setLookParams(p);
  LookTick k = s.runStill(400, 10, -12);
  assert(near(k.lookXErrDeg, -12, 0.1f));
  assert(near(k.lookYErrDeg, 10, 0.1f));
  assert(k.velocity.vxCps > 0 && k.velocity.vyCps > 0);
  p.vsrc = 1; p.invertX = false;
  s.c.setLookParams(p);
  k = s.step(10, -12);
  assert(k.lookXErrDeg == k.lookYErrDeg);
  assert(k.velocity.vxCps < 0 && k.velocity.vyCps < 0);
}

void test_yaw_integration_leak_and_reset() {
  Sim s;
  MouseLookParams p = s.c.lookParams();
  p.hsrc = p.vsrc = 2; p.yawTauS = 0;
  s.c.setLookParams(p);
  for (int i = 0; i < 20; ++i) s.step(0, 0, true, makeVec3(0, 0, -10));
  assert(s.c.yawDeg() > 1);
  s.c.requestArm();
  s.step(0, 0);
  assert(s.c.yawDeg() == 0);  // calibration start
  // Yaw accumulates during calibration, but the completed calibration resets it.
  LookTick k;
  do { k = s.step(0, 0); } while (k.state == LOOK_CALIBRATING);
  assert(k.state == LOOK_WAIT_NEUTRAL && s.c.yawDeg() == 0);
  s.c.setOtherInputsNeutral(false);
  for (int i = 0; i < 10; ++i) s.step(0, 0, true, makeVec3(0.4f, -0.2f, -1.9f));
  assert(near(s.c.yawDeg(), 0.2f, 0.001f));  // bias-corrected -Z, right positive
  s.c.setOtherInputsNeutral(true);
  do { k = s.step(0, 0); } while (k.state == LOOK_WAIT_NEUTRAL);
  assert(k.state == LOOK_ACTIVE && s.c.yawDeg() == 0);
  assert(k.lookXErrDeg == 0 && k.lookYErrDeg == 0);
  for (int i = 0; i < 100; ++i)
    k = s.step(0, 0, true, makeVec3(0.4f, -0.2f, -9.9f));
  assert(near(s.c.yawDeg(), 10, 0.001f));
  assert(near(k.lookXErrDeg, 10, 0.001f) && k.lookXErrDeg == k.lookYErrDeg);
  s.runStill(100);
  assert(near(s.c.yawDeg(), 10, 0.001f));  // tau 0: no leak
  p.yawTauS = 1;
  s.c.setLookParams(p);
  s.runStill(100);
  assert(near(s.c.yawDeg(), 10 * std::exp(-1.0f), 0.002f));
  // Exact constant-rate leaky integration, independent of sample count.
  for (int i = 0; i < 100; ++i)
    s.step(0, 0, true, makeVec3(0.4f, -0.2f, -9.9f));
  const float expected = 10 * std::exp(-2.0f) + 10 * (1 - std::exp(-1.0f));
  assert(near(s.c.yawDeg(), expected, 0.003f));

  LookController mounted;
  assert(mounted.setMount(AXIS_NEG_Y, AXIS_POS_Z));
  const Vec3 a = makeVec3(0, -1, 0);
  mounted.tick(10000, sample(10000, a, makeVec3(0, 10, 0)), true);
  mounted.tick(20000, sample(20000, a, makeVec3(0, 10, 0)), true);
  assert(mounted.yawDeg() > 0.09f);  // sensor +Y -> body -Z
}

void test_look_per_axis_dead_and_neutral_gate() {
  MouseLookMapper m;
  MouseLookParams p = m.params();
  p.deadbandDeg[0] = 4; p.deadbandDeg[1] = 10;
  m.setParams(p);
  LookVelocity v = m.update(6, 6);
  assert(v.engagedX && !v.engagedY);
  v = m.update(12, 12);
  assert(v.engagedX && v.engagedY);
  v = m.update(7, 7);  // X release 2, Y release 8
  assert(v.engagedX && !v.engagedY);
  p.hystMode = 1;
  m.setParams(p); m.reset();
  v = m.update(7, 11);  // X engage 6, Y engage 12
  assert(v.engagedX && !v.engagedY);
  m.update(12, 13);
  v = m.update(5, 9);   // X release 4, Y release 10
  assert(v.engagedX && !v.engagedY);

  Sim s;
  s.c.setAlpha(0.01f);
  s.c.requestArm(); s.step(0, 0); s.runStill(305);
  p = s.c.lookParams();
  p.hsrc = p.vsrc = 1;
  p.deadbandDeg[0] = 4; p.deadbandDeg[1] = 10;
  s.c.setLookParams(p);
  assert(s.runStill(100, 0, 7).state == LOOK_WAIT_NEUTRAL);
  p.deadbandDeg[0] = 10; p.deadbandDeg[1] = 4;
  s.c.setLookParams(p);
  assert(s.runStill(100, 0, 7).state == LOOK_WAIT_NEUTRAL);
  p.hsrc = p.vsrc = 0;  // unmapped pitch must no longer block neutral
  s.c.setLookParams(p);
  assert(s.runStill(40, 0, 12).state == LOOK_ACTIVE);
}

void test_look_new_param_sanitization() {
  MouseLookParams p = defaultMouseLookParams();
  p.hsrc = 255; p.vsrc = 3; p.yawTauS = 200;
  p.deadbandDeg[0] = -2; p.deadbandDeg[1] = 30;
  assert(sanitizeMouseLookParams(&p));
  assert(p.hsrc == 2 && p.vsrc == 2 && p.yawTauS == 120);
  assert(p.deadbandDeg[0] == 0 && p.deadbandDeg[1] == 15);
  p.yawTauS = NAN; p.deadbandDeg[1] = NAN;
  assert(sanitizeMouseLookParams(&p));
  assert(p.yawTauS == 0 && p.deadbandDeg[1] == 0);
}

void test_monotonic_clock() {
  MonotonicClock64 c;
  assert(c.update(10U) == 10U);
  assert(c.update(0xFFFFFFF0U) == 0xFFFFFFF0ULL);
  assert(c.update(5U) == 0x100000005ULL);
}

}  // namespace

void run_game_input_tests();  // test_game_input.cpp
void run_channel_tests();     // test_channels.cpp

int main() {
  test_parse_axis();
  test_mount_frame();
  test_filter_static_and_gap();
  test_filter_tracks_gyro_and_rejects_linear_accel();
  test_filter_alpha_is_dt_robust();
  test_still_calibration();
  test_mapper_inner_hysteresis_spec_velocity();
  test_mapper_outer_and_ramp_modes_and_invert();
  test_mapper_engage_confirmation();
  test_sanitize();
  test_delta_accumulator();
  test_controller_powerup_disarmed_no_output();
  test_controller_full_arm_flow_and_mapping_sign();
  test_controller_neutral_gate();
  test_controller_neutral_gate_waits_for_other_inputs();
  test_controller_disarm_sources_release_all();
  test_controller_fault_latch();
  test_controller_startup_without_imu_faults();
  test_controller_cal_motion_returns_disarmed();
  test_look_sources_swap_and_same_source();
  test_yaw_integration_leak_and_reset();
  test_look_per_axis_dead_and_neutral_gate();
  test_look_new_param_sanitization();
  test_monotonic_clock();
  run_game_input_tests();
  run_channel_tests();
  std::printf("all IMU, game input and channel logic tests passed\n");
  return 0;
}
