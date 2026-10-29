// Frozen v0.8.0 protocol: synthetic physics and controller/profile scenarios.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "LookController.h"

namespace {
const float kRad = 0.01745329252f;
const Vec3 kZero = {0, 0, 0};
bool near(float a, float b, float tol = 0.001f) { return std::fabs(a-b) <= tol; }
void errorIs(const char* actual, const char* expected) {
  assert(actual && std::strcmp(actual, expected) == 0);
}
Vec3 gravity(float roll, float pitch) {
  const float r = roll*kRad, p = pitch*kRad;
  return makeVec3(-std::sin(p), std::sin(r)*std::cos(p), std::cos(r)*std::cos(p));
}
Vec3 cross(const Vec3& a, const Vec3& b) {
  return makeVec3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
Vec3 rotatingAccel(const Vec3& grav, const Vec3& w, const Vec3& wd, const Vec3& r) {
  const Vec3 c = cross(w, cross(w, r)), t = cross(wd, r);
  return makeVec3(grav.x+(c.x+t.x)/9.80665f, grav.y+(c.y+t.y)/9.80665f,
                  grav.z+(c.z+t.z)/9.80665f);
}
struct Sim {
  LookController c;
  uint64_t t;
  Sim() : t(1000000) {}
  LookTick step(float roll = 0, float pitch = 0, Vec3 gyro = kZero,
                bool fresh = true, bool enable = true) {
    t += 10000;
    ImuInput in = {fresh, t, gravity(roll, pitch), gyro};
    return c.tick(t, in, enable);
  }
  LookTick still(int n, float roll = 0, float pitch = 0) {
    LookTick out = step(roll, pitch);
    for (int i = 1; i < n; ++i) out = step(roll, pitch);
    return out;
  }
  void arm() {
    c.requestArm(); step(); still(305); still(31);
    assert(c.state() == LOOK_ACTIVE && c.calibrated());
  }
};
void poses(FineCalibration* f) {
  assert(f->setPose('n', makeVec3(5, -2, 0)));
  assert(f->setPose('l', makeVec3(-5, -2, -8)));
  assert(f->setPose('r', makeVec3(25, -2, 12)));
  assert(f->setPose('d', makeVec3(5, 8, 0)));
  assert(f->setPose('u', makeVec3(5, -22, 0)));
}

void test_fuse0_v070_golden_trace() {
  // Recorded from the unmodified v0.7.0 source, including bumps and gaps.
  const uint32_t golden[12][2] = {
    {0xc0053edeU, 0x40121216U}, {0xbfaa291bU, 0x3f6bd7fbU},
    {0xbd2888b6U, 0x401115d9U}, {0xbdab0d42U, 0x3f642e52U},
    {0x407f610eU, 0x3f08eafeU}, {0x4006edb1U, 0x3e287fa3U},
    {0xbf7861deU, 0xbf9dfc90U}, {0xbebcc952U, 0xbf02f351U},
    {0x40203371U, 0xc03eab75U}, {0x3f50d4b8U, 0xbf940edeU},
    {0xc0379303U, 0xc093bb3cU}, {0xbf9faf23U, 0xbff264d4U}
  };
  ComplementaryFilter f;
  f.seed(makeVec3(0.12f, -0.09f, 0.98f));
  for (int i = 0; i < 600; ++i) {
    const Vec3 a = makeVec3((i%11-5)*0.03f, (i%7-3)*0.05f,
                            i%19 == 0 ? 2.0f : 0.97f);
    const Vec3 g = makeVec3((i%13-6)*2.5f, (i%17-8)*1.75f, (i%23-11)*3.0f);
    const float dt = i%101 == 0 ? 0.15f : (i%3 == 0 ? 0.005f : 0.01f);
    f.update(a, g, dt);
    if (i%50 == 49) {
      float r, p;
      std::memcpy(&r, &golden[i/50][0], sizeof(r));
      std::memcpy(&p, &golden[i/50][1], sizeof(p));
      // Allow libm rounding differences across host platforms.
      assert(near(f.rollDeg(), r, 0.00001f) && near(f.pitchDeg(), p, 0.00001f));
    }
  }
}

void test_fuse1_lever_arm_constant_and_ramp() {
  for (unsigned ramp = 0; ramp < 2; ++ramp) {
    ComplementaryFilter enhanced, legacy;
    enhanced.seed(gravity(0, 0)); legacy.seed(gravity(0, 0));
    float maxEnhanced = 0, maxLegacy = 0;
    for (int i = 0; i < 1000; ++i) {
      const float w = ramp ? std::fmin(2.0f, i*0.01f) : 2.0f;
      const float wd = ramp && i < 200 ? 1.0f : 0;
      const Vec3 gyro = makeVec3(0, 0, w/kRad);
      const Vec3 a = rotatingAccel(gravity(0, 0), makeVec3(0, 0, w),
                                  makeVec3(0, 0, wd), makeVec3(0.1f, 0, 0));
      assert(enhanced.updateEnhanced(a, gyro, 0.01f, 0.1f, 0, 0.15f));
      legacy.update(a, gyro, 0.01f);
      maxEnhanced = std::fmax(maxEnhanced, std::fmax(std::fabs(enhanced.rollDeg()),
                                                   std::fabs(enhanced.pitchDeg())));
      maxLegacy = std::fmax(maxLegacy, std::fmax(std::fabs(legacy.rollDeg()),
                                               std::fabs(legacy.pitchDeg())));
    }
    assert(maxEnhanced < 1 && maxLegacy > 1);
    assert(enhanced.linearG() < 0.001f && enhanced.rotationalG() > 0.04f);
    std::printf("fuse1 %s: max compensated %.3f deg, legacy %.3f deg\n",
                ramp ? "ramp" : "constant", maxEnhanced, maxLegacy);
  }
}

void test_fuse1_rejection_trust_gap_and_guard() {
  ComplementaryFilter f;
  f.seed(gravity(0, 0));
  f.updateEnhanced(makeVec3(0, 1, 2), makeVec3(10, 0, 0), 0.01f, 0, 0, 0.15f);
  assert(f.accelTrust() == 0 && !f.lastUsedAccel());
  assert(near(f.rollDeg(), 0.1f) && f.pitchDeg() == 0 && f.linearG() > 1);
  f.updateEnhanced(makeVec3(0, 0, 1.05f), kZero, 0.01f, 0, 0, 0.2f);
  assert(near(f.accelTrust(), 1));
  f.updateEnhanced(makeVec3(0, 0, 1.15f), kZero, 0.01f, 0, 0, 0.2f);
  assert(near(f.accelTrust(), 0.5f));
  f.updateEnhanced(makeVec3(0, 0, 1.2f), kZero, 0.01f, 0, 0, 0.2f);
  assert(near(f.accelTrust(), 0));
  f.updateEnhanced(gravity(5, 20), makeVec3(0, 0, 500), 0.2f, 0, 0, 0.15f);
  assert(near(f.rollDeg(), 5) && near(f.pitchDeg(), 20));
  f.seed(gravity(10, 89));
  assert(vecFinite(f.eulerRates(makeVec3(20, 30, 40))));
  const float before = f.rollDeg();
  assert(!f.updateEnhanced(makeVec3(NAN, 0, 1), kZero, 0.01f, 0, 0, 0.15f));
  assert(f.rollDeg() == before);
}

void test_fuse1_euler_pitch_then_yaw_source() {
  Sim s;
  FusionParams f = defaultFusionParams(); f.mode = 1; f.armX = f.armZ = 0;
  s.c.setFusionParams(f);
  MouseLookParams p = s.c.lookParams(); p.hsrc = 2; p.yawTauS = 0;
  s.c.setLookParams(p);
  s.arm();
  p.lmode = 1; s.c.setLookParams(p);
  for (int i = 1; i <= 100; ++i) s.step(0, i*0.3f, makeVec3(0, 30, 0));
  assert(near(s.c.filter().pitchDeg(), 30, 0.02f));
  const Vec3 bodyYaw = makeVec3(-45, 0, 90*std::cos(30*kRad));
  LookTick out;
  for (int i = 0; i < 100; ++i) out = s.step(0, 30, bodyYaw);
  assert(near(out.rollDeg, 0, 0.02f) && near(out.pitchDeg, 30, 0.02f));
  assert(near(s.c.yawDeg(), -90, 0.03f));
  assert(near(out.lookXErrDeg, -90, 0.03f));
  assert(near(out.velocity.vxCps, -8*87, 0.03f) && near(out.velocity.vyCps, 0));
}

void test_lmode_gyro_hybrid_clamp_invert_and_scale() {
  MouseLookMapper m;
  MouseLookParams p = m.params();
  p.lmode = 1; p.kgx = 8; p.kgy = 10; p.gyroDeadDps = 3;
  m.setParams(p);
  LookVelocity v = m.update(20, -20, 0.01f, 13, -5);
  assert(near(v.vxCps, 80) && near(v.vyCps, -20));
  v = m.update(20, 20, 0.01f, 3, -2);
  assert(v.vxCps == 0 && v.vyCps == 0);
  p.lmode = 2; p.kxCpsPerDeg = 10; p.kyCpsPerDeg = 20;
  p.invertX = p.invertY = true; m.setParams(p);
  v = m.update(10, -10, 0.01f, 13, -5, -2, 0.5f);
  assert(near(v.vxCps, 60) && near(v.vyCps, 210));
  p.lmode = 1; m.setParams(p);
  v = m.update(0, 0, 0.01f, 1000, -1000, 2, 2);
  assert(v.vxCps == -4000 && v.vyCps == 4000);
  p.lmode = 0; m.setParams(p);
  v = m.update(10, -10, 0.01f, 1000, -1000);
  assert(near(v.vxCps, -100) && near(v.vyCps, 200));
}

void test_lmode_selected_rates_and_only_active() {
  Sim s;
  MouseLookParams p = s.c.lookParams();
  p.lmode = 1; p.hsrc = 2; p.vsrc = 1; p.kgx = p.kgy = 8;
  s.c.setLookParams(p);
  LookTick out = s.step(0, 0, makeVec3(0, 10, -13));
  assert(out.velocity.vxCps == 0 && out.velocity.vyCps == 0);
  s.arm();
  out = s.step(0, 0, makeVec3(0, 10, -13));
  assert(near(out.velocity.vxCps, 80) && near(out.velocity.vyCps, 56));
  s.c.requestDisarm();
  out = s.step(0, 0, makeVec3(0, 10, -13));
  assert(out.state == LOOK_DISARMED && out.velocity.vxCps == 0);
  s.c.reportFault("test"); out = s.step();
  assert(out.state == LOOK_FAULT && out.velocity.vxCps == 0);
  Sim waiting;
  waiting.c.setLookParams(p); waiting.c.setOtherInputsNeutral(false);
  waiting.c.requestArm(); waiting.step();
  assert(waiting.step(0, 0, makeVec3(0, 1, 1)).velocity.vyCps == 0);
  waiting.still(305);
  assert(waiting.c.state() == LOOK_WAIT_NEUTRAL);
  out = waiting.step(0, 0, makeVec3(0, 10, -13));
  assert(out.velocity.vxCps == 0 && out.velocity.vyCps == 0);
}

void test_fcal_capture_window_average_and_motion() {
  FineCalibration f;
  errorIs(f.start('r', 0), "fcal_need_neutral");
  assert(!f.start('n', 1000));
  assert(f.addSample(1000, makeVec3(99, 99, 99), true) == 0); // not fresh
  for (int i = 1; i <= 50; ++i) {
    const float roll = i % 2 ? 9 : 11;
    const int result = f.addSample(1000+i*10000U, makeVec3(roll, 2, 7), true);
    assert(result == (i == 50 ? 1 : 0));
  }
  assert(near(f.pose(0).x, 10) && f.pose(0).y == 2 && f.pose(0).z == 0);
  assert(f.capturedNeutralYaw() == 7);
  assert(f.setPose('r', makeVec3(20, 2, 10)));
  assert(!f.start('r', 1000000));
  assert(f.addSample(1010000, makeVec3(0, 0, 0), true) == 0);
  assert(f.addSample(1500000, makeVec3(4.01f, 0, 0), true) == -1);
  assert(f.pose(2).x == 20); // preserve old pose on failure
  for (unsigned axis = 1; axis <= 2; ++axis) {
    assert(!f.start('l', 2000000));
    assert(f.addSample(2010000, kZero, true) == 0);
    const Vec3 moved = axis == 1 ? makeVec3(0, 4.1f, 0) : makeVec3(0, 0, 4.1f);
    assert(f.addSample(2500000, moved, true) == -1);
  }
  assert(!f.start('l', 3000000));
  f.addSample(3010000, kZero, false);
  assert(f.addSample(3500000, makeVec3(4, 4, 100), false) == 1); // yaw unknown
  assert(!f.start('n', 4000000));
  f.addSample(4010000, makeVec3(1, 2, 8), true);
  assert(f.addSample(4500000, makeVec3(1, 2, 8), true) == 1);
  assert(f.has(1) && f.has(2));  // new N keeps other poses
  assert(!f.start('n', 5000000));
  f.addSample(5100000, makeVec3(1, 2, 3), true);
  assert(f.addSample(5500001, makeVec3(99, 99, 99), true) == 1);
  assert(f.pose(0).x == 1 && f.pose(0).y == 2); // after-window sample excluded
}

void test_fcal_validation_auto_ties_and_yaw_eligibility() {
  FineCalibration f; uint8_t h = 0, v = 1;
  errorIs(f.enable(true, &h, &v), "fcal_incomplete");
  poses(&f);
  assert(!f.enable(true, &h, &v) && h == 0 && v == 1);
  f.off(); assert(f.has(0) && f.has(4));
  f.setPose('l', makeVec3(15, -2, -8)); // same-sign roll, valid yaw
  assert(!f.enable(true, &h, &v) && h == 2);
  errorIs(f.enable(false, &h, &v), "fcal_bad_h");
  assert(!f.on());
  f.setPose('l', makeVec3(2.01f, -2, 0)); // below 3, yaw invalid
  errorIs(f.enable(true, &h, &v), "fcal_bad_h");
  f.setPose('l', makeVec3(2, -2, 0)); // exactly 3 accepted
  assert(!f.enable(false, &h, &v) && h == 0);
  f.setAutoSource(false); h = 1;
  errorIs(f.enable(true, &h, &v), "fcal_bad_h");
  h = 0; v = 0;
  errorIs(f.enable(true, &h, &v), "fcal_bad_v");
  v = 1; assert(!f.enable(true, &h, &v));
  f.clear(); assert(!f.on() && !f.has(0) && !f.has(4));
  poses(&f); f.setAutoSource(true);
  f.setPose('l', makeVec3(-5, -12, -10));
  f.setPose('r', makeVec3(15, 8, 10));
  assert(!f.enable(true, &h, &v) && h == 0); // tie: roll first
  f.setPose('l', makeVec3(5, -12, -10));
  f.setPose('r', makeVec3(5, 8, 10));
  assert(!f.enable(true, &h, &v) && h == 1); // pitch wins yaw tie
}

void test_fcal_asymmetric_mapping_and_signed_gyro_scale() {
  FineCalibration f; poses(&f); uint8_t h = 0, v = 1;
  assert(!f.enable(true, &h, &v));
  assert(near(f.map(0, h, 25, 20), 20) && near(f.map(0, h, -5, 20), -20));
  assert(near(f.map(0, h, 15, 20), 10) && near(f.map(0, h, 0, 20), -10));
  assert(near(f.map(1, v, 8, 20), 20) && near(f.map(1, v, -22, 20), -20));
  assert(f.map(0, h, 5, 20) == 0);
  assert(near(f.gyroScale(0, h, 20), 40.0f/30));
  f.setPose('r', makeVec3(-15, -2, 0));
  f.setPose('l', makeVec3(15, -2, 0));
  assert(!f.enable(true, &h, &v));
  assert(near(f.map(0, h, -15, 20), 20) && near(f.map(0, h, 15, 20), -20));
  assert(near(f.gyroScale(0, h, 20), -40.0f/30));
}

void test_fcal_val_round_trip_and_off_clear() {
  FineCalibration f, replay; poses(&f);
  const char* letters = "nlrdu";
  for (unsigned i = 0; i < 5; ++i) {
    const Vec3 a = f.pose(i);
    char line[128], pose; float roll, pitch, yaw;
    std::snprintf(line, sizeof(line), "fcal val %c %.2f %.2f %.2f", letters[i], a.x, a.y, a.z);
    assert(std::sscanf(line, "fcal val %c %f %f %f", &pose, &roll, &pitch, &yaw) == 4);
    assert(replay.setPose(pose, makeVec3(roll, pitch, yaw)));
    const Vec3 b = replay.pose(i);
    assert(a.x == b.x && a.y == b.y && a.z == b.z);
  }
  uint8_t h = 0, v = 1; assert(!replay.enable(true, &h, &v));
  replay.off(); assert(!replay.on() && replay.has(4));
  replay.clear(); for (unsigned i = 0; i < 5; ++i) assert(!replay.has(i));
  assert(replay.setPose('r', kZero));  // direct profile poses may be replayed in any order
  assert(!replay.setPose('?', kZero));
  assert(!replay.setPose('n', makeVec3(NAN, 0, 0)));
}

void test_fcal_controller_capture_rejections_and_yaw_reference() {
  Sim s;
  errorIs(s.c.startFineCapture('l', s.t), "fcal_need_neutral");
  assert(!s.c.startFineCapture('n', s.t));
  s.still(49);
  assert(!s.c.fineCalibration().has(0));
  LookTick out = s.step();
  assert(out.fineEvent && !std::strcmp(out.fineEvent, "captured") && out.finePose == 'n');
  s.arm();
  errorIs(s.c.startFineCapture('n', s.t), "fcal_active");
  s.c.requestDisarm(); s.step();
  MouseLookParams p = s.c.lookParams(); p.yawTauS = 0; s.c.setLookParams(p);
  for (int i = 0; i < 100; ++i) s.step(0, 0, makeVec3(0, 0, -10));
  assert(near(s.c.yawDeg(), 10));
  assert(!s.c.startFineCapture('n', s.t));
  out = s.still(50);
  assert(out.fineEvent && !std::strcmp(out.fineEvent, "captured"));
  assert(near(s.c.yawDeg(), 0) && s.c.fineCalibration().pose(0).z == 0);
  for (int i = 0; i < 100; ++i) s.step(0, 0, makeVec3(0, 0, -10));
  assert(!s.c.startFineCapture('r', s.t));
  out = s.still(50);
  assert(out.fineEvent && s.c.fineCalibration().has(2));
  assert(near(s.c.fineCalibration().pose(2).z, 10));
  s.c.reportFault("test"); s.step();
  errorIs(s.c.startFineCapture('n', s.t), "fcal_fault");
}

void test_fcal_controller_mapping_gate_scale_source_and_mount() {
  Sim s;
  s.c.setAlpha(0.01f);
  poses(&s.c.fineCalibration());
  assert(!s.c.enableFineCalibration());
  MouseLookParams p = s.c.lookParams();
  p.lmode = 1; p.kgx = p.kgy = 8; s.c.setLookParams(p);
  s.c.setOtherInputsNeutral(false);
  s.c.requestArm(); s.step(); s.still(305);
  assert(s.c.state() == LOOK_WAIT_NEUTRAL);
  s.c.setOtherInputsNeutral(true);
  // Basic neutral is zero, but fine neutral roll=5,pitch=-2 (5 maps to -10).
  assert(s.still(60).state == LOOK_WAIT_NEUTRAL);
  assert(s.still(50, 5, -2).state == LOOK_ACTIVE);
  LookTick out = s.step(5, -2, makeVec3(13, 0, 0));
  assert(near(out.lookXErrDeg, 0, 0.01f));
  assert(near(out.velocity.vxCps, 8*10*40.0f/30, 0.02f));
  s.c.requestDisarm(); s.step(5, -2);
  s.c.fineCalibration().off();
  out = s.still(50, 5, -2);
  assert(near(out.lookXErrDeg, 5, 0.01f) && near(out.lookYErrDeg, -2, 0.01f));
  assert(!s.c.enableFineCalibration());
  p = s.c.lookParams(); p.hsrc = 2;
  assert(!s.c.setLookParams(p) && s.c.fineCalibration().on()); // valid yaw
  p.hsrc = 1;
  assert(s.c.setLookParams(p) && !s.c.fineCalibration().on());
  assert(s.c.fineCalibration().has(4));
  assert(s.c.setMount(AXIS_NEG_Y, AXIS_POS_Z));
  assert(!s.c.fineCalibration().has(0));
  out = s.step(); assert(!s.c.calibrated() && out.state == LOOK_DISARMED);
}

void test_fcal_capture_freshness_and_state_transition_cancel() {
  Sim s;
  assert(!s.c.startFineCapture('n', s.t));
  s.still(25);
  for (int i = 0; i < 10; ++i) s.step(0, 0, kZero, false);
  assert(s.still(15).fineEvent);  // 40 fresh samples, no stale samples averaged
  assert(s.c.fineCalibration().pose(0).x == 0);
  assert(!s.c.startFineCapture('l', s.t));
  s.c.reportFault("test");
  LookTick out = s.step();
  errorIs(out.fineEvent, "fcal_fault");
  assert(!s.c.fineCalibration().capturing() && !s.c.fineCalibration().has(1));

  Sim waiting;
  waiting.c.requestArm(); waiting.step(); waiting.still(305);
  assert(waiting.c.state() == LOOK_WAIT_NEUTRAL);
  assert(!waiting.c.startFineCapture('n', waiting.t));
  bool cancelled = false;
  for (int i = 0; i < 40; ++i) {
    out = waiting.step();
    if (out.fineEvent) {
      errorIs(out.fineEvent, "fcal_active");
      cancelled = true;
    }
  }
  assert(cancelled && !waiting.c.fineCalibration().has(0));
}

void test_v080_sanitize_and_legacy_defaults() {
  FusionParams f = defaultFusionParams();
  assert(f.mode == 0 && f.armX == 0.10f && f.armZ == -0.05f && f.accelRejectG == 0.15f);
  MouseLookParams p = defaultMouseLookParams();
  assert(p.lmode == 0 && p.kgx == 8 && p.kgy == 8 && p.gyroDeadDps == 3);
  LookController c; assert(!c.fineCalibration().on() && c.fineCalibration().autoSource());
  f.mode = 9; f.armX = -1; f.armZ = 1; f.accelRejectG = -1;
  sanitizeFusionParams(&f);
  assert(f.mode == 1 && f.armX == -0.3f && f.armZ == 0.3f && f.accelRejectG == 0.02f);
  f.armX = f.armZ = f.accelRejectG = NAN; sanitizeFusionParams(&f);
  assert(f.armX == 0.1f && f.armZ == -0.05f && f.accelRejectG == 0.15f);
  f.accelRejectG = 2; sanitizeFusionParams(&f); assert(f.accelRejectG == 1);
  p.lmode = 9; p.kgx = -1; p.kgy = 200; p.gyroDeadDps = 100;
  assert(sanitizeMouseLookParams(&p));
  assert(p.lmode == 2 && p.kgx == 0 && p.kgy == 100 && p.gyroDeadDps == 50);
}
}  // namespace

void run_v080_tests() {
  test_fuse0_v070_golden_trace();
  test_fuse1_lever_arm_constant_and_ramp();
  test_fuse1_rejection_trust_gap_and_guard();
  test_fuse1_euler_pitch_then_yaw_source();
  test_lmode_gyro_hybrid_clamp_invert_and_scale();
  test_lmode_selected_rates_and_only_active();
  test_fcal_capture_window_average_and_motion();
  test_fcal_validation_auto_ties_and_yaw_eligibility();
  test_fcal_asymmetric_mapping_and_signed_gyro_scale();
  test_fcal_val_round_trip_and_off_clear();
  test_fcal_controller_capture_rejections_and_yaw_reference();
  test_fcal_controller_mapping_gate_scale_source_and_mount();
  test_fcal_capture_freshness_and_state_transition_cancel();
  test_v080_sanitize_and_legacy_defaults();
  std::puts("all v0.8.0 fusion, look-law and fine-calibration tests passed");
}
