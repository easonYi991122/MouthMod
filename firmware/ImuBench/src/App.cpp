#include "App.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "hardware/HidOutput.h"
#include "hardware/Hx710b.h"
#include "hardware/ImuReader.h"
#include "hardware/Watchdog.h"
#include "logic/GameInput.h"
#include "logic/JoystickMapper.h"
#include "logic/LookController.h"
#include "logic/MonotonicClock.h"
#include "logic/MouseLookMapper.h"
#include "logic/PressureChannel.h"
#include "logic/SipArmGate.h"

namespace {

HidOutput gHid;
ImuReader gImu;
LookController gLook;
MonotonicClock64 gClock;
DeltaAccumulator gAccX(cfg::kMaxPendingCounts);
DeltaAccumulator gAccY(cfg::kMaxPendingCounts);
GameInput gGame;

// Body channels (ported from the team's joystick and puff sketches).
JoystickMapper gJoy;
PressureChannel gBreath;  // module #1: puff -> right button, sip -> disarm
PressureChannel gBite;    // module #2: bite -> Space
Hx710b gBreathHx(cfg::kPinBreathOut, cfg::kPinBreathSck);
Hx710b gBiteHx(cfg::kPinBiteOut, cfg::kPinBiteSck);
bool gUseJoy = cfg::kUseJoystick;
bool gUseBreath = cfg::kUseBreath;
bool gUseBite = cfg::kUseBite;
uint64_t gBreathSinceUs = 0;  // enable time, starts the startup grace
uint64_t gBiteSinceUs = 0;
uint64_t gSpanUntilUs = 0;    // "jspan" capture end; 0 = idle
bool gSipFired = false;
SipArmGate gSipArm(cfg::kSipArmUs);
Watchdog gWdt;
uint32_t gResetReason = 0;
int gJoyRaw[2] = {0, 0};
uint64_t gNextSensorUs = 0;

// Bench-only action source (serial "act"/"tap"), OR-ed with the channels.
GameIntent gBench = noGameIntent();
uint64_t gActUntilUs[6] = {0, 0, 0, 0, 0, 0};  // 0 = held until "act x 0"
bool gTapJump = false;
bool gTapUse = false;
uint8_t gLoggedKeys[6] = {0, 0, 0, 0, 0, 0};
uint8_t gLoggedButtons = 0;

// Bench-only sensitivity test (serial "turn"): exact count of mouse X counts.
int32_t gTurnLeft = 0;
int32_t gTurnTotal = 0;
float gTurnRateCps = 0.0f;
float gTurnFrac = 0.0f;

bool gHidEnabled = cfg::kHidEnabledAtBoot;
bool gFocusGate = false;       // "fgate 1": real HID only while Minecraft is front
uint64_t gFocusUntilUs = 0;    // refreshed by the host's "focus 1"
bool gFocusLogged = true;      // last reported gate result (for #F lines)
uint16_t gTelemetryHz = cfg::kDefaultTelemetryHz;

uint64_t gNextTickUs = 0;
uint64_t gLastTickUs = 0;
uint32_t gLoopDtUs = 0;
uint32_t gMaxLoopDtUs = 0;
uint64_t gNextTelemetryUs = 0;

bool gSerialWasOpen = false;
bool gEnableWasClosed = false;
int gDxSum = 0;
int gDySum = 0;
bool gImuNewSinceLine = false;
const char* gLineEvent = 0;
uint32_t gLogDropped = 0;
uint32_t gImuReadErrors = 0;

char gCmd[80];
uint8_t gCmdLen = 0;

uint64_t nowUs() { return gClock.update(micros()); }

// Bounded write: never blocks the safety loop when the host is not reading.
void emit(const char* line) {
  if (!Serial) return;
  const int len = static_cast<int>(strlen(line));
  if (Serial.availableForWrite() < len) {
    ++gLogDropped;
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t*>(line), len);
}

void emitf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void emitf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  emit(buf);
}

bool enableInputClosed() {
  bool ok = true;
  if (cfg::kUseD6Enable) ok = ok && digitalRead(cfg::kPinEnable) == LOW;
  if (cfg::kDisarmOnSerialClose) ok = ok && static_cast<bool>(Serial);
  return ok;
}

void printHeader() {
  emit("#H,t_ms,loop_dt_us,state,imu_new,imu_age_ms,ax_g,ay_g,az_g,"
       "gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,roll_err,pitch_err,"
       "vx_cps,vy_cps,dx,dy,hid,engaged,log_dropped,event\n");
  emit("#HS,t_ms,joy_x_raw,joy_y_raw,joy_x_norm,joy_y_norm,joy_keys,"
       "breath_raw,breath_baseline,breath_p,puff,sip,bite_raw,bite_baseline,"
       "bite_p,bite,breath_age_ms\n");
}

void printChannels() {
  const JoystickParams& j = gJoy.params();
  const PressureParams& b = gBreath.params();
  const PressureParams& k = gBite.params();
  emitf("#Q,joy=%u,jcenter=%.0f/%.0f,jon=%.3f,joff=%.3f,jspan=%.0f/%.0f/%.0f/%.0f,"
        "jswap=%u,jinvx=%u,jinvy=%u,breath=%u,bsign=%d,bon=%ld,boff=%ld,son=%ld,"
        "soff=%ld,brel_ms=%lu,slock_ms=%lu,bite=%u,ksign=%d,kon=%ld,koff=%ld\n",
        gUseJoy ? 1U : 0U, gJoy.center(0), gJoy.center(1), j.onNorm, j.offNorm,
        j.spanPos[0], j.spanNeg[0], j.spanPos[1], j.spanNeg[1], j.swapXY ? 1U : 0U,
        j.invertX ? 1U : 0U, j.invertY ? 1U : 0U, gUseBreath ? 1U : 0U, b.sign,
        static_cast<long>(b.onCounts), static_cast<long>(b.offCounts),
        static_cast<long>(b.sipOnCounts), static_cast<long>(b.sipOffCounts),
        static_cast<unsigned long>(b.releaseHoldUs / 1000U),
        static_cast<unsigned long>(b.sipLockoutUs / 1000U), gUseBite ? 1U : 0U,
        k.sign, static_cast<long>(k.onCounts), static_cast<long>(k.offCounts));
}

void printParams() {
  const MouseLookParams& p = gLook.lookParams();
  emitf("#P,kx=%.2f,ky=%.2f,dead=%.2f,hyst=%.2f,sat=%.2f,hmode=%u,vmode=%u,"
        "confirm=%.0f,invx=%u,invy=%u,alpha=%.4f,mount=%s%s,hid=%u,rate=%u,"
        "fgate=%u\n",
        p.kxCpsPerDeg, p.kyCpsPerDeg, p.deadbandDeg, p.hysteresisDeg,
        p.saturationDeg, p.hystMode, p.velMode, p.engageConfirmS * 1e3f,
        p.invertX ? 1U : 0U,
        p.invertY ? 1U : 0U, gLook.alpha(), signedAxisName(gLook.mount().up()),
        signedAxisName(gLook.mount().forward()), gHidEnabled ? 1U : 0U,
        gTelemetryHz, gFocusGate ? 1U : 0U);
}

void printIdentity() {
  emitf("#I,fw=%s,ver=%s,bsp=%s,imu_ok=%u,whoami=0x%02X,odr=%u,acc_g=%u,"
        "gyro_dps=%u,d6_enable=%u,wdt=%u,reset=%s/0x%lx,sip_arm=%u,"
        "max_loop_dt_us=%lu\n",
        cfg::kFirmwareName, cfg::kFirmwareVersion, ARDUINO_BSP_VERSION,
        gImu.ok() ? 1U : 0U, gImu.whoAmI(), cfg::kImuOdrHz, cfg::kAccelRangeG,
        cfg::kGyroRangeDps, cfg::kUseD6Enable ? 1U : 0U,
        gWdt.running() ? 1U : 0U, Watchdog::resetReasonName(gResetReason),
        static_cast<unsigned long>(gResetReason), cfg::kSipArms ? 1U : 0U,
        static_cast<unsigned long>(gMaxLoopDtUs));
}

void printCalibration() {
  const CalibrationResult& r = gLook.calibration().result();
  emitf("#C,status=%s,samples=%lu,bias=%.3f/%.3f/%.3f,gyro_std=%.3f/%.3f/%.3f,"
        "roll0=%.2f,pitch0=%.2f\n",
        calibrationStatusName(gLook.calibration().status()),
        static_cast<unsigned long>(r.samples), r.gyroBiasDps.x,
        r.gyroBiasDps.y, r.gyroBiasDps.z, r.gyroStdDps.x, r.gyroStdDps.y,
        r.gyroStdDps.z, r.roll0Deg, r.pitch0Deg);
}

// Channel thresholds: jon joff jswap jinvx jinvy | bsign bon boff son soff
// brel(ms) slock(ms) | ksign kon koff. Takes effect at once; copy final values into
// the defaults in JoystickMapper.cpp / PressureChannel.cpp.
bool setChannelParam(const char* name, float v) {
  JoystickParams j = gJoy.params();
  PressureParams b = gBreath.params();
  PressureParams k = gBite.params();
  const int8_t sign = v < 0 ? -1 : 1;
  const int32_t counts = static_cast<int32_t>(v);
  if (!strcmp(name, "jon")) j.onNorm = v;
  else if (!strcmp(name, "joff")) j.offNorm = v;
  else if (!strcmp(name, "jswap")) j.swapXY = v != 0.0f;
  else if (!strcmp(name, "jinvx")) j.invertX = v != 0.0f;
  else if (!strcmp(name, "jinvy")) j.invertY = v != 0.0f;
  else if (!strcmp(name, "bsign")) b.sign = sign;
  else if (!strcmp(name, "bon")) b.onCounts = counts;
  else if (!strcmp(name, "boff")) b.offCounts = counts;
  else if (!strcmp(name, "son")) b.sipOnCounts = counts;
  else if (!strcmp(name, "soff")) b.sipOffCounts = counts;
  else if (!strcmp(name, "brel")) b.releaseHoldUs = static_cast<uint32_t>(v * 1000.0f);
  else if (!strcmp(name, "slock")) b.sipLockoutUs = static_cast<uint32_t>(v * 1000.0f);
  else if (!strcmp(name, "ksign")) k.sign = sign;
  else if (!strcmp(name, "kon")) k.onCounts = counts;
  else if (!strcmp(name, "koff")) k.offCounts = counts;
  else return false;
  gJoy.setParams(j);
  gBreath.setParams(b);
  gBite.setParams(k);
  printChannels();
  return true;
}

bool setParam(const char* name, const char* value) {
  char* end = 0;
  const float v = strtof(value, &end);
  if (end == value) return false;
  if (setChannelParam(name, v)) return true;
  MouseLookParams p = gLook.lookParams();
  if (!strcmp(name, "kx")) p.kxCpsPerDeg = v;
  else if (!strcmp(name, "ky")) p.kyCpsPerDeg = v;
  else if (!strcmp(name, "dead")) p.deadbandDeg = v;
  else if (!strcmp(name, "hyst")) p.hysteresisDeg = v;
  else if (!strcmp(name, "sat")) p.saturationDeg = v;
  else if (!strcmp(name, "hmode")) p.hystMode = static_cast<uint8_t>(v);
  else if (!strcmp(name, "vmode")) p.velMode = static_cast<uint8_t>(v);
  else if (!strcmp(name, "invx")) p.invertX = v != 0.0f;
  else if (!strcmp(name, "invy")) p.invertY = v != 0.0f;
  else if (!strcmp(name, "confirm")) p.engageConfirmS = v * 1e-3f;  // ms
  else if (!strcmp(name, "alpha")) {
    gLook.setAlpha(v);
    return true;
  } else if (!strcmp(name, "rate")) {
    gTelemetryHz = static_cast<uint16_t>(v < 1 ? 0 : (v > 100 ? 100 : v));
    return true;
  } else {
    return false;
  }
  gLook.setLookParams(p);
  return true;
}

void clearBenchActions() {
  gBench = noGameIntent();
  memset(gActUntilUs, 0, sizeof(gActUntilUs));
  gTapJump = gTapUse = false;
  if (gTurnLeft != 0) {
    emitf("#ERR,turn aborted,remaining=%ld\n", static_cast<long>(gTurnLeft));
  }
  gTurnLeft = 0;
  gTurnFrac = 0.0f;
}

bool* benchField(const char* n, int* index) {
  bool* const fields[6] = {&gBench.forward, &gBench.back, &gBench.left,
                           &gBench.right,   &gBench.jump, &gBench.use};
  static const char* const names[6] = {"w", "s", "a", "d", "jump", "use"};
  for (int i = 0; i < 6; ++i) {
    if (!strcmp(n, names[i])) {
      *index = i;
      return fields[i];
    }
  }
  return 0;
}

// act <w|a|s|d|jump|use> <0|1|ms>  |  act clear
// 1 holds until "act x 0"; a value >= 2 holds for that many ms.
void handleAct(int argc, char** argv) {
  if (!strcmp(argv[1], "clear")) {
    gBench = noGameIntent();
    memset(gActUntilUs, 0, sizeof(gActUntilUs));
    emit("#OK,act clear\n");
    return;
  }
  int index = 0;
  bool* field = argc == 3 ? benchField(argv[1], &index) : 0;
  if (!field) {
    emit("#ERR,act <w|a|s|d|jump|use> <0|1|ms> or act clear\n");
    return;
  }
  const long v = strtol(argv[2], 0, 10);
  if (v != 0 && gLook.state() != LOOK_ACTIVE) {
    emit("#ERR,act needs ACTIVE\n");
    return;
  }
  *field = v != 0;
  gActUntilUs[index] = v >= 2 ? nowUs() + static_cast<uint64_t>(v) * 1000U : 0;
  emitf("#OK,act %s=%ld\n", argv[1], v);
}

void expireBenchActions(uint64_t t) {
  int index = 0;
  static const char* const names[6] = {"w", "s", "a", "d", "jump", "use"};
  for (int i = 0; i < 6; ++i) {
    if (gActUntilUs[i] != 0 && t >= gActUntilUs[i]) {
      gActUntilUs[i] = 0;
      *benchField(names[i], &index) = false;
    }
  }
}

// turn <counts> [rate_cps]: sends exactly <counts> mouse X counts (head look
// paused meanwhile). Used to measure Minecraft's degrees per count and to
// check that slow and fast turns of equal counts give equal angles.
void startTurn(int argc, char** argv) {
  const long counts = strtol(argv[1], 0, 10);
  const float rate = argc >= 3 ? strtof(argv[2], 0) : 400.0f;
  if (gLook.state() != LOOK_ACTIVE) {
    emit("#ERR,turn needs ACTIVE\n");
  } else if (counts == 0 || counts > 20000 || counts < -20000 || rate < 10.0f ||
             rate > 5000.0f) {
    emit("#ERR,turn <counts 1..20000, signed> [rate 10..5000 cps]\n");
  } else {
    gTurnLeft = gTurnTotal = static_cast<int32_t>(counts);
    gTurnRateCps = rate;
    gTurnFrac = 0.0f;
    emitf("#OK,turn %ld @ %.0f cps\n", counts, rate);
  }
}

// #K line whenever the committed keyboard/button state changes.
void logGameOutput(uint64_t t) {
  const uint8_t* keys = gGame.committedKeys();
  const uint8_t buttons = gGame.committedButtons();
  if (!memcmp(keys, gLoggedKeys, 6) && buttons == gLoggedButtons) return;
  memcpy(gLoggedKeys, keys, 6);
  gLoggedButtons = buttons;
  char names[48] = "";
  for (int i = 0; i < 6 && keys[i]; ++i) {
    const char* n = keys[i] == mc::kKeyForward ? "W"
                    : keys[i] == mc::kKeyBack  ? "S"
                    : keys[i] == mc::kKeyLeft  ? "A"
                    : keys[i] == mc::kKeyRight ? "D"
                    : keys[i] == mc::kKeyJump  ? "SPACE"
                                               : "?";
    if (names[0]) strncat(names, "+", sizeof(names) - strlen(names) - 1);
    strncat(names, n, sizeof(names) - strlen(names) - 1);
  }
  emitf("#K,%lu,keys=%s,left=%u,hid=%u\n", static_cast<unsigned long>(t / 1000U),
        names[0] ? names : "-", (buttons & mc::kButtonPuff) ? 1U : 0U,
        gHidEnabled ? 1U : 0U);
}

// chan <joy|breath|bite> <0|1>: lets one build run with or without each
// sensor. Any change disarms; the channel restarts its calibration/grace.
void setChannel(const char* name, bool on) {
  const uint64_t t = nowUs();
  if (!strcmp(name, "joy")) {
    gUseJoy = on;
    gSpanUntilUs = 0;
  } else if (!strcmp(name, "breath")) {
    gUseBreath = on;
    gBreath.reset();
    gBreathSinceUs = t;
  } else if (!strcmp(name, "bite")) {
    gUseBite = on;
    gBite.reset();
    gBiteSinceUs = t;
  } else {
    emit("#ERR,chan <joy|breath|bite> <0|1>\n");
    return;
  }
  if (gLook.state() != LOOK_DISARMED && gLook.state() != LOOK_FAULT) {
    gLook.requestDisarm();
  }
  printChannels();
}

void startSpanCapture() {
  if (!gUseJoy || gLook.state() != LOOK_DISARMED) {
    emit("#ERR,jspan needs joy=1 and DISARMED\n");
    return;
  }
  gJoy.beginSpanCapture();
  gSpanUntilUs = nowUs() + cfg::kJoySpanCaptureUs;
  emit("#OK,jspan: keep released now, then push all 4 directions to the "
       "comfortable limit within 6 s\n");
}

void handleCommand(char* line) {
  char* argv[4] = {0, 0, 0, 0};
  int argc = 0;
  for (char* tok = strtok(line, " \t\r"); tok && argc < 4;
       tok = strtok(0, " \t\r")) {
    argv[argc++] = tok;
  }
  if (argc == 0) return;
  const char* c = argv[0];
  if (!strcmp(c, "arm")) {
    if (gSpanUntilUs != 0) {
      emit("#ERR,arm: jspan running\n");
      return;
    }
    gLook.requestArm();
    emit("#OK,arm\n");
  } else if (!strcmp(c, "disarm") || !strcmp(c, "x")) {
    gLook.requestDisarm();
    emit("#OK,disarm\n");
  } else if (!strcmp(c, "ack")) {
    gLook.acknowledgeFault();
    emit("#OK,ack\n");
  } else if (!strcmp(c, "hid") && argc == 2) {
    gHidEnabled = atoi(argv[1]) != 0;
    gHid.requestReleaseAll();  // real host goes neutral either way
    clearBenchActions();
    gGame.releaseAll();        // committed state restarts from neutral
    emitf("#OK,hid=%u\n", gHidEnabled ? 1U : 0U);
  } else if (!strcmp(c, "fgate") && argc == 2) {
    gFocusGate = atoi(argv[1]) != 0;
    emitf("#OK,fgate=%u\n", gFocusGate ? 1U : 0U);
  } else if (!strcmp(c, "focus") && argc == 2) {
    // Quiet on purpose: the host repeats this several times a second.
    gFocusUntilUs = atoi(argv[1]) != 0 ? nowUs() + cfg::kFocusHoldUs : 0;
  } else if (!strcmp(c, "chan") && argc == 3) {
    setChannel(argv[1], atoi(argv[2]) != 0);
  } else if (!strcmp(c, "hang")) {
    // Fault injection for the watchdog test: stop feeding, expect a reset
    // that comes back DISARMED with reset=watchdog in #I.
    if (!gWdt.running()) {
      emit("#ERR,hang needs the watchdog running (cfg::kWatchdogStarted)\n");
    } else {
      emit("#OK,hang: loop stopped, expect watchdog reset\n");
      Serial.flush();
      while (true) {
      }
    }
  } else if (!strcmp(c, "jspan")) {
    startSpanCapture();
  } else if (!strcmp(c, "act") && argc >= 2) {
    handleAct(argc, argv);
  } else if (!strcmp(c, "tap") && argc == 2) {
    if (gLook.state() != LOOK_ACTIVE) {
      emit("#ERR,tap needs ACTIVE\n");
    } else if (!strcmp(argv[1], "jump")) {
      gTapJump = true;
      emit("#OK,tap jump\n");
    } else if (!strcmp(argv[1], "use")) {
      gTapUse = true;
      emit("#OK,tap use\n");
    } else {
      emit("#ERR,tap jump|use\n");
    }
  } else if (!strcmp(c, "turn") && argc >= 2) {
    startTurn(argc, argv);
  } else if (!strcmp(c, "set") && argc == 3) {
    if (setParam(argv[1], argv[2])) {
      printParams();
    } else {
      emitf("#ERR,set %s\n", argv[1]);
    }
  } else if (!strcmp(c, "mount") && argc == 3) {
    SignedAxis up, fwd;
    if (parseSignedAxis(argv[1], &up) && parseSignedAxis(argv[2], &fwd) &&
        gLook.setMount(up, fwd)) {
      printParams();
    } else {
      emit("#ERR,mount needs two perpendicular axes, e.g. mount +Z +X\n");
    }
  } else if (!strcmp(c, "get")) {
    printIdentity();
    printParams();
    printChannels();
    printCalibration();
    printHeader();
  } else {
    emitf("#ERR,unknown %s\n", c);
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\n') {
      gCmd[gCmdLen] = '\0';
      handleCommand(gCmd);
      gCmdLen = 0;
    } else if (gCmdLen < sizeof(gCmd) - 1) {
      gCmd[gCmdLen++] = static_cast<char>(ch);
    }
  }
}

void writeTelemetry(uint64_t t, const LookTick& k) {
  if (gTelemetryHz == 0 || t < gNextTelemetryUs) return;
  // Fixed cadence: advance from the schedule, not from now, so 10 ms tick
  // jitter does not stretch the period to the next tick.
  const uint32_t period = 1000000U / gTelemetryHz;
  gNextTelemetryUs += period;
  if (gNextTelemetryUs + period < t) gNextTelemetryUs = t + period;
  const Vec3 a = gLook.lastAccelBody();
  const Vec3 g = gLook.lastGyroBody();
  const uint32_t ageMs = gLook.hasSample()
                             ? static_cast<uint32_t>((t - gLook.lastSampleUs()) / 1000U)
                             : 0xFFFFU;
  char buf[320];
  snprintf(buf, sizeof(buf),
           "D,%lu,%lu,%s,%u,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
           "%.2f,%.1f,%.1f,%d,%d,%u,%u%u,%lu,%s\n",
           static_cast<unsigned long>(t / 1000U),
           static_cast<unsigned long>(gLoopDtUs), lookStateName(k.state),
           gImuNewSinceLine ? 1U : 0U, static_cast<unsigned long>(ageMs), a.x,
           a.y, a.z, g.x, g.y, g.z, k.rollDeg, k.pitchDeg, k.rollErrDeg,
           k.pitchErrDeg, k.velocity.vxCps, k.velocity.vyCps, gDxSum, gDySum,
           gHidEnabled ? 1U : 0U, k.velocity.engagedX ? 1U : 0U,
           k.velocity.engagedY ? 1U : 0U,
           static_cast<unsigned long>(gLogDropped),
           gLineEvent ? gLineEvent : "");
  emit(buf);
  gDxSum = 0;
  gDySum = 0;
  gImuNewSinceLine = false;
  gLineEvent = 0;
}

int readAdcAverage(uint8_t pin) {
  analogRead(pin);  // discard the first sample after switching channel
  long total = 0;
  for (int i = 0; i < cfg::kJoyAdcAverage; ++i) total += analogRead(pin);
  return static_cast<int>(total / cfg::kJoyAdcAverage);
}

// Step 2 of the loop: joystick ADC, and whichever HX710B has OUT low.
void readChannels(uint64_t t) {
  if (gUseJoy) {
    gJoyRaw[0] = readAdcAverage(cfg::kPinJoyX);
    gJoyRaw[1] = readAdcAverage(cfg::kPinJoyY);
    gJoy.update(static_cast<float>(gJoyRaw[0]), static_cast<float>(gJoyRaw[1]),
                gLoopDtUs);
    if (gSpanUntilUs != 0 && t >= gSpanUntilUs) {
      gSpanUntilUs = 0;
      const bool ok = gJoy.finishSpanCapture();
      emitf("#E,%lu,%s,%s\n", static_cast<unsigned long>(t / 1000U),
            ok ? "jspan_done" : "jspan_partial", lookStateName(gLook.state()));
      printChannels();
    }
  }
  uint32_t bits = 0;
  if (gUseBreath && gBreathHx.tryRead(&bits)) {
    gBreath.addSample(t, hx710bToSigned(bits));
  }
  if (gUseBite && gBiteHx.tryRead(&bits)) {
    gBite.addSample(t, hx710bToSigned(bits));
  }
}

// Step 3: a pressure channel that stops converting latches FAULT.
void checkPressureFresh(const PressureChannel& c, bool use, uint64_t since,
                        const char* fault, uint64_t t) {
  if (!use || gLook.state() == LOOK_FAULT) return;
  const bool fresh = c.hasSample() && c.lastSampleUs() >= since;
  const uint64_t ref = fresh ? c.lastSampleUs() : since;
  const uint32_t limit = fresh ? cfg::kPressureTimeoutUs : cfg::kPressureGraceUs;
  if (t > ref && t - ref > limit) gLook.reportFault(fault);
}

bool otherInputsNeutral() {
  return (!gUseJoy || gJoy.neutral()) && (!gUseBreath || gBreath.neutral()) &&
         (!gUseBite || gBite.neutral());
}

void writeSensorLine(uint64_t t) {
  if (t < gNextSensorUs) return;
  gNextSensorUs = t + 1000000U / cfg::kSensorLineHz;
  char keys[6] = "";
  int n = 0;
  if (gJoy.forward()) keys[n++] = 'W';
  if (gJoy.back()) keys[n++] = 'S';
  if (gJoy.left()) keys[n++] = 'A';
  if (gJoy.right()) keys[n++] = 'D';
  keys[n] = '\0';
  const uint32_t ageMs =
      gBreath.hasSample() ? static_cast<uint32_t>((t - gBreath.lastSampleUs()) / 1000U)
                          : 0xFFFFU;
  emitf("S,%lu,%d,%d,%.3f,%.3f,%s,%ld,%ld,%ld,%u,%u,%ld,%ld,%ld,%u,%lu\n",
        static_cast<unsigned long>(t / 1000U), gJoyRaw[0], gJoyRaw[1],
        gJoy.norm(0), gJoy.norm(1), n ? keys : "-",
        static_cast<long>(gBreath.raw()), static_cast<long>(gBreath.baseline()),
        static_cast<long>(gBreath.pressure()), gBreath.active() ? 1U : 0U,
        gBreath.sip() ? 1U : 0U, static_cast<long>(gBite.raw()),
        static_cast<long>(gBite.baseline()), static_cast<long>(gBite.pressure()),
        gBite.active() ? 1U : 0U, static_cast<unsigned long>(ageMs));
}

void controlTick(uint64_t t) {
  // USB (re)enumeration or resume: host key state is unknown, stay DISARMED.
  if (gHid.remounted()) {
    if (gLook.state() != LOOK_DISARMED) gLook.requestDisarm();
    gGame.releaseAll();
  }

  // 1. Read inputs.
  ImuInput in;
  in.fresh = false;
  in.sampleUs = t;
  in.accelG = makeVec3(0, 0, 0);
  in.gyroDps = makeVec3(0, 0, 0);
  const ImuReader::PollResult pr = gImu.poll(&in.accelG, &in.gyroDps);
  if (pr == ImuReader::POLL_OK) {
    in.fresh = true;
    gImuNewSinceLine = true;
  } else if (pr == ImuReader::POLL_ERROR) {
    ++gImuReadErrors;
  }

  readChannels(t);

  // 3. Validate: pressure freshness.
  checkPressureFresh(gBreath, gUseBreath, gBreathSinceUs, "fault_breath_timeout", t);
  checkPressureFresh(gBite, gUseBite, gBiteSinceUs, "fault_bite_timeout", t);

  // 4. Disarm first: a sip held >= 150 ms (once per sip).
  if (gUseBreath && cfg::kSipDisarms &&
      gBreath.sipHeldUs(t) >= cfg::kSipDisarmUs) {
    const LookState s = gLook.state();
    if (!gSipFired && s != LOOK_DISARMED && s != LOOK_FAULT) {
      gLook.requestDisarm();
      emitf("#E,%lu,sip_disarm,%s\n", static_cast<unsigned long>(t / 1000U),
            lookStateName(s));
    }
    gSipFired = true;
  } else if (!gBreath.sip()) {
    gSipFired = false;
  }
  gLook.setOtherInputsNeutral(otherInputsNeutral());

  // Arm: DISARMED + D6 closed + a sip that started while DISARMED, >= 1 s.
  if (gUseBreath && cfg::kSipArms && gBreath.seeded()) {
    const bool disarmed = gLook.state() == LOOK_DISARMED && gSpanUntilUs == 0;
    if (gSipArm.update(disarmed, enableInputClosed(), gBreath.sip(),
                       gBreath.sipHeldUs(t))) {
      gLook.requestArm();
      emitf("#E,%lu,sip_arm,%s\n", static_cast<unsigned long>(t / 1000U),
            lookStateName(gLook.state()));
    }
  }

  // Validation done: feed the watchdog (never from a timer).
  gWdt.feed();

  // Caregiver switch toggle (open -> closed) acknowledges FAULT.
  const bool enable = enableInputClosed();
  if (cfg::kUseD6Enable) {
    const bool d6Closed = digitalRead(cfg::kPinEnable) == LOW;
    if (d6Closed && gEnableWasClosed == false && gLook.state() == LOOK_FAULT) {
      gLook.acknowledgeFault();
    }
    gEnableWasClosed = d6Closed;
  }

  // 2-4. Validate, safety, state.
  const LookTick k = gLook.tick(t, in, enable);
  if (k.event) {
    gLineEvent = k.event;
    emitf("#E,%lu,%s,%s\n", static_cast<unsigned long>(t / 1000U), k.event,
          lookStateName(k.state));
    if (!strcmp(k.event, "cal_done") || !strncmp(k.event, "fail", 4)) {
      printCalibration();
    }
    // Channel calibration rides on the same 3 s still window.
    if (!strcmp(k.event, "cal_start")) {
      gJoy.beginCenter();
      // After a long-sip arm the sip is still on: keep the tracked baseline
      // (it is frozen during the sip) instead of seeding from sip pressure.
      if (!gBreath.seeded() || gBreath.neutral()) gBreath.reset();
      if (!gBite.seeded() || gBite.neutral()) gBite.reset();
    } else if (!strcmp(k.event, "cal_done")) {
      const char* fail = 0;
      if (gUseJoy && !gJoy.finishCenter()) fail = "fail_joystick_center";
      else if (gUseBreath && !gBreath.seeded()) fail = "fail_breath_baseline";
      else if (gUseBite && !gBite.seeded()) fail = "fail_bite_baseline";
      if (fail) {
        gLook.requestDisarm();
        emitf("#E,%lu,%s,%s\n", static_cast<unsigned long>(t / 1000U), fail,
              lookStateName(k.state));
      }
      printChannels();
    }
  }
  if (k.releaseAll) {
    gGame.releaseAll();
    clearBenchActions();
    gAccX.clear();
    gAccY.clear();
  }

  // 5. Channels -> desired output. Velocity -> integer deltas over the real
  // loop interval; game actions -> GameInput (bench: serial act/tap).
  // Focus gate: with real HID on and the gate enabled, game output only
  // while the host reports Minecraft frontmost; leaving it releases all.
  const bool focusOk = !gHidEnabled || !gFocusGate || t < gFocusUntilUs;
  if (focusOk != gFocusLogged) {
    gFocusLogged = focusOk;
    emitf("#F,%lu,gate=%u,front=%u\n", static_cast<unsigned long>(t / 1000U),
          gFocusGate ? 1U : 0U, focusOk ? 1U : 0U);
  }
  const bool active = (k.state == LOOK_ACTIVE) && focusOk;
  const float dt = gLoopDtUs * 1e-6f;
  const bool turning = active && gTurnLeft != 0;
  if (active && !turning) {
    gAccX.add(k.velocity.vxCps, dt);
    gAccY.add(k.velocity.vyCps, dt);
  } else {
    gAccX.clear();
    gAccY.clear();
  }
  int8_t dx = gAccX.peek();
  int8_t dy = gAccY.peek();
  if (turning) {
    gTurnFrac += gTurnRateCps * dt;
    if (gTurnFrac > 127.0f) gTurnFrac = 127.0f;
    const int32_t left = gTurnLeft < 0 ? -gTurnLeft : gTurnLeft;
    int32_t step = static_cast<int32_t>(gTurnFrac);
    if (step > left) step = left;
    dx = static_cast<int8_t>(gTurnLeft < 0 ? -step : step);
    dy = 0;
  }

  expireBenchActions(t);
  GameIntent intent = gBench;
  if (gUseJoy) {
    intent.forward = intent.forward || gJoy.forward();
    intent.back = intent.back || gJoy.back();
    intent.left = intent.left || gJoy.left();
    intent.right = intent.right || gJoy.right();
  }
  if (gUseBreath) intent.use = intent.use || gBreath.active();
  if (gUseBite) intent.jump = intent.jump || gBite.active();
  intent.jump = intent.jump || gTapJump;
  intent.use = intent.use || gTapUse;
  gTapJump = gTapUse = false;
  gGame.update(t, intent, active);

  // 6. One report per loop: pending hardware release first, then GameInput's
  // plan. With HID off the plan is committed virtually for the demo page.
  int8_t sentX = 0, sentY = 0;
  if (!gHid.flushRelease()) {
    const HidPlan p = gGame.plan(dx, dy);
    bool ok = false;
    if (p.slot != HID_NONE) {
      if (!gHidEnabled) {
        ok = true;
      } else if (p.slot == HID_KEYBOARD) {
        ok = gHid.sendKeyboard(p.modifier, p.keys);
      } else {
        ok = gHid.sendMouse(p.buttons, p.dx, p.dy);
      }
    }
    if (ok) {
      gGame.committed(p, t);
      if (p.slot == HID_MOUSE) {
        sentX = p.dx;
        sentY = p.dy;
      }
      logGameOutput(t);
    }
  }
  if (turning) {
    gTurnFrac -= sentX < 0 ? -sentX : sentX;
    gTurnLeft -= sentX;
    if (gTurnLeft == 0) {
      emitf("#OK,turn done,counts=%ld\n", static_cast<long>(gTurnTotal));
    }
  } else {
    gAccX.commit(sentX);
    gAccY.commit(sentY);
  }
  gDxSum += sentX;
  gDySum += sentY;

  // 7. Log.
  writeTelemetry(t, k);
  writeSensorLine(t);
}

}  // namespace

void appSetup() {
  gResetReason = Watchdog::captureResetReason();
  gHid.begin();  // HID interface must be registered before enumeration
  Serial.begin(cfg::kSerialBaud);  // no while(!Serial): run without a host
  if (cfg::kUseD6Enable) pinMode(cfg::kPinEnable, INPUT_PULLUP);

  SignedAxis up, fwd;
  if (parseSignedAxis(cfg::kDefaultMountUp, &up) &&
      parseSignedAxis(cfg::kDefaultMountForward, &fwd)) {
    gLook.setMount(up, fwd);
  }
  LookTiming timing = gLook.timing();
  timing.imuTimeoutUs = cfg::kImuTimeoutUs;
  gLook.setTiming(timing);
  gLook.setAlpha(cfg::kAlpha);
  MouseLookParams p = defaultMouseLookParams();
  p.kxCpsPerDeg = cfg::kKxCpsPerDeg;
  p.kyCpsPerDeg = cfg::kKyCpsPerDeg;
  p.deadbandDeg = cfg::kDeadbandDeg;
  p.hysteresisDeg = cfg::kHysteresisDeg;
  p.saturationDeg = cfg::kSaturationDeg;
  p.velMode = cfg::kVelMode;
  p.engageConfirmS = cfg::kEngageConfirmMs * 1e-3f;
  gLook.setLookParams(p);

  // Joystick ADC: [Z] section 6 start configuration (12-bit, 3.6 V range).
  analogReadResolution(12);
  analogReference(AR_INTERNAL);
  analogSampleTime(40);
  analogCalibrateOffset();
  gBite.setParams(defaultBiteParams());
  gBreathHx.begin();
  gBiteHx.begin();

  if (!gImu.begin()) {
    gLook.reportFault("fault_imu_init");
  }
  if (cfg::kWatchdogStarted) gWdt.begin(cfg::kWatchdogTimeoutMs);
  const uint64_t t = nowUs();
  gBreathSinceUs = t;
  gBiteSinceUs = t;
  gNextTickUs = t;
  gLastTickUs = t;
}

void appLoop() {
  pollSerialCommands();

  const bool open = static_cast<bool>(Serial);
  if (open && !gSerialWasOpen) {
    printIdentity();
    printParams();
    printChannels();
    printHeader();
  }
  gSerialWasOpen = open;

  const uint64_t t = nowUs();
  if (t < gNextTickUs) return;
  gLoopDtUs = static_cast<uint32_t>(t - gLastTickUs);
  if (gLoopDtUs > gMaxLoopDtUs && gLastTickUs != 0) gMaxLoopDtUs = gLoopDtUs;
  gLastTickUs = t;
  gNextTickUs += cfg::kLoopPeriodUs;
  if (t > gNextTickUs + cfg::kLoopPeriodUs) {
    gNextTickUs = t + cfg::kLoopPeriodUs;  // overran: do not burst-catch-up
  }
  controlTick(t);
}
