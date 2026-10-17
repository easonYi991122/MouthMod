#include "App.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "hardware/HidOutput.h"
#include "hardware/ImuReader.h"
#include "logic/LookController.h"
#include "logic/MonotonicClock.h"
#include "logic/MouseLookMapper.h"

namespace {

HidOutput gHid;
ImuReader gImu;
LookController gLook;
MonotonicClock64 gClock;
DeltaAccumulator gAccX(cfg::kMaxPendingCounts);
DeltaAccumulator gAccY(cfg::kMaxPendingCounts);

bool gHidEnabled = cfg::kHidEnabledAtBoot;
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
}

void printParams() {
  const MouseLookParams& p = gLook.lookParams();
  emitf("#P,kx=%.2f,ky=%.2f,dead=%.2f,hyst=%.2f,sat=%.2f,hmode=%u,vmode=%u,"
        "invx=%u,invy=%u,alpha=%.4f,mount=%s%s,hid=%u,rate=%u\n",
        p.kxCpsPerDeg, p.kyCpsPerDeg, p.deadbandDeg, p.hysteresisDeg,
        p.saturationDeg, p.hystMode, p.velMode, p.invertX ? 1U : 0U,
        p.invertY ? 1U : 0U, gLook.alpha(), signedAxisName(gLook.mount().up()),
        signedAxisName(gLook.mount().forward()), gHidEnabled ? 1U : 0U,
        gTelemetryHz);
}

void printIdentity() {
  emitf("#I,fw=%s,ver=%s,bsp=%s,imu_ok=%u,whoami=0x%02X,odr=%u,acc_g=%u,"
        "gyro_dps=%u,d6_enable=%u,wdt=%u,max_loop_dt_us=%lu\n",
        cfg::kFirmwareName, cfg::kFirmwareVersion, ARDUINO_BSP_VERSION,
        gImu.ok() ? 1U : 0U, gImu.whoAmI(), cfg::kImuOdrHz, cfg::kAccelRangeG,
        cfg::kGyroRangeDps, cfg::kUseD6Enable ? 1U : 0U,
        cfg::kWatchdogStarted ? 1U : 0U,
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

bool setParam(const char* name, const char* value) {
  char* end = 0;
  const float v = strtof(value, &end);
  if (end == value) return false;
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
    if (!gHidEnabled) gHid.requestReleaseAll();
    emitf("#OK,hid=%u\n", gHidEnabled ? 1U : 0U);
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

void controlTick(uint64_t t) {
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
  }
  if (k.releaseAll) {
    gHid.requestReleaseAll();
    gAccX.clear();
    gAccY.clear();
  }

  // 5-6. Output: velocity -> integer deltas over the real loop interval.
  const bool active = (k.state == LOOK_ACTIVE);
  if (active) {
    const float dt = gLoopDtUs * 1e-6f;
    gAccX.add(k.velocity.vxCps, dt);
    gAccY.add(k.velocity.vyCps, dt);
  } else {
    gAccX.clear();
    gAccY.clear();
  }
  const int8_t dx = gAccX.peek();
  const int8_t dy = gAccY.peek();
  int8_t sentX = 0, sentY = 0;
  if (gHidEnabled) {
    gHid.service(active, dx, dy, &sentX, &sentY);
  } else {
    gHid.service(false, 0, 0, &sentX, &sentY);  // still flush releases
    if (active) {
      sentX = dx;  // virtual output for the demo page
      sentY = dy;
    }
  }
  gAccX.commit(sentX);
  gAccY.commit(sentY);
  gDxSum += sentX;
  gDySum += sentY;

  // 7. Log.
  writeTelemetry(t, k);
}

}  // namespace

void appSetup() {
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
  gLook.setLookParams(p);

  if (!gImu.begin()) {
    gLook.reportFault("fault_imu_init");
  }
  const uint64_t t = nowUs();
  gNextTickUs = t;
  gLastTickUs = t;
}

void appLoop() {
  pollSerialCommands();

  const bool open = static_cast<bool>(Serial);
  if (open && !gSerialWasOpen) {
    printIdentity();
    printParams();
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
