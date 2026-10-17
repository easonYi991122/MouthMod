// MouthMod ImuLookMini: head tilt -> USB mouse look, single-file version.
// Board: Seeed nRF52 Boards (non-mbed) > Seeed XIAO nRF52840 Sense.
// Libraries: Seeed Arduino LSM6DS3 2.0.7; Adafruit TinyUSB (bundled 1.7.0).
//
// Flow: DISARMED --arm--> CALIBRATING (hold still 3 s) --> WAIT_NEUTRAL
//       (head back to neutral 300 ms) --> ACTIVE (tilt -> mouse velocity).
//       IMU silent > 100 ms -> FAULT (latched until "ack").
// Mapping (V5 spec): vx = kx*(roll-roll0), vy = ky*(pitch-pitch0) in counts/s,
//       dead-band 3 deg, hysteresis 2 deg (release below 1 deg), saturation 20 deg.
// Serial commands (newline-terminated): arm | disarm | x | ack | hid 0|1 |
//       set kx|ky|dead|hyst|sat|alpha|rate|invx|invy <value> | mount <up> <fwd> | get
// Telemetry "D," lines match tools/look_demo, so the test page still works.
#include <Adafruit_TinyUSB.h>
#include <LSM6DS3.h>
#include <Wire.h>

// ------------------------------------------------------------------ config
const uint32_t LOOP_US = 10000;         // 100 Hz control loop
const uint32_t IMU_TIMEOUT_US = 100000; // no fresh sample -> FAULT
const uint32_t CAL_US = 3000000;        // still window
const uint32_t NEUTRAL_US = 300000;     // neutral hold before ACTIVE
const float CAL_MAX_GYRO_STD = 3.0f;    // dps; 1.0 was too strict when worn
const uint32_t CAL_MIN_SAMPLES = 250;
const float GYRO_LSB = 8.75e-3f;        // dps/LSB at +/-245 dps
const float ACC_LSB = 0.061e-3f;        // g/LSB at +/-2 g
const bool DISARM_ON_SERIAL_CLOSE = true;

float kx = 40, ky = 30, dead = 3, hyst = 2, sat = 20, alpha = 0.98f;
bool invX = false, invY = false, hidOn = false;
uint16_t telemHz = 50;
int upAx = 2, upSg = 1, fwAx = 0, fwSg = 1;  // mount: +Z up, +X forward

// ------------------------------------------------------------------ USB HID
enum { RID_KEYBOARD = 1, RID_MOUSE = 2 };
uint8_t const hidDesc[] = {TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
                           TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(RID_MOUSE))};
Adafruit_USBD_HID hid;
bool kbReleasePending = true, mouseReleasePending = true, wasMounted = false;

// ------------------------------------------------------------------ state
enum State { DISARMED, CALIBRATING, WAIT_NEUTRAL, ACTIVE, FAULT };
const char* const STATE_NAME[] = {"DISARMED", "CALIBRATING", "WAIT_NEUTRAL", "ACTIVE", "FAULT"};
State state = DISARMED;

LSM6DS3 imu(I2C_MODE, 0x6A);
bool imuOk = false;
float acc[3], gyr[3];              // body frame (x fwd, y left, z up); gyro bias removed
float bias[3] = {0, 0, 0};
float roll = 0, pitch = 0, roll0 = 0, pitch0 = 0;
bool filterInit = false, calibrated = false;
uint32_t lastSampleUs = 0, stateSinceUs = 0, neutralSinceUs = 0;
bool neutralTiming = false, engX = false, engY = false;

uint32_t calN = 0;                  // calibration sums
double calG[3], calG2[3], calA[3];

float accX = 0, accY = 0;           // fractional HID counts
int dxSum = 0, dySum = 0;
uint32_t lastTickUs = 0, loopDtUs = 0, nextTelemUs = 0, dropped = 0;
bool imuNewSinceLine = false;
const char* lineEvent = "";
char cmd[64];
uint8_t cmdLen = 0;

// ------------------------------------------------------------------ helpers
void emit(const char* s) {
  int n = strlen(s);
  if (!Serial) return;
  if (Serial.availableForWrite() < n) { dropped++; return; }  // never block
  Serial.write(s, n);
}

void emitf(const char* fmt, ...) {
  char b[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  emit(b);
}

void enter(State s, const char* ev) {
  if (state == ACTIVE || s == FAULT || s == CALIBRATING) {
    kbReleasePending = mouseReleasePending = true;  // release everything
  }
  state = s;
  stateSinceUs = micros();
  neutralTiming = false;
  engX = engY = false;
  accX = accY = 0;
  lineEvent = ev;
  emitf("#E,%lu,%s,%s\n", millis(), ev, STATE_NAME[s]);
}

bool readRegs(uint8_t reg, uint8_t* out, uint8_t len) {
  Wire1.beginTransmission(0x6A);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((uint8_t)0x6A, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) out[i] = Wire1.read();
  return true;
}

// Sensor axis i -> body axis via the mount; left = up x forward.
void toBody(const float s[3], float b[3]) {
  int lfAx = 3 - upAx - fwAx;
  int lfSg = upSg * fwSg * (((fwAx - upAx + 3) % 3 == 1) ? 1 : -1);
  b[0] = fwSg * s[fwAx];
  b[1] = lfSg * s[lfAx];
  b[2] = upSg * s[upAx];
}

// Returns true when a fresh sample was read into acc/gyr (raw gyro in rawG).
bool pollImu(float rawG[3]) {
  uint8_t st, d[12];
  if (!imuOk || !readRegs(0x1E, &st, 1) || (st & 0x03) != 0x03) return false;
  if (!readRegs(0x22, d, 12)) return false;  // gyro xyz, accel xyz
  float g[3], a[3];
  for (int i = 0; i < 3; i++) {
    g[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8)) * GYRO_LSB;
    a[i] = (int16_t)(d[6 + 2 * i] | (d[7 + 2 * i] << 8)) * ACC_LSB;
  }
  toBody(g, rawG);
  toBody(a, acc);
  for (int i = 0; i < 3; i++) gyr[i] = rawG[i] - bias[i];
  return true;
}

float accRoll(const float a[3]) { return atan2f(a[1], a[2]) * RAD_TO_DEG; }
float accPitch(const float a[3]) { return atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * RAD_TO_DEG; }

// Complementary filter; alpha is defined at 10 ms and scaled to the real dt.
void updateFilter(float dt) {
  float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (!filterInit || dt <= 0 || dt > 0.1f) {
    roll = accRoll(acc);
    pitch = accPitch(acc);
    filterInit = true;
    return;
  }
  float r = roll * DEG_TO_RAD, p = pitch * DEG_TO_RAD;
  float tp = fabsf(cosf(p)) > 1e-3f ? tanf(p) : 0;
  roll += (gyr[0] + (sinf(r) * gyr[1] + cosf(r) * gyr[2]) * tp) * dt;  // no yaw
  pitch += (cosf(r) * gyr[1] - sinf(r) * gyr[2]) * dt;
  if (n > 0.7f && n < 1.3f) {  // ignore gravity during strong linear accel
    float tau = alpha * 0.01f / (1 - alpha);
    float w = dt / (tau + dt);
    roll += w * (accRoll(acc) - roll);
    pitch += w * (accPitch(acc) - pitch);
  }
}

// Dead-band + hysteresis + saturation for one axis -> counts/s.
float lookAxis(float e, float k, bool& eng) {
  float m = fabsf(e);
  if (eng && m < max(0.0f, dead - hyst)) eng = false;
  else if (!eng && m >= dead) eng = true;
  if (!eng) return 0;
  return (e < 0 ? -k : k) * min(m, sat);
}

// One HID report per loop; pending releases go first.
void hidService(int8_t dx, int8_t dy, int8_t& sx, int8_t& sy) {
  sx = sy = 0;
  bool mounted = TinyUSBDevice.mounted() && !TinyUSBDevice.suspended();
  if (mounted && !wasMounted) kbReleasePending = mouseReleasePending = true;
  wasMounted = mounted;
  if (!mounted || !hid.ready()) return;
  if (kbReleasePending) { if (hid.keyboardRelease(RID_KEYBOARD)) kbReleasePending = false; return; }
  if (mouseReleasePending) { if (hid.mouseReport(RID_MOUSE, 0, 0, 0, 0, 0)) mouseReleasePending = false; return; }
  if ((dx || dy) && hid.mouseReport(RID_MOUSE, 0, dx, dy, 0, 0)) { sx = dx; sy = dy; }
}

bool parseAxis(const char* t, int& ax, int& sg) {
  sg = 1;
  if (*t == '+') t++;
  else if (*t == '-') { sg = -1; t++; }
  char c = toupper(*t);
  if (c < 'X' || c > 'Z' || t[1]) return false;
  ax = c - 'X';
  return true;
}

void printParams() {
  const char* ax = "XYZ";
  emitf("#P,kx=%.2f,ky=%.2f,dead=%.2f,hyst=%.2f,sat=%.2f,alpha=%.4f,invx=%d,invy=%d,"
        "mount=%c%c%c%c,hid=%d,rate=%u\n", kx, ky, dead, hyst, sat, alpha, invX, invY,
        upSg > 0 ? '+' : '-', ax[upAx], fwSg > 0 ? '+' : '-', ax[fwAx], hidOn, telemHz);
}

void handleCommand(char* line) {
  char* a = strtok(line, " \r");
  char* b = strtok(NULL, " \r");
  char* c = strtok(NULL, " \r");
  if (!a) return;
  if (!strcmp(a, "arm")) {
    if (state == DISARMED) {
      calN = 0;
      for (int i = 0; i < 3; i++) calG[i] = calG2[i] = calA[i] = 0;
      enter(CALIBRATING, "cal_start");
    }
  } else if (!strcmp(a, "disarm") || !strcmp(a, "x")) {
    if (state != DISARMED && state != FAULT) enter(DISARMED, "disarm_cmd");
  } else if (!strcmp(a, "ack")) {
    if (state == FAULT && micros() - lastSampleUs < IMU_TIMEOUT_US) enter(DISARMED, "fault_ack");
  } else if (!strcmp(a, "hid") && b) {
    hidOn = atoi(b) != 0;
    kbReleasePending = mouseReleasePending = true;
    emitf("#OK,hid=%d\n", hidOn);
  } else if (!strcmp(a, "set") && b && c) {
    float v = atof(c);
    if (!strcmp(b, "kx")) kx = v;
    else if (!strcmp(b, "ky")) ky = v;
    else if (!strcmp(b, "dead")) dead = v;
    else if (!strcmp(b, "hyst")) hyst = v;
    else if (!strcmp(b, "sat")) sat = v;
    else if (!strcmp(b, "alpha") && v > 0 && v < 1) alpha = v;
    else if (!strcmp(b, "rate")) telemHz = constrain((int)v, 0, 100);
    else if (!strcmp(b, "invx")) invX = v != 0;
    else if (!strcmp(b, "invy")) invY = v != 0;
    else return;  // hmode/vmode from the test page: not in the mini build
    printParams();
  } else if (!strcmp(a, "mount") && b && c) {
    int ua, us, fa, fs;
    if (parseAxis(b, ua, us) && parseAxis(c, fa, fs) && ua != fa) {
      upAx = ua; upSg = us; fwAx = fa; fwSg = fs;
      calibrated = false; filterInit = false;
      for (int i = 0; i < 3; i++) bias[i] = 0;
      if (state != DISARMED && state != FAULT) enter(DISARMED, "disarm_mount_change");
      printParams();
    }
  } else if (!strcmp(a, "get")) {
    emitf("#I,fw=MouthMod-ImuLookMini,ver=0.1.0,bsp=%s,imu_ok=%d\n", ARDUINO_BSP_VERSION, imuOk);
    printParams();
  }
}

// ------------------------------------------------------------------ main
void setup() {
  hid.setPollInterval(2);
  hid.setReportDescriptor(hidDesc, sizeof hidDesc);
  hid.begin();
  Serial.begin(115200);  // never 1200 (bootloader touch); no while(!Serial)

  imu.settings.gyroRange = 245;
  imu.settings.gyroSampleRate = 104;
  imu.settings.accelRange = 2;
  imu.settings.accelSampleRate = 104;
  imu.settings.accelBandWidth = 100;
  imu.settings.tempEnabled = 0;
  uint8_t who = 0;
  imuOk = imu.begin() == IMU_SUCCESS;  // powers the IMU, starts Wire1
  Wire1.setClock(400000);
  imuOk = imuOk && readRegs(0x0F, &who, 1) && who == 0x6A;
  lastSampleUs = lastTickUs = micros();
  if (!imuOk) enter(FAULT, "fault_imu_init");
}

void loop() {
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch == '\n') { cmd[cmdLen] = 0; handleCommand(cmd); cmdLen = 0; }
    else if (cmdLen < sizeof cmd - 1) cmd[cmdLen++] = ch;
  }
  uint32_t now = micros();
  if (now - lastTickUs < LOOP_US) return;
  loopDtUs = now - lastTickUs;
  lastTickUs = now;

  // 1. Read the IMU.
  float rawG[3];
  if (pollImu(rawG)) {
    updateFilter((now - lastSampleUs) * 1e-6f);
    lastSampleUs = now;
    imuNewSinceLine = true;
    if (state == CALIBRATING) {
      calN++;
      for (int i = 0; i < 3; i++) {
        calG[i] += rawG[i]; calG2[i] += (double)rawG[i] * rawG[i]; calA[i] += acc[i];
      }
    }
  }

  // 2. Safety: IMU timeout latches FAULT; closing the serial port disarms.
  if (state != FAULT && now - lastSampleUs > IMU_TIMEOUT_US) enter(FAULT, "fault_imu_timeout");
  if (DISARM_ON_SERIAL_CLOSE && !Serial && state != DISARMED && state != FAULT) {
    enter(DISARMED, "disarm_serial_closed");
  }

  // 3. Calibration and neutral gate.
  if (state == CALIBRATING && now - stateSinceUs >= CAL_US) {
    float stdMax = 0, a[3];
    for (int i = 0; i < 3; i++) {
      double m = calN ? calG[i] / calN : 0;
      stdMax = max(stdMax, (float)sqrt(max(0.0, calN ? calG2[i] / calN - m * m : 0)));
      bias[i] = m;
      a[i] = calN ? calA[i] / calN : 0;
    }
    float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const char* fail = calN < CAL_MIN_SAMPLES ? "fail_samples"
                     : stdMax > CAL_MAX_GYRO_STD ? "fail_motion"
                     : (an < 0.9f || an > 1.1f) ? "fail_accel" : NULL;
    emitf("#C,status=%s,samples=%lu,bias=%.3f/%.3f/%.3f,gyro_std_max=%.3f,limit=%.1f\n",
          fail ? fail : "done", calN, bias[0], bias[1], bias[2], stdMax, CAL_MAX_GYRO_STD);
    if (fail) {
      for (int i = 0; i < 3; i++) bias[i] = 0;
      enter(DISARMED, fail);
    } else {
      roll = roll0 = accRoll(a);
      pitch = pitch0 = accPitch(a);
      calibrated = true;
      enter(WAIT_NEUTRAL, "cal_done");
    }
  }
  float re = calibrated ? roll - roll0 : 0, pe = calibrated ? pitch - pitch0 : 0;
  if (state == WAIT_NEUTRAL) {
    if (fabsf(re) >= dead || fabsf(pe) >= dead) neutralTiming = false;
    else if (!neutralTiming) { neutralTiming = true; neutralSinceUs = now; }
    else if (now - neutralSinceUs >= NEUTRAL_US) enter(ACTIVE, "active");
  }

  // 4. Output: velocity -> integer counts over the real loop interval.
  float vx = 0, vy = 0;
  if (state == ACTIVE) {
    vx = lookAxis(re, kx, engX) * (invX ? -1 : 1);  // + = turn right
    vy = lookAxis(pe, ky, engY) * (invY ? -1 : 1);  // + = look down
    float dt = loopDtUs * 1e-6f;
    accX = vx ? constrain(accX + vx * dt, -64.0f, 64.0f) : 0;  // cap stale motion
    accY = vy ? constrain(accY + vy * dt, -64.0f, 64.0f) : 0;
  }
  int8_t dx = (int8_t)constrain(truncf(accX), -127, 127);
  int8_t dy = (int8_t)constrain(truncf(accY), -127, 127);
  int8_t sx, sy;
  hidService(hidOn && state == ACTIVE ? dx : 0, hidOn && state == ACTIVE ? dy : 0, sx, sy);
  if (!hidOn && state == ACTIVE) { sx = dx; sy = dy; }  // virtual output for the page
  accX -= sx; accY -= sy;
  dxSum += sx; dySum += sy;

  // 5. Telemetry (same columns as the full ImuBench build).
  if (telemHz && (int32_t)(now - nextTelemUs) >= 0) {
    nextTelemUs += 1000000UL / telemHz;
    if ((int32_t)(now - nextTelemUs) > 0) nextTelemUs = now + 1000000UL / telemHz;
    char b[300];
    snprintf(b, sizeof b,
             "D,%lu,%lu,%s,%d,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
             "%.1f,%.1f,%d,%d,%d,%d%d,%lu,%s\n",
             millis(), loopDtUs, STATE_NAME[state], imuNewSinceLine, (now - lastSampleUs) / 1000,
             acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], roll, pitch, re, pe, vx, vy,
             dxSum, dySum, hidOn, engX, engY, dropped, lineEvent);
    emit(b);
    dxSum = dySum = 0;
    imuNewSinceLine = false;
    lineEvent = "";
  }
}
