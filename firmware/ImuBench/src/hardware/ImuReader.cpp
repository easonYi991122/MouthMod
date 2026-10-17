#include "ImuReader.h"

#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>

#include "../Config.h"

#ifndef TARGET_SEEED_XIAO_NRF52840_SENSE
#error "Select board: Seeed nRF52 Boards > Seeed XIAO nRF52840 Sense (non-mbed)"
#endif

namespace {
// LSM6DS3TR-C registers (datasheet / Seeed library definitions).
const uint8_t kRegWhoAmI = 0x0F;
const uint8_t kRegStatus = 0x1E;
const uint8_t kRegOutGyroXL = 0x22;  // 0x22..0x27 gyro, 0x28..0x2D accel
const uint8_t kWhoAmIExpected = 0x6A;
const uint8_t kStatusXLDA = 0x01;
const uint8_t kStatusGDA = 0x02;

LSM6DS3 gImu(I2C_MODE, cfg::kImuI2cAddress);

float gyroScaleDps() {
  switch (cfg::kGyroRangeDps) {
    case 125: return 4.375e-3f;
    case 245: return 8.75e-3f;
    case 500: return 17.5e-3f;
    case 1000: return 35.0e-3f;
    default: return 70.0e-3f;  // 2000
  }
}

float accelScaleG() {
  switch (cfg::kAccelRangeG) {
    case 2: return 0.061e-3f;
    case 4: return 0.122e-3f;
    case 8: return 0.244e-3f;
    default: return 0.488e-3f;  // 16
  }
}

int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

ImuReader::ImuReader() : ok_(false), whoAmI_(0), errors_(0) {}

bool ImuReader::begin() {
  gImu.settings.gyroEnabled = 1;
  gImu.settings.gyroRange = cfg::kGyroRangeDps;
  gImu.settings.gyroSampleRate = cfg::kImuOdrHz;
  gImu.settings.gyroFifoEnabled = 0;
  gImu.settings.accelEnabled = 1;
  gImu.settings.accelRange = cfg::kAccelRangeG;
  gImu.settings.accelSampleRate = cfg::kImuOdrHz;
  gImu.settings.accelBandWidth = 100;
  gImu.settings.accelFifoEnabled = 0;
  gImu.settings.tempEnabled = 0;

  const status_t st = gImu.begin();  // powers the sensor, starts Wire1
  Wire1.setClock(400000);            // 14-byte poll ~0.4 ms instead of ~1.5 ms
  uint8_t who = 0;
  ok_ = (st == IMU_SUCCESS) && readRegs(kRegWhoAmI, &who, 1) &&
        who == kWhoAmIExpected;
  whoAmI_ = who;
  return ok_;
}

bool ImuReader::readRegs(uint8_t reg, uint8_t* out, uint8_t len) {
  Wire1.beginTransmission(cfg::kImuI2cAddress);
  Wire1.write(reg);  // auto-increment (IF_INC) is on by default
  if (Wire1.endTransmission(false) != 0) {
    ++errors_;
    return false;
  }
  const uint8_t got = Wire1.requestFrom(cfg::kImuI2cAddress, len);
  if (got != len) {
    while (Wire1.available()) Wire1.read();
    ++errors_;
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) out[i] = static_cast<uint8_t>(Wire1.read());
  return true;
}

ImuReader::PollResult ImuReader::poll(Vec3* accelG, Vec3* gyroDps) {
  if (!ok_) {
    return POLL_ERROR;
  }
  uint8_t status = 0;
  if (!readRegs(kRegStatus, &status, 1)) {
    return POLL_ERROR;
  }
  // Wait for both: accel and gyro share the same ODR here.
  if ((status & (kStatusXLDA | kStatusGDA)) != (kStatusXLDA | kStatusGDA)) {
    return POLL_NO_DATA;
  }
  uint8_t b[12];
  if (!readRegs(kRegOutGyroXL, b, sizeof(b))) {
    return POLL_ERROR;
  }
  const float gs = gyroScaleDps();
  const float as = accelScaleG();
  *gyroDps = makeVec3(le16(b + 0) * gs, le16(b + 2) * gs, le16(b + 4) * gs);
  *accelG = makeVec3(le16(b + 6) * as, le16(b + 8) * as, le16(b + 10) * as);
  return POLL_OK;
}
