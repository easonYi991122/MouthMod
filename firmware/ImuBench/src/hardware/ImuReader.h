// On-board LSM6DS3TR-C (XIAO nRF52840 Sense, internal Wire1, 0x6A), polled.
// The Seeed library is used only for power-up and configuration; samples are
// read directly from Wire1 so every I2C transfer is checked.
#ifndef MOUTHMOD_IMU_READER_H
#define MOUTHMOD_IMU_READER_H

#include <stdint.h>

#include "../logic/Vec3.h"

class ImuReader {
 public:
  enum PollResult { POLL_NO_DATA = 0, POLL_OK, POLL_ERROR };

  ImuReader();

  // Powers the sensor and applies ODR/range from Config.h. Call once in setup.
  bool begin();
  bool ok() const { return ok_; }
  uint8_t whoAmI() const { return whoAmI_; }

  // Non-blocking: returns POLL_NO_DATA when STATUS_REG has no new sample.
  // Units: accel in g, gyro in dps, sensor frame.
  PollResult poll(Vec3* accelG, Vec3* gyroDps);

  uint32_t errorCount() const { return errors_; }

 private:
  bool readRegs(uint8_t reg, uint8_t* out, uint8_t len);

  bool ok_;
  uint8_t whoAmI_;
  uint32_t errors_;
};

#endif
