#include "LookController.h"

#include <math.h>

const char* lookStateName(LookState s) {
  switch (s) {
    case LOOK_DISARMED:
      return "DISARMED";
    case LOOK_CALIBRATING:
      return "CALIBRATING";
    case LOOK_WAIT_NEUTRAL:
      return "WAIT_NEUTRAL";
    case LOOK_ACTIVE:
      return "ACTIVE";
    case LOOK_FAULT:
      return "FAULT";
  }
  return "?";
}

LookController::LookController()
    : state_(LOOK_DISARMED),
      armRequested_(false),
      disarmRequested_(false),
      ackRequested_(false),
      mountChanged_(false),
      pendingFault_(0),
      hasSample_(false),
      lastSampleUs_(0U),
      bootUs_(0U),
      bootSet_(false),
      neutralSinceUs_(0U),
      neutralTiming_(false),
      calibrated_(false),
      bias_(makeVec3(0.0f, 0.0f, 0.0f)),
      roll0_(0.0f),
      pitch0_(0.0f),
      lastAccelBody_(makeVec3(0.0f, 0.0f, 0.0f)),
      lastGyroBody_(makeVec3(0.0f, 0.0f, 0.0f)) {
  timing_.imuTimeoutUs = 100000U;
  timing_.neutralHoldUs = 300000U;
}

bool LookController::setMount(SignedAxis up, SignedAxis forward) {
  if (!mount_.configure(up, forward)) {
    return false;
  }
  mountChanged_ = true;
  return true;
}

void LookController::reportFault(const char* reason) {
  pendingFault_ = reason ? reason : "fault";
}

void LookController::enter(LookState s, const char* event, LookTick* out) {
  const bool wasActive = (state_ == LOOK_ACTIVE);
  state_ = s;
  if (event) out->event = event;
  if (s != LOOK_ACTIVE) {
    mapper_.reset();
    if (wasActive || s == LOOK_FAULT || s == LOOK_CALIBRATING) {
      out->releaseAll = true;
    }
  }
  if (s == LOOK_CALIBRATING) {
    calibration_.cancel();
  }
  neutralTiming_ = false;
}

LookTick LookController::tick(uint64_t nowUs, const ImuInput& imu,
                              bool enableInput) {
  LookTick out;
  out.state = state_;
  out.releaseAll = false;
  out.event = 0;
  out.velocity.vxCps = 0.0f;
  out.velocity.vyCps = 0.0f;
  out.velocity.engagedX = false;
  out.velocity.engagedY = false;

  if (!bootSet_) {
    bootSet_ = true;
    bootUs_ = nowUs;
  }

  // 1. Sensor update (always, so telemetry shows the pose while disarmed).
  bool fresh = false;
  if (imu.fresh && vecFinite(imu.accelG) && vecFinite(imu.gyroDps)) {
    const Vec3 a = mount_.toBody(imu.accelG);
    const Vec3 g = vecSub(mount_.toBody(imu.gyroDps), bias_);
    const float dt = hasSample_ && imu.sampleUs > lastSampleUs_
                         ? static_cast<float>(imu.sampleUs - lastSampleUs_) * 1e-6f
                         : 0.0f;
    if (dt > 0.0f) {
      filter_.update(a, g, dt);
    } else {
      filter_.seed(a);
    }
    lastAccelBody_ = a;
    lastGyroBody_ = g;
    hasSample_ = true;
    lastSampleUs_ = imu.sampleUs;
    fresh = true;
  }

  // 2. Fault checks (latched). Startup grace equals the timeout.
  const uint64_t ref = hasSample_ ? lastSampleUs_ : bootUs_;
  const bool timedOut = nowUs > ref && nowUs - ref > timing_.imuTimeoutUs;
  if (state_ != LOOK_FAULT) {
    if (pendingFault_) {
      enter(LOOK_FAULT, pendingFault_, &out);
    } else if (timedOut) {
      enter(LOOK_FAULT, "fault_imu_timeout", &out);
    }
  }
  pendingFault_ = 0;

  if (state_ == LOOK_FAULT) {
    if (ackRequested_ && !timedOut) {
      enter(LOOK_DISARMED, "fault_ack", &out);
    }
    armRequested_ = false;
    disarmRequested_ = false;
  }
  ackRequested_ = false;

  // 3. Disarm sources take priority over everything except FAULT.
  if (state_ != LOOK_FAULT && state_ != LOOK_DISARMED) {
    if (!enableInput) {
      enter(LOOK_DISARMED, "disarm_enable_open", &out);
    } else if (disarmRequested_) {
      enter(LOOK_DISARMED, "disarm_cmd", &out);
    } else if (mountChanged_) {
      enter(LOOK_DISARMED, "disarm_mount_change", &out);
    }
  }
  if (mountChanged_) {
    calibrated_ = false;
    bias_ = makeVec3(0.0f, 0.0f, 0.0f);
    filter_.reset();
    mountChanged_ = false;
  }
  disarmRequested_ = false;

  // 4. Arm.
  if (armRequested_) {
    armRequested_ = false;
    if (state_ == LOOK_DISARMED && enableInput) {
      enter(LOOK_CALIBRATING, "cal_start", &out);
      calibration_.start(nowUs);
    }
  }

  // 5. Calibration and neutral gate.
  if (state_ == LOOK_CALIBRATING && fresh) {
    // Calibration statistics use the raw (unbiased) body gyro.
    const Vec3 rawGyro = mount_.toBody(imu.gyroDps);
    const CalibrationStatus cs =
        calibration_.addSample(imu.sampleUs, lastAccelBody_, rawGyro);
    if (cs == CAL_DONE) {
      const CalibrationResult& r = calibration_.result();
      bias_ = r.gyroBiasDps;
      roll0_ = r.roll0Deg;
      pitch0_ = r.pitch0Deg;
      calibrated_ = true;
      filter_.seed(r.meanAccelG);
      enter(LOOK_WAIT_NEUTRAL, "cal_done", &out);
    } else if (cs != CAL_COLLECTING) {
      enter(LOOK_DISARMED, calibrationStatusName(cs), &out);
    }
  }

  out.rollDeg = filter_.rollDeg();
  out.pitchDeg = filter_.pitchDeg();
  out.rollErrDeg = calibrated_ ? out.rollDeg - roll0_ : 0.0f;
  out.pitchErrDeg = calibrated_ ? out.pitchDeg - pitch0_ : 0.0f;

  if (state_ == LOOK_WAIT_NEUTRAL) {
    const float edge = LookAxis::engageEdge(mapper_.params());
    const bool neutral =
        fabsf(out.rollErrDeg) < edge && fabsf(out.pitchErrDeg) < edge;
    if (!neutral) {
      neutralTiming_ = false;
    } else if (!neutralTiming_) {
      neutralTiming_ = true;
      neutralSinceUs_ = nowUs;
    } else if (nowUs - neutralSinceUs_ >= timing_.neutralHoldUs) {
      mapper_.reset();
      enter(LOOK_ACTIVE, "active", &out);
    }
  }

  // 6. Output.
  if (state_ == LOOK_ACTIVE) {
    out.velocity = mapper_.update(out.rollErrDeg, out.pitchErrDeg);
  }
  out.state = state_;
  return out;
}
