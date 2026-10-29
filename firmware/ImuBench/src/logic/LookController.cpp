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
    : fusion_(defaultFusionParams()),
      sourceRates_(makeVec3(0, 0, 0)),
      state_(LOOK_DISARMED),
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
      otherNeutral_(true),
      lastMapUs_(0U),
      calibrated_(false),
      bias_(makeVec3(0.0f, 0.0f, 0.0f)),
      roll0_(0.0f),
      pitch0_(0.0f),
      yaw_(0.0f),
      lastAccelBody_(makeVec3(0.0f, 0.0f, 0.0f)),
      lastGyroBody_(makeVec3(0.0f, 0.0f, 0.0f)) {
  timing_.imuTimeoutUs = 100000U;
  timing_.neutralHoldUs = 300000U;
}

bool LookController::setMount(SignedAxis up, SignedAxis forward) {
  if (!mount_.configure(up, forward)) {
    return false;
  }
  fine_.clear();
  mountChanged_ = true;
  return true;
}

bool LookController::setLookParams(const MouseLookParams& p) {
  const uint8_t oldH = mapper_.params().hsrc, oldV = mapper_.params().vsrc;
  mapper_.setParams(p);
  const MouseLookParams& next = mapper_.params();
  if (fine_.on() && (oldH != next.hsrc || oldV != next.vsrc) &&
      !fine_.validSources(calibrated_, next.hsrc, next.vsrc)) {
    fine_.off();
    return true;
  }
  return false;
}

void LookController::setFusionParams(const FusionParams& p) {
  if (fusion_.mode != p.mode) filter_.resetDerivative();
  fusion_ = p;
  sanitizeFusionParams(&fusion_);
}

const char* LookController::startFineCapture(char pose, uint64_t nowUs) {
  if (state_ == LOOK_ACTIVE) return "fcal_active";
  if (state_ == LOOK_FAULT) return "fcal_fault";
  return fine_.start(pose, nowUs);
}

const char* LookController::enableFineCalibration() {
  MouseLookParams p = mapper_.params();
  const char* error = fine_.enable(calibrated_, &p.hsrc, &p.vsrc);
  if (!error) mapper_.setParams(p);
  return error;
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
  if ((s == LOOK_ACTIVE || s == LOOK_FAULT) && fine_.capturing()) {
    out->finePose = fine_.capturePose();
    out->fineEvent = s == LOOK_ACTIVE ? "fcal_active" : "fcal_fault";
    fine_.cancel();
  }
  if (s == LOOK_CALIBRATING || s == LOOK_ACTIVE) yaw_ = 0.0f;
  if (s == LOOK_CALIBRATING) {
    calibration_.cancel();
  }
  neutralTiming_ = false;
}

void LookController::poseErrors(LookTick* out) const {
  out->rollDeg = filter_.rollDeg();
  out->pitchDeg = filter_.pitchDeg();
  out->rollErrDeg = calibrated_ ? out->rollDeg - roll0_ : 0.0f;
  out->pitchErrDeg = calibrated_ ? out->pitchDeg - pitch0_ : 0.0f;
  const float errors[3] = {out->rollErrDeg, out->pitchErrDeg,
                          calibrated_ ? yaw_ : 0.0f};
  out->lookXErrDeg = errors[mapper_.params().hsrc];
  out->lookYErrDeg = errors[mapper_.params().vsrc];
  if (fine_.on()) {
    const MouseLookParams& p = mapper_.params();
    const float angles[3] = {out->rollDeg, out->pitchDeg, yaw_};
    out->lookXErrDeg = fine_.map(0, p.hsrc, angles[p.hsrc], p.saturationDeg);
    out->lookYErrDeg = fine_.map(1, p.vsrc, angles[p.vsrc], p.saturationDeg);
  }
}

LookTick LookController::tick(uint64_t nowUs, const ImuInput& imu,
                              bool enableInput) {
  LookTick out;
  out.state = state_;
  out.releaseAll = false;
  out.event = 0;
  out.fineEvent = 0;
  out.finePose = '-';
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
  if (imu.fresh && (!hasSample_ || imu.sampleUs > lastSampleUs_) &&
      vecFinite(imu.accelG) && vecFinite(imu.gyroDps)) {
    const Vec3 a = mount_.toBody(imu.accelG);
    const Vec3 g = vecSub(mount_.toBody(imu.gyroDps), bias_);
    const float dt = hasSample_ && imu.sampleUs > lastSampleUs_
                         ? static_cast<float>(imu.sampleUs - lastSampleUs_) * 1e-6f
                         : 0.0f;
    if (dt > 0.0f) {
      sourceRates_ = filter_.eulerRates(g);
      if (fusion_.mode == 1) {
        filter_.updateEnhanced(a, g, dt, fusion_.armX, fusion_.armZ,
                               fusion_.accelRejectG);
      } else {
        sourceRates_.z = -g.z;  // v0.7.0 yaw source remains body -Z
        filter_.update(a, g, dt);
      }
      // Body axes are forward/left/up: rightward head turn is negative Z.
      // Match the filter's gap guard; never integrate across a stale sample.
      if (dt <= ComplementaryFilter::kMaxDtS) {
        const float rate = sourceRates_.z;
        const float tau = mapper_.params().yawTauS;
        if (tau > 0.0f) {
          const float leak = -expm1f(-dt / tau);
          yaw_ += (rate * tau - yaw_) * leak;
        } else {
          yaw_ += rate * dt;
        }
      }
    } else {
      if (fusion_.mode == 1) {
        filter_.updateEnhanced(a, g, ComplementaryFilter::kNominalDtS,
                               fusion_.armX, fusion_.armZ, fusion_.accelRejectG);
      } else {
        filter_.seed(a);
      }
      sourceRates_ = filter_.eulerRates(g);
      if (fusion_.mode == 0) sourceRates_.z = -g.z;
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
    sourceRates_ = makeVec3(0, 0, 0);
    yaw_ = 0.0f;
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
      yaw_ = 0.0f;
      filter_.seed(r.meanAccelG);
      filter_.resetDerivative();
      sourceRates_ = makeVec3(0, 0, 0);
      enter(LOOK_WAIT_NEUTRAL, "cal_done", &out);
    } else if (cs != CAL_COLLECTING) {
      enter(LOOK_DISARMED, calibrationStatusName(cs), &out);
    }
  }

  if (fresh && fine_.capturing() && state_ != LOOK_ACTIVE && state_ != LOOK_FAULT) {
    out.finePose = fine_.capturePose();
    const int result = fine_.addSample(imu.sampleUs,
        makeVec3(filter_.rollDeg(), filter_.pitchDeg(), yaw_), calibrated_);
    if (result != 0) {
      out.fineEvent = result > 0 ? "captured" : "fail_motion";
      // Keep the live yaw in the same N-relative frame as captured poses.
      // Required calibration/ACTIVE yaw resets also restore N.yaw == 0.
      if (result > 0 && out.finePose == 'n') yaw_ -= fine_.capturedNeutralYaw();
      const MouseLookParams& p = mapper_.params();
      if (fine_.on() && !fine_.validSources(calibrated_, p.hsrc, p.vsrc)) fine_.off();
    }
  }

  poseErrors(&out);

  if (state_ == LOOK_WAIT_NEUTRAL) {
    const MouseLookParams& p = mapper_.params();
    const bool neutral = otherNeutral_ &&
        fabsf(out.lookXErrDeg) < LookAxis::engageEdge(p, 0) &&
        fabsf(out.lookYErrDeg) < LookAxis::engageEdge(p, 1);
    if (!neutral) {
      neutralTiming_ = false;
    } else if (!neutralTiming_) {
      neutralTiming_ = true;
      neutralSinceUs_ = nowUs;
    } else if (nowUs - neutralSinceUs_ >= timing_.neutralHoldUs) {
      mapper_.reset();
      lastMapUs_ = nowUs;
      enter(LOOK_ACTIVE, "active", &out);
    }
  }

  poseErrors(&out);  // entering ACTIVE resets yaw before this tick maps it

  // 6. Output.
  if (state_ == LOOK_ACTIVE) {
    float dtS = static_cast<float>(nowUs - lastMapUs_) * 1e-6f;
    if (dtS > 0.1f) dtS = 0.1f;  // a stall must not count as a long hold
    lastMapUs_ = nowUs;
    const MouseLookParams& p = mapper_.params();
    const float rates[3] = {sourceRates_.x, sourceRates_.y, sourceRates_.z};
    const float sx = fine_.on() ? fine_.gyroScale(0, p.hsrc, p.saturationDeg) : 1;
    const float sy = fine_.on() ? fine_.gyroScale(1, p.vsrc, p.saturationDeg) : 1;
    out.velocity = mapper_.update(out.lookXErrDeg, out.lookYErrDeg, dtS,
                                  rates[p.hsrc], rates[p.vsrc], sx, sy);
  }
  out.state = state_;
  return out;
}
