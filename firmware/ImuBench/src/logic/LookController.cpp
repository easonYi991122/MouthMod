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
      frame_(makeHeadFrame(makeVec3(0, 0, 1))),
      headRates_(makeVec3(0, 0, 0)),
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

void LookController::setFusionParams(const FusionParams& p) {
  if (fusion_.mode != p.mode) filter_.resetDerivative();
  fusion_ = p;
  sanitizeFusionParams(&fusion_);
}

const char* LookController::startFineCapture(char pose, uint64_t nowUs) {
  if (state_ == LOOK_ACTIVE) return "fcal_active";
  if (state_ == LOOK_FAULT) return "fcal_fault";
  return fine_.start(pose, nowUs, calibrated_);
}

const char* LookController::enableFineCalibration() {
  return fine_.enable();
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
  // D40: rotation since the basic-calibration neutral; no +-180 wrap.
  const Vec3 d = calibrated_
      ? headDeviation(frame_, upFromRollPitch(out->rollDeg, out->pitchDeg), yaw_)
      : makeVec3(0.0f, 0.0f, 0.0f);
  out->rollErrDeg = d.x;
  out->pitchErrDeg = d.y;
  const float errors[3] = {d.x, d.y, d.z};
  out->lookXErrDeg = errors[mapper_.params().hsrc];
  out->lookYErrDeg = errors[mapper_.params().vsrc];
  // D39: fine calibration replaces the single source with the head's own axes.
  if (fine_.on()) fine_.project(d, &out->lookXErrDeg, &out->lookYErrDeg);
}

LookTick LookController::tick(uint64_t nowUs, const ImuInput& imu,
                              bool enableInput) {
  LookTick out;
  out.state = state_;
  out.releaseAll = false;
  out.event = 0;
  out.fineEvent = 0;
  out.fineError = 0;
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
      if (fusion_.mode == 1) {
        filter_.updateEnhanced(a, g, dt, fusion_.armX, fusion_.armZ,
                               fusion_.accelRejectG);
      } else {
        filter_.update(a, g, dt);
      }
      // D40: yaw turns about the measured vertical, not about body Z, so a
      // board mounted on its edge in front of the mouth still sees head turns.
      headRates_ = headRates(
          frame_, upFromRollPitch(filter_.rollDeg(), filter_.pitchDeg()), g);
      // Match the filter's gap guard; never integrate across a stale sample.
      if (dt <= ComplementaryFilter::kMaxDtS) {
        const float rate = headRates_.z;
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
      headRates_ = headRates(
          frame_, upFromRollPitch(filter_.rollDeg(), filter_.pitchDeg()), g);
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
    frame_ = makeHeadFrame(makeVec3(0, 0, 1));
    headRates_ = makeVec3(0, 0, 0);
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
      frame_ = makeHeadFrame(upFromRollPitch(roll0_, pitch0_));
      headRates_ = makeVec3(0, 0, 0);
      // D40: keep the fine-calibration axes, move only its neutral.
      if (fine_.has(0) && fine_.setNeutral(roll0_, pitch0_)) {
        out.fineEvent = "neutral";
        out.finePose = 'n';
      }
      enter(LOOK_WAIT_NEUTRAL, "cal_done", &out);
    } else if (cs != CAL_COLLECTING) {
      enter(LOOK_DISARMED, calibrationStatusName(cs), &out);
    }
  }

  if (fresh && fine_.capturing() && state_ != LOOK_ACTIVE && state_ != LOOK_FAULT) {
    out.finePose = fine_.capturePose();
    const int result = fine_.addSample(imu.sampleUs,
        upFromRollPitch(filter_.rollDeg(), filter_.pitchDeg()), yaw_);
    if (result != 0) {
      out.fineEvent = result > 0 ? "captured" : "fail_motion";
      // Keep the live yaw in the same N-relative frame as captured poses.
      // Required calibration/ACTIVE yaw resets also restore N.yaw == 0.
      if (result > 0 && out.finePose == 'n') yaw_ -= fine_.capturedNeutralYaw();
      // A complete pose set takes effect at once (and re-derives the axes).
      if (result > 0 && fine_.complete()) out.fineError = fine_.enable();
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
    const float rates[3] = {headRates_.x, headRates_.y, headRates_.z};
    float rx = rates[p.hsrc], ry = rates[p.vsrc];
    if (fine_.on()) fine_.project(headRates_, &rx, &ry);
    out.velocity = mapper_.update(out.lookXErrDeg, out.lookYErrDeg, dtS, rx, ry);
  }
  out.state = state_;
  return out;
}
