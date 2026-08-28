#pragma once

#include <cmath>

constexpr float DEFAULT_STRAIGHT_SPEED_FPS = 1.5f;
constexpr float DEFAULT_TURN_SPEED_FPS = 0.7f;

// Hold a ground speed rather than a throttle position. A fixed pulse is an
// assumption about the whole vehicle at once, and the brine tank breaks it as
// it drains: the rover gets lighter over a mission, so the pulse that held
// cruise on a full tank runs an empty one fast. The integrator is what absorbs
// that -- it carries the load estimate and walks the throttle back down as the
// weight comes off.
//
// The closed-loop time constant is roughly this load's microseconds-per-ft/s
// divided by INTEGRAL_US_PER_FT, so the gain sets how fast a pass reaches
// cruise. What caps it is the empty rover, where the same gain buys the most
// speed per microsecond and the loop is closest to ringing against the delay in
// the measurement -- the phone medians five poses before sending, and a pose is
// allowed to be 250ms old. Against a model carrying that delay, 100 settles a
// full tank in about five seconds and overshoots an empty one by under 1%; 150
// overshoots by 6% and 200 by 13%. Commit 3ce5c37 ran 75 on this hardware
// (15us per ft/s of error every 200ms) and held 1.8 ft/s, which is the evidence
// that this range is drivable at all.
struct SpeedPI {
  // Small on purpose. This term acts directly on the medianed, lagged speed
  // estimate, so it is the one that would ring first; the integrator does the
  // work of finding the operating point.
  static constexpr float PROPORTIONAL_US_PER_FPS = 30.0f;
  static constexpr float INTEGRAL_US_PER_FT = 100.0f;
  // 3ce5c37 measured this rover settling at +190us light and +370us loaded to
  // hold 1.8 ft/s, and recorded the ESC having room to about 1900us. 400 is
  // that room; a lower ceiling would let a heavy rover pin and run slow with
  // nothing in the log to say it had run out of throttle.
  static constexpr float MAX_CORRECTION_US = 300.0f;
  static constexpr float MAX_OFFSET_US = 400.0f;
  static constexpr float BREAKAWAY_RATE_US_PER_SECOND = 100.0f;
  static constexpr float MAX_BREAKAWAY_US = 100.0f;
  static constexpr float MOTION_THRESHOLD_FPS = 0.05f;
  static constexpr float NO_MOTION_TIMEOUT_SECONDS = 1.0f;
  static constexpr float MAX_DT_SECONDS = 0.5f;
  static constexpr float MAX_TARGET_SLEW_FPS2 = 1.0f;

  float feedForwardUs = 120.0f;
  float integralUs = 0.0f;
  float breakawayUs = 0.0f;
  float noMotionSeconds = 0.0f;
  float lastCorrectionUs = 0.0f;
  float rampedTargetFps = 0.0f;
  int lastOffsetUs = 0;
  bool targetInitialized = false;
  bool stalled = false;

  int update(float target, float measured, float dt) {
    if (!std::isfinite(target) || !std::isfinite(measured) ||
        !std::isfinite(dt) || dt <= 0.0f || dt > MAX_DT_SECONDS) {
      return lastOffsetUs;
    }
    if (stalled) {
      lastOffsetUs = 0;
      return 0;
    }
    if (std::fabs(target) < 0.01f) {
      integralUs = 0.0f;
      breakawayUs = 0.0f;
      noMotionSeconds = 0.0f;
      lastCorrectionUs = 0.0f;
      rampedTargetFps = 0.0f;
      targetInitialized = false;
      lastOffsetUs = 0;
      return 0;
    }

    // Nothing resets the loop between a straight and the turn that follows it
    // -- a direction change does, but a forward-only route never has one -- so
    // the integrator meets the turn still wound for cruise. The target flips
    // the moment the route index enters the turn, so a step would hand the
    // controller a 0.8 ft/s error in one update and let it drive the integrator
    // hard the other way; walking the target down over the turn entry keeps the
    // error small enough that the wind-off stays monotonic. It does not brake
    // before the turn -- deciding that would mean reading ahead in the route,
    // which this controller does not see.
    //
    // The first target after a reset is taken whole. Ramping up from a
    // standstill would soften exactly the push that breaks the rover away,
    // which is the one thing the stall timeout is watching for.
    if (!targetInitialized || rampedTargetFps * target < 0.0f) {
      rampedTargetFps = target;
      targetInitialized = true;
    } else {
      const float step = MAX_TARGET_SLEW_FPS2 * dt;
      const float remaining = target - rampedTargetFps;
      if (remaining > step) rampedTargetFps += step;
      else if (remaining < -step) rampedTargetFps -= step;
      else rampedTargetFps = target;
    }

    const float direction = rampedTargetFps > 0.0f ? 1.0f : -1.0f;
    const float error = rampedTargetFps - measured;
    const bool moving = std::fabs(measured) >= MOTION_THRESHOLD_FPS;

    float candidateIntegral = integralUs;
    if (moving) {
      noMotionSeconds = 0.0f;
      breakawayUs = 0.0f;
      candidateIntegral += INTEGRAL_US_PER_FT * error * dt;
    } else {
      noMotionSeconds += dt;
      breakawayUs += BREAKAWAY_RATE_US_PER_SECOND * dt;
      if (breakawayUs > MAX_BREAKAWAY_US) breakawayUs = MAX_BREAKAWAY_US;
      if (noMotionSeconds >= NO_MOTION_TIMEOUT_SECONDS - 0.001f) {
        stalled = true;
        lastCorrectionUs = 0.0f;
        lastOffsetUs = 0;
        return 0;
      }
    }

    const float proportional = PROPORTIONAL_US_PER_FPS * error;
    const float breakaway = moving ? 0.0f : direction * breakawayUs;
    const float rawCorrection = proportional + candidateIntegral + breakaway;
    const float feedForward = direction * std::fabs(feedForwardUs);
    const float rawOffset = feedForward + rawCorrection;

    const bool saturatesHigh = rawCorrection > MAX_CORRECTION_US ||
                               rawOffset > MAX_OFFSET_US;
    const bool saturatesLow = rawCorrection < -MAX_CORRECTION_US ||
                              rawOffset < -MAX_OFFSET_US;
    if (!moving || (!saturatesHigh && !saturatesLow) ||
        (saturatesHigh && error < 0.0f) ||
        (saturatesLow && error > 0.0f)) {
      integralUs = candidateIntegral;
    }

    float correction = proportional + integralUs + breakaway;
    if (correction > MAX_CORRECTION_US) correction = MAX_CORRECTION_US;
    if (correction < -MAX_CORRECTION_US) correction = -MAX_CORRECTION_US;
    lastCorrectionUs = correction;

    float offset = feedForward + correction;
    if (offset > MAX_OFFSET_US) offset = MAX_OFFSET_US;
    if (offset < -MAX_OFFSET_US) offset = -MAX_OFFSET_US;
    lastOffsetUs = static_cast<int>(std::lround(offset));
    return lastOffsetUs;
  }

  void reset() {
    integralUs = 0.0f;
    breakawayUs = 0.0f;
    noMotionSeconds = 0.0f;
    lastCorrectionUs = 0.0f;
    rampedTargetFps = 0.0f;
    targetInitialized = false;
    lastOffsetUs = 0;
    stalled = false;
  }
};
