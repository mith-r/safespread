#pragma once

#include <cmath>

constexpr float DEFAULT_STRAIGHT_SPEED_FPS = 1.0f;
constexpr float DEFAULT_TURN_SPEED_FPS = 0.7f;

struct SpeedPI {
  static constexpr float PROPORTIONAL_US_PER_FPS = 30.0f;
  static constexpr float INTEGRAL_US_PER_FT = 8.0f;
  static constexpr float MAX_CORRECTION_US = 250.0f;
  static constexpr float MIN_FEED_FORWARD_US = 40.0f;
  static constexpr float MAX_OFFSET_US = 350.0f;
  static constexpr float BREAKAWAY_RATE_US_PER_SECOND = 100.0f;
  static constexpr float MAX_BREAKAWAY_US = 200.0f;
  static constexpr float MOTION_THRESHOLD_FPS = 0.05f;
  // DirectionState independently requires 0.30 ft within two seconds. Match
  // that verification window so a loaded rover gets the full bounded
  // breakaway ramp instead of being faulted halfway through it.
  static constexpr float NO_MOTION_TIMEOUT_SECONDS = 2.0f;
  static constexpr float MAX_DT_SECONDS = 0.5f;

  float feedForwardUs = 120.0f;
  float integralUs = 0.0f;
  float breakawayUs = 0.0f;
  float noMotionSeconds = 0.0f;
  float lastCorrectionUs = 0.0f;
  int lastOffsetUs = 0;
  bool stalled = false;
  bool configurationInvalid = false;

  int update(float target, float measured, float dt) {
    if (!std::isfinite(target) || !std::isfinite(measured) ||
        !std::isfinite(dt) || dt <= 0.0f || dt > MAX_DT_SECONDS) {
      return lastOffsetUs;
    }
    if (stalled) {
      lastOffsetUs = 0;
      return 0;
    }
    if (!std::isfinite(feedForwardUs) ||
        feedForwardUs < MIN_FEED_FORWARD_US ||
        feedForwardUs > MAX_OFFSET_US) {
      configurationInvalid = true;
      integralUs = 0.0f;
      breakawayUs = 0.0f;
      noMotionSeconds = 0.0f;
      lastCorrectionUs = 0.0f;
      lastOffsetUs = 0;
      return 0;
    }
    configurationInvalid = false;
    if (std::fabs(target) < 0.01f) {
      integralUs = 0.0f;
      breakawayUs = 0.0f;
      noMotionSeconds = 0.0f;
      lastCorrectionUs = 0.0f;
      lastOffsetUs = 0;
      return 0;
    }

    const float direction = target > 0.0f ? 1.0f : -1.0f;
    const float error = target - measured;
    // Opposite-sign motion is not progress and must not clear the no-motion
    // timer (notably while a loaded rover is still coasting into a reversal).
    const bool moving = measured * direction >= MOTION_THRESHOLD_FPS;

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
    lastOffsetUs = 0;
    stalled = false;
    configurationInvalid = false;
  }
};
