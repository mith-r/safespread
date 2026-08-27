#pragma once

#include <cmath>
#include <cstdint>

enum DrivePhase : uint8_t {
  D_NEUTRAL = 0,
  D_BRAKE = 1,
  D_COMMAND = 2,
  D_VERIFY = 3,
  D_READY = 4,
  D_FAILED = 5,
};

inline bool driveDirectionChanged(bool directionRequested,
                                  bool currentReverse,
                                  bool requestedReverse) {
  return directionRequested && currentReverse != requestedReverse;
}

// One ESC direction sequence used everywhere the rover can move. The caller
// supplies pose samples; this object owns the timing and refuses to call a
// direction ready until measured displacement agrees with the command.
struct DirectionState {
  static constexpr uint32_t NEUTRAL_MS = 800;
  static constexpr uint32_t BRAKE_MS = 300;
  static constexpr uint32_t REVERSE_REARM_MS = 100;
  static constexpr uint32_t VERIFY_TIMEOUT_MS = 2000;
  static constexpr float VERIFY_DISTANCE_FT = 0.30f;

  DrivePhase phase = D_NEUTRAL;
  bool reverse = false;
  bool brakeRequired = false;
  bool brakeTapped = false;
  bool wrongDirection = false;
  bool noDisplacement = false;
  int commandPulseUs = 1500;
  int neutralPulseUs = 1500;
  int outputPulseUs = 1500;
  uint32_t phaseStartedMs = 0;
  uint32_t verifyStartedMs = 0;
  float verifyX = 0.0f;
  float verifyY = 0.0f;
  float headingDeg = 0.0f;

  void begin(bool requestedReverse, bool needsReverseBrake,
             uint32_t nowMs, float x, float y, float initialHeadingDeg,
             int requestedPulseUs, int neutralUs) {
    phase = D_NEUTRAL;
    reverse = requestedReverse;
    brakeRequired = requestedReverse && needsReverseBrake;
    brakeTapped = false;
    wrongDirection = false;
    noDisplacement = false;
    commandPulseUs = requestedPulseUs;
    neutralPulseUs = neutralUs;
    outputPulseUs = neutralUs;
    phaseStartedMs = nowMs;
    verifyStartedMs = nowMs;
    verifyX = x;
    verifyY = y;
    headingDeg = initialHeadingDeg;
  }

  int update(uint32_t nowMs, float x, float y) {
    switch (phase) {
      case D_NEUTRAL:
        outputPulseUs = neutralPulseUs;
        if (nowMs - phaseStartedMs >=
            (brakeTapped ? REVERSE_REARM_MS : NEUTRAL_MS)) {
          phase = brakeRequired && !brakeTapped ? D_BRAKE : D_COMMAND;
          phaseStartedMs = nowMs;
          if (phase == D_COMMAND) startVerification(nowMs, x, y);
          outputPulseUs = commandPulseUs;
        }
        break;

      case D_BRAKE:
        outputPulseUs = commandPulseUs;
        if (nowMs - phaseStartedMs >= BRAKE_MS) {
          // A reverse tap applies the ESC's brake. It must see neutral again
          // before the next reverse pulse is interpreted as reverse motion.
          brakeTapped = true;
          phase = D_NEUTRAL;
          phaseStartedMs = nowMs;
          outputPulseUs = neutralPulseUs;
        }
        break;

      case D_COMMAND:
        outputPulseUs = commandPulseUs;
        phase = D_VERIFY;
        break;

      case D_VERIFY: {
        outputPulseUs = commandPulseUs;
        const float radians = headingDeg * 3.14159265358979323846f / 180.0f;
        const float along = (x - verifyX) * std::sinf(radians) +
                            (y - verifyY) * std::cosf(radians);
        if (std::fabs(along) >= VERIFY_DISTANCE_FT) {
          const bool signMatches = reverse ? along < 0.0f : along > 0.0f;
          if (signMatches) {
            phase = D_READY;
          } else {
            wrongDirection = true;
            fail();
          }
        } else if (nowMs - verifyStartedMs >= VERIFY_TIMEOUT_MS) {
          noDisplacement = true;
          fail();
        }
        break;
      }

      case D_READY:
        outputPulseUs = commandPulseUs;
        break;

      case D_FAILED:
      default:
        outputPulseUs = neutralPulseUs;
        break;
    }
    return outputPulseUs;
  }

  void stop() {
    phase = D_NEUTRAL;
    wrongDirection = false;
    noDisplacement = false;
    brakeTapped = false;
    outputPulseUs = neutralPulseUs;
  }

  bool ready() const { return phase == D_READY; }
  bool failed() const { return phase == D_FAILED; }

 private:
  void startVerification(uint32_t nowMs, float x, float y) {
    verifyStartedMs = nowMs;
    verifyX = x;
    verifyY = y;
  }

  void fail() {
    phase = D_FAILED;
    outputPulseUs = neutralPulseUs;
  }
};
