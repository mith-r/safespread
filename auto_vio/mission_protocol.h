#ifndef SAFESPREAD_MISSION_PROTOCOL_H
#define SAFESPREAD_MISSION_PROTOCOL_H

#include <cmath>
#include <cstdint>
#include "protocol_v2.h"
#include "safety.h"

class MissionProtocol {
 public:
  // Keep the expected rectangle-frame staging pose centralized: route setup
  // may move the run-in point without changing the safety checks themselves.
  static constexpr float EXPECTED_START_X_FT = 0.0f;
  static constexpr float EXPECTED_START_Y_FT = -1.0f;
  static constexpr float EXPECTED_START_HEADING_DEG = 0.0f;
  static constexpr float START_POSITION_TOLERANCE_FT = 0.75f;
  static constexpr float START_HEADING_TOLERANCE_DEG = 5.0f;
  static constexpr float START_MAX_SPEED_FPS = 0.10f;

  MissionState state() const { return state_; }
  FaultCode fault() const { return fault_; }
  uint16_t epoch() const { return epoch_; }
  uint16_t calibrationId() const { return calibrationId_; }
  uint32_t droppedPoses() const { return droppedPoses_; }
  uint32_t lastPoseReceivedAtMs() const { return poseReceivedAtMs_; }
  FaultCode lastPoseRejectFault() const { return lastPoseRejectFault_; }
  bool lastCommandWasDuplicate() const { return lastCommandWasDuplicate_; }
  bool lastSetupWasDuplicate() const { return lastSetupWasDuplicate_; }
  const protocol_v2::RectangleV2 &rectangle() const { return rectangle_; }
  const protocol_v2::CalibrationV2 &calibration() const { return calibration_; }

  bool allowsLegacyDiagnostics() const { return state_ == S_IDLE; }
  bool allowsLegacyArm() const { return false; }
  void setPwmReady(bool ready) { pwmReady_ = ready; }

  protocol_v2::AckV2 acceptCalibration(
      const protocol_v2::CalibrationV2 &message, uint32_t /* nowMs */) {
    lastSetupWasDuplicate_ = false;
    if (setupCached_ && cachedSetupType_ == 0x4b && message.epoch == cachedSetupEpoch_ &&
        message.commandId == cachedSetupCommandId_) {
      lastSetupWasDuplicate_ = true;
      return cachedSetupAck_;
    }
    if (state_ != S_IDLE ||
        (hasAcceptedEpoch_ && !epochIsNewer(message.epoch, lastAcceptedEpoch_))) {
      return ack(message.epoch, message.commandId, F_ROUTE);
    }
    if (message.flags != 0 || message.schemaVersion != SUPPORTED_CALIBRATION_SCHEMA) {
      return ack(message.epoch, message.commandId, F_CALIBRATION);
    }
    epoch_ = message.epoch;
    lastAcceptedEpoch_ = message.epoch;
    hasAcceptedEpoch_ = true;
    calibrationId_ = message.calibrationId;
    calibration_ = message;
    hasCalibration_ = true;
    hasRectangle_ = false;
    hasPose_ = false;
    posePending_ = false;
    hasPoseSequence_ = false;
    armedStartViolation_ = false;
    latestPose_ = {};
    lastPoseSequence_ = 0;
    poseReceivedAtMs_ = 0;
    droppedPoses_ = 0;
    fault_ = F_NONE;
    clearCommandCache();
    protocol_v2::AckV2 result = ack(message.epoch, message.commandId, F_NONE);
    cacheSetup(0x4b, message.epoch, message.commandId, result);
    return result;
  }

  protocol_v2::AckV2 acceptRectangle(
      const protocol_v2::RectangleV2 &message, uint32_t /* nowMs */) {
    lastSetupWasDuplicate_ = false;
    if (setupCached_ && cachedSetupType_ == 0x44 && message.epoch == cachedSetupEpoch_ &&
        message.commandId == cachedSetupCommandId_) {
      lastSetupWasDuplicate_ = true;
      return cachedSetupAck_;
    }
    if (state_ != S_IDLE || !hasCalibration_ || message.epoch != epoch_) {
      return ack(message.epoch, message.commandId, F_ROUTE);
    }
    if (message.calibrationId != calibrationId_) {
      return ack(message.epoch, message.commandId, F_CALIBRATION);
    }
    rectangle_ = message;
    hasRectangle_ = true;
    // Poses sent before this ACK are in the pre-rectangle coordinate frame.
    // Do not let one satisfy ARM, reach control, or seed innovation checks in
    // the rectangle frame. Retain the sequence watermark so a delayed replay
    // of that old pose cannot become the required post-rectangle sample.
    hasPose_ = false;
    posePending_ = false;
    armedStartViolation_ = false;
    latestPose_ = {};
    poseReceivedAtMs_ = 0;
    state_ = S_CONFIGURED;
    protocol_v2::AckV2 result = ack(message.epoch, message.commandId, F_NONE);
    cacheSetup(0x44, message.epoch, message.commandId, result);
    return result;
  }

  protocol_v2::AckV2 overrideLastSetupWithFault(FaultCode fault) {
    setFault(fault);
    if (setupCached_) {
      cachedSetupAck_.state = static_cast<uint8_t>(state_);
      cachedSetupAck_.faultCode = static_cast<uint16_t>(fault_);
      return cachedSetupAck_;
    }
    return ack(epoch_, 0, fault);
  }

  bool acceptPose(const protocol_v2::PoseV2 &message, uint32_t receivedAtMs) {
    lastPoseRejectFault_ = F_NONE;
    if (state_ != S_IDLE && state_ != S_CONFIGURED && state_ != S_ARMED && state_ != S_RUNNING) {
      return rejectPose(F_POSE_INVALID);
    }
    if (state_ == S_IDLE && !hasCalibration_) return rejectPose(F_CALIBRATION);
    if (message.epoch != epoch_ || message.calibrationId != calibrationId_) {
      return rejectPose(F_CALIBRATION);
    }
    if ((message.flags & 0x01) == 0) return rejectPose(F_TRACKING_ERROR);
    if ((message.flags & 0x04) == 0) return rejectPose(F_CALIBRATION);
    if (message.ageMs > MAX_POSE_AGE_MS ||
        (hasPoseSequence_ && message.sequence <= lastPoseSequence_)) {
      return rejectPose(F_POSE_INVALID);
    }
    if (std::fabs(message.speedFps) > MAX_SPEED_FPS ||
        std::fabs(message.yawRateDps) > MAX_YAW_RATE_DPS) {
      return rejectPose(F_POSE_INVALID);
    }
    if (state_ == S_ARMED && !poseAtExpectedStart(message)) {
      // The firmware caller turns any rejected ARMED pose into a latched
      // hardware fault. Keep a local latch too, so START cannot proceed if a
      // caller observes the rejection but has not yet performed that handoff.
      armedStartViolation_ = true;
      return rejectPose(F_TRACKING_ERROR);
    }
    if (hasPose_) {
      // BLE delivery jitter is not vehicle motion. Reconstruct the capture
      // interval from each packet's age instead of differentiating solely by
      // receive time. Unsigned receive subtraction intentionally handles the
      // millis() wrap as long as consecutive poses are less than 2^31 ms apart.
      const uint32_t receivedDeltaMs = receivedAtMs - poseReceivedAtMs_;
      if (receivedDeltaMs > 0x7fffffffu) return rejectPose(F_POSE_INVALID);
      const int64_t captureDeltaMs = static_cast<int64_t>(receivedDeltaMs) +
          static_cast<int64_t>(latestPose_.ageMs) -
          static_cast<int64_t>(message.ageMs);
      if (captureDeltaMs <= 0) return rejectPose(F_POSE_INVALID);
      const float dt = captureDeltaMs / 1000.0f;
      if (std::fabs(message.speedFps - latestPose_.speedFps) / dt > MAX_ACCEL_FPS2) {
        return rejectPose(F_POSE_JUMP);
      }
      const float dx = message.x - latestPose_.x;
      const float dy = message.y - latestPose_.y;
      const float allowedDistance =
          std::fmax(std::fabs(message.speedFps), std::fabs(latestPose_.speedFps)) * dt +
          POSITION_INNOVATION_FT;
      if (std::sqrt(dx * dx + dy * dy) > allowedDistance) {
        return rejectPose(F_POSE_JUMP);
      }
      const float headingDelta = std::fabs(wrappedAngleDiff(
          message.heading, latestPose_.heading));
      const float allowedHeading =
          std::fmax(std::fabs(message.yawRateDps), std::fabs(latestPose_.yawRateDps)) * dt +
          HEADING_INNOVATION_DEG;
      if (headingDelta > allowedHeading) return rejectPose(F_POSE_JUMP);
    }
    if (posePending_) droppedPoses_++;
    latestPose_ = message;
    lastPoseSequence_ = message.sequence;
    hasPoseSequence_ = true;
    poseReceivedAtMs_ = receivedAtMs;
    hasPose_ = true;
    posePending_ = true;
    return true;
  }

  bool poseFresh(uint32_t nowMs) const {
    if (!hasPose_) return false;
    const uint32_t queuedAge = nowMs - poseReceivedAtMs_;
    return latestPose_.ageMs <= MAX_POSE_AGE_MS &&
           queuedAge <= MAX_POSE_AGE_MS - latestPose_.ageMs;
  }

  bool takePose(protocol_v2::PoseV2 &out) {
    if (!posePending_) return false;
    out = latestPose_;
    posePending_ = false;
    return true;
  }

  protocol_v2::AckV2 acceptCommand(
      const protocol_v2::CommandV2 &message, uint32_t nowMs,
      bool calibrationAllowed = false) {
    lastCommandWasDuplicate_ = false;
    for (uint8_t index = 0; index < commandCacheCount_; ++index) {
      const CommandCacheEntry &entry = commandCache_[index];
      if (message.epoch == entry.epoch && message.commandId == entry.commandId &&
          message.opcode == entry.opcode) {
        lastCommandWasDuplicate_ = true;
        return entry.response;
      }
    }
    if (message.opcode != 3 && message.epoch == epoch_ && hasCommandWatermark_ &&
        message.commandId <= commandWatermark_) {
      protocol_v2::AckV2 rejected = ack(message.epoch, message.commandId, F_ROUTE);
      cacheCommand(message, rejected);
      return rejected;
    }

    protocol_v2::AckV2 result = {};
    if (message.opcode == 3) {
      state_ = S_IDLE;
      fault_ = F_NONE;
      hasPose_ = false;
      posePending_ = false;
      hasPoseSequence_ = false;
      armedStartViolation_ = false;
      latestPose_ = {};
      lastPoseSequence_ = 0;
      poseReceivedAtMs_ = 0;
      hasRectangle_ = false;
      hasCalibration_ = false;
      calibrationId_ = 0;
      clearSetupCache();
      result = ack(message.epoch, message.commandId, F_NONE);
    } else if (message.epoch != epoch_) {
      result = ack(message.epoch, message.commandId, F_ROUTE);
    } else if (message.opcode == 1) {
      if (state_ != S_CONFIGURED || !hasRectangle_) {
        result = ack(message.epoch, message.commandId, F_ROUTE);
      } else if (!poseFresh(nowMs)) {
        result = ack(message.epoch, message.commandId, F_POSE_TIMEOUT);
      } else if (!poseAtExpectedStart(latestPose_)) {
        result = ack(message.epoch, message.commandId, F_TRACKING_ERROR);
      } else if (!pwmReady_) {
        result = ack(message.epoch, message.commandId, F_PWM);
      } else {
        armedStartViolation_ = false;
        state_ = S_ARMED;
        result = ack(message.epoch, message.commandId, F_NONE);
      }
    } else if (message.opcode == 2) {
      if (state_ != S_ARMED) {
        result = ack(message.epoch, message.commandId, F_ROUTE);
      } else if (!poseFresh(nowMs)) {
        result = ack(message.epoch, message.commandId, F_POSE_TIMEOUT);
      } else if (armedStartViolation_ || !poseAtExpectedStart(latestPose_)) {
        result = ack(message.epoch, message.commandId, F_TRACKING_ERROR);
      } else if (!pwmReady_) {
        result = ack(message.epoch, message.commandId, F_PWM);
      } else {
        state_ = S_RUNNING;
        result = ack(message.epoch, message.commandId, F_NONE);
      }
    } else if (message.opcode == 4) {
      // The retained self-test drives both directions. Treat it as a dry
      // calibration movement, not as an idle diagnostic that can bypass pose,
      // calibration, or actuator readiness gates.
      if (state_ != S_IDLE || !hasCalibration_ || !calibrationAllowed) {
        result = ack(message.epoch, message.commandId, F_CALIBRATION);
      } else if (!poseFresh(nowMs)) {
        result = ack(message.epoch, message.commandId, F_POSE_TIMEOUT);
      } else if (!pwmReady_) {
        result = ack(message.epoch, message.commandId, F_PWM);
      } else {
        result = ack(message.epoch, message.commandId, F_NONE);
      }
    } else if (message.opcode == 8) {
      result = state_ == S_IDLE || state_ == S_FAULT
          ? ack(message.epoch, message.commandId, F_NONE)
          : ack(message.epoch, message.commandId, F_ROUTE);
    } else if (message.opcode >= 5 && message.opcode <= 7) {
      if (state_ != S_IDLE || !hasCalibration_ || !calibrationAllowed) {
        result = ack(message.epoch, message.commandId, F_CALIBRATION);
      } else if (!poseFresh(nowMs)) {
        result = ack(message.epoch, message.commandId, F_POSE_TIMEOUT);
      } else if (!pwmReady_) {
        result = ack(message.epoch, message.commandId, F_PWM);
      } else {
        result = ack(message.epoch, message.commandId, F_NONE);
      }
    } else {
      result = ack(message.epoch, message.commandId, F_ROUTE);
    }
    if (message.epoch == epoch_ &&
        (!hasCommandWatermark_ || message.commandId > commandWatermark_)) {
      commandWatermark_ = message.commandId;
      hasCommandWatermark_ = true;
    }
    cacheCommand(message, result);
    return result;
  }

  void setFault(FaultCode fault) {
    if (fault == F_NONE) return;
    state_ = S_FAULT;
    fault_ = fault;
  }

  void complete() {
    if (state_ == S_RUNNING) state_ = S_COMPLETE;
  }

  void onDisconnect() {
    if (state_ == S_CONFIGURED || state_ == S_ARMED || state_ == S_RUNNING) setFault(F_BLE);
  }

 private:
  static constexpr uint32_t MAX_POSE_AGE_MS = 250;
  static constexpr uint16_t SUPPORTED_CALIBRATION_SCHEMA = 1;
  static constexpr float MAX_SPEED_FPS = 8.0f;
  static constexpr float MAX_ACCEL_FPS2 = 15.0f;
  static constexpr float MAX_YAW_RATE_DPS = 180.0f;
  static constexpr float POSITION_INNOVATION_FT = 0.75f;
  static constexpr float HEADING_INNOVATION_DEG = 5.0f;
  static constexpr uint8_t COMMAND_CACHE_CAPACITY = 8;
  struct CommandCacheEntry {
    uint8_t opcode;
    uint16_t epoch;
    uint32_t commandId;
    protocol_v2::AckV2 response;
  };
  MissionState state_ = S_IDLE;
  FaultCode fault_ = F_NONE;
  uint16_t epoch_ = 0;
  uint16_t calibrationId_ = 0;
  bool hasCalibration_ = false;
  bool hasAcceptedEpoch_ = false;
  uint16_t lastAcceptedEpoch_ = 0;
  bool pwmReady_ = false;
  bool hasRectangle_ = false;
  protocol_v2::CalibrationV2 calibration_ = {};
  protocol_v2::RectangleV2 rectangle_ = {};
  bool hasPose_ = false;
  bool posePending_ = false;
  bool hasPoseSequence_ = false;
  bool armedStartViolation_ = false;
  protocol_v2::PoseV2 latestPose_ = {};
  uint32_t lastPoseSequence_ = 0;
  uint32_t poseReceivedAtMs_ = 0;
  uint32_t droppedPoses_ = 0;
  FaultCode lastPoseRejectFault_ = F_NONE;
  bool lastCommandWasDuplicate_ = false;
  bool lastSetupWasDuplicate_ = false;
  CommandCacheEntry commandCache_[COMMAND_CACHE_CAPACITY] = {};
  uint8_t commandCacheCount_ = 0;
  uint8_t commandCacheNext_ = 0;
  bool hasCommandWatermark_ = false;
  uint32_t commandWatermark_ = 0;
  bool setupCached_ = false;
  uint8_t cachedSetupType_ = 0;
  uint16_t cachedSetupEpoch_ = 0;
  uint32_t cachedSetupCommandId_ = 0;
  protocol_v2::AckV2 cachedSetupAck_ = {};

  protocol_v2::AckV2 ack(uint16_t messageEpoch, uint32_t commandId, FaultCode fault) const {
    protocol_v2::AckV2 result = {};
    result.state = static_cast<uint8_t>(state_);
    result.epoch = messageEpoch;
    result.commandId = commandId;
    result.faultCode = static_cast<uint16_t>(fault);
    result.calibrationId = calibrationId_;
    return result;
  }

  bool rejectPose(FaultCode fault) {
    lastPoseRejectFault_ = fault;
    return false;
  }

  static float wrappedAngleDiff(float a, float b) {
    return std::fmod(a - b + 540.0f, 360.0f) - 180.0f;
  }

  static bool poseAtExpectedStart(const protocol_v2::PoseV2 &pose) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.heading) || !std::isfinite(pose.speedFps)) {
      return false;
    }
    const float dx = pose.x - EXPECTED_START_X_FT;
    const float dy = pose.y - EXPECTED_START_Y_FT;
    return std::sqrt(dx * dx + dy * dy) <= START_POSITION_TOLERANCE_FT &&
           std::fabs(wrappedAngleDiff(
               pose.heading, EXPECTED_START_HEADING_DEG)) <=
               START_HEADING_TOLERANCE_DEG &&
           std::fabs(pose.speedFps) <= START_MAX_SPEED_FPS;
  }

  static bool epochIsNewer(uint16_t candidate, uint16_t previous) {
    const uint16_t delta = static_cast<uint16_t>(candidate - previous);
    return delta != 0 && delta < 0x8000;
  }

  void cacheCommand(
      const protocol_v2::CommandV2 &message, const protocol_v2::AckV2 &response) {
    commandCache_[commandCacheNext_] = {
      message.opcode, message.epoch, message.commandId, response
    };
    commandCacheNext_ = (commandCacheNext_ + 1) % COMMAND_CACHE_CAPACITY;
    if (commandCacheCount_ < COMMAND_CACHE_CAPACITY) commandCacheCount_++;
  }

  void cacheSetup(uint8_t type, uint16_t epoch, uint32_t commandId,
                  const protocol_v2::AckV2 &response) {
    setupCached_ = true;
    cachedSetupType_ = type;
    cachedSetupEpoch_ = epoch;
    cachedSetupCommandId_ = commandId;
    cachedSetupAck_ = response;
  }

  void clearCommandCache() {
    commandCacheCount_ = 0;
    commandCacheNext_ = 0;
    hasCommandWatermark_ = false;
    commandWatermark_ = 0;
    for (uint8_t index = 0; index < COMMAND_CACHE_CAPACITY; ++index) {
      commandCache_[index] = {};
    }
  }

  void clearSetupCache() {
    setupCached_ = false;
    cachedSetupType_ = 0;
    cachedSetupEpoch_ = 0;
    cachedSetupCommandId_ = 0;
    cachedSetupAck_ = {};
  }
};

#endif
