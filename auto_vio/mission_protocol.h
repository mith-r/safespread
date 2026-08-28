#ifndef SAFESPREAD_MISSION_PROTOCOL_H
#define SAFESPREAD_MISSION_PROTOCOL_H

#include <cmath>
#include <cstdint>
#include "protocol_v2.h"
#include "safety.h"

// Why a pose was refused, with the numbers that refused it. Without these a
// rejected stream is indistinguishable from a phone that stopped talking.
struct PoseRejectDetail {
  FaultCode fault;
  float dtSeconds;
  float measured;
  float allowed;
};

class MissionProtocol {
 public:
  MissionState state() const { return state_; }
  FaultCode fault() const { return fault_; }
  uint16_t epoch() const { return epoch_; }
  uint16_t calibrationId() const { return calibrationId_; }
  uint32_t droppedPoses() const { return droppedPoses_; }
  uint32_t lastPoseReceivedAtMs() const { return poseReceivedAtMs_; }
  FaultCode lastPoseRejectFault() const { return lastPoseReject_.fault; }
  const PoseRejectDetail &lastPoseReject() const { return lastPoseReject_; }
  bool lastCommandWasDuplicate() const { return lastCommandWasDuplicate_; }
  bool lastSetupWasDuplicate() const { return lastSetupWasDuplicate_; }
  const protocol_v2::RectangleV2 &rectangle() const { return rectangle_; }
  const protocol_v2::CalibrationV2 &calibration() const { return calibration_; }

  bool allowsLegacyDiagnostics() const { return state_ == S_IDLE; }
  bool allowsLegacyArm() const { return false; }
  void setPwmReady(bool ready) { pwmReady_ = ready; }
  // Only the rover knows where a resumed pass begins, so only the rover can
  // check the operator put it there. Refusing the ARM leaves the mission
  // CONFIGURED so the rover can be moved and armed again.
  void setStartPointReached(bool reached) { startPointReached_ = reached; }

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
    lastPoseSequence_ = 0;
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
    lastPoseReject_ = {F_NONE, 0.0f, 0.0f, 0.0f};
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
        (hasPose_ && message.sequence <= lastPoseSequence_)) {
      return rejectPose(F_POSE_INVALID);
    }
    if (std::fabs(message.speedFps) > MAX_SPEED_FPS ||
        std::fabs(message.yawRateDps) > MAX_YAW_RATE_DPS) {
      return rejectPose(F_POSE_INVALID);
    }
    // ARKit can relocalize position, heading, or inferred velocity in one
    // frame. Accept that discontinuity and keep the control stream alive; pose
    // jump fault code 4 remains reserved only for wire compatibility.
    if (posePending_) droppedPoses_++;
    latestPose_ = message;
    lastPoseSequence_ = message.sequence;
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
      bool diagnosticMotionAllowed = false) {
    lastCommandWasDuplicate_ = false;
    for (uint8_t index = 0; index < commandCacheCount_; ++index) {
      const CommandCacheEntry &entry = commandCache_[index];
      if (message.epoch == entry.epoch && message.commandId == entry.commandId &&
          message.opcode == entry.opcode) {
        lastCommandWasDuplicate_ = true;
        return entry.response;
      }
    }
    if (message.opcode != 3 && message.opcode != 4 && message.epoch == epoch_ &&
        hasCommandWatermark_ && message.commandId <= commandWatermark_) {
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
      hasRectangle_ = false;
      hasCalibration_ = false;
      calibrationId_ = 0;
      clearSetupCache();
      result = ack(message.epoch, message.commandId, F_NONE);
    } else if (message.opcode == 4) {
      // The self test answers "why will nothing move", so it cannot require a
      // configured mission: an operator whose PWM chip is not answering has no
      // way to produce one, and the I2C probe is the only thing that names the
      // actual fault. Handled ahead of the epoch check for the same reason Stop
      // is, and accepted from S_FAULT for the same reason the fault dump is --
      // those are precisely the states it is needed in. Still refused while
      // armed or running, where the rover is under mission control.
      result = state_ == S_IDLE || state_ == S_FAULT
          ? ack(message.epoch, message.commandId, F_NONE)
          : ack(message.epoch, message.commandId, F_ROUTE);
    } else if (message.epoch != epoch_) {
      result = ack(message.epoch, message.commandId, F_ROUTE);
    } else if (message.opcode == 1) {
      if (state_ != S_CONFIGURED || !hasRectangle_) {
        result = ack(message.epoch, message.commandId, F_ROUTE);
      } else if (!poseFresh(nowMs)) {
        result = ack(message.epoch, message.commandId, F_POSE_TIMEOUT);
      } else if (!pwmReady_) {
        result = ack(message.epoch, message.commandId, F_PWM);
      } else if (!startPointReached_) {
        result = ack(message.epoch, message.commandId, F_START_POINT);
      } else {
        state_ = S_ARMED;
        result = ack(message.epoch, message.commandId, F_NONE);
      }
    } else if (message.opcode == 2) {
      if (state_ != S_ARMED) {
        result = ack(message.epoch, message.commandId, F_ROUTE);
      } else if (!poseFresh(nowMs)) {
        result = ack(message.epoch, message.commandId, F_POSE_TIMEOUT);
      } else if (!pwmReady_) {
        result = ack(message.epoch, message.commandId, F_PWM);
      } else {
        state_ = S_RUNNING;
        result = ack(message.epoch, message.commandId, F_NONE);
      }
    } else if (message.opcode == 8) {
      result = state_ == S_IDLE || state_ == S_FAULT
          ? ack(message.epoch, message.commandId, F_NONE)
          : ack(message.epoch, message.commandId, F_ROUTE);
    } else if (message.opcode == 5) {
      if (state_ != S_IDLE || !hasCalibration_ || !diagnosticMotionAllowed) {
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
  static constexpr float MAX_YAW_RATE_DPS = 180.0f;
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
  bool startPointReached_ = true;
  bool hasRectangle_ = false;
  protocol_v2::CalibrationV2 calibration_ = {};
  protocol_v2::RectangleV2 rectangle_ = {};
  bool hasPose_ = false;
  bool posePending_ = false;
  protocol_v2::PoseV2 latestPose_ = {};
  uint32_t lastPoseSequence_ = 0;
  uint32_t poseReceivedAtMs_ = 0;
  uint32_t droppedPoses_ = 0;
  PoseRejectDetail lastPoseReject_ = {F_NONE, 0.0f, 0.0f, 0.0f};
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

  bool rejectPose(FaultCode fault, float dtSeconds = 0.0f, float measured = 0.0f,
                  float allowed = 0.0f) {
    lastPoseReject_ = {fault, dtSeconds, measured, allowed};
    return false;
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
