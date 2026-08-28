#ifndef SAFESPREAD_MISSION_PROTOCOL_H
#define SAFESPREAD_MISSION_PROTOCOL_H

#include <cmath>
#include <cstdint>
#include "protocol_v2.h"
#include "safety.h"

class MissionProtocol {
 public:
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
    // The rectangle defines the coordinate frame poses arrive in: the app
    // streams the rover's raw world pose until it is configured, then switches
    // to rectangle-relative coordinates. Discard the pre-rectangle pose so that
    // one-time frame shift is not read as a position/heading jump -- exactly as
    // acceptCalibration clears it when the epoch's frame of reference changes.
    hasPose_ = false;
    posePending_ = false;
    lastPoseSequence_ = 0;
    droppedPoses_ = 0;
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
    // Transport artifacts, not stream corruption: iOS coalesces BLE writes,
    // so a duplicate, out-of-order, or same-millisecond packet arrives in
    // normal operation, and a pose can sit in the app's queue past its age
    // budget. Drop these without a fault -- freshness is enforced separately
    // (poseFresh watchdog faults with F_POSE_TIMEOUT if nothing usable
    // arrives) -- so one coalesced packet cannot kill an armed mission.
    if (message.ageMs > MAX_POSE_AGE_MS ||
        (hasPose_ && message.sequence <= lastPoseSequence_)) {
      return rejectPoseBenign();
    }
    if (std::fabs(message.speedFps) > MAX_SPEED_FPS ||
        std::fabs(message.yawRateDps) > MAX_YAW_RATE_DPS) {
      return rejectPose(F_POSE_INVALID);
    }
    if (hasPose_) {
      // The jump gates bound how far the pose may move between camera
      // captures, so dt must be capture-to-capture time. Arrival spacing is
      // wrong for that: iOS coalesces BLE writes, so poses captured ~100 ms
      // apart routinely land ~2 ms apart, which explodes the acceleration
      // ratio and collapses the distance/heading envelopes (seen in the field
      // as F_POSE_JUMP the moment the rover starts creeping). Reconstruct the
      // capture instant from the packet's own age.
      const int64_t captureMs = (int64_t)receivedAtMs - (int64_t)message.ageMs;
      const float dt = (captureMs - lastCaptureMs_) / 1000.0f;
      if (dt <= 0.0f) return rejectPoseBenign();  // coalesced or duplicate frame
      // Transit jitter (~10 ms) corrupts short intervals, so evaluate the
      // acceleration ratio over at least 50 ms of capture time.
      const float accelDt = std::fmax(dt, 0.05f);
      if (std::fabs(message.speedFps - latestPose_.speedFps) / accelDt > MAX_ACCEL_FPS2) {
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
    poseReceivedAtMs_ = receivedAtMs;
    lastCaptureMs_ = (int64_t)receivedAtMs - (int64_t)message.ageMs;
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
  protocol_v2::PoseV2 latestPose_ = {};
  uint32_t lastPoseSequence_ = 0;
  uint32_t poseReceivedAtMs_ = 0;
  int64_t lastCaptureMs_ = 0;
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

  /** Drop the packet without blaming the stream: lastPoseRejectFault stays
   *  F_NONE so the caller does not escalate to a mission fault. */
  bool rejectPoseBenign() {
    lastPoseRejectFault_ = F_NONE;
    return false;
  }

  static float wrappedAngleDiff(float a, float b) {
    return std::fmod(a - b + 540.0f, 360.0f) - 180.0f;
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
