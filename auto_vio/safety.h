#ifndef SAFESPREAD_SAFETY_H
#define SAFESPREAD_SAFETY_H

#include <cstdint>

enum MissionState : uint8_t {
  S_IDLE = 0,
  S_CONFIGURED = 1,
  S_ARMED = 2,
  S_RUNNING = 3,
  S_COMPLETE = 4,
  S_FAULT = 5,
};

enum FaultCode : uint8_t {
  F_NONE = 0,
  F_BLE = 1,
  F_POSE_TIMEOUT = 2,
  // Reserved for protocol compatibility with older app/firmware builds.
  // Invalid pose packets are dropped and can only lead to F_POSE_TIMEOUT if
  // valid tracking does not resume; they never become a mission fault.
  F_POSE_INVALID = 3,
  F_POSE_JUMP = 4,
  F_PWM = 5,
  F_I2C = 6,
  F_STALL = 7,
  F_WRONG_DIRECTION = 8,
  F_TRACKING_ERROR = 9,
  F_ROUTE = 10,
  F_CALIBRATION = 11,
  F_HEADLAND = 12,
  // Not a mission fault: the rover refuses to arm because it is not standing
  // on the pass the operator asked to resume from. Move it and arm again.
  F_START_POINT = 13,
};

struct SafetyInput {
  bool bleConnected;
  bool poseFresh;
  bool poseValid;
  bool poseJumped;
  bool pwmReady;
  bool i2cReady;
  bool stalled;
  bool wrongDirection;
  bool trackingNormal;
  bool routeValid;
  bool calibrationValid;
  bool headlandValid;
};

inline FaultCode evaluateSafety(MissionState state, const SafetyInput &input) {
  if (state == S_IDLE || state == S_COMPLETE || state == S_FAULT) return F_NONE;
  if (!input.bleConnected) return F_BLE;
  if (!input.poseFresh) return F_POSE_TIMEOUT;
  // Pose discontinuities are accepted. Fault 4 remains reserved for protocol
  // compatibility but is not a runtime safety stop.
  if (!input.pwmReady) return F_PWM;
  if (!input.i2cReady) return F_I2C;
  if (input.stalled) return F_STALL;
  if (input.wrongDirection) return F_WRONG_DIRECTION;
  if (!input.trackingNormal) return F_TRACKING_ERROR;
  if (!input.routeValid) return F_ROUTE;
  if (!input.calibrationValid) return F_CALIBRATION;
  if (!input.headlandValid) return F_HEADLAND;
  return F_NONE;
}

inline bool rejectedPoseRequiresFault(FaultCode fault) {
  return fault != F_NONE && fault != F_POSE_INVALID && fault != F_POSE_JUMP;
}

enum MissionEvent : uint8_t {
  E_CONFIGURE,
  E_ARM,
  E_START,
  E_COMPLETE,
  E_STOP,
  E_SAFETY_FAULT,
};

struct TransitionResult {
  MissionState state;
  FaultCode fault;
  bool accepted;
};

inline TransitionResult transitionMission(
    MissionState state, MissionEvent event, FaultCode fault) {
  if (event == E_STOP) return {S_IDLE, F_NONE, true};
  if (event == E_SAFETY_FAULT && fault != F_NONE) return {S_FAULT, fault, true};
  if (state == S_IDLE && event == E_CONFIGURE) return {S_CONFIGURED, F_NONE, true};
  if (state == S_CONFIGURED && event == E_ARM) return {S_ARMED, F_NONE, true};
  if (state == S_ARMED && event == E_START) return {S_RUNNING, F_NONE, true};
  if (state == S_RUNNING && event == E_COMPLETE) return {S_COMPLETE, F_NONE, true};
  return {state, state == S_FAULT ? fault : F_NONE, false};
}

#endif
