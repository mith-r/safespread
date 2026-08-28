#include <cassert>
#include <cstdio>
#include "../safety.h"

static SafetyInput safeInput() {
  SafetyInput in = {};
  in.bleConnected = true;
  in.poseFresh = true;
  in.poseValid = true;
  in.poseJumped = false;
  in.pwmReady = true;
  in.i2cReady = true;
  in.stalled = false;
  in.wrongDirection = false;
  in.trackingNormal = true;
  in.routeValid = true;
  in.calibrationValid = true;
  in.headlandValid = true;
  return in;
}

int main() {
  MissionState state = S_IDLE;
  TransitionResult result = transitionMission(state, E_CONFIGURE, F_NONE);
  assert(result.accepted && result.state == S_CONFIGURED);
  result = transitionMission(result.state, E_ARM, F_NONE);
  assert(result.accepted && result.state == S_ARMED);
  result = transitionMission(result.state, E_START, F_NONE);
  assert(result.accepted && result.state == S_RUNNING);
  result = transitionMission(result.state, E_COMPLETE, F_NONE);
  assert(result.accepted && result.state == S_COMPLETE);

  for (int raw = S_IDLE; raw <= S_FAULT; ++raw) {
    result = transitionMission(static_cast<MissionState>(raw), E_STOP, F_NONE);
    assert(result.accepted && result.state == S_IDLE);
  }
  result = transitionMission(S_IDLE, E_START, F_NONE);
  assert(!result.accepted && result.state == S_IDLE);
  result = transitionMission(S_CONFIGURED, E_START, F_NONE);
  assert(!result.accepted && result.state == S_CONFIGURED);
  result = transitionMission(S_RUNNING, E_SAFETY_FAULT, F_STALL);
  assert(result.accepted && result.state == S_FAULT && result.fault == F_STALL);

  SafetyInput in = safeInput();
  assert(evaluateSafety(S_RUNNING, in) == F_NONE);
  in.bleConnected = false; assert(evaluateSafety(S_RUNNING, in) == F_BLE); in = safeInput();
  in.poseFresh = false; assert(evaluateSafety(S_RUNNING, in) == F_POSE_TIMEOUT); in = safeInput();
  // Invalid packets are ignored; loss of a usable stream is handled by the
  // independent freshness timeout instead of an immediate fault 3.
  in.poseValid = false; assert(evaluateSafety(S_RUNNING, in) == F_NONE); in = safeInput();
  assert(!rejectedPoseRequiresFault(F_NONE));
  assert(!rejectedPoseRequiresFault(F_POSE_INVALID));
  assert(!rejectedPoseRequiresFault(F_POSE_JUMP));
  in.poseJumped = true; assert(evaluateSafety(S_RUNNING, in) == F_NONE); in = safeInput();
  in.pwmReady = false; assert(evaluateSafety(S_RUNNING, in) == F_PWM); in = safeInput();
  in.i2cReady = false; assert(evaluateSafety(S_RUNNING, in) == F_I2C); in = safeInput();
  in.stalled = true; assert(evaluateSafety(S_RUNNING, in) == F_STALL); in = safeInput();
  in.wrongDirection = true; assert(evaluateSafety(S_RUNNING, in) == F_WRONG_DIRECTION); in = safeInput();
  in.trackingNormal = false; assert(evaluateSafety(S_RUNNING, in) == F_TRACKING_ERROR); in = safeInput();
  in.routeValid = false; assert(evaluateSafety(S_RUNNING, in) == F_ROUTE); in = safeInput();
  in.calibrationValid = false; assert(evaluateSafety(S_RUNNING, in) == F_CALIBRATION); in = safeInput();
  in.headlandValid = false; assert(evaluateSafety(S_RUNNING, in) == F_HEADLAND);

  // Idle diagnostics do not fault merely because drive prerequisites are absent.
  assert(evaluateSafety(S_IDLE, SafetyInput{}) == F_NONE);
  std::printf("safety_test: all assertions passed\n");
  return 0;
}
