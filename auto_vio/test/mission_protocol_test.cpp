#include <cassert>
#include <cstdio>
#include "../mission_protocol.h"

using namespace protocol_v2;

static CalibrationV2 calibration(uint16_t epoch = 7, uint16_t id = 3, uint32_t command = 1) {
  return {0, epoch, command, id, -0.5f, 0.0f, 1};
}
static RectangleV2 rectangle(uint16_t epoch = 7, uint16_t id = 3, uint32_t command = 2) {
  return {6, epoch, command, 20.0f, 8.0f, 4.0f, 6.0f, id};
}
static PoseV2 pose(uint32_t sequence, uint32_t age = 20, uint16_t epoch = 7, uint16_t id = 3) {
  return {7, epoch, sequence, age, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, id};
}

static void configure(MissionProtocol &protocol, uint32_t now = 1000, bool pwmReady = true) {
  AckV2 ack = protocol.acceptCalibration(calibration(), now);
  assert(ack.state == S_IDLE && ack.faultCode == F_NONE);
  AckV2 duplicateCalibration = protocol.acceptCalibration(calibration(), now + 1);
  assert(duplicateCalibration.state == ack.state && duplicateCalibration.commandId == ack.commandId);
  ack = protocol.acceptRectangle(rectangle(), now);
  assert(ack.state == S_CONFIGURED && ack.faultCode == F_NONE);
  AckV2 duplicateRectangle = protocol.acceptRectangle(rectangle(), now + 1);
  assert(duplicateRectangle.state == ack.state && duplicateRectangle.commandId == ack.commandId);
  protocol.setPwmReady(pwmReady);
}

int main() {
  MissionProtocol radiusGate;
  radiusGate.setPwmReady(true);
  assert(radiusGate.acceptCalibration(calibration(), 900).faultCode == F_NONE);
  assert(radiusGate.acceptPose(pose(1, 10), 900));
  assert(radiusGate.acceptCommand({5, 7, 2}, 900).faultCode == F_CALIBRATION);

  MissionProtocol dryRadius;
  dryRadius.setPwmReady(true);
  assert(dryRadius.acceptCalibration(calibration(), 900).faultCode == F_NONE);
  assert(dryRadius.acceptPose(pose(1, 10), 900));
  AckV2 radiusAck = dryRadius.acceptCommand({5, 7, 2}, 900, true);
  assert(radiusAck.state == S_IDLE && radiusAck.faultCode == F_NONE);
  assert(dryRadius.acceptCommand({6, 7, 3}, 1200, true).faultCode == F_ROUTE);

  MissionProtocol hardwareGate;
  configure(hardwareGate, 1000, false);
  assert(hardwareGate.acceptPose(pose(1, 10), 1000));
  AckV2 hardwareAck = hardwareGate.acceptCommand({1, 7, 3}, 1000);
  assert(hardwareAck.state == S_CONFIGURED && hardwareAck.faultCode == F_PWM);

  MissionProtocol protocol;
  assert(protocol.state() == S_IDLE);
  assert(protocol.allowsLegacyDiagnostics());
  assert(!protocol.allowsLegacyArm());

  configure(protocol);
  assert(!protocol.allowsLegacyDiagnostics());

  // ARM requires a fresh, normal, calibration-valid pose from this epoch.
  AckV2 ack = protocol.acceptCommand({1, 7, 3}, 1000);
  assert(ack.state == S_CONFIGURED && ack.faultCode == F_POSE_TIMEOUT);
  PoseV2 stale = pose(1, 251);
  assert(!protocol.acceptPose(stale, 1000));
  PoseV2 degraded = pose(2);
  degraded.flags &= ~1;
  assert(!protocol.acceptPose(degraded, 1000));
  assert(protocol.acceptPose(pose(3, 250), 1000));
  assert(protocol.poseFresh(1000));
  assert(!protocol.poseFresh(1001));
  assert(protocol.acceptPose(pose(4, 20), 1010));

  ack = protocol.acceptCommand({1, 7, 4}, 1010);
  assert(ack.state == S_ARMED && ack.faultCode == F_NONE);
  AckV2 duplicate = protocol.acceptCommand({1, 7, 4}, 99999);
  assert(duplicate.state == ack.state && duplicate.commandId == ack.commandId &&
         duplicate.faultCode == ack.faultCode);
  assert(protocol.lastCommandWasDuplicate());
  ack = protocol.acceptCommand({2, 7, 5}, 1010);
  assert(ack.state == S_RUNNING && ack.faultCode == F_NONE);

  PoseV2 consumed = {};
  assert(protocol.takePose(consumed) && consumed.sequence == 4);
  assert(!protocol.takePose(consumed));
  assert(protocol.acceptPose(pose(5), 1020));
  assert(!protocol.acceptPose(pose(5), 1021));
  assert(protocol.lastPoseRejectFault() == F_POSE_INVALID);
  assert(!protocol.acceptPose(pose(4), 1022));

  PoseV2 tooFast = pose(6);
  tooFast.speedFps = 8.01f;
  assert(!protocol.acceptPose(tooFast, 1030));
  PoseV2 jumped = pose(7);
  jumped.x = 2.0f;
  assert(protocol.acceptPose(jumped, 1030));
  assert(protocol.lastPoseRejectFault() == F_NONE);
  PoseV2 headingJump = pose(8);
  headingJump.heading = 20.0f;
  assert(protocol.acceptPose(headingJump, 1040));
  assert(protocol.lastPoseRejectFault() == F_NONE);

  // STOP is always accepted, including epoch zero used as the safe v2 probe.
  ack = protocol.acceptCommand({3, 0, 99}, 1020);
  assert(ack.state == S_IDLE && ack.faultCode == F_NONE && ack.epoch == 0);
  assert(protocol.allowsLegacyDiagnostics());
  assert(protocol.acceptCalibration(calibration(7, 3, 8), 1500).faultCode == F_ROUTE);

  // Exact delayed retries replay their original ACK but are marked duplicate,
  // so hardware effects cannot run again after a later Stop.
  ack = protocol.acceptCommand({4, 7, 100}, 1600);
  assert(ack.faultCode == F_NONE);
  protocol_v2::AckV2 originalSelfTestAck = ack;
  ack = protocol.acceptCommand({3, 7, 101}, 1601);
  assert(ack.state == S_IDLE);
  ack = protocol.acceptCommand({4, 7, 100}, 1602);
  assert(protocol.lastCommandWasDuplicate());
  assert(ack.commandId == originalSelfTestAck.commandId &&
         ack.faultCode == originalSelfTestAck.faultCode);

  // A new epoch may replace old configuration only from IDLE.
  ack = protocol.acceptCalibration(calibration(8, 4, 10), 2000);
  assert(ack.faultCode == F_NONE && protocol.epoch() == 8);
  ack = protocol.acceptRectangle(rectangle(8, 4, 11), 2000);
  assert(ack.state == S_CONFIGURED);
  AckV2 rejected = protocol.acceptCalibration(calibration(9, 5, 12), 2000);
  assert(rejected.faultCode != F_NONE && protocol.epoch() == 8);

  // Wrong epoch/calibration and non-normal poses are never exposed to control.
  assert(!protocol.acceptPose(pose(20, 10, 7, 4), 2010));
  assert(!protocol.acceptPose(pose(21, 10, 8, 99), 2010));
  PoseV2 noCalibrationFlag = pose(22, 10, 8, 4);
  noCalibrationFlag.flags &= ~4;
  assert(!protocol.acceptPose(noCalibrationFlag, 2010));

  // Queue replacement is visible and only the newest accepted pose is consumed.
  assert(protocol.acceptPose(pose(23, 10, 8, 4), 2020));
  assert(protocol.acceptPose(pose(24, 10, 8, 4), 2030));
  assert(protocol.droppedPoses() == 1);
  assert(protocol.takePose(consumed) && consumed.sequence == 24);

  protocol.onDisconnect();
  assert(protocol.state() == S_FAULT && protocol.fault() == F_BLE);
  ack = protocol.acceptCommand({3, 8, 13}, 2040);
  assert(ack.state == S_IDLE);
  assert(protocol.acceptCalibration(calibration(7, 5, 14), 2050).faultCode == F_ROUTE);

  // The self test is the only command that reports which I2C failure is
  // actually happening, so nothing about a mission may be required to run it.
  // A rover whose PWM chip is not answering cannot produce a mission epoch in
  // the first place, which is what used to make this unreachable.
  MissionProtocol diagnostics;
  assert(diagnostics.state() == S_IDLE);
  diagnostics.setPwmReady(false);            // the condition being diagnosed
  AckV2 unconfigured = diagnostics.acceptCommand({4, 4242, 9}, 3000);
  assert(unconfigured.faultCode == F_NONE);  // never configured, epoch unknown

  // Available from S_FAULT too, like the fault dump it accompanies -- that is
  // the state an operator is in when they need it.
  diagnostics.setFault(F_PWM);
  assert(diagnostics.state() == S_FAULT);
  assert(diagnostics.acceptCommand({4, 4242, 10}, 3010).faultCode == F_NONE);

  // A rewound command id must not lock diagnostics out; the app restarting is
  // not a reason to refuse the servo test.
  assert(diagnostics.acceptCommand({4, 4242, 1}, 3020).faultCode == F_NONE);

  // It stays refused once the rover is under mission control, where motion is
  // the route's to command.
  MissionProtocol armedGate;
  configure(armedGate);
  assert(armedGate.acceptPose(pose(1), 1000));
  assert(armedGate.acceptCommand({1, 7, 3}, 1000).state == S_ARMED);
  assert(armedGate.acceptCommand({4, 7, 4}, 1000).faultCode == F_ROUTE);

  // Only the rover knows where a resumed pass begins. Refusing the ARM must be
  // retryable -- the operator moves the rover and arms again -- not a fault.
  MissionProtocol resume;
  configure(resume, 4000);
  assert(resume.acceptPose(pose(1, 20), 4000));
  resume.setStartPointReached(false);
  AckV2 refused = resume.acceptCommand({1, 7, 40}, 4000);
  assert(refused.faultCode == F_START_POINT && resume.state() == S_CONFIGURED);
  resume.setStartPointReached(true);
  AckV2 armed = resume.acceptCommand({1, 7, 41}, 4000);
  assert(armed.faultCode == F_NONE && resume.state() == S_ARMED);

  // A stationary rover still reports a small, sign-flipping speed estimate.
  // Sixty seconds of that noise must not look like a discontinuity: this is
  // the field symptom the additive floors exist to stop.
  MissionProtocol resting;
  resting.setPwmReady(true);
  configure(resting, 3000);
  uint32_t restingNow = 3000;
  for (uint32_t index = 0; index < 2000; ++index) {
    const uint32_t age = 10 + (index % 3) * 5;
    PoseV2 sample = pose(100 + index, age);
    sample.speedFps = (index % 2) ? 0.22f : -0.21f;
    sample.yawRateDps = (index % 2) ? 1.5f : -1.4f;
    sample.x = ((index % 4) - 1.5f) * 0.01f;
    sample.y = ((index % 3) - 1.0f) * 0.01f;
    sample.heading = (index % 2) ? 0.4f : 359.6f;
    restingNow += (index % 2) ? 16 : 17;
    assert(resting.acceptPose(sample, restingNow));
    assert(resting.lastPoseRejectFault() == F_NONE);
  }

  // Two packets landing inside the same millisecond are a BLE artefact, not a
  // teleport; the capture-time floor keeps them usable.
  PoseV2 bunchedFirst = pose(9000, 20);
  bunchedFirst.speedFps = 0.3f;
  assert(resting.acceptPose(bunchedFirst, restingNow + 20));
  PoseV2 bunchedSecond = pose(9001, 20);
  bunchedSecond.speedFps = 0.34f;
  bunchedSecond.x = 0.02f;
  assert(resting.acceptPose(bunchedSecond, restingNow + 20));
  assert(resting.lastPoseRejectFault() == F_NONE);

  // Genuine relocalisation snaps and velocity lurches remain usable poses.
  PoseV2 snap = pose(9002, 20);
  snap.x = 4.0f;
  assert(resting.acceptPose(snap, restingNow + 40));
  assert(resting.lastPoseRejectFault() == F_NONE);
  PoseV2 lurch = pose(9003, 20);
  lurch.speedFps = 4.0f;
  assert(resting.acceptPose(lurch, restingNow + 60));
  assert(resting.lastPoseRejectFault() == F_NONE);
  assert(resting.poseFresh(restingNow + 60));
  assert(!resting.poseFresh(restingNow + 60 + 231));

  std::printf("mission_protocol_test: all assertions passed\n");
  return 0;
}
