#include <cassert>
#include <cstdio>
#include "../mission_protocol.h"
#include "../route.h"

using namespace protocol_v2;

static_assert(INITIAL_RUN_IN_FT == 1.0f, "direction verification needs a one-foot run-in");
static_assert(MissionProtocol::EXPECTED_START_Y_FT == -INITIAL_RUN_IN_FT,
              "ARM staging pose must match the route's initial run-in");

static CalibrationV2 calibration(uint16_t epoch = 7, uint16_t id = 3, uint32_t command = 1) {
  return {0, epoch, command, id, -0.5f, 0.0f, 1};
}
static RectangleV2 rectangle(uint16_t epoch = 7, uint16_t id = 3, uint32_t command = 2) {
  return {6, epoch, command, 20.0f, 8.0f, 4.0f, 6.0f, id};
}
static PoseV2 pose(uint32_t sequence, uint32_t age = 20, uint16_t epoch = 7, uint16_t id = 3) {
  return {
    7, epoch, sequence, age,
    MissionProtocol::EXPECTED_START_X_FT,
    MissionProtocol::EXPECTED_START_Y_FT,
    MissionProtocol::EXPECTED_START_HEADING_DEG,
    0.0f, 0.0f, id
  };
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
  MissionProtocol calibrationGate;
  calibrationGate.setPwmReady(true);
  assert(calibrationGate.acceptCalibration(calibration(), 900).faultCode == F_NONE);
  assert(calibrationGate.acceptPose(pose(1, 10), 900));
  assert(calibrationGate.acceptCommand({5, 7, 2}, 900).faultCode == F_CALIBRATION);

  MissionProtocol dryCalibration;
  dryCalibration.setPwmReady(true);
  assert(dryCalibration.acceptCalibration(calibration(), 900).faultCode == F_NONE);
  assert(dryCalibration.acceptPose(pose(1, 10), 900));
  AckV2 calibrationAck = dryCalibration.acceptCommand({5, 7, 2}, 900, true);
  assert(calibrationAck.state == S_IDLE && calibrationAck.faultCode == F_NONE);
  assert(dryCalibration.acceptCommand({6, 7, 3}, 1200, true).faultCode == F_POSE_TIMEOUT);

  MissionProtocol drySelfTest;
  drySelfTest.setPwmReady(true);
  assert(drySelfTest.acceptCalibration(calibration(), 900).faultCode == F_NONE);
  assert(drySelfTest.acceptCommand({4, 7, 2}, 900, true).faultCode == F_POSE_TIMEOUT);
  assert(drySelfTest.acceptPose(pose(1, 10), 900));
  assert(drySelfTest.acceptCommand({4, 7, 3}, 900).faultCode == F_CALIBRATION);
  assert(drySelfTest.acceptCommand({4, 7, 4}, 900, true).faultCode == F_NONE);

  MissionProtocol hardwareGate;
  configure(hardwareGate, 1000, false);
  assert(hardwareGate.acceptPose(pose(1, 10), 1000));
  AckV2 hardwareAck = hardwareGate.acceptCommand({1, 7, 3}, 1000);
  assert(hardwareAck.state == S_CONFIGURED && hardwareAck.faultCode == F_PWM);

  // Rectangle acceptance is a coordinate-frame boundary. A pose queued before
  // it cannot satisfy ARM or reach control, and even a delayed replay of that
  // sequence remains invalid. The app must provide a newer rectangle-frame
  // pose; an exact rectangle retry must not discard that new pose again.
  MissionProtocol rectangleFrameGate;
  rectangleFrameGate.setPwmReady(true);
  assert(rectangleFrameGate.acceptCalibration(calibration(), 1000).faultCode == F_NONE);
  PoseV2 beforeRectangle = pose(1, 10);
  assert(rectangleFrameGate.acceptPose(beforeRectangle, 1000));
  assert(rectangleFrameGate.acceptRectangle(rectangle(), 1001).faultCode == F_NONE);
  PoseV2 discarded = {};
  assert(!rectangleFrameGate.poseFresh(1001));
  assert(!rectangleFrameGate.takePose(discarded));
  AckV2 frameAck = rectangleFrameGate.acceptCommand({1, 7, 3}, 1001);
  assert(frameAck.state == S_CONFIGURED && frameAck.faultCode == F_POSE_TIMEOUT);
  assert(!rectangleFrameGate.acceptPose(beforeRectangle, 1002));
  assert(rectangleFrameGate.lastPoseRejectFault() == F_POSE_INVALID);
  assert(rectangleFrameGate.acceptPose(pose(2, 10), 1002));
  assert(rectangleFrameGate.acceptRectangle(rectangle(), 1003).faultCode == F_NONE);
  assert(rectangleFrameGate.lastSetupWasDuplicate());
  assert(rectangleFrameGate.poseFresh(1003));
  frameAck = rectangleFrameGate.acceptCommand({1, 7, 4}, 1003);
  assert(frameAck.state == S_ARMED && frameAck.faultCode == F_NONE);

  // ARM requires a fresh rectangle-frame pose inside the configurable staging
  // position/heading envelope and with negligible measured speed.
  MissionProtocol displacedArmGate;
  configure(displacedArmGate);
  PoseV2 displacedStart = pose(1);
  displacedStart.x += MissionProtocol::START_POSITION_TOLERANCE_FT + 0.01f;
  assert(displacedArmGate.acceptPose(displacedStart, 1000));
  AckV2 startGateAck = displacedArmGate.acceptCommand({1, 7, 3}, 1000);
  assert(startGateAck.state == S_CONFIGURED && startGateAck.faultCode == F_TRACKING_ERROR);

  MissionProtocol headingArmGate;
  configure(headingArmGate);
  PoseV2 misalignedStart = pose(1);
  misalignedStart.heading += MissionProtocol::START_HEADING_TOLERANCE_DEG + 0.01f;
  assert(headingArmGate.acceptPose(misalignedStart, 1000));
  startGateAck = headingArmGate.acceptCommand({1, 7, 3}, 1000);
  assert(startGateAck.state == S_CONFIGURED && startGateAck.faultCode == F_TRACKING_ERROR);

  MissionProtocol movingArmGate;
  configure(movingArmGate);
  PoseV2 movingStart = pose(1);
  movingStart.speedFps = MissionProtocol::START_MAX_SPEED_FPS + 0.01f;
  assert(movingArmGate.acceptPose(movingStart, 1000));
  startGateAck = movingArmGate.acceptCommand({1, 7, 3}, 1000);
  assert(startGateAck.state == S_CONFIGURED && startGateAck.faultCode == F_TRACKING_ERROR);

  // A rover that moves after ARM is rejected with a tracking fault. START is
  // also latched out if the caller has not yet converted that rejection into
  // the firmware's hardware fault transition.
  MissionProtocol armedDisplacementGate;
  configure(armedDisplacementGate);
  assert(armedDisplacementGate.acceptPose(pose(1), 1000));
  assert(armedDisplacementGate.acceptCommand({1, 7, 3}, 1000).faultCode == F_NONE);
  PoseV2 armedDisplacement = pose(2);
  armedDisplacement.y += MissionProtocol::START_POSITION_TOLERANCE_FT + 0.01f;
  assert(!armedDisplacementGate.acceptPose(armedDisplacement, 1010));
  assert(armedDisplacementGate.lastPoseRejectFault() == F_TRACKING_ERROR);
  assert(armedDisplacementGate.state() == S_ARMED);
  startGateAck = armedDisplacementGate.acceptCommand({2, 7, 4}, 1010);
  assert(startGateAck.state == S_ARMED && startGateAck.faultCode == F_TRACKING_ERROR);

  MissionProtocol armedSpeedGate;
  configure(armedSpeedGate);
  assert(armedSpeedGate.acceptPose(pose(1), 1000));
  assert(armedSpeedGate.acceptCommand({1, 7, 3}, 1000).faultCode == F_NONE);
  PoseV2 armedMotion = pose(2);
  armedMotion.speedFps = MissionProtocol::START_MAX_SPEED_FPS + 0.01f;
  assert(!armedSpeedGate.acceptPose(armedMotion, 1010));
  assert(armedSpeedGate.lastPoseRejectFault() == F_TRACKING_ERROR);
  startGateAck = armedSpeedGate.acceptCommand({2, 7, 4}, 1010);
  assert(startGateAck.state == S_ARMED && startGateAck.faultCode == F_TRACKING_ERROR);

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
  jumped.x = MissionProtocol::EXPECTED_START_X_FT + 2.0f;
  assert(!protocol.acceptPose(jumped, 1030));
  assert(protocol.lastPoseRejectFault() == F_POSE_JUMP);
  PoseV2 headingJump = pose(8);
  headingJump.heading = MissionProtocol::EXPECTED_START_HEADING_DEG + 20.0f;
  assert(!protocol.acceptPose(headingJump, 1040));
  assert(protocol.lastPoseRejectFault() == F_POSE_JUMP);

  // Innovation timing follows camera capture time rather than BLE arrival
  // jitter. This speed change spans 250 ms at capture even though notifications
  // arrived only 100 ms apart, so it remains under the acceleration limit.
  MissionProtocol jittered;
  configure(jittered);
  PoseV2 delayedFirst = pose(1, 200);
  assert(jittered.acceptPose(delayedFirst, 1000));
  PoseV2 quickSecond = pose(2, 50);
  quickSecond.speedFps = 2.0f;
  assert(jittered.acceptPose(quickSecond, 1100));

  MissionProtocol repeatedCapture;
  configure(repeatedCapture);
  assert(repeatedCapture.acceptPose(pose(1, 20), 1000));
  assert(!repeatedCapture.acceptPose(pose(2, 220), 1200));
  assert(repeatedCapture.lastPoseRejectFault() == F_POSE_INVALID);

  // STOP is always accepted, including epoch zero used as the safe v2 probe.
  ack = protocol.acceptCommand({3, 0, 99}, 1020);
  assert(ack.state == S_IDLE && ack.faultCode == F_NONE && ack.epoch == 0);
  assert(protocol.allowsLegacyDiagnostics());
  assert(protocol.acceptCalibration(calibration(7, 3, 8), 1500).faultCode == F_ROUTE);

  // Exact delayed retries replay their original ACK but are marked duplicate,
  // so hardware effects cannot run again after a later Stop.
  assert(protocol.acceptCalibration(calibration(8, 3, 100), 1600).faultCode == F_NONE);
  assert(protocol.acceptPose(pose(1, 10, 8, 3), 1600));
  ack = protocol.acceptCommand({4, 8, 101}, 1600, true);
  assert(ack.faultCode == F_NONE);
  protocol_v2::AckV2 originalSelfTestAck = ack;
  ack = protocol.acceptCommand({3, 8, 102}, 1601);
  assert(ack.state == S_IDLE);
  ack = protocol.acceptCommand({4, 8, 101}, 1602, true);
  assert(protocol.lastCommandWasDuplicate());
  assert(ack.commandId == originalSelfTestAck.commandId &&
         ack.faultCode == originalSelfTestAck.faultCode);

  // A new epoch may replace old configuration only from IDLE.
  ack = protocol.acceptCalibration(calibration(9, 4, 110), 2000);
  assert(ack.faultCode == F_NONE && protocol.epoch() == 9);
  ack = protocol.acceptRectangle(rectangle(9, 4, 111), 2000);
  assert(ack.state == S_CONFIGURED);
  AckV2 rejected = protocol.acceptCalibration(calibration(10, 5, 112), 2000);
  assert(rejected.faultCode != F_NONE && protocol.epoch() == 9);

  // Wrong epoch/calibration and non-normal poses are never exposed to control.
  assert(!protocol.acceptPose(pose(20, 10, 8, 4), 2010));
  assert(!protocol.acceptPose(pose(21, 10, 9, 99), 2010));
  PoseV2 noCalibrationFlag = pose(22, 10, 9, 4);
  noCalibrationFlag.flags &= ~4;
  assert(!protocol.acceptPose(noCalibrationFlag, 2010));

  // Queue replacement is visible and only the newest accepted pose is consumed.
  assert(protocol.acceptPose(pose(23, 10, 9, 4), 2020));
  assert(protocol.acceptPose(pose(24, 10, 9, 4), 2030));
  assert(protocol.droppedPoses() == 1);
  assert(protocol.takePose(consumed) && consumed.sequence == 24);

  protocol.onDisconnect();
  assert(protocol.state() == S_FAULT && protocol.fault() == F_BLE);
  ack = protocol.acceptCommand({3, 9, 113}, 2040);
  assert(ack.state == S_IDLE);
  assert(protocol.acceptCalibration(calibration(8, 5, 114), 2050).faultCode == F_ROUTE);

  std::printf("mission_protocol_test: all assertions passed\n");
  return 0;
}
