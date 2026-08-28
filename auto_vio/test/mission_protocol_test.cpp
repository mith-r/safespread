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
  assert(!protocol.acceptPose(pose(5), 1021));   // duplicate: dropped, not blamed
  assert(protocol.lastPoseRejectFault() == F_NONE);
  assert(!protocol.acceptPose(pose(4), 1022));   // regression: dropped, not blamed
  assert(protocol.lastPoseRejectFault() == F_NONE);

  PoseV2 tooFast = pose(6);
  tooFast.speedFps = 8.01f;
  assert(!protocol.acceptPose(tooFast, 1030));
  PoseV2 jumped = pose(7);
  jumped.x = 2.0f;
  assert(!protocol.acceptPose(jumped, 1030));
  assert(protocol.lastPoseRejectFault() == F_POSE_JUMP);
  PoseV2 headingJump = pose(8);
  headingJump.heading = 20.0f;
  assert(!protocol.acceptPose(headingJump, 1040));
  assert(protocol.lastPoseRejectFault() == F_POSE_JUMP);

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

  // Accepting a rectangle changes the coordinate frame poses arrive in (the app
  // streams the rover's raw world pose until Configure, then switches to
  // rectangle-relative coordinates). That one-time shift is larger than the
  // per-sample innovation, so acceptRectangle must clear the pose baseline the
  // way acceptCalibration does; otherwise the first pose of the mission reads as
  // F_POSE_JUMP and the mission faults at its first step.
  MissionProtocol frameSwitch;
  assert(frameSwitch.acceptCalibration(calibration(), 3000).faultCode == F_NONE);
  PoseV2 worldPose = pose(1, 20);
  worldPose.y = -2.0f;   // rover's raw VIO position before it is configured
  assert(frameSwitch.acceptPose(worldPose, 3000));
  AckV2 rectAck = frameSwitch.acceptRectangle(rectangle(), 3010);
  assert(rectAck.state == S_CONFIGURED && rectAck.faultCode == F_NONE);
  PoseV2 framePose = pose(2, 20);
  framePose.y = 0.0f;    // same rover, now at the rectangle origin: a 2 ft shift
  assert(frameSwitch.acceptPose(framePose, 3020));
  assert(frameSwitch.lastPoseRejectFault() == F_NONE);
  // A genuine jump within the rectangle frame is still rejected afterwards.
  PoseV2 realJump = pose(3, 20);
  realJump.y = 4.0f;
  assert(!frameSwitch.acceptPose(realJump, 3030));
  assert(frameSwitch.lastPoseRejectFault() == F_POSE_JUMP);

  // Transport artifacts are dropped without a fault code, so a coalesced BLE
  // write cannot fault an armed mission: duplicates/regressions, same-tick
  // pairs, and stale ages reject with F_NONE. Real corruption still blames
  // the stream (checked above: accel, position and heading jumps).
  MissionProtocol artifacts;
  assert(artifacts.acceptCalibration(calibration(), 5000).faultCode == F_NONE);
  assert(artifacts.acceptPose(pose(10, 20), 5000));
  assert(!artifacts.acceptPose(pose(10, 20), 5016));           // duplicate sequence
  assert(artifacts.lastPoseRejectFault() == F_NONE);
  assert(!artifacts.acceptPose(pose(9, 20), 5016));            // regression
  assert(artifacts.lastPoseRejectFault() == F_NONE);
  assert(!artifacts.acceptPose(pose(11, 20), 5000));           // same-tick pair
  assert(artifacts.lastPoseRejectFault() == F_NONE);
  assert(!artifacts.acceptPose(pose(12, 251), 5030));          // stale age
  assert(artifacts.lastPoseRejectFault() == F_NONE);
  assert(artifacts.acceptPose(pose(13, 20), 5032));            // stream continues
  assert(artifacts.poseFresh(5032));

  // Jump gates divide by capture spacing, not arrival spacing: coalescing can
  // deliver poses captured a frame apart within a couple of milliseconds, and
  // arrival-time dt turned a creeping rover's first real speed change into an
  // impossible-acceleration F_POSE_JUMP.
  MissionProtocol coalesced;
  assert(coalesced.acceptCalibration(calibration(), 6000).faultCode == F_NONE);
  PoseV2 slow = pose(20, 120);                 // captured at 5880
  assert(coalesced.acceptPose(slow, 6000));
  PoseV2 burst = pose(21, 20);                 // captured at 5982, arrives 2 ms later
  burst.speedFps = 0.9f;                       // 8.8 fps^2 over capture time
  assert(coalesced.acceptPose(burst, 6002));   // arrival-dt math would read 450 fps^2
  PoseV2 teleport = pose(22, 20);              // captured at 6080
  teleport.speedFps = 6.0f;                    // 52 fps^2 over capture time
  assert(!coalesced.acceptPose(teleport, 6100));
  assert(coalesced.lastPoseRejectFault() == F_POSE_JUMP);

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

  std::printf("mission_protocol_test: all assertions passed\n");
  return 0;
}
