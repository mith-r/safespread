#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include "../protocol_v2.h"

using namespace protocol_v2;

static const uint8_t POSE_FIXTURE[POSE_SIZE] = {
  0x21,0x56,0x02,0x07,0x34,0x12,0x04,0x03,0x02,0x01,0x7d,0x00,0x00,0x00,0xc0,0x3f,
  0x00,0x00,0x10,0xc0,0x00,0xc0,0xb3,0x43,0x85,0xff,0xd7,0x11,0xef,0xbe,0xb7,0xe9
};
static const uint8_t RECTANGLE_FIXTURE[RECTANGLE_SIZE] = {
  0x21,0x44,0x02,0x05,0x04,0x00,0x09,0x00,0x00,0x00,0x00,0x00,0xa0,0x41,0x00,0x00,
  0x00,0x41,0x00,0x00,0x80,0x40,0x00,0x00,0xc0,0x40,0x03,0x00,0x00,0x00,0xe9,0xc0
};
static const uint8_t CALIBRATION_FIXTURE[CALIBRATION_SIZE] = {
  0x21,0x4b,0x02,0x00,0x04,0x00,0x0a,0x00,0x00,0x00,0x03,0x00,
  0x00,0x00,0x00,0xbf,0x00,0x00,0x80,0x3e,0x01,0x00,0x78,0xa5
};
static const uint8_t COMMAND_FIXTURE[COMMAND_SIZE] = {
  0x21,0x43,0x02,0x08,0x04,0x00,0x0b,0x00,0x00,0x00,0x59,0xc1
};

static void rewriteCrc(uint8_t *bytes, size_t size) {
  const uint16_t crc = crc16Ccitt(bytes, size - 2);
  bytes[size - 2] = static_cast<uint8_t>(crc & 0xff);
  bytes[size - 1] = static_cast<uint8_t>(crc >> 8);
}

int main() {
  PoseV2 pose = {};
  assert(parsePoseV2(POSE_FIXTURE, sizeof(POSE_FIXTURE), pose));
  assert(pose.flags == 7);
  assert(pose.epoch == 0x1234);
  assert(pose.sequence == 0x01020304);
  assert(pose.ageMs == 125);
  assert(pose.x == 1.5f);
  assert(pose.y == -2.25f);
  assert(pose.heading == 359.5f);
  assert(std::fabs(pose.speedFps - -1.23f) < 0.0001f);
  assert(std::fabs(pose.yawRateDps - 45.67f) < 0.0001f);
  assert(pose.calibrationId == 0xbeef);

  uint8_t builtPose[POSE_SIZE];
  assert(buildPoseV2(pose, builtPose, sizeof(builtPose)));
  assert(std::memcmp(builtPose, POSE_FIXTURE, sizeof(builtPose)) == 0);

  RectangleV2 rectangle = {5, 4, 9, 20.0f, 8.0f, 4.0f, 6.0f, 3};
  uint8_t builtRectangle[RECTANGLE_SIZE];
  assert(buildRectangleV2(rectangle, builtRectangle, sizeof(builtRectangle)));
  assert(std::memcmp(builtRectangle, RECTANGLE_FIXTURE, sizeof(builtRectangle)) == 0);

  // A resume carries the pass to start on in what used to be reserved space,
  // so an old-style packet (zeroes) still means "drive the whole rectangle".
  RectangleV2 resumed = {5, 4, 9, 20.0f, 8.0f, 4.0f, 6.0f, 3, 4};
  uint8_t builtResume[RECTANGLE_SIZE];
  assert(buildRectangleV2(resumed, builtResume, sizeof(builtResume)));
  RectangleV2 parsedResume = {};
  assert(parseRectangleV2(builtResume, sizeof(builtResume), parsedResume));
  assert(parsedResume.startPassIndex == 4 && parsedResume.calibrationId == 3);
  RectangleV2 parsedFixture = {};
  assert(parseRectangleV2(RECTANGLE_FIXTURE, RECTANGLE_SIZE, parsedFixture));
  assert(parsedFixture.startPassIndex == 0);

  CalibrationV2 calibration = {0, 4, 10, 3, -0.5f, 0.25f, 1};
  uint8_t builtCalibration[CALIBRATION_SIZE];
  assert(buildCalibrationV2(calibration, builtCalibration, sizeof(builtCalibration)));
  assert(std::memcmp(builtCalibration, CALIBRATION_FIXTURE, sizeof(builtCalibration)) == 0);

  CommandV2 command = {8, 4, 11};
  uint8_t builtCommand[COMMAND_SIZE];
  assert(buildCommandV2(command, builtCommand, sizeof(builtCommand)));
  assert(std::memcmp(builtCommand, COMMAND_FIXTURE, sizeof(builtCommand)) == 0);

  PoseV2 sentinel = {};
  sentinel.sequence = 77;
  sentinel.x = 88.0f;
  uint8_t invalid[POSE_SIZE];
  std::memcpy(invalid, POSE_FIXTURE, sizeof(invalid));
  invalid[12] ^= 1;
  assert(!parsePoseV2(invalid, sizeof(invalid), sentinel));
  assert(sentinel.sequence == 77 && sentinel.x == 88.0f);

  std::memcpy(invalid, POSE_FIXTURE, sizeof(invalid));
  invalid[2] = 1;
  rewriteCrc(invalid, sizeof(invalid));
  assert(!parsePoseV2(invalid, sizeof(invalid), sentinel));

  std::memcpy(invalid, POSE_FIXTURE, sizeof(invalid));
  invalid[12] = 0x00;
  invalid[13] = 0x00;
  invalid[14] = 0xc0;
  invalid[15] = 0x7f;
  rewriteCrc(invalid, sizeof(invalid));
  assert(!parsePoseV2(invalid, sizeof(invalid), sentinel));

  AckV2 ack = {5, 0x1234, 0x01020304, 9, 0xbeef};
  uint8_t ackBytes[ACK_SIZE];
  assert(buildAckV2(ack, ackBytes, sizeof(ackBytes)));
  AckV2 parsedAck = {};
  assert(parseAckV2(ackBytes, sizeof(ackBytes), parsedAck));
  assert(parsedAck.state == 5 && parsedAck.faultCode == 9);

  TelemetryV2 telemetry = {3, 7, 99, 4, 18, -0.25f, 12.5f, -0.75f,
                           1510, 1420, 5, 0, 12, 80};
  uint8_t telemetryBytes[TELEMETRY_SIZE];
  assert(buildTelemetryV2(telemetry, telemetryBytes, sizeof(telemetryBytes)));
  TelemetryV2 parsedTelemetry = {};
  assert(parseTelemetryV2(telemetryBytes, sizeof(telemetryBytes), parsedTelemetry));
  assert(parsedTelemetry.routeIndex == 4);
  assert(std::fabs(parsedTelemetry.crossTrackFt - -0.25f) < 0.0001f);

  FaultSampleV2 sample = {3, 7, 1234, 4, 5, 8, -0.1f, 2.25f, 0.8f,
                          1490, 1540, 5, 2, 3};
  uint8_t sampleBytes[FAULT_SAMPLE_SIZE];
  assert(buildFaultSampleV2(sample, sampleBytes, sizeof(sampleBytes)));
  FaultSampleV2 parsedSample = {};
  assert(parseFaultSampleV2(sampleBytes, sizeof(sampleBytes), parsedSample));
  assert(parsedSample.sampleIndex == 4 && parsedSample.sampleCount == 5);
  assert(std::fabs(parsedSample.headingErrorDeg - 2.25f) < 0.0001f);

  PoseV2 invalidPose = pose;
  invalidPose.x = std::numeric_limits<float>::quiet_NaN();
  uint8_t untouched[POSE_SIZE];
  std::memset(untouched, 0xa5, sizeof(untouched));
  assert(!buildPoseV2(invalidPose, untouched, sizeof(untouched)));
  for (size_t i = 0; i < sizeof(untouched); ++i) assert(untouched[i] == 0xa5);

  assert(!parsePoseV2(POSE_FIXTURE, POSE_SIZE - 1, sentinel));
  std::printf("protocol_v2_test: all assertions passed\n");
  return 0;
}
