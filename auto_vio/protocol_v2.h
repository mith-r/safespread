#ifndef SAFESPREAD_PROTOCOL_V2_H
#define SAFESPREAD_PROTOCOL_V2_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace protocol_v2 {

constexpr uint8_t VERSION = 2;
constexpr uint8_t MAGIC = 0x21;
constexpr size_t POSE_SIZE = 32;
constexpr size_t RECTANGLE_SIZE = 32;
constexpr size_t CALIBRATION_SIZE = 24;
constexpr size_t COMMAND_SIZE = 12;
constexpr size_t ACK_SIZE = 16;
constexpr size_t TELEMETRY_SIZE = 32;
constexpr size_t FAULT_SAMPLE_SIZE = 32;

struct PoseV2 {
  uint8_t flags;
  uint16_t epoch;
  uint32_t sequence;
  uint32_t ageMs;
  float x;
  float y;
  float heading;
  float speedFps;
  float yawRateDps;
  uint16_t calibrationId;
};

struct RectangleV2 {
  uint8_t flags;
  uint16_t epoch;
  uint32_t commandId;
  float mFt;
  float nFt;
  float startClearFt;
  float endClearFt;
  uint16_t calibrationId;
  // Sprayed pass to begin on, counted in route order from zero. A fault used
  // to cost the whole rectangle because every restart began at pass zero;
  // this lets the operator re-drive only what is left. Zero is the old
  // behaviour, so an unset field means "from the start".
  uint16_t startPassIndex;
};

struct CalibrationV2 {
  uint8_t flags;
  uint16_t epoch;
  uint32_t commandId;
  uint16_t calibrationId;
  float sprayForwardFt;
  float sprayRightFt;
  uint16_t schemaVersion;
};

struct CommandV2 {
  uint8_t opcode;
  uint16_t epoch;
  uint32_t commandId;
};

struct AckV2 {
  uint8_t state;
  uint16_t epoch;
  uint32_t commandId;
  uint16_t faultCode;
  uint16_t calibrationId;
};

struct TelemetryV2 {
  uint8_t state;
  uint16_t epoch;
  uint32_t consumedPoseSequence;
  uint16_t routeIndex;
  uint16_t routeCount;
  float crossTrackFt;
  float headingErrorDeg;
  float speedFps;
  uint16_t steeringUs;
  uint16_t throttleUs;
  uint8_t flags;
  uint8_t faultCode;
  uint16_t droppedPackets;
  uint16_t poseAgeMs;
};

struct FaultSampleV2 {
  uint8_t flags;
  uint16_t epoch;
  uint32_t sequence;
  uint16_t sampleIndex;
  uint16_t sampleCount;
  uint16_t routeIndex;
  float crossTrackFt;
  float headingErrorDeg;
  float speedFps;
  uint16_t steeringUs;
  uint16_t throttleUs;
  uint8_t state;
  uint8_t faultCode;
  uint16_t droppedPackets;
};

inline uint16_t crc16Ccitt(const uint8_t *bytes, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(bytes[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

namespace detail {

inline void writeU16(uint8_t *out, size_t offset, uint16_t value) {
  out[offset] = static_cast<uint8_t>(value & 0xff);
  out[offset + 1] = static_cast<uint8_t>(value >> 8);
}

inline void writeU32(uint8_t *out, size_t offset, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) out[offset + i] = static_cast<uint8_t>(value >> (i * 8));
}

inline void writeI16(uint8_t *out, size_t offset, int16_t value) {
  writeU16(out, offset, static_cast<uint16_t>(value));
}

inline uint16_t readU16(const uint8_t *in, size_t offset) {
  return static_cast<uint16_t>(in[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(in[offset + 1]) << 8);
}

inline uint32_t readU32(const uint8_t *in, size_t offset) {
  return static_cast<uint32_t>(in[offset]) |
         (static_cast<uint32_t>(in[offset + 1]) << 8) |
         (static_cast<uint32_t>(in[offset + 2]) << 16) |
         (static_cast<uint32_t>(in[offset + 3]) << 24);
}

inline int16_t readI16(const uint8_t *in, size_t offset) {
  return static_cast<int16_t>(readU16(in, offset));
}

inline void writeFloat(uint8_t *out, size_t offset, float value) {
  static_assert(sizeof(float) == sizeof(uint32_t), "protocol requires 32-bit float");
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  writeU32(out, offset, bits);
}

inline float readFloat(const uint8_t *in, size_t offset) {
  const uint32_t bits = readU32(in, offset);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline int16_t scaled(float value) {
  const double scaledValue = static_cast<double>(value) * 100.0;
  if (scaledValue >= 32767.0) return 32767;
  if (scaledValue <= -32768.0) return -32768;
  return static_cast<int16_t>(std::lround(scaledValue));
}

inline void begin(uint8_t *bytes, size_t size, uint8_t type, uint8_t byte3) {
  std::memset(bytes, 0, size);
  bytes[0] = MAGIC;
  bytes[1] = type;
  bytes[2] = VERSION;
  bytes[3] = byte3;
}

inline void finish(uint8_t *bytes, size_t size) {
  writeU16(bytes, size - 2, crc16Ccitt(bytes, size - 2));
}

inline bool valid(const uint8_t *bytes, size_t size, size_t expectedSize, uint8_t type) {
  return bytes != nullptr && size == expectedSize && bytes[0] == MAGIC &&
         bytes[1] == type && bytes[2] == VERSION &&
         readU16(bytes, size - 2) == crc16Ccitt(bytes, size - 2);
}

inline bool copyOut(const uint8_t *built, size_t size, uint8_t *out, size_t outSize) {
  if (out == nullptr || outSize != size) return false;
  std::memcpy(out, built, size);
  return true;
}

}  // namespace detail

inline bool buildPoseV2(const PoseV2 &value, uint8_t *out, size_t size) {
  if (out == nullptr || size != POSE_SIZE || value.flags > 7 ||
      !std::isfinite(value.x) || !std::isfinite(value.y) ||
      !std::isfinite(value.heading) || value.heading < 0.0f || value.heading >= 360.0f ||
      !std::isfinite(value.speedFps) || !std::isfinite(value.yawRateDps)) return false;
  uint8_t bytes[POSE_SIZE];
  detail::begin(bytes, sizeof(bytes), 0x56, value.flags);
  detail::writeU16(bytes, 4, value.epoch);
  detail::writeU32(bytes, 6, value.sequence);
  detail::writeU16(bytes, 10, value.ageMs > 0xffff ? 0xffff : static_cast<uint16_t>(value.ageMs));
  detail::writeFloat(bytes, 12, value.x);
  detail::writeFloat(bytes, 16, value.y);
  detail::writeFloat(bytes, 20, value.heading);
  detail::writeI16(bytes, 24, detail::scaled(value.speedFps));
  detail::writeI16(bytes, 26, detail::scaled(value.yawRateDps));
  detail::writeU16(bytes, 28, value.calibrationId);
  detail::finish(bytes, sizeof(bytes));
  return detail::copyOut(bytes, sizeof(bytes), out, size);
}

inline bool parsePoseV2(const uint8_t *bytes, size_t size, PoseV2 &out) {
  if (!detail::valid(bytes, size, POSE_SIZE, 0x56) || bytes[3] > 7) return false;
  PoseV2 parsed = {};
  parsed.flags = bytes[3];
  parsed.epoch = detail::readU16(bytes, 4);
  parsed.sequence = detail::readU32(bytes, 6);
  parsed.ageMs = detail::readU16(bytes, 10);
  parsed.x = detail::readFloat(bytes, 12);
  parsed.y = detail::readFloat(bytes, 16);
  parsed.heading = detail::readFloat(bytes, 20);
  parsed.speedFps = detail::readI16(bytes, 24) / 100.0f;
  parsed.yawRateDps = detail::readI16(bytes, 26) / 100.0f;
  parsed.calibrationId = detail::readU16(bytes, 28);
  if (!std::isfinite(parsed.x) || !std::isfinite(parsed.y) ||
      !std::isfinite(parsed.heading) || parsed.heading < 0.0f || parsed.heading >= 360.0f) return false;
  out = parsed;
  return true;
}

inline bool buildRectangleV2(const RectangleV2 &value, uint8_t *out, size_t size) {
  if (out == nullptr || size != RECTANGLE_SIZE || value.flags > 7 ||
      !std::isfinite(value.mFt) || value.mFt <= 0.0f ||
      !std::isfinite(value.nFt) || value.nFt <= 0.0f ||
      !std::isfinite(value.startClearFt) || value.startClearFt < 0.0f ||
      !std::isfinite(value.endClearFt) || value.endClearFt < 0.0f) return false;
  uint8_t bytes[RECTANGLE_SIZE];
  detail::begin(bytes, sizeof(bytes), 0x44, value.flags);
  detail::writeU16(bytes, 4, value.epoch);
  detail::writeU32(bytes, 6, value.commandId);
  detail::writeFloat(bytes, 10, value.mFt);
  detail::writeFloat(bytes, 14, value.nFt);
  detail::writeFloat(bytes, 18, value.startClearFt);
  detail::writeFloat(bytes, 22, value.endClearFt);
  detail::writeU16(bytes, 26, value.calibrationId);
  detail::writeU16(bytes, 28, value.startPassIndex);
  detail::finish(bytes, sizeof(bytes));
  return detail::copyOut(bytes, sizeof(bytes), out, size);
}

inline bool parseRectangleV2(const uint8_t *bytes, size_t size, RectangleV2 &out) {
  if (!detail::valid(bytes, size, RECTANGLE_SIZE, 0x44) || bytes[3] > 7) return false;
  RectangleV2 parsed = {};
  parsed.flags = bytes[3];
  parsed.epoch = detail::readU16(bytes, 4);
  parsed.commandId = detail::readU32(bytes, 6);
  parsed.mFt = detail::readFloat(bytes, 10);
  parsed.nFt = detail::readFloat(bytes, 14);
  parsed.startClearFt = detail::readFloat(bytes, 18);
  parsed.endClearFt = detail::readFloat(bytes, 22);
  parsed.calibrationId = detail::readU16(bytes, 26);
  parsed.startPassIndex = detail::readU16(bytes, 28);
  if (!std::isfinite(parsed.mFt) || parsed.mFt <= 0.0f ||
      !std::isfinite(parsed.nFt) || parsed.nFt <= 0.0f ||
      !std::isfinite(parsed.startClearFt) || parsed.startClearFt < 0.0f ||
      !std::isfinite(parsed.endClearFt) || parsed.endClearFt < 0.0f) return false;
  out = parsed;
  return true;
}

inline bool buildCalibrationV2(const CalibrationV2 &value, uint8_t *out, size_t size) {
  if (out == nullptr || size != CALIBRATION_SIZE || value.flags != 0 ||
      !std::isfinite(value.sprayForwardFt) || !std::isfinite(value.sprayRightFt)) return false;
  uint8_t bytes[CALIBRATION_SIZE];
  detail::begin(bytes, sizeof(bytes), 0x4b, 0);
  detail::writeU16(bytes, 4, value.epoch);
  detail::writeU32(bytes, 6, value.commandId);
  detail::writeU16(bytes, 10, value.calibrationId);
  detail::writeFloat(bytes, 12, value.sprayForwardFt);
  detail::writeFloat(bytes, 16, value.sprayRightFt);
  detail::writeU16(bytes, 20, value.schemaVersion);
  detail::finish(bytes, sizeof(bytes));
  return detail::copyOut(bytes, sizeof(bytes), out, size);
}

inline bool parseCalibrationV2(const uint8_t *bytes, size_t size, CalibrationV2 &out) {
  if (!detail::valid(bytes, size, CALIBRATION_SIZE, 0x4b) || bytes[3] != 0) return false;
  CalibrationV2 parsed = {};
  parsed.flags = 0;
  parsed.epoch = detail::readU16(bytes, 4);
  parsed.commandId = detail::readU32(bytes, 6);
  parsed.calibrationId = detail::readU16(bytes, 10);
  parsed.sprayForwardFt = detail::readFloat(bytes, 12);
  parsed.sprayRightFt = detail::readFloat(bytes, 16);
  parsed.schemaVersion = detail::readU16(bytes, 20);
  if (!std::isfinite(parsed.sprayForwardFt) || !std::isfinite(parsed.sprayRightFt)) return false;
  out = parsed;
  return true;
}

inline bool buildCommandV2(const CommandV2 &value, uint8_t *out, size_t size) {
  if (out == nullptr || size != COMMAND_SIZE || value.opcode < 1 || value.opcode > 8) return false;
  uint8_t bytes[COMMAND_SIZE];
  detail::begin(bytes, sizeof(bytes), 0x43, value.opcode);
  detail::writeU16(bytes, 4, value.epoch);
  detail::writeU32(bytes, 6, value.commandId);
  detail::finish(bytes, sizeof(bytes));
  return detail::copyOut(bytes, sizeof(bytes), out, size);
}

inline bool parseCommandV2(const uint8_t *bytes, size_t size, CommandV2 &out) {
  if (!detail::valid(bytes, size, COMMAND_SIZE, 0x43) || bytes[3] < 1 || bytes[3] > 8) return false;
  CommandV2 parsed = {bytes[3], detail::readU16(bytes, 4), detail::readU32(bytes, 6)};
  out = parsed;
  return true;
}

inline bool buildAckV2(const AckV2 &value, uint8_t *out, size_t size) {
  if (out == nullptr || size != ACK_SIZE || value.state > 5) return false;
  uint8_t bytes[ACK_SIZE];
  detail::begin(bytes, sizeof(bytes), 0x41, value.state);
  detail::writeU16(bytes, 4, value.epoch);
  detail::writeU32(bytes, 6, value.commandId);
  detail::writeU16(bytes, 10, value.faultCode);
  detail::writeU16(bytes, 12, value.calibrationId);
  detail::finish(bytes, sizeof(bytes));
  return detail::copyOut(bytes, sizeof(bytes), out, size);
}

inline bool parseAckV2(const uint8_t *bytes, size_t size, AckV2 &out) {
  if (!detail::valid(bytes, size, ACK_SIZE, 0x41) || bytes[3] > 5) return false;
  AckV2 parsed = {bytes[3], detail::readU16(bytes, 4), detail::readU32(bytes, 6),
                  detail::readU16(bytes, 10), detail::readU16(bytes, 12)};
  out = parsed;
  return true;
}

inline bool buildTelemetryV2(const TelemetryV2 &value, uint8_t *out, size_t size) {
  if (out == nullptr || size != TELEMETRY_SIZE || value.state > 5 ||
      !std::isfinite(value.crossTrackFt) || !std::isfinite(value.headingErrorDeg) ||
      !std::isfinite(value.speedFps)) return false;
  uint8_t bytes[TELEMETRY_SIZE];
  detail::begin(bytes, sizeof(bytes), 0x54, value.state);
  detail::writeU16(bytes, 4, value.epoch);
  detail::writeU32(bytes, 6, value.consumedPoseSequence);
  detail::writeU16(bytes, 10, value.routeIndex);
  detail::writeU16(bytes, 12, value.routeCount);
  detail::writeI16(bytes, 14, detail::scaled(value.crossTrackFt));
  detail::writeI16(bytes, 16, detail::scaled(value.headingErrorDeg));
  detail::writeI16(bytes, 18, detail::scaled(value.speedFps));
  detail::writeU16(bytes, 20, value.steeringUs);
  detail::writeU16(bytes, 22, value.throttleUs);
  bytes[24] = value.flags;
  bytes[25] = value.faultCode;
  detail::writeU16(bytes, 26, value.droppedPackets);
  detail::writeU16(bytes, 28, value.poseAgeMs);
  detail::finish(bytes, sizeof(bytes));
  return detail::copyOut(bytes, sizeof(bytes), out, size);
}

inline bool parseTelemetryV2(const uint8_t *bytes, size_t size, TelemetryV2 &out) {
  if (!detail::valid(bytes, size, TELEMETRY_SIZE, 0x54) || bytes[3] > 5) return false;
  TelemetryV2 parsed = {};
  parsed.state = bytes[3];
  parsed.epoch = detail::readU16(bytes, 4);
  parsed.consumedPoseSequence = detail::readU32(bytes, 6);
  parsed.routeIndex = detail::readU16(bytes, 10);
  parsed.routeCount = detail::readU16(bytes, 12);
  parsed.crossTrackFt = detail::readI16(bytes, 14) / 100.0f;
  parsed.headingErrorDeg = detail::readI16(bytes, 16) / 100.0f;
  parsed.speedFps = detail::readI16(bytes, 18) / 100.0f;
  parsed.steeringUs = detail::readU16(bytes, 20);
  parsed.throttleUs = detail::readU16(bytes, 22);
  parsed.flags = bytes[24];
  parsed.faultCode = bytes[25];
  parsed.droppedPackets = detail::readU16(bytes, 26);
  parsed.poseAgeMs = detail::readU16(bytes, 28);
  out = parsed;
  return true;
}

inline bool buildFaultSampleV2(const FaultSampleV2 &value, uint8_t *out, size_t size) {
  if (out == nullptr || size != FAULT_SAMPLE_SIZE || value.flags > 3 || value.state > 5 ||
      !std::isfinite(value.crossTrackFt) || !std::isfinite(value.headingErrorDeg) ||
      !std::isfinite(value.speedFps)) return false;
  uint8_t bytes[FAULT_SAMPLE_SIZE];
  detail::begin(bytes, sizeof(bytes), 0x42, value.flags);
  detail::writeU16(bytes, 4, value.epoch);
  detail::writeU32(bytes, 6, value.sequence);
  detail::writeU16(bytes, 10, value.sampleIndex);
  detail::writeU16(bytes, 12, value.sampleCount);
  detail::writeU16(bytes, 14, value.routeIndex);
  detail::writeI16(bytes, 16, detail::scaled(value.crossTrackFt));
  detail::writeI16(bytes, 18, detail::scaled(value.headingErrorDeg));
  detail::writeI16(bytes, 20, detail::scaled(value.speedFps));
  detail::writeU16(bytes, 22, value.steeringUs);
  detail::writeU16(bytes, 24, value.throttleUs);
  bytes[26] = value.state;
  bytes[27] = value.faultCode;
  detail::writeU16(bytes, 28, value.droppedPackets);
  detail::finish(bytes, sizeof(bytes));
  return detail::copyOut(bytes, sizeof(bytes), out, size);
}

inline bool parseFaultSampleV2(const uint8_t *bytes, size_t size, FaultSampleV2 &out) {
  if (!detail::valid(bytes, size, FAULT_SAMPLE_SIZE, 0x42) || bytes[3] > 3 || bytes[26] > 5) return false;
  FaultSampleV2 parsed = {};
  parsed.flags = bytes[3];
  parsed.epoch = detail::readU16(bytes, 4);
  parsed.sequence = detail::readU32(bytes, 6);
  parsed.sampleIndex = detail::readU16(bytes, 10);
  parsed.sampleCount = detail::readU16(bytes, 12);
  parsed.routeIndex = detail::readU16(bytes, 14);
  parsed.crossTrackFt = detail::readI16(bytes, 16) / 100.0f;
  parsed.headingErrorDeg = detail::readI16(bytes, 18) / 100.0f;
  parsed.speedFps = detail::readI16(bytes, 20) / 100.0f;
  parsed.steeringUs = detail::readU16(bytes, 22);
  parsed.throttleUs = detail::readU16(bytes, 24);
  parsed.state = bytes[26];
  parsed.faultCode = bytes[27];
  parsed.droppedPackets = detail::readU16(bytes, 28);
  out = parsed;
  return true;
}

}  // namespace protocol_v2

#endif
