#include <algorithm>
#include <cerrno>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_math.h"
#include "protocol_v2.h"
#include "route.h"
#include "safety.h"

namespace {

constexpr uint64_t MAX_POSE_AGE_MS = 250;
constexpr float MAX_CROSS_TRACK_FT = 2.55f / 12.0f;
constexpr float P95_CROSS_TRACK_FT = MAX_CROSS_TRACK_FT / 2.0f;
constexpr unsigned STEERING_LOW_SATURATION_US = 800;
constexpr unsigned STEERING_HIGH_SATURATION_US = 2300;
constexpr double MAX_STEERING_SATURATION_PERCENT = 10.0;

const std::vector<std::string> REQUIRED_HEADER = {
    "phone_ms", "sequence", "epoch", "x_ft", "y_ft", "heading_deg",
    "speed_fps", "yaw_rate_dps", "tracking_valid", "route_index",
    "cross_track_ft", "heading_error_deg", "steering_us", "throttle_us",
    "fault"};

struct CsvRecord {
  std::vector<std::string> fields;
  size_t line;
};

struct Sample {
  uint64_t phoneMs;
  uint32_t sequence;
  uint16_t epoch;
  float xFt;
  float yFt;
  float headingDeg;
  float speedFps;
  float yawRateDps;
  bool trackingValid;
  uint16_t routeIndex;
  float crossTrackFt;
  float headingErrorDeg;
  uint16_t steeringUs;
  uint16_t throttleUs;
  uint8_t fault;
};

std::string trim(const std::string &value) {
  const size_t first = value.find_first_not_of(" \t");
  if (first == std::string::npos) return "";
  return value.substr(first, value.find_last_not_of(" \t") - first + 1);
}

[[noreturn]] void fail(size_t line, const std::string &message) {
  throw std::runtime_error("line " + std::to_string(line) + ": " + message);
}

std::vector<CsvRecord> readCsv(std::istream &input) {
  std::vector<CsvRecord> records;
  std::vector<std::string> fields;
  std::string field;
  size_t line = 1;
  size_t recordLine = 1;
  bool quoted = false;
  bool afterQuote = false;

  auto finishRecord = [&]() {
    fields.push_back(field);
    field.clear();
    records.push_back({fields, recordLine});
    fields.clear();
    recordLine = line + 1;
  };

  char ch;
  while (input.get(ch)) {
    if (quoted) {
      if (ch == '"') {
        if (input.peek() == '"') {
          input.get(ch);
          field.push_back('"');
        } else {
          quoted = false;
          afterQuote = true;
        }
      } else {
        field.push_back(ch);
        if (ch == '\n') ++line;
      }
      continue;
    }

    if (afterQuote) {
      if (ch == ',') {
        fields.push_back(field);
        field.clear();
        afterQuote = false;
      } else if (ch == '\n') {
        finishRecord();
        afterQuote = false;
        ++line;
      } else if (ch != '\r') {
        fail(recordLine, "malformed quoted field");
      }
      continue;
    }

    if (ch == '"') {
      if (!field.empty()) fail(recordLine, "malformed quoted field");
      quoted = true;
    } else if (ch == ',') {
      fields.push_back(field);
      field.clear();
    } else if (ch == '\n') {
      finishRecord();
      ++line;
    } else if (ch != '\r') {
      field.push_back(ch);
    }
  }

  if (quoted) fail(recordLine, "unterminated quoted field");
  if (afterQuote || !field.empty() || !fields.empty()) {
    fields.push_back(field);
    records.push_back({fields, recordLine});
  }
  return records;
}

template <typename T>
T parseUnsigned(const std::string &raw, size_t line, const char *name) {
  const std::string value = trim(raw);
  uint64_t parsed = 0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (value.empty() || result.ec != std::errc() || result.ptr != value.data() + value.size() ||
      parsed > static_cast<uint64_t>(std::numeric_limits<T>::max())) {
    fail(line, std::string("invalid ") + name);
  }
  return static_cast<T>(parsed);
}

float parseFloat(const std::string &raw, size_t line, const char *name) {
  const std::string value = trim(raw);
  if (value.empty()) fail(line, std::string("invalid ") + name);
  char *end = nullptr;
  errno = 0;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end != value.c_str() + value.size() || errno == ERANGE) {
    fail(line, std::string("invalid ") + name);
  }
  if (!std::isfinite(parsed) ||
      parsed < -std::numeric_limits<float>::max() ||
      parsed > std::numeric_limits<float>::max()) {
    fail(line, std::string("non-finite ") + name);
  }
  return static_cast<float>(parsed);
}

bool parseBool(const std::string &raw, size_t line) {
  const std::string value = trim(raw);
  if (value == "true" || value == "1") return true;
  if (value == "false" || value == "0") return false;
  fail(line, "invalid tracking_valid");
}

bool isCompleteControlRecord(const CsvRecord &record) {
  if (record.fields.size() != REQUIRED_HEADER.size()) return false;
  for (const std::string &field : record.fields) {
    if (trim(field).empty()) return false;
  }
  return true;
}

bool hasControlMetrics(const CsvRecord &record) {
  if (record.fields.size() != REQUIRED_HEADER.size()) return false;
  for (size_t index = 9; index <= 13; ++index) {
    if (!trim(record.fields[index]).empty()) return true;
  }
  return false;
}

FaultCode faultFromCell(const std::string &raw) {
  const std::string value = trim(raw);
  if (value.empty()) return F_NONE;
  uint64_t numeric = 0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), numeric);
  if (result.ec == std::errc() && result.ptr == value.data() + value.size() &&
      numeric <= std::numeric_limits<uint8_t>::max()) {
    const FaultCode fault = static_cast<FaultCode>(numeric);
    return fault == F_POSE_INVALID ? F_POSE_TIMEOUT : fault;
  }
  std::string lower;
  lower.reserve(value.size());
  for (unsigned char ch : value) lower.push_back(static_cast<char>(std::tolower(ch)));
  if (lower.find("ble") != std::string::npos ||
      lower.find("transport") != std::string::npos ||
      lower.find("disconnect") != std::string::npos) return F_BLE;
  if (lower.find("tracking") != std::string::npos) return F_TRACKING_ERROR;
  if (lower.find("calibration") != std::string::npos) return F_CALIBRATION;
  if (lower.find("headland") != std::string::npos) return F_HEADLAND;
  if (lower.find("pwm") != std::string::npos) return F_PWM;
  if (lower.find("i2c") != std::string::npos) return F_I2C;
  if (lower.find("stall") != std::string::npos) return F_STALL;
  if (lower.find("direction") != std::string::npos) return F_WRONG_DIRECTION;
  if (lower.find("pose") != std::string::npos) return F_POSE_TIMEOUT;
  return F_ROUTE;
}

Sample parseSample(const CsvRecord &record) {
  if (record.fields.size() != REQUIRED_HEADER.size()) fail(record.line, "malformed row");
  for (size_t index = 0; index < REQUIRED_HEADER.size(); ++index) {
    if (trim(record.fields[index]).empty()) fail(record.line, "malformed row");
  }
  return {
      parseUnsigned<uint64_t>(record.fields[0], record.line, "phone_ms"),
      parseUnsigned<uint32_t>(record.fields[1], record.line, "sequence"),
      parseUnsigned<uint16_t>(record.fields[2], record.line, "epoch"),
      parseFloat(record.fields[3], record.line, "x_ft"),
      parseFloat(record.fields[4], record.line, "y_ft"),
      parseFloat(record.fields[5], record.line, "heading_deg"),
      parseFloat(record.fields[6], record.line, "speed_fps"),
      parseFloat(record.fields[7], record.line, "yaw_rate_dps"),
      parseBool(record.fields[8], record.line),
      parseUnsigned<uint16_t>(record.fields[9], record.line, "route_index"),
      parseFloat(record.fields[10], record.line, "cross_track_ft"),
      parseFloat(record.fields[11], record.line, "heading_error_deg"),
      parseUnsigned<uint16_t>(record.fields[12], record.line, "steering_us"),
      parseUnsigned<uint16_t>(record.fields[13], record.line, "throttle_us"),
      parseUnsigned<uint8_t>(record.fields[14], record.line, "fault")};
}

SafetyInput readySafetyInput() {
  return {true, true, true, false, true, true,
          false, false, true, true, true, true};
}

int replay(const std::vector<CsvRecord> &records) {
  if (records.empty() || records[0].fields != REQUIRED_HEADER) {
    fail(1, "missing required CSV header");
  }

  size_t accepted = 0;
  size_t rejected = 0;
  uint64_t maxPoseAgeMs = 0;
  uint64_t previousPhoneMs = 0;
  bool havePhoneMs = false;
  uint32_t previousSequence = 0;
  bool haveSequence = false;
  uint16_t epoch = 0;
  bool haveEpoch = false;
  uint16_t finalRouteIndex = 0;
  bool haveRouteIndex = false;
  FaultCode missionFault = F_NONE;
  size_t saturatedSteering = 0;
  std::vector<float> crossTrack;

  for (size_t index = 1; index < records.size(); ++index) {
    const CsvRecord &record = records[index];
    if (record.fields.size() == 1 && trim(record.fields[0]).empty()) continue;
    if (record.fields.size() != REQUIRED_HEADER.size()) fail(record.line, "malformed row");
    if (!isCompleteControlRecord(record)) {
      const FaultCode eventFault = faultFromCell(record.fields[14]);
      if (eventFault != F_NONE) {
        if (missionFault == F_NONE) missionFault = eventFault;
        continue;
      }
      if (hasControlMetrics(record)) fail(record.line, "malformed row");
      continue;
    }
    const Sample sample = parseSample(record);

    if (!haveEpoch) {
      epoch = sample.epoch;
      haveEpoch = true;
    } else if (sample.epoch != epoch) {
      fail(record.line, "epoch changed from " + std::to_string(epoch) + " to " +
                            std::to_string(sample.epoch));
    }

    uint64_t poseAgeMs = 0;
    if (havePhoneMs) {
      if (sample.phoneMs < previousPhoneMs) fail(record.line, "phone_ms moved backward");
      poseAgeMs = sample.phoneMs - previousPhoneMs;
    }
    previousPhoneMs = sample.phoneMs;
    havePhoneMs = true;
    maxPoseAgeMs = std::max(maxPoseAgeMs, poseAgeMs);

    const bool ordered = !haveSequence || sample.sequence > previousSequence;
    previousSequence = sample.sequence;
    haveSequence = true;

    protocol_v2::PoseV2 encoded = {
        static_cast<uint8_t>((sample.trackingValid ? 1 : 0) | 2 | 4),
        sample.epoch,
        sample.sequence,
        static_cast<uint32_t>(std::min<uint64_t>(poseAgeMs, 0xffffffffULL)),
        sample.xFt,
        sample.yFt,
        sample.headingDeg,
        sample.speedFps,
        sample.yawRateDps,
        1};
    uint8_t packet[protocol_v2::POSE_SIZE] = {};
    protocol_v2::PoseV2 decoded = {};
    const bool protocolValid = protocol_v2::buildPoseV2(encoded, packet, sizeof(packet)) &&
                               protocol_v2::parsePoseV2(packet, sizeof(packet), decoded) &&
                               decoded.epoch == sample.epoch &&
                               decoded.sequence == sample.sequence;

    if (haveRouteIndex &&
        !routeIndexAdvanceIsPossible(finalRouteIndex, sample.routeIndex) &&
        missionFault == F_NONE) {
      missionFault = F_ROUTE;
    }
    finalRouteIndex = sample.routeIndex;
    haveRouteIndex = true;

    if (sample.fault != F_NONE && missionFault == F_NONE) {
      missionFault = static_cast<FaultCode>(sample.fault);
    }

    SafetyInput safety = readySafetyInput();
    safety.poseFresh = poseAgeMs <= MAX_POSE_AGE_MS;
    safety.poseValid = ordered && protocolValid;
    safety.trackingNormal = sample.trackingValid;
    const FaultCode safetyFault = evaluateSafety(S_RUNNING, safety);
    const bool staleOnly = !safety.poseFresh && safety.poseValid && safety.trackingNormal;
    if (staleOnly) {
      ++rejected;
      continue;
    }
    if (safetyFault != F_NONE) {
      if (missionFault == F_NONE) missionFault = safetyFault;
      ++rejected;
      continue;
    }

    const float curvature = lineFollowCurvature(
        sample.crossTrackFt, sample.headingErrorDeg, 1.5f);
    if (!std::isfinite(curvature) && missionFault == F_NONE) missionFault = F_ROUTE;

    const float absCrossTrack = std::fabs(sample.crossTrackFt);
    crossTrack.push_back(absCrossTrack);
    if (sample.steeringUs <= STEERING_LOW_SATURATION_US ||
        sample.steeringUs >= STEERING_HIGH_SATURATION_US) {
      ++saturatedSteering;
    }
    ++accepted;
  }

  if (accepted == 0 && missionFault == F_NONE) {
    throw std::runtime_error("no replayable control samples");
  }
  std::sort(crossTrack.begin(), crossTrack.end());
  const float maximumCrossTrack = crossTrack.empty() ? 0.0f : crossTrack.back();
  const size_t p95Index = crossTrack.empty() ? 0 : static_cast<size_t>(
      std::ceil(0.95 * static_cast<double>(crossTrack.size()))) - 1;
  const float p95CrossTrack = crossTrack.empty() ? 0.0f : crossTrack[p95Index];
  const double saturationPercent =
      accepted == 0 ? 0.0 :
      100.0 * static_cast<double>(saturatedSteering) / static_cast<double>(accepted);
  const bool acceptedMission = missionFault == F_NONE && rejected == 0 && haveRouteIndex &&
      maximumCrossTrack <= MAX_CROSS_TRACK_FT && p95CrossTrack <= P95_CROSS_TRACK_FT &&
      saturationPercent <= MAX_STEERING_SATURATION_PERCENT;

  std::cout << "accepted=" << accepted
            << " rejected=" << rejected
            << std::fixed << std::setprecision(3)
            << " max_cross_track_ft=" << maximumCrossTrack
            << " p95_cross_track_ft=" << p95CrossTrack
            << " max_pose_age_ms=" << maxPoseAgeMs
            << std::setprecision(1)
            << " steering_saturation_pct=" << saturationPercent
            << " fault=" << static_cast<unsigned>(missionFault)
            << " final_route_index=" << finalRouteIndex
            << " acceptance=" << (acceptedMission ? "accepted" : "rejected")
            << '\n';
  return acceptedMission ? 0 : 2;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: replay <mission.csv>\n";
    return 1;
  }
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) {
    std::cerr << "cannot open " << argv[1] << '\n';
    return 1;
  }
  try {
    return replay(readCsv(input));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
