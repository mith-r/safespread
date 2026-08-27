#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "steering_map.h"

constexpr int MAX_CALIBRATION_SAMPLES = 32;
constexpr int MAX_CALIBRATION_KNOTS = 9;
constexpr float MIN_CALIBRATION_SWEEP_DEG = 60.0f;
constexpr int MAX_SPEED_CALIBRATION_SAMPLES = 8;
constexpr int MIN_SPEED_CALIBRATION_OFFSET_US = 40;
constexpr int MAX_SPEED_CALIBRATION_OFFSET_US = 350;
constexpr int DEFAULT_SPEED_CALIBRATION_OFFSET_US = 120;
constexpr float SPEED_CALIBRATION_TARGET_FPS = 1.0f;
constexpr float MAX_SPEED_CALIBRATION_FPS = 2.5f;
constexpr float MIN_SPEED_CALIBRATION_BRACKET_FPS = 0.05f;

struct SteeringCalibrationSample {
  int pulseUs;
  float curvaturePerFt;
  float sweepDeg;
  int8_t direction;
};

struct SteeringCalibrationFit {
  SteeringKnot knots[MAX_CALIBRATION_KNOTS];
  uint8_t count;
  bool valid;
};

struct SpeedCalibrationSample {
  int pulseUs;
  float speedFps;
  float distanceFt;
  int8_t direction;
};

inline float calibrationMedian(float *values, int count) {
  for (int index = 1; index < count; ++index) {
    const float value = values[index];
    int insert = index;
    while (insert > 0 && values[insert - 1] > value) {
      values[insert] = values[insert - 1];
      --insert;
    }
    values[insert] = value;
  }
  const int middle = count / 2;
  return count % 2 ? values[middle] : (values[middle - 1] + values[middle]) * 0.5f;
}

inline bool fitSteeringCalibration(const SteeringCalibrationSample *samples,
                                   int count, int straightPulseUs,
                                   SteeringCalibrationFit &out) {
  out = {};
  if (samples == nullptr || count < 2 || count > MAX_CALIBRATION_SAMPLES ||
      straightPulseUs < 500 || straightPulseUs > 2500) return false;

  struct PulseGroup {
    int pulseUs;
    float curvatures[MAX_CALIBRATION_SAMPLES];
    int count;
  } groups[MAX_CALIBRATION_KNOTS - 1] = {};
  int groupCount = 0;

  for (int index = 0; index < count; ++index) {
    const SteeringCalibrationSample &sample = samples[index];
    if ((sample.direction != 1 && sample.direction != -1) ||
        sample.pulseUs < 500 || sample.pulseUs > 2500 ||
        sample.pulseUs == straightPulseUs ||
        !std::isfinite(sample.curvaturePerFt) ||
        !std::isfinite(sample.sweepDeg) ||
        std::fabs(sample.sweepDeg) < MIN_CALIBRATION_SWEEP_DEG ||
        std::fabs(sample.curvaturePerFt) < 1e-4f) return false;

    int group = -1;
    for (int candidate = 0; candidate < groupCount; ++candidate) {
      if (groups[candidate].pulseUs == sample.pulseUs) {
        group = candidate;
        break;
      }
    }
    if (group < 0) {
      if (groupCount >= MAX_CALIBRATION_KNOTS - 1) return false;
      group = groupCount++;
      groups[group].pulseUs = sample.pulseUs;
    }
    const float normalized = sample.direction > 0
        ? sample.curvaturePerFt : -sample.curvaturePerFt;
    groups[group].curvatures[groups[group].count++] = normalized;
  }
  if (groupCount < 2) return false;

  out.count = static_cast<uint8_t>(groupCount + 1);
  for (int group = 0; group < groupCount; ++group) {
    out.knots[group] = {
      groups[group].pulseUs,
      calibrationMedian(groups[group].curvatures, groups[group].count),
    };
  }
  out.knots[groupCount] = {straightPulseUs, 0.0f};

  for (int index = 1; index < out.count; ++index) {
    const SteeringKnot knot = out.knots[index];
    int insert = index;
    while (insert > 0 && out.knots[insert - 1].curvaturePerFt > knot.curvaturePerFt) {
      out.knots[insert] = out.knots[insert - 1];
      --insert;
    }
    out.knots[insert] = knot;
  }
  out.valid = validSteeringMap(out.knots, out.count);
  return out.valid;
}

inline bool fitSpeedFeedForward(const SpeedCalibrationSample *samples, int count,
                                int8_t expectedDirection, int neutralPulseUs,
                                float &feedForwardUs) {
  feedForwardUs = 0.0f;
  if (samples == nullptr || count < 3 || count > MAX_SPEED_CALIBRATION_SAMPLES ||
      (expectedDirection != 1 && expectedDirection != -1) ||
      neutralPulseUs < 500 || neutralPulseUs > 2500) return false;

  struct SpeedPoint {
    int offsetUs;
    float speeds[MAX_SPEED_CALIBRATION_SAMPLES];
    int count;
    float speedFps;
  } points[MAX_SPEED_CALIBRATION_SAMPLES] = {};
  int pointCount = 0;

  for (int index = 0; index < count; ++index) {
    const SpeedCalibrationSample &sample = samples[index];
    const int offset = sample.pulseUs - neutralPulseUs;
    if (sample.direction != expectedDirection ||
        !std::isfinite(sample.speedFps) || !std::isfinite(sample.distanceFt) ||
        sample.distanceFt < 3.0f || sample.speedFps * expectedDirection <= 0.0f ||
        std::fabs(sample.speedFps) > MAX_SPEED_CALIBRATION_FPS ||
        offset * expectedDirection <= 0 ||
        std::abs(offset) < MIN_SPEED_CALIBRATION_OFFSET_US ||
        std::abs(offset) > MAX_SPEED_CALIBRATION_OFFSET_US) return false;

    const int magnitude = std::abs(offset);
    int point = -1;
    for (int candidate = 0; candidate < pointCount; ++candidate) {
      if (points[candidate].offsetUs == magnitude) {
        point = candidate;
        break;
      }
    }
    if (point < 0) {
      point = pointCount++;
      points[point].offsetUs = magnitude;
    }
    points[point].speeds[points[point].count++] = std::fabs(sample.speedFps);
  }
  if (pointCount < 3) return false;

  for (int point = 0; point < pointCount; ++point) {
    points[point].speedFps = calibrationMedian(points[point].speeds,
                                               points[point].count);
  }
  for (int index = 1; index < pointCount; ++index) {
    const SpeedPoint point = points[index];
    int insert = index;
    while (insert > 0 && points[insert - 1].offsetUs > point.offsetUs) {
      points[insert] = points[insert - 1];
      --insert;
    }
    points[insert] = point;
  }

  // More throttle must not produce materially less speed. A little tolerance
  // is retained for VIO quantization and pavement texture, but a reversed or
  // badly scattered curve is not safe to use as feed-forward.
  constexpr float MONOTONIC_NOISE_FPS = 0.10f;
  for (int index = 1; index < pointCount; ++index) {
    if (points[index].speedFps + MONOTONIC_NOISE_FPS <
        points[index - 1].speedFps) return false;
  }

  int below = -1;
  int above = -1;
  for (int index = 0; index < pointCount; ++index) {
    if (points[index].speedFps <= SPEED_CALIBRATION_TARGET_FPS &&
        (below < 0 || points[index].speedFps > points[below].speedFps)) {
      below = index;
    }
    if (points[index].speedFps >= SPEED_CALIBRATION_TARGET_FPS &&
        (above < 0 || points[index].speedFps < points[above].speedFps)) {
      above = index;
    }
  }

  float slowest = points[0].speedFps;
  float fastest = points[0].speedFps;
  for (int index = 1; index < pointCount; ++index) {
    if (points[index].speedFps < slowest) slowest = points[index].speedFps;
    if (points[index].speedFps > fastest) fastest = points[index].speedFps;
  }
  if (below < 0 || above < 0 ||
      slowest > SPEED_CALIBRATION_TARGET_FPS - MIN_SPEED_CALIBRATION_BRACKET_FPS ||
      fastest < SPEED_CALIBRATION_TARGET_FPS + MIN_SPEED_CALIBRATION_BRACKET_FPS) {
    return false;
  }

  if (below == above) {
    feedForwardUs = static_cast<float>(points[below].offsetUs);
  } else {
    if (points[above].offsetUs <= points[below].offsetUs) return false;
    const float speedSpan = points[above].speedFps - points[below].speedFps;
    if (speedSpan < 0.02f) return false;
    const float fraction = (SPEED_CALIBRATION_TARGET_FPS -
                            points[below].speedFps) / speedSpan;
    feedForwardUs = points[below].offsetUs +
        fraction * (points[above].offsetUs - points[below].offsetUs);
  }
  return std::isfinite(feedForwardUs) &&
         feedForwardUs >= MIN_SPEED_CALIBRATION_OFFSET_US &&
         feedForwardUs <= MAX_SPEED_CALIBRATION_OFFSET_US;
}

// Pick the next safe throttle magnitude from measurements collected so far.
// The caller records failed/too-slow attempts in minimumUsefulOffsetUs, which
// prevents retrying a pulse already proved unable to move the loaded rover.
// This is deliberately bounded: failure at 350 us is a drivetrain/load fault,
// not permission for calibration to keep raising throttle.
inline bool nextSpeedCalibrationOffset(const SpeedCalibrationSample *samples,
                                       int count, int8_t expectedDirection,
                                       int neutralPulseUs,
                                       int minimumUsefulOffsetUs,
                                       int &nextOffsetUs) {
  nextOffsetUs = 0;
  if (count < 0 || count > MAX_SPEED_CALIBRATION_SAMPLES ||
      (count > 0 && samples == nullptr) ||
      (expectedDirection != 1 && expectedDirection != -1) ||
      neutralPulseUs < 500 || neutralPulseUs > 2500 ||
      minimumUsefulOffsetUs < 0 ||
      minimumUsefulOffsetUs >= MAX_SPEED_CALIBRATION_OFFSET_US) return false;

  const int floorUs = minimumUsefulOffsetUs > 0
      ? minimumUsefulOffsetUs + 10 : MIN_SPEED_CALIBRATION_OFFSET_US;
  if (count == 0) {
    nextOffsetUs = DEFAULT_SPEED_CALIBRATION_OFFSET_US;
    if (nextOffsetUs < floorUs) nextOffsetUs = floorUs;
    return nextOffsetUs <= MAX_SPEED_CALIBRATION_OFFSET_US;
  }

  int minimumMeasuredOffset = MAX_SPEED_CALIBRATION_OFFSET_US + 1;
  int maximumMeasuredOffset = 0;
  float maximumSpeed = 0.0f;
  int belowOffset = 0;
  int aboveOffset = 0;
  float belowSpeed = -1.0f;
  float aboveSpeed = MAX_SPEED_CALIBRATION_FPS + 1.0f;
  for (int index = 0; index < count; ++index) {
    const SpeedCalibrationSample &sample = samples[index];
    const int signedOffset = sample.pulseUs - neutralPulseUs;
    const int offset = std::abs(signedOffset);
    const float speed = std::fabs(sample.speedFps);
    if (sample.direction != expectedDirection ||
        !std::isfinite(sample.speedFps) ||
        !std::isfinite(sample.distanceFt) || sample.distanceFt < 3.0f ||
        sample.speedFps * expectedDirection <= 0.0f ||
        signedOffset * expectedDirection <= 0 ||
        offset < MIN_SPEED_CALIBRATION_OFFSET_US ||
        offset > MAX_SPEED_CALIBRATION_OFFSET_US ||
        speed > MAX_SPEED_CALIBRATION_FPS) return false;
    if (offset < minimumMeasuredOffset) minimumMeasuredOffset = offset;
    if (offset > maximumMeasuredOffset) maximumMeasuredOffset = offset;
    if (speed > maximumSpeed) maximumSpeed = speed;
    if (speed <= SPEED_CALIBRATION_TARGET_FPS -
                     MIN_SPEED_CALIBRATION_BRACKET_FPS &&
        speed > belowSpeed) {
      belowSpeed = speed;
      belowOffset = offset;
    }
    if (speed >= SPEED_CALIBRATION_TARGET_FPS +
                     MIN_SPEED_CALIBRATION_BRACKET_FPS &&
        speed < aboveSpeed) {
      aboveSpeed = speed;
      aboveOffset = offset;
    }
  }

  int candidate = 0;
  if (belowSpeed < 0.0f) {
    // Every measured point is too fast. Bisect the remaining interval between
    // the proven-too-weak floor and the slowest successful pulse.
    const int upperUs = minimumMeasuredOffset - 10;
    if (floorUs > upperUs) return false;
    candidate = floorUs + (upperUs - floorUs) / 2;
  } else if (aboveSpeed > MAX_SPEED_CALIBRATION_FPS) {
    // Every point is too slow. Explore upward in bounded increments scaled by
    // the remaining speed error, never by an unbounded controller integral.
    int stepUs = static_cast<int>(std::lround(
        (SPEED_CALIBRATION_TARGET_FPS - maximumSpeed) * 80.0f));
    if (stepUs < 30) stepUs = 30;
    if (stepUs > 80) stepUs = 80;
    candidate = maximumMeasuredOffset + stepUs;
  } else {
    const float speedSpan = aboveSpeed - belowSpeed;
    if (speedSpan < 0.02f || aboveOffset <= belowOffset) return false;
    const float fraction = (SPEED_CALIBRATION_TARGET_FPS - belowSpeed) / speedSpan;
    candidate = static_cast<int>(std::lround(
        belowOffset + fraction * (aboveOffset - belowOffset)));

    // Three independent pulse levels are required. If interpolation lands on
    // an existing level, probe the widest remaining side of the bracket.
    bool duplicate = false;
    for (int index = 0; index < count; ++index) {
      if (std::abs(std::abs(samples[index].pulseUs - neutralPulseUs) - candidate) < 10) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      const int lowGap = candidate - belowOffset;
      const int highGap = aboveOffset - candidate;
      candidate = highGap >= lowGap
          ? candidate + (highGap > 20 ? highGap / 2 : 10)
          : candidate - (lowGap > 20 ? lowGap / 2 : 10);
    }
  }

  if (candidate < floorUs) candidate = floorUs;
  if (candidate > MAX_SPEED_CALIBRATION_OFFSET_US) return false;
  for (int index = 0; index < count; ++index) {
    if (std::abs(std::abs(samples[index].pulseUs - neutralPulseUs) - candidate) < 5) {
      return false;
    }
  }
  nextOffsetUs = candidate;
  return true;
}

constexpr uint32_t COMPACT_CALIBRATION_MAGIC = 0x5343414c;  // SCAL
constexpr uint16_t COMPACT_CALIBRATION_FORMAT_VERSION = 2;

struct CompactMotionCalibration {
  uint32_t magic;
  uint16_t formatVersion;
  uint16_t schemaVersion;
  uint16_t calibrationId;
  uint32_t hardwareTagHash;
  SteeringKnot knots[MAX_CALIBRATION_KNOTS];
  uint8_t knotCount;
  float forwardFeedForwardUs;
  float reverseFeedForwardUs;
  uint8_t reverseVerified;
  uint16_t checksum;
};

inline uint16_t calibrationChecksum(const CompactMotionCalibration &calibration) {
  uint16_t crc = 0xffff;
  auto addByte = [&](uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  };
  auto addValue = [&](const void *value, size_t size) {
    const uint8_t *bytes = static_cast<const uint8_t *>(value);
    for (size_t index = 0; index < size; ++index) addByte(bytes[index]);
  };
  addValue(&calibration.magic, sizeof(calibration.magic));
  addValue(&calibration.formatVersion, sizeof(calibration.formatVersion));
  addValue(&calibration.schemaVersion, sizeof(calibration.schemaVersion));
  addValue(&calibration.calibrationId, sizeof(calibration.calibrationId));
  addValue(&calibration.hardwareTagHash, sizeof(calibration.hardwareTagHash));
  addValue(calibration.knots, sizeof(calibration.knots));
  addValue(&calibration.knotCount, sizeof(calibration.knotCount));
  addValue(&calibration.forwardFeedForwardUs, sizeof(calibration.forwardFeedForwardUs));
  addValue(&calibration.reverseFeedForwardUs, sizeof(calibration.reverseFeedForwardUs));
  addValue(&calibration.reverseVerified, sizeof(calibration.reverseVerified));
  return crc;
}

inline CompactMotionCalibration makeCompactCalibration(
    uint16_t schemaVersion, uint16_t calibrationId, uint32_t hardwareTagHash,
    const SteeringCalibrationFit &steering, float forwardFeedForwardUs,
    float reverseFeedForwardUs, bool reverseVerified) {
  CompactMotionCalibration result = {};
  result.magic = COMPACT_CALIBRATION_MAGIC;
  result.formatVersion = COMPACT_CALIBRATION_FORMAT_VERSION;
  result.schemaVersion = schemaVersion;
  result.calibrationId = calibrationId;
  result.hardwareTagHash = hardwareTagHash;
  result.knotCount = steering.valid ? steering.count : 0;
  for (int index = 0; index < result.knotCount && index < MAX_CALIBRATION_KNOTS; ++index) {
    result.knots[index] = steering.knots[index];
  }
  result.forwardFeedForwardUs = forwardFeedForwardUs;
  result.reverseFeedForwardUs = reverseFeedForwardUs;
  result.reverseVerified = reverseVerified ? 1 : 0;
  result.checksum = calibrationChecksum(result);
  return result;
}

inline bool compactCalibrationValid(const CompactMotionCalibration &calibration) {
  return calibration.magic == COMPACT_CALIBRATION_MAGIC &&
         calibration.formatVersion == COMPACT_CALIBRATION_FORMAT_VERSION &&
         calibration.schemaVersion == 1 && calibration.hardwareTagHash != 0 &&
         calibration.knotCount >= 3 && calibration.knotCount <= MAX_CALIBRATION_KNOTS &&
         validSteeringMap(calibration.knots, calibration.knotCount) &&
         std::isfinite(calibration.forwardFeedForwardUs) &&
         calibration.forwardFeedForwardUs >= MIN_SPEED_CALIBRATION_OFFSET_US &&
         calibration.forwardFeedForwardUs <= MAX_SPEED_CALIBRATION_OFFSET_US &&
         std::isfinite(calibration.reverseFeedForwardUs) &&
         calibration.reverseFeedForwardUs >= MIN_SPEED_CALIBRATION_OFFSET_US &&
         calibration.reverseFeedForwardUs <= MAX_SPEED_CALIBRATION_OFFSET_US &&
         calibration.reverseVerified == 1 &&
         calibration.checksum == calibrationChecksum(calibration);
}

inline bool calibrationIdentityMatches(const CompactMotionCalibration &calibration,
                                       uint16_t schemaVersion, uint16_t calibrationId,
                                       uint32_t hardwareTagHash) {
  return compactCalibrationValid(calibration) &&
         calibration.schemaVersion == schemaVersion &&
         calibration.calibrationId == calibrationId &&
         calibration.hardwareTagHash == hardwareTagHash;
}

template <typename Store>
class MotionCalibrationPersistence {
 public:
  explicit MotionCalibrationPersistence(Store &store) : store_(store) {}
  bool save(const CompactMotionCalibration &calibration) {
    return compactCalibrationValid(calibration) &&
           store_.write(reinterpret_cast<const uint8_t *>(&calibration), sizeof(calibration));
  }
  bool load(CompactMotionCalibration &calibration) {
    CompactMotionCalibration candidate = {};
    if (!store_.read(reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate)) ||
        !compactCalibrationValid(candidate)) return false;
    calibration = candidate;
    return true;
  }

 private:
  Store &store_;
};
