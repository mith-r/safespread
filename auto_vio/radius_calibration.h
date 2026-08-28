#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "nav_math.h"
#include "turn_radius/circle_fit.h"

constexpr uint32_t TURN_RADIUS_MAGIC = 0x53535244;  // SSRD
constexpr uint16_t TURN_RADIUS_SCHEMA = 1;

struct StoredTurnRadii {
  uint32_t magic;
  uint16_t schemaVersion;
  uint32_t hardwareTagHash;
  float leftFt;
  float rightFt;
  uint16_t checksum;
};

inline uint16_t turnRadiusChecksum(const StoredTurnRadii &radii) {
  uint16_t crc = 0xffff;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&radii);
  const size_t size = offsetof(StoredTurnRadii, checksum);
  for (size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint16_t>(bytes[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline bool validTurnRadius(float radiusFt) {
  return std::isfinite(radiusFt) && radiusFt >= 1.0f && radiusFt <= 40.0f;
}

inline StoredTurnRadii makeStoredTurnRadii(uint32_t hardwareTagHash,
                                           float leftFt, float rightFt) {
  StoredTurnRadii result = {};
  result.magic = TURN_RADIUS_MAGIC;
  result.schemaVersion = TURN_RADIUS_SCHEMA;
  result.hardwareTagHash = hardwareTagHash;
  result.leftFt = leftFt;
  result.rightFt = rightFt;
  result.checksum = turnRadiusChecksum(result);
  return result;
}

inline bool storedTurnRadiiValid(const StoredTurnRadii &radii,
                                 uint32_t hardwareTagHash) {
  return radii.magic == TURN_RADIUS_MAGIC &&
         radii.schemaVersion == TURN_RADIUS_SCHEMA &&
         radii.hardwareTagHash == hardwareTagHash &&
         validTurnRadius(radii.leftFt) && validTurnRadius(radii.rightFt) &&
         radii.checksum == turnRadiusChecksum(radii);
}

template <typename Store>
class TurnRadiusPersistence {
 public:
  explicit TurnRadiusPersistence(Store &store) : store_(store) {}

  bool save(const StoredTurnRadii &radii, uint32_t hardwareTagHash) {
    return storedTurnRadiiValid(radii, hardwareTagHash) &&
           store_.write(reinterpret_cast<const uint8_t *>(&radii), sizeof(radii));
  }

  bool load(StoredTurnRadii &radii, uint32_t hardwareTagHash) {
    StoredTurnRadii candidate = {};
    if (!store_.read(reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate)) ||
        !storedTurnRadiiValid(candidate, hardwareTagHash)) return false;
    radii = candidate;
    return true;
  }

 private:
  Store &store_;
};

struct RadiusSweepResult {
  bool valid;
  float radiusFt;
  float circleRadiusFt;
  float arcRadiusFt;
  float sweepDeg;
  float pathFt;
  float rmsFt;
  int samples;
};

// Collects one deliberate full-lock sweep. Positions are retained every 0.2 ft
// for an actual circle fit; travelled distance / heading sweep is an
// independent cross-check, not the source of the answer.
class RadiusSweep {
 public:
  static constexpr int MAX_SAMPLES = 192;
  static constexpr float SAMPLE_SPACING_FT = 0.20f;
  static constexpr float MAX_POSE_STEP_FT = 0.50f;
  static constexpr float MIN_SWEEP_DEG = 80.0f;
  static constexpr float MIN_PATH_FT = 3.0f;

  void begin(uint32_t sequence, float x, float y, float headingDeg) {
    active_ = true;
    failed_ = false;
    sequence_ = sequence;
    lastX_ = sampleX_ = x;
    lastY_ = sampleY_ = y;
    lastHeadingDeg_ = headingDeg;
    pathFt_ = 0.0f;
    sweepDeg_ = 0.0f;
    sampleCount_ = 1;
    xs_[0] = x;
    ys_[0] = y;
  }

  void add(uint32_t sequence, float x, float y, float headingDeg) {
    if (!active_ || failed_ || sequence == sequence_) return;
    sequence_ = sequence;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(headingDeg)) {
      failed_ = true;
      return;
    }
    const float dx = x - lastX_;
    const float dy = y - lastY_;
    const float stepFt = std::sqrt(dx * dx + dy * dy);
    if (stepFt >= MAX_POSE_STEP_FT) {
      failed_ = true;
      return;
    }
    pathFt_ += stepFt;
    sweepDeg_ += angleDiffDeg(headingDeg, lastHeadingDeg_);
    lastX_ = x;
    lastY_ = y;
    lastHeadingDeg_ = headingDeg;

    const float sampleDx = x - sampleX_;
    const float sampleDy = y - sampleY_;
    if (sampleCount_ < MAX_SAMPLES &&
        std::sqrt(sampleDx * sampleDx + sampleDy * sampleDy) >= SAMPLE_SPACING_FT) {
      xs_[sampleCount_] = x;
      ys_[sampleCount_] = y;
      sampleCount_++;
      sampleX_ = x;
      sampleY_ = y;
    }
  }

  bool reachedTarget() const { return std::fabs(sweepDeg_) >= 90.0f; }
  bool failed() const { return failed_; }
  float pathFt() const { return pathFt_; }

  RadiusSweepResult finish(bool expectRight) const {
    RadiusSweepResult result = {};
    if (!active_ || failed_ || sampleCount_ < 8 ||
        std::fabs(sweepDeg_) < MIN_SWEEP_DEG || pathFt_ < MIN_PATH_FT ||
        (expectRight ? sweepDeg_ <= 0.0f : sweepDeg_ >= 0.0f)) return result;
    const CircleFit circle = fitCircle(xs_, ys_, sampleCount_);
    const float arc = arcRadius(pathFt_, std::fabs(sweepDeg_));
    if (!circle.ok || !validTurnRadius(circle.r) || !validTurnRadius(arc) ||
        circle.rms > 0.35f) return result;
    const float disagreement = std::fabs(circle.r - arc) /
                               std::fmax(circle.r, arc);
    if (disagreement > 0.20f) return result;
    result.valid = true;
    result.radiusFt = circle.r;
    result.circleRadiusFt = circle.r;
    result.arcRadiusFt = arc;
    result.sweepDeg = sweepDeg_;
    result.pathFt = pathFt_;
    result.rmsFt = circle.rms;
    result.samples = sampleCount_;
    return result;
  }

 private:
  bool active_ = false;
  bool failed_ = false;
  uint32_t sequence_ = 0;
  float lastX_ = 0.0f;
  float lastY_ = 0.0f;
  float sampleX_ = 0.0f;
  float sampleY_ = 0.0f;
  float lastHeadingDeg_ = 0.0f;
  float pathFt_ = 0.0f;
  float sweepDeg_ = 0.0f;
  int sampleCount_ = 0;
  float xs_[MAX_SAMPLES] = {};
  float ys_[MAX_SAMPLES] = {};
};
