#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include "../calibration.h"

struct MemoryStore {
  uint8_t bytes[sizeof(CompactMotionCalibration)] = {};
  bool present = false;
  bool write(const uint8_t *value, size_t size) {
    if (size != sizeof(bytes)) return false;
    std::memcpy(bytes, value, size);
    present = true;
    return true;
  }
  bool read(uint8_t *value, size_t size) {
    if (!present || size != sizeof(bytes)) return false;
    std::memcpy(value, bytes, size);
    return true;
  }
};

int main() {
  const SteeringCalibrationSample arcs[] = {
      {2300, -0.21f, 72.0f, +1},
      {2300, -0.23f, 69.0f, +1},
      {2300, +0.22f, 75.0f, -1},  // reverse observation normalizes to left
      {1100, +0.24f, 70.0f, +1},
      {1100, +0.26f, 74.0f, +1},
      {1100, -0.25f, 71.0f, -1},  // reverse observation normalizes to right
  };
  SteeringCalibrationFit steering = {};
  assert(fitSteeringCalibration(arcs, 6, 1709, steering));
  assert(steering.valid && steering.count == 3);
  assert(steering.knots[1].pulseUs == 1709);
  assert(steering.knots[1].curvaturePerFt == 0.0f);  // directly measured straight
  assert(steering.knots[0].curvaturePerFt < 0.0f);
  assert(steering.knots[2].curvaturePerFt > 0.0f);
  assert(std::fabs(steering.knots[0].curvaturePerFt + 0.22f) < 0.011f);
  assert(std::fabs(steering.knots[2].curvaturePerFt - 0.25f) < 0.011f);
  assert(validSteeringMap(steering.knots, steering.count));

  SteeringCalibrationFit rejected = {};
  SteeringCalibrationSample invalidDirection[] = {{2300, -0.2f, 70.0f, 0}};
  assert(!fitSteeringCalibration(invalidDirection, 1, 1709, rejected));
  SteeringCalibrationSample shortSweep[] = {{2300, -0.2f, 59.9f, +1}};
  assert(!fitSteeringCalibration(shortSweep, 1, 1709, rejected));

  const SpeedCalibrationSample forwardSpeed[] = {
      {1680, 0.40f, 4.0f, +1},
      {1740, 0.80f, 4.2f, +1},
      {1840, 1.40f, 4.1f, +1},
  };
  const SpeedCalibrationSample reverseSpeed[] = {
      {1300, -0.40f, 4.0f, -1},
      {1240, -0.90f, 4.2f, -1},
      {1180, -1.40f, 4.1f, -1},
  };
  float forwardFeedForward = 0.0f, reverseFeedForward = 0.0f;
  assert(fitSpeedFeedForward(forwardSpeed, 3, +1, 1500, forwardFeedForward));
  assert(fitSpeedFeedForward(reverseSpeed, 3, -1, 1500, reverseFeedForward));
  assert(std::fabs(forwardFeedForward - 273.3333f) < 0.001f);
  assert(std::fabs(reverseFeedForward - 272.0f) < 0.001f);

  SpeedCalibrationSample wrongSign[] = {
      {1680, -0.5f, 4.0f, +1},
      {1760, 1.0f, 4.0f, +1},
      {1840, 1.5f, 4.0f, +1},
  };
  assert(!fitSpeedFeedForward(wrongSign, 3, +1, 1500, forwardFeedForward));

  SpeedCalibrationSample allTooSlow[] = {
      {1680, 0.3f, 4.0f, +1},
      {1760, 0.5f, 4.0f, +1},
      {1840, 0.8f, 4.0f, +1},
  };
  assert(!fitSpeedFeedForward(allTooSlow, 3, +1, 1500, forwardFeedForward));
  SpeedCalibrationSample nonMonotonic[] = {
      {1680, 0.7f, 4.0f, +1},
      {1760, 1.4f, 4.0f, +1},
      {1840, 0.9f, 4.0f, +1},
  };
  assert(!fitSpeedFeedForward(nonMonotonic, 3, +1, 1500, forwardFeedForward));

  int nextOffset = 0;
  assert(nextSpeedCalibrationOffset(nullptr, 0, +1, 1500, 0, nextOffset));
  assert(nextOffset == DEFAULT_SPEED_CALIBRATION_OFFSET_US);
  assert(nextSpeedCalibrationOffset(nullptr, 0, +1, 1500, 230, nextOffset));
  assert(nextOffset == 240);  // never retry a pulse already proved too weak

  const SpeedCalibrationSample oneSlow[] = {{1680, 0.40f, 4.0f, +1}};
  assert(nextSpeedCalibrationOffset(oneSlow, 1, +1, 1500, 0, nextOffset));
  assert(nextOffset == 228);
  const SpeedCalibrationSample oneFast[] = {{1800, 1.50f, 4.0f, +1}};
  assert(nextSpeedCalibrationOffset(oneFast, 1, +1, 1500, 200, nextOffset));
  assert(nextOffset == 250);
  const SpeedCalibrationSample bracket[] = {
      {1700, 0.60f, 4.0f, +1},
      {1780, 1.40f, 4.0f, +1},
  };
  assert(nextSpeedCalibrationOffset(bracket, 2, +1, 1500, 0, nextOffset));
  assert(nextOffset == 240);
  const SpeedCalibrationSample atCeiling[] = {{1850, 0.50f, 4.0f, +1}};
  assert(!nextSpeedCalibrationOffset(atCeiling, 1, +1, 1500, 0, nextOffset));
  const SpeedCalibrationSample selectorWrongSign[] = {{1680, -0.50f, 4.0f, +1}};
  assert(!nextSpeedCalibrationOffset(selectorWrongSign, 1, +1, 1500, 0,
                                     nextOffset));

  CompactMotionCalibration compact = makeCompactCalibration(
      1, 0x4321, 0x89abcdef, steering, 273.3333f, 272.0f, true);
  assert(compactCalibrationValid(compact));
  assert(compact.formatVersion == COMPACT_CALIBRATION_FORMAT_VERSION);
  assert(calibrationIdentityMatches(compact, 1, 0x4321, 0x89abcdef));
  assert(!calibrationIdentityMatches(compact, 1, 0x4322, 0x89abcdef));
  compact.forwardFeedForwardUs += 1.0f;
  assert(!compactCalibrationValid(compact));

  compact = makeCompactCalibration(1, 0x4321, 0x89abcdef,
                                   steering, 273.3333f, 272.0f, true);
  CompactMotionCalibration legacyFormat = compact;
  legacyFormat.formatVersion = 1;
  legacyFormat.checksum = calibrationChecksum(legacyFormat);
  assert(!compactCalibrationValid(legacyFormat));
  MemoryStore store;
  MotionCalibrationPersistence<MemoryStore> persistence(store);
  assert(persistence.save(compact));
  CompactMotionCalibration loaded = {};
  assert(persistence.load(loaded));
  assert(calibrationIdentityMatches(loaded, 1, 0x4321, 0x89abcdef));
  store.bytes[5] ^= 0x40;
  assert(!persistence.load(loaded));

  std::printf("calibration_test: all assertions passed\n");
  return 0;
}
