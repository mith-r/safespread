#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "../radius_calibration.h"

struct MemoryStore {
  uint8_t bytes[sizeof(StoredTurnRadii)] = {};
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

static RadiusSweepResult measure(float radiusFt, bool right) {
  RadiusSweep sweep;
  uint32_t sequence = 1;
  float x = 0.0f, y = 0.0f, heading = 0.0f;
  sweep.begin(sequence, x, y, heading);
  const float direction = right ? 1.0f : -1.0f;
  for (int step = 0; step < 1000 && !sweep.reachedTarget(); ++step) {
    const float ds = 0.08f;
    heading += direction * ds / radiusFt * 180.0f / (float)M_PI;
    const float radians = heading * (float)M_PI / 180.0f;
    x += ds * sinf(radians);
    y += ds * cosf(radians);
    sweep.add(++sequence, x, y, fmodf(heading + 360.0f, 360.0f));
  }
  return sweep.finish(right);
}

int main() {
  for (float radius = 2.5f; radius <= 15.0f; radius += 0.5f) {
    for (int direction = 0; direction < 2; ++direction) {
      const bool right = direction == 1;
      const RadiusSweepResult result = measure(radius, right);
      assert(result.valid);
      assert(std::fabs(result.radiusFt - radius) < 0.08f);
      assert(std::fabs(result.sweepDeg) >= 90.0f);
    }
  }

  MemoryStore store;
  TurnRadiusPersistence<MemoryStore> persistence(store);
  const uint32_t hardware = 0x89abcdef;
  StoredTurnRadii radii = makeStoredTurnRadii(hardware, 7.50f, 6.25f);
  assert(persistence.save(radii, hardware));
  StoredTurnRadii loaded = {};
  assert(persistence.load(loaded, hardware));
  assert(std::fabs(loaded.leftFt - 7.50f) < 0.001f);
  assert(std::fabs(loaded.rightFt - 6.25f) < 0.001f);
  store.bytes[3] ^= 0x40;
  assert(!persistence.load(loaded, hardware));

  std::printf("radius_calibration_test: both locks measured and persisted\n");
  return 0;
}
