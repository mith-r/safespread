#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "../fault_buffer.h"

using namespace protocol_v2;

static ControlSample sample(uint32_t sequence) {
  ControlSample value = {};
  value.sequence = sequence;
  value.routeIndex = static_cast<uint16_t>(sequence);
  value.crossTrackFt = -0.1f * sequence;
  value.headingErrorDeg = 0.25f * sequence;
  value.speedFps = 0.5f;
  value.steeringUs = 1500 + sequence;
  value.throttleUs = 1400 + sequence;
  value.state = S_RUNNING;
  value.fault = F_NONE;
  value.droppedPackets = static_cast<uint16_t>(sequence * 10);
  return value;
}

struct FakeStore {
  uint8_t bytes[FAULT_SUMMARY_SIZE] = {};
  bool present = false;
  int writes = 0;
  bool read(uint8_t *out, size_t size) {
    if (!present || size != sizeof(bytes)) return false;
    std::memcpy(out, bytes, size);
    return true;
  }
  bool write(const uint8_t *input, size_t size) {
    if (size != sizeof(bytes)) return false;
    std::memcpy(bytes, input, size);
    present = true;
    writes++;
    return true;
  }
};

int main() {
  FaultBuffer<4> buffer;
  for (uint32_t sequence = 1; sequence <= 6; ++sequence) buffer.push(sample(sequence));
  assert(buffer.size() == 4);
  for (size_t index = 0; index < 4; ++index) assert(buffer.at(index).sequence == index + 3);

  buffer.freeze(F_STALL);
  buffer.push(sample(7));
  assert(buffer.size() == 4);
  assert(buffer.at(0).sequence == 3 && buffer.at(3).sequence == 6);
  assert(buffer.frozen() && buffer.fault() == F_STALL);

  for (uint16_t index = 0; index < buffer.size(); ++index) {
    uint8_t packet[FAULT_SAMPLE_SIZE];
    assert(buffer.buildPacket(index, 42, packet, sizeof(packet)));
    FaultSampleV2 parsed = {};
    assert(parseFaultSampleV2(packet, sizeof(packet), parsed));
    assert(parsed.epoch == 42);
    assert(parsed.sampleIndex == index && parsed.sampleCount == 4);
    assert((parsed.flags & 1) == (index == 0 ? 1 : 0));
    assert(((parsed.flags >> 1) & 1) == (index == 3 ? 1 : 0));
    assert(parsed.sequence == index + 3);
    assert(parsed.faultCode == F_STALL);
    assert(parsed.droppedPackets == (index + 3) * 10);
  }
  uint8_t tooSmall[8];
  assert(!buffer.buildPacket(4, 42, tooSmall, sizeof(tooSmall)));

  buffer.reset();
  assert(!buffer.frozen() && buffer.fault() == F_NONE && buffer.size() == 0);
  buffer.push(sample(20));
  assert(buffer.size() == 1 && buffer.at(0).sequence == 20);
  buffer.freeze(F_PWM);
  assert(buffer.frozen() && buffer.fault() == F_PWM);

  FakeStore store;
  FaultSummaryPersistence<FakeStore> persistence(store, 2);
  FaultSummary summary = {2, 42, F_STALL, 8, 11, 12, 1234};
  assert(persistence.persistOnce(summary));
  assert(!persistence.persistOnce(summary));
  assert(store.writes == 1);
  FaultSummary loaded = {};
  assert(persistence.load(loaded));
  assert(loaded.epoch == 42 && loaded.fault == F_STALL && loaded.controlSequence == 1234);

  store.bytes[8] ^= 1;
  assert(!persistence.load(loaded));
  store.bytes[8] ^= 1;
  FaultSummaryPersistence<FakeStore> wrongSchema(store, 3);
  assert(!wrongSchema.load(loaded));

  std::printf("fault_buffer_test: all assertions passed\n");
  return 0;
}
