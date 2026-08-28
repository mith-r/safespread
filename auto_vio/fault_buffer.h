#ifndef SAFESPREAD_FAULT_BUFFER_H
#define SAFESPREAD_FAULT_BUFFER_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include "protocol_v2.h"
#include "safety.h"

struct ControlSample {
  uint32_t sequence;
  uint16_t routeIndex;
  float crossTrackFt;
  float headingErrorDeg;
  float speedFps;
  uint16_t steeringUs;
  uint16_t throttleUs;
  MissionState state;
  FaultCode fault;
  uint16_t droppedPackets;
};

template <size_t N>
class FaultBuffer {
 public:
  static_assert(N > 0, "fault buffer capacity must be positive");

  void push(const ControlSample &sample) {
    if (isFrozen_) return;
    if (count_ < N) {
      samples_[(start_ + count_) % N] = sample;
      count_++;
    } else {
      samples_[start_] = sample;
      start_ = (start_ + 1) % N;
    }
  }

  void freeze(FaultCode fault) {
    if (isFrozen_) return;
    isFrozen_ = true;
    frozenFault_ = fault;
  }

  void reset() {
    start_ = 0;
    count_ = 0;
    isFrozen_ = false;
    frozenFault_ = F_NONE;
  }

  size_t size() const { return count_; }
  bool frozen() const { return isFrozen_; }
  FaultCode fault() const { return frozenFault_; }
  const ControlSample &at(size_t index) const { return samples_[(start_ + index) % N]; }

  bool buildPacket(uint16_t index, uint16_t epoch, uint8_t *out, size_t outSize) const {
    if (!isFrozen_ || index >= count_ || outSize != protocol_v2::FAULT_SAMPLE_SIZE) return false;
    const ControlSample &source = at(index);
    protocol_v2::FaultSampleV2 packet = {};
    packet.flags = static_cast<uint8_t>((index == 0 ? 1 : 0) | (index + 1 == count_ ? 2 : 0));
    packet.epoch = epoch;
    packet.sequence = source.sequence;
    packet.sampleIndex = index;
    packet.sampleCount = static_cast<uint16_t>(count_);
    packet.routeIndex = source.routeIndex;
    packet.crossTrackFt = source.crossTrackFt;
    packet.headingErrorDeg = source.headingErrorDeg;
    packet.speedFps = source.speedFps;
    packet.steeringUs = source.steeringUs;
    packet.throttleUs = source.throttleUs;
    packet.state = static_cast<uint8_t>(source.state);
    packet.faultCode = static_cast<uint8_t>(frozenFault_);
    packet.droppedPackets = source.droppedPackets;
    return protocol_v2::buildFaultSampleV2(packet, out, outSize);
  }

 private:
  ControlSample samples_[N] = {};
  size_t start_ = 0;
  size_t count_ = 0;
  bool isFrozen_ = false;
  FaultCode frozenFault_ = F_NONE;
};

constexpr uint16_t FAULT_SUMMARY_SCHEMA = 2;
constexpr size_t FAULT_SUMMARY_SIZE = 18;

struct FaultSummary {
  uint16_t schema;
  uint16_t epoch;
  FaultCode fault;
  uint16_t routeIndex;
  uint16_t droppedPackets;
  uint16_t invalidPackets;
  uint32_t controlSequence;
};

namespace fault_summary_detail {
inline void put16(uint8_t *out, size_t at, uint16_t value) {
  out[at] = static_cast<uint8_t>(value);
  out[at + 1] = static_cast<uint8_t>(value >> 8);
}
inline void put32(uint8_t *out, size_t at, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) out[at + i] = static_cast<uint8_t>(value >> (8 * i));
}
inline uint16_t get16(const uint8_t *in, size_t at) {
  return static_cast<uint16_t>(in[at]) | static_cast<uint16_t>(in[at + 1] << 8);
}
inline uint32_t get32(const uint8_t *in, size_t at) {
  return static_cast<uint32_t>(in[at]) |
         (static_cast<uint32_t>(in[at + 1]) << 8) |
         (static_cast<uint32_t>(in[at + 2]) << 16) |
         (static_cast<uint32_t>(in[at + 3]) << 24);
}
inline void encode(const FaultSummary &summary, uint8_t *out) {
  std::memset(out, 0, FAULT_SUMMARY_SIZE);
  put16(out, 0, summary.schema);
  put16(out, 2, summary.epoch);
  out[4] = static_cast<uint8_t>(summary.fault);
  put16(out, 6, summary.routeIndex);
  put16(out, 8, summary.droppedPackets);
  put16(out, 10, summary.invalidPackets);
  put32(out, 12, summary.controlSequence);
  put16(out, 16, protocol_v2::crc16Ccitt(out, 16));
}
inline bool decode(const uint8_t *in, uint16_t schema, FaultSummary &out) {
  if (get16(in, 16) != protocol_v2::crc16Ccitt(in, 16) || get16(in, 0) != schema ||
      in[4] > F_HEADLAND) return false;
  FaultSummary parsed = {};
  parsed.schema = get16(in, 0);
  parsed.epoch = get16(in, 2);
  parsed.fault = static_cast<FaultCode>(in[4]);
  parsed.routeIndex = get16(in, 6);
  parsed.droppedPackets = get16(in, 8);
  parsed.invalidPackets = get16(in, 10);
  parsed.controlSequence = get32(in, 12);
  out = parsed;
  return true;
}
}  // namespace fault_summary_detail

template <typename Store>
class FaultSummaryPersistence {
 public:
  FaultSummaryPersistence(Store &store, uint16_t schema) : store_(store), schema_(schema) {}

  bool persistOnce(const FaultSummary &summary) {
    if (written_) return false;
    if (summary.schema != schema_) return false;
    uint8_t bytes[FAULT_SUMMARY_SIZE];
    fault_summary_detail::encode(summary, bytes);
    written_ = store_.write(bytes, sizeof(bytes));
    return written_;
  }

  bool load(FaultSummary &out) {
    uint8_t bytes[FAULT_SUMMARY_SIZE];
    if (!store_.read(bytes, sizeof(bytes))) return false;
    return fault_summary_detail::decode(bytes, schema_, out);
  }

 private:
  Store &store_;
  uint16_t schema_;
  bool written_ = false;
};

#endif
