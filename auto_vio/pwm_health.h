#pragma once

#include <cstdint>

// A single missed I2C poll is not proof that the PWM controller is gone. Once
// the chip has been verified, allow two transient misses and fault on the third
// consecutive failure. A successful check immediately resets the streak.
class PwmReadinessGate {
 public:
  static constexpr uint8_t FAILURES_BEFORE_FAULT = 3;

  bool observe(bool rawReady) {
    if (rawReady) {
      verifiedOnce_ = true;
      consecutiveFailures_ = 0;
      return true;
    }
    if (consecutiveFailures_ < 0xff) consecutiveFailures_++;
    return verifiedOnce_ &&
           consecutiveFailures_ < FAILURES_BEFORE_FAULT;
  }

  uint8_t consecutiveFailures() const { return consecutiveFailures_; }
  bool verifiedOnce() const { return verifiedOnce_; }

 private:
  bool verifiedOnce_ = false;
  uint8_t consecutiveFailures_ = 0;
};
