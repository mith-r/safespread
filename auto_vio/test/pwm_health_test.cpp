#include <cassert>
#include <cstdio>
#include "../pwm_health.h"

int main() {
  PwmReadinessGate gate;

  // Never allow a mission before the PWM controller has answered once.
  assert(!gate.observe(false));
  assert(!gate.verifiedOnce());

  assert(gate.observe(true));
  assert(gate.verifiedOnce());
  assert(gate.consecutiveFailures() == 0);

  // Two isolated half-second misses are tolerated. The third consecutive miss
  // removes readiness and allows fault 5 to latch.
  assert(gate.observe(false));
  assert(gate.observe(false));
  assert(!gate.observe(false));
  assert(gate.consecutiveFailures() == 3);

  // Recovery is automatic and clears the accumulated failures.
  assert(gate.observe(true));
  assert(gate.consecutiveFailures() == 0);
  assert(gate.observe(false));

  std::printf("pwm_health_test: all assertions passed\n");
  return 0;
}
