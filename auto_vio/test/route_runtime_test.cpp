#include <cassert>
#include <cmath>
#include <cstdio>

#include "../route_runtime.h"

int main() {
  const ApplicatorPoint north = applicatorPoint(2.0f, 3.0f, 0.0f, -0.5f, 0.25f);
  assert(std::fabs(north.x - 2.25f) < 1e-5f);
  assert(std::fabs(north.y - 2.50f) < 1e-5f);

  const ApplicatorPoint east = applicatorPoint(2.0f, 3.0f, 90.0f, -0.5f, 0.25f);
  assert(std::fabs(east.x - 1.50f) < 1e-5f);
  assert(std::fabs(east.y - 2.75f) < 1e-5f);

  assert(applicatorWithinPassY(0.0f, 21.0f));
  assert(applicatorWithinPassY(21.0f, 21.0f));
  assert(!applicatorWithinPassY(-0.001f, 21.0f));
  assert(!applicatorWithinPassY(21.001f, 21.0f));

  RoutePoint route[] = {
      {0.0f, 0.0f, true, false, false, false},
      {0.0f, 1.0f, true, false, false, false},
      {0.0f, 2.0f, true, false, false, true},
  };
  assert(routeReadyForStart(route, 3));
  assert(!routeTerminalReachedOrPassed(route, 3, 0, {0.0f, 2.0f}, 0.2f));
  assert(!routeTerminalReachedOrPassed(route, 3, 1, {0.0f, 1.7f}, 0.2f));
  assert(routeTerminalReachedOrPassed(route, 3, 1, {0.0f, 1.81f}, 0.2f));
  assert(routeTerminalReachedOrPassed(route, 3, 2, {0.1f, 2.4f}, 0.2f));
  assert(!routeTerminalReachedOrPassed(route, 3, 2, {0.3f, 2.4f}, 0.2f));

  route[2].terminal = false;
  assert(!routeReadyForStart(route, 3));
  assert(!routeTerminalReachedOrPassed(route, 3, 2, {0.0f, 2.0f}, 0.2f));

  std::printf("route_runtime_test: all assertions passed\n");
  return 0;
}
