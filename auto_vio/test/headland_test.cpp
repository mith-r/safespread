#include <cassert>
#include <cmath>
#include <cstdio>
#include "../headland.h"
#include "../route.h"

static const float BAR = 21.0f / 12.0f;
static const float OVERLAP = 0.0f;
static const float RL = 5.54f;
static const float RR = 5.05f;
static RoutePoint route[6000];

int main() {
  const float passLength = 21.91f;
  int count = buildRoute(passLength, 21.91f, BAR, OVERLAP, RL, RR,
                         route, 6000);
  assert(count > 0);

  float minY = 1e9f;
  float maxY = -1e9f;
  int reversals = 0;
  for (int index = 0; index < count; ++index) {
    const float heading = routeRoverHeadingDeg(route, count, index) *
                          (float)M_PI / 180.0f;
    const float forwards[2] = {-ROVER_FOOTPRINT.rearFt, ROVER_FOOTPRINT.frontFt};
    const float rights[2] = {-ROVER_FOOTPRINT.halfWidthFt, ROVER_FOOTPRINT.halfWidthFt};
    for (float forward : forwards) for (float right : rights) {
      const float y = route[index].y - right * sinf(heading) + forward * cosf(heading);
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    if (index == 0) continue;
    if (route[index].reverse != route[index - 1].reverse) reversals++;
  }

  RouteRequirements requirements = inspectRoute(route, count, passLength);
  assert(!requirements.truncated);
  assert(std::fabs(requirements.beforeStartFt -
                   (std::fmax(0.0f, -minY) + ROUTE_EXTREMA_ALLOWANCE_FT)) < 0.001f);
  assert(std::fabs(requirements.beyondEndFt -
                   (std::fmax(0.0f, maxY - passLength) + ROUTE_EXTREMA_ALLOWANCE_FT)) < 0.001f);
  assert(requirements.reversals == reversals && reversals > 0);
  assert(headlandFits(requirements, requirements.beforeStartFt,
                      requirements.beyondEndFt));
  assert(!headlandFits(requirements, requirements.beforeStartFt - 0.01f,
                       requirements.beyondEndFt));
  assert(!headlandFits(requirements, requirements.beforeStartFt,
                       requirements.beyondEndFt - 0.01f));

  RoutePoint shortRoute[100];
  int shortCount = buildRoute(passLength, 21.91f, BAR, OVERLAP, RL, RR,
                              shortRoute, 100);
  RouteRequirements truncated = inspectRoute(shortRoute, shortCount, passLength);
  assert(truncated.truncated);
  assert(!headlandFits(truncated, 1000.0f, 1000.0f));

  std::printf("headland_test: needs %.2f ft before and %.2f ft beyond, %d reversals\n",
              requirements.beforeStartFt, requirements.beyondEndFt,
              requirements.reversals);
  std::printf("headland_test: all assertions passed\n");
  return 0;
}
