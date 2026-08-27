#include <cassert>
#include <cmath>
#include <cstdio>
#include "../headland.h"
#include "../route.h"

static const float BAR = 17.0f / 12.0f;
static const float OVERLAP = 0.15f;
static const float RL = 4.33f;
static const float RR = 2.92f;
static RoutePoint route[6000];

int main() {
  const float passLength = 21.91f;
  int count = buildRoute(passLength, 21.91f, BAR, OVERLAP, RL, RR,
                         route, 6000);
  assert(count > 0);
  assert(route[count - 1].terminal);
  assert(std::fabs(route[0].y + INITIAL_RUN_IN_FT) < 0.001f && !route[0].spray);

  float minY = route[0].y;
  float maxY = route[0].y;
  int reversals = 0;
  for (int index = 1; index < count; ++index) {
    if (route[index].y < minY) minY = route[index].y;
    if (route[index].y > maxY) maxY = route[index].y;
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

  // Isolate the initial staging approach from turn extrema: for a one-lane
  // route it is the only geometry before y=0, so inspection must reserve it
  // (plus the normal continuous-path allowance) for either route style.
  RoutePoint straightRoute[100];
  int straightCount = buildRoute(20.0f, 1.0f, BAR, OVERLAP, RL, RR,
                                 straightRoute, 100);
  RouteRequirements straight = inspectRoute(straightRoute, straightCount, 20.0f);
  assert(straightCount > 0 && straightRoute[straightCount - 1].terminal);
  assert(!straight.truncated);
  assert(std::fabs(straight.beforeStartFt -
                   (INITIAL_RUN_IN_FT + ROUTE_EXTREMA_ALLOWANCE_FT)) < 0.001f);
  assert(std::fabs(straight.beyondEndFt) < 0.001f);

  RoutePoint forwardStraightRoute[100];
  int forwardStraightCount = buildForwardOnlyRoute(
      20.0f, 1.0f, BAR, OVERLAP, RL, RR, forwardStraightRoute, 100);
  RouteRequirements forwardStraight = inspectRoute(
      forwardStraightRoute, forwardStraightCount, 20.0f);
  assert(forwardStraightCount > 0 &&
         forwardStraightRoute[forwardStraightCount - 1].terminal);
  assert(!forwardStraight.truncated);
  assert(std::fabs(forwardStraight.beforeStartFt -
                   (INITIAL_RUN_IN_FT + ROUTE_EXTREMA_ALLOWANCE_FT)) < 0.001f);
  assert(std::fabs(forwardStraight.beyondEndFt) < 0.001f);

  std::printf("headland_test: needs %.2f ft before and %.2f ft beyond, %d reversals\n",
              requirements.beforeStartFt, requirements.beyondEndFt,
              requirements.reversals);
  std::printf("headland_test: all assertions passed\n");
  return 0;
}
