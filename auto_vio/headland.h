#pragma once
#include <cmath>
#include "route.h"

struct RouteRequirements {
  float beforeStartFt;
  float beyondEndFt;
  int reversals;
  bool truncated;
};

// Route arcs are sampled every 0.5 ft. At the smallest planned turn radius
// (5.05 ft measured), an extremum between samples can
// exceed both adjacent waypoints by just under 0.01 ft. Reserve that amount
// whenever the route leaves an end of the rectangle so exact entered
// clearance remains conservative for the continuous physical path.
constexpr float ROUTE_EXTREMA_ALLOWANCE_FT = 0.01f;

inline RouteRequirements inspectRoute(const RoutePoint *route, int count,
                                      float passLengthFt) {
  RouteRequirements result = {0.0f, 0.0f, 0, true};
  if (route == nullptr || count <= 0 || passLengthFt <= 0.0f) return result;

  float minY = route[0].y;
  float maxY = route[0].y;
  for (int index = 1; index < count; ++index) {
    if (route[index].y < minY) minY = route[index].y;
    if (route[index].y > maxY) maxY = route[index].y;
    if (route[index].reverse != route[index - 1].reverse) result.reversals++;
  }
  const float sampledBefore = std::fmax(0.0f, -minY);
  const float sampledBeyond = std::fmax(0.0f, maxY - passLengthFt);
  result.beforeStartFt = sampledBefore > 0.0f
      ? sampledBefore + ROUTE_EXTREMA_ALLOWANCE_FT : 0.0f;
  result.beyondEndFt = sampledBeyond > 0.0f
      ? sampledBeyond + ROUTE_EXTREMA_ALLOWANCE_FT : 0.0f;
  result.truncated = !route[count - 1].terminal;
  return result;
}

inline bool headlandFits(const RouteRequirements &requirements,
                         float availableStartFt, float availableEndFt) {
  if (requirements.truncated || availableStartFt < 0.0f || availableEndFt < 0.0f) {
    return false;
  }
  const float tolerance = 1e-4f;
  return availableStartFt + tolerance >= requirements.beforeStartFt &&
         availableEndFt + tolerance >= requirements.beyondEndFt;
}

enum RouteStyle : uint8_t {
  ROUTE_NONE = 0,
  ROUTE_FORWARD_ONLY = 1,
  ROUTE_THREE_POINT = 2,
};

struct RouteSelection {
  int count;
  RouteStyle style;
  RouteRequirements requirements;
};

inline RouteSelection selectRoute(float fieldPassFt, float fieldWidthFt,
                                  float barWidthFt, float overlapFraction,
                                  float rLeftFt, float rRightFt,
                                  float availableStartFt, float availableEndFt,
                                  bool preferForwardOnly,
                                  RoutePoint *out, int maxOut) {
  // Adjacent-lane three-point turns have the smallest headland envelope. Use
  // them whenever they fit, even if the caller permits a wide forward loop.
  const int threePointCount = buildRoute(
      fieldPassFt, fieldWidthFt, barWidthFt, overlapFraction,
      rLeftFt, rRightFt, out, maxOut);
  const RouteRequirements threePoint = inspectRoute(out, threePointCount, fieldPassFt);
  if (headlandFits(threePoint, availableStartFt, availableEndFt)) {
    return {threePointCount, ROUTE_THREE_POINT, threePoint};
  }

  if (preferForwardOnly) {
    const int forwardCount = buildForwardOnlyRoute(
        fieldPassFt, fieldWidthFt, barWidthFt, overlapFraction,
        rLeftFt, rRightFt, out, maxOut);
    const RouteRequirements forward = inspectRoute(out, forwardCount, fieldPassFt);
    if (headlandFits(forward, availableStartFt, availableEndFt)) {
      return {forwardCount, ROUTE_FORWARD_ONLY, forward};
    }
  }
  return {0, ROUTE_NONE, threePoint};
}
