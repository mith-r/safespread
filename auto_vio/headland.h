#pragma once
#include <cmath>
#include "route.h"

struct RouteRequirements {
  float beforeStartFt;
  float beyondEndFt;
  int reversals;
  bool truncated;
};

// Route arcs are sampled every 0.5 ft. Reserve the same allowance used by the
// continuous turn-envelope calculation so a between-point extremum remains
// conservative.
constexpr float ROUTE_EXTREMA_ALLOWANCE_FT = TURN_ENVELOPE_ALLOWANCE_FT;

inline float routeRoverHeadingDeg(const RoutePoint *route, int count, int index) {
  int other = index;
  if (index + 1 < count && route[index + 1].reverse == route[index].reverse) other = index + 1;
  else if (index > 0 && route[index - 1].reverse == route[index].reverse) other = index - 1;
  if (other == index) return 0.0f;
  const int from = other > index ? index : other;
  const int to = other > index ? other : index;
  float heading = bearingToWaypointDeg(route[to].x - route[from].x,
                                       route[to].y - route[from].y);
  if (route[index].reverse) heading = fmodf(heading + 180.0f, 360.0f);
  return heading;
}

inline RouteRequirements inspectRoute(const RoutePoint *route, int count,
                                      float passLengthFt) {
  RouteRequirements result = {0.0f, 0.0f, 0, true};
  if (route == nullptr || count <= 0 || passLengthFt <= 0.0f) return result;

  float minY = 1e9f;
  float maxY = -1e9f;
  for (int index = 0; index < count; ++index) {
    const float heading = turnRad(routeRoverHeadingDeg(route, count, index));
    const float forwards[2] = {-ROVER_FOOTPRINT.rearFt, ROVER_FOOTPRINT.frontFt};
    const float rights[2] = {-ROVER_FOOTPRINT.halfWidthFt, ROVER_FOOTPRINT.halfWidthFt};
    for (float forward : forwards) {
      for (float right : rights) {
        const float cornerY = route[index].y - right * sinf(heading) +
                              forward * cosf(heading);
        if (cornerY < minY) minY = cornerY;
        if (cornerY > maxY) maxY = cornerY;
      }
    }
    if (index == 0) continue;
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
