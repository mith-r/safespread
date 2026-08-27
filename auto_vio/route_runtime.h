#pragma once

#include <cmath>

#include "route.h"

// The route is tracked by the rover pose, but application limits and mission
// completion belong to the physical applicator. Calibration offsets use the
// rover's +forward/+right frame and headings are clockwise from field +Y.
struct ApplicatorPoint {
  float x;
  float y;
};

inline ApplicatorPoint applicatorPoint(float robotX, float robotY,
                                       float headingDeg, float forwardFt,
                                       float rightFt) {
  const float headingRad =
      headingDeg * 3.14159265358979323846f / 180.0f;
  return {
      robotX + rightFt * std::cosf(headingRad) +
          forwardFt * std::sinf(headingRad),
      robotY - rightFt * std::sinf(headingRad) +
          forwardFt * std::cosf(headingRad),
  };
}

inline bool applicatorWithinPassY(float applicatorY, float fieldPassFt) {
  return std::isfinite(applicatorY) && std::isfinite(fieldPassFt) &&
         fieldPassFt >= 0.0f && applicatorY >= 0.0f &&
         applicatorY <= fieldPassFt;
}

inline bool routeReadyForStart(const RoutePoint *points, int count) {
  return points != nullptr && count >= 2 && points[count - 1].terminal;
}

// Completion is possible only near the end of a fully emitted route. Merely
// reaching the penultimate tracker index is not enough: the applicator must
// reach the endpoint or pass its perpendicular finish line while remaining
// close to the terminal segment.
inline bool routeTerminalReachedOrPassed(const RoutePoint *points, int count,
                                         int currentIndex,
                                         const ApplicatorPoint &applicator,
                                         float toleranceFt) {
  if (!routeReadyForStart(points, count) || currentIndex < count - 2 ||
      !std::isfinite(applicator.x) || !std::isfinite(applicator.y) ||
      !std::isfinite(toleranceFt) || toleranceFt < 0.0f) {
    return false;
  }

  const RoutePoint &terminal = points[count - 1];
  const float fromTerminalX = applicator.x - terminal.x;
  const float fromTerminalY = applicator.y - terminal.y;
  const float toleranceSq = toleranceFt * toleranceFt;
  if (fromTerminalX * fromTerminalX + fromTerminalY * fromTerminalY <=
      toleranceSq) {
    return true;
  }

  int previousIndex = count - 2;
  float segmentX = 0.0f;
  float segmentY = 0.0f;
  float segmentLengthSq = 0.0f;
  while (previousIndex >= 0) {
    segmentX = terminal.x - points[previousIndex].x;
    segmentY = terminal.y - points[previousIndex].y;
    segmentLengthSq = segmentX * segmentX + segmentY * segmentY;
    if (segmentLengthSq > 1e-8f) break;
    --previousIndex;
  }
  if (previousIndex < 0) return false;

  const float along = fromTerminalX * segmentX + fromTerminalY * segmentY;
  if (along <= 0.0f) return false;

  const float cross = fromTerminalX * segmentY - fromTerminalY * segmentX;
  return cross * cross <= toleranceSq * segmentLengthSq;
}
