#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>

inline bool parsePosePacket(const uint8_t* d, size_t n, float& x, float& y, float& heading) {
  if (n < 15 || d[0] != 0x21 || d[1] != 0x50) return false;

  uint8_t sum = 0;
  for (int i = 0; i < 14; i++) sum += d[i];
  uint8_t crc = (uint8_t)(~sum);
  if (crc != d[14]) return false;

  memcpy(&x, d + 2, 4);
  memcpy(&y, d + 6, 4);
  memcpy(&heading, d + 10, 4);
  return true;
}

// 11-byte '!D' packet: field dimensions in feet, set from the app.
inline bool parseAreaPacket(const uint8_t* d, size_t n, float& widthFt, float& lengthFt) {
  if (n < 11 || d[0] != 0x21 || d[1] != 0x44) return false;

  uint8_t sum = 0;
  for (int i = 0; i < 10; i++) sum += d[i];
  if ((uint8_t)(~sum) != d[10]) return false;

  memcpy(&widthFt, d + 2, 4);
  memcpy(&lengthFt, d + 6, 4);
  return true;
}

// --- Lane geometry -------------------------------------------------------
// Lanes are spaced so the spray bar tiles the width with a little overlap.
// The first lane sits at x=0, i.e. exactly where the rover is standing when
// the mission starts: placing the rover is how you aim the first pass, and it
// must therefore drive straight ahead rather than immediately steering onto a
// lane offset from it. Coverage consequently begins half a bar-width to the
// left of the start point. Lanes stop before the far edge, so a width not
// divisible by the bar is under-covered at the far side, never overrun.

inline float laneSpacing(float barWidthFt, float overlapFraction) {
  float s = barWidthFt * (1.0f - overlapFraction);
  return (s > 0.01f) ? s : 0.01f;
}

inline float laneCenterX(int lane, float barWidthFt, float overlapFraction) {
  return lane * laneSpacing(barWidthFt, overlapFraction);
}

inline int laneCount(float widthFt, float barWidthFt, float overlapFraction) {
  if (widthFt <= barWidthFt) return 1;
  float usable = widthFt - barWidthFt;
  int n = (int)floorf(usable / laneSpacing(barWidthFt, overlapFraction)) + 1;
  return (n < 1) ? 1 : n;
}

// --- Pure pursuit --------------------------------------------------------
// Steer at a point a fixed distance ahead on the planned path. Drift pulls the
// lookahead point off to one side, which steers back onto the line, so the
// rover recovers from error instead of re-deciding where it was going.

/** Index of the route point nearest the rover, searched forward only so the
 *  rover cannot latch onto an earlier part of a path that crosses itself. */
template <typename PointT>
inline int nearestRouteIndex(const PointT *route, int count, int fromIndex,
                             float x, float y, int searchWindow) {
  int best = fromIndex;
  float bestDist = 1e18f;
  int end = fromIndex + searchWindow;
  if (end > count) end = count;
  for (int i = fromIndex; i < end; i++) {
    float dx = route[i].x - x;
    float dy = route[i].y - y;
    float d = dx * dx + dy * dy;
    if (d < bestDist) { bestDist = d; best = i; }
  }
  return best;
}

/** First point at least `lookaheadFt` beyond `fromIndex`. */
template <typename PointT>
inline int lookaheadRouteIndex(const PointT *route, int count, int fromIndex,
                               float x, float y, float lookaheadFt) {
  float need = lookaheadFt * lookaheadFt;
  for (int i = fromIndex; i < count; i++) {
    float dx = route[i].x - x;
    float dy = route[i].y - y;
    if (dx * dx + dy * dy >= need) return i;
  }
  return count - 1;
}

/** Curvature that intersects a lookahead point in the rover's travel frame.
 *  Unlike a proportional steering command, this reproduces the radius of a
 *  planned arc instead of waiting for cross-track error before steering hard. */
inline float purePursuitCurvature(float headingErrorDeg, float targetDistanceFt) {
  if (!(targetDistanceFt > 0.01f) || !std::isfinite(headingErrorDeg) ||
      !std::isfinite(targetDistanceFt)) return 0.0f;
  const float radians = headingErrorDeg * static_cast<float>(M_PI) / 180.0f;
  return 2.0f * std::sin(radians) / targetDistanceFt;
}

// --- Straight-line following ---------------------------------------------
// Pure pursuit chases a point ahead of the rover, and chasing lags: it reaches
// the line carrying heading error, crosses it, and comes back. On a pass that
// is only a few lookaheads long the rover is still weaving when it gets to the
// far end, which is why every pass except the first -- the only one that
// starts on its line already aligned -- comes out curved.
//
// For a straight line there is a better law. Steer on how far off the line the
// rover is and how far off parallel it is, in the ratio that makes the
// approach critically damped, and it settles onto the line without ever
// crossing it.
//
// Writing e for the sideways offset and s for distance travelled, a steered
// vehicle obeys e'' = curvature, so commanding
//     curvature = (2/T) * headingError - (1/T^2) * e
// gives e'' + (2/T) e' + (1/T^2) e = 0: a critically damped approach with
// distance constant T. T is in feet and sets how sharply it converges --
// roughly 3*T feet to close a 1 ft error, with no overshoot at any T.

/** Signed distance from the line through (lineX, lineY) heading lineHeadingDeg,
 *  positive when the rover is off to the right of it. */
inline float crossTrackFt(float lineX, float lineY, float lineHeadingDeg,
                          float x, float y) {
  float h = lineHeadingDeg * (float)M_PI / 180.0f;
  float dx = sinf(h), dy = cosf(h);
  return (x - lineX) * dy - (y - lineY) * dx;
}

/** Curvature (1/ft, positive turns right) that converges onto the line. */
inline float lineFollowCurvature(float crossFt, float headingErrDeg,
                                 float distanceConstantFt) {
  float T = (distanceConstantFt > 0.1f) ? distanceConstantFt : 0.1f;
  float psi = headingErrDeg * (float)M_PI / 180.0f;
  return (2.0f / T) * psi - (1.0f / (T * T)) * crossFt;
}

/** Curvature as a steering command, scaled so `maxOffset` is full lock. Each
 *  side is measured against its own tightest circle, so the command means the
 *  same fraction of the turn available whichever way the rover is bending. */
inline float curvatureToCommand(float kappa, float rLeftFt, float rRightFt,
                                float maxOffset) {
  float kMax = (kappa >= 0.0f) ? (1.0f / rRightFt) : (1.0f / rLeftFt);
  if (!(kMax > 0.0f)) return 0.0f;
  float cmd = maxOffset * (kappa / kMax);
  if (cmd > maxOffset)  cmd = maxOffset;
  if (cmd < -maxOffset) cmd = -maxOffset;
  return cmd;
}

inline float angleDiffDeg(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

inline float bearingToWaypointDeg(float dx, float dy) {
  float b = atan2f(dx, dy) * (180.0f / (float)M_PI);
  if (b < 0.0f) b += 360.0f;
  return b;
}

inline bool waypointReached(float dx, float dy, float toleranceFt) {
  return (dx * dx + dy * dy) <= (toleranceFt * toleranceFt);
}
