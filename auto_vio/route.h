#pragma once
#include "nav_math.h"
#include "dubins.h"
#include "turn.h"

// The whole mission, computed once before the rover moves: a dense list of
// points to follow, each flagged for spray and for reversing. Planning up
// front is what stops the rover re-deciding its route from its own tracking
// error, which is how small position errors used to turn into completely
// different maneuvers.
//
// Lanes are driven in order -- 0, 1, 2, ... -- the way a person mowing a lawn
// does it, joined by three-point turns. That ordering only became affordable
// once turns could reverse: a forward-only turn between neighbouring lanes has
// to loop right out and come back, and used to cost 28.7 ft against 12.6 ft
// for a lane a full turning diameter away, which is why the route used to skip
// lanes and pick them up on later sweeps. A three-point turn does the same
// move in 10.1 ft, so the simple order is now also the cheap one.

struct RoutePoint {
  float x;
  float y;
  bool  spray;
  bool  reverse;
  // Set on the arcs of a turn. The tracker aims at a point a fixed distance
  // ahead, and a lookahead longer than the arc's radius cuts the corner
  // instead of following it -- with a 2.9 ft turning circle, the 2.5 ft
  // lookahead that suits a straight pass misses the turn completely.
  bool  turning;
  // True only on the final point of a fully generated route. A full output
  // buffer can otherwise look like a valid partial pass, so completion must be
  // explicit rather than inferred from the last coordinate.
  bool  terminal;
};

const float ROUTE_STEP_FT = 0.5f;

// advanceRouteIndex searches no more than this many points per control update.
// Telemetry/replay can therefore detect a backward index or a forward jump the
// production tracker itself could not have produced in one accepted sample.
constexpr int ROUTE_PROGRESS_SEARCH_WINDOW = 80;

inline bool routeIndexAdvanceIsPossible(uint16_t previous, uint16_t current) {
  return current >= previous &&
         static_cast<uint32_t>(current - previous) <= ROUTE_PROGRESS_SEARCH_WINDOW;
}

// Turn directly at the rectangle edge. Segment-bounded lookahead prevents an
// early turn, and geometric turn tracking lands aligned on the next lane, so
// the old 3.5 ft runout only made the rover drive needlessly far away.
const float HEADLAND_MARGIN_FT = 0.0f;

// Use the rover's measured minimum radii. Turn speed is already reduced, and
// inflating these by 30% was the main source of unnecessarily wide headlands.
const float TURN_PLANNING_MARGIN = 1.0f;
const int MAX_FORWARD_LANES = 64;

inline int forwardLaneSkip(float turnRadiusFt, float barWidthFt,
                           float overlapFraction, int totalLanes) {
  int skip = static_cast<int>(lroundf(
      (2.0f * turnRadiusFt) / laneSpacing(barWidthFt, overlapFraction)));
  if (skip < 1) skip = 1;
  if (totalLanes > 1 && skip > totalLanes - 1) skip = totalLanes - 1;
  return skip;
}

// Ported from the earlier measured-radius navigator: aim a turning diameter
// away, then switch to the nearest remaining lane when that far landing has
// already been covered. This deliberately alternates far and near indices;
// the long jumps make easy U-turns and the short joins are handled by Dubins
// loops when enough continuous headland exists.
inline int buildAlternatingFarLaneOrder(int totalLanes, int skip,
                                        int *out, int maxOut) {
  if (totalLanes <= 0 || totalLanes > MAX_FORWARD_LANES || maxOut < totalLanes) return 0;
  bool covered[MAX_FORWARD_LANES] = {};
  int current = 0;
  for (int count = 0; count < totalLanes; ++count) {
    out[count] = current;
    covered[current] = true;
    if (count + 1 == totalLanes) return totalLanes;

    int next = current + skip;
    if (next >= totalLanes || covered[next]) {
      next = current - skip;
    }
    if (next < 0 || next >= totalLanes || covered[next]) {
      float bestDistance = 1e9f;
      next = -1;
      for (int lane = 0; lane < totalLanes; ++lane) {
        if (covered[lane]) continue;
        const float distance = fabsf(static_cast<float>(lane - current));
        if (distance < bestDistance) {
          bestDistance = distance;
          next = lane;
        }
      }
    }
    if (next < 0) return 0;
    current = next;
  }
  return 0;
}

inline float routeHeadingToMath(float headingDeg) {
  return (90.0f - headingDeg) * static_cast<float>(M_PI) / 180.0f;
}

/** Points along a straight line, excluding the start (the previous segment
 *  already ended there) and landing exactly on the end, so consecutive
 *  segments join without a gap the tracker would read as a jump. */
inline int emitLineTo(RoutePoint *out, int maxOut, float x1, float y1,
                      float x2, float y2, bool spray, bool reverse) {
  float dx = x2 - x1, dy = y2 - y1;
  float len = sqrtf(dx * dx + dy * dy);
  if (len < 1e-6f || maxOut <= 0) return 0;

  int n = 0;
  int steps = (int)(len / ROUTE_STEP_FT);
  for (int k = 1; k <= steps && n < maxOut; k++) {
    float t = (k * ROUTE_STEP_FT) / len;
    out[n].x = x1 + dx * t;
    out[n].y = y1 + dy * t;
    out[n].spray = spray;
    out[n].reverse = reverse;
    out[n].turning = false;
    out[n].terminal = false;
    n++;
  }
  if (n < maxOut &&
      (n == 0 || fabsf(out[n - 1].x - x2) > 1e-4f || fabsf(out[n - 1].y - y2) > 1e-4f)) {
    out[n].x = x2; out[n].y = y2; out[n].spray = spray; out[n].reverse = reverse;
    out[n].turning = false;
    out[n].terminal = false;
    n++;
  }
  return n;
}

/** Points along a headland turn, transformed from the maneuver's own frame
 *  into the field. The rover's local +X is its right hand, which points along
 *  world +X on an outbound pass and world -X coming back. */
inline int emitTurn(RoutePoint *out, int maxOut, const TurnPlan &p,
                    float x0, float y0, float h0Deg) {
  if (maxOut <= 0) return 0;
  float ch = cosf(turnRad(h0Deg)), sh = sinf(turnRad(h0Deg));
  int n = 0;

  // Sample each leg independently and always emit its exact endpoint. When a
  // cusp fell between the old fixed-distance samples, the direction flag
  // changed at a point up to half a foot before or after the real cusp. The
  // rover then cut the three-point turn and entered the next pass beside its
  // intended line. Exact cusp points keep every forward/reverse leg intact.
  float completed = 0.0f;
  for (int legIndex = 0; legIndex < p.legCount && n < maxOut; ++legIndex) {
    const float legLength = turnLegLength(p.leg[legIndex]);
    if (legLength <= 1e-6f) continue;

    for (float local = ROUTE_STEP_FT;
         local < legLength - 1e-4f && n < maxOut;
         local += ROUTE_STEP_FT) {
      float lx, ly, lh; bool rev;
      turnPoseAt(p, completed + local, lx, ly, lh, rev);
      out[n].x = x0 + lx * ch + ly * sh;
      out[n].y = y0 - lx * sh + ly * ch;
      out[n].spray = false;
      out[n].reverse = p.leg[legIndex].reverse;
      out[n].turning = true;
      out[n].terminal = false;
      n++;
    }

    completed += legLength;
    if (n < maxOut) {
      float lx, ly, lh; bool rev;
      turnPoseAt(p, completed, lx, ly, lh, rev);
      out[n].x = x0 + lx * ch + ly * sh;
      out[n].y = y0 - lx * sh + ly * ch;
      out[n].spray = false;
      out[n].reverse = (legIndex + 1 < p.legCount)
          ? p.leg[legIndex].reverse
          : false;              // leave the completed turn driving forward
      out[n].turning = true;
      out[n].terminal = false;
      n++;
    }
  }
  return n;
}

/** How many sprayed passes the plan contains, counted in the order they are
 *  driven rather than by lane number -- a forward-only plan visits lanes out
 *  of order, and what an operator resumes is the pass, not the lane. */
inline int routePassCount(const RoutePoint *route, int count) {
  int passes = 0;
  for (int i = 0; i < count; ++i) {
    if (route[i].spray && (i == 0 || !route[i - 1].spray)) passes++;
  }
  return passes;
}

/** First point of the given sprayed pass, or -1 when the plan has no such
 *  pass. Resuming a faulted mission starts here instead of at point zero. */
inline int routePassStartIndex(const RoutePoint *route, int count, int pass) {
  if (pass < 0) return -1;
  int seen = 0;
  for (int i = 0; i < count; ++i) {
    if (!route[i].spray || (i > 0 && route[i - 1].spray)) continue;
    if (seen == pass) return i;
    seen++;
  }
  return -1;
}

/** Which sprayed pass a route index sits on, in the same route order. Used to
 *  report where a resumed mission actually starts. */
inline int routePassIndexAt(const RoutePoint *route, int count, int index) {
  int pass = -1;
  for (int i = 0; i < count && i <= index; ++i) {
    if (route[i].spray && (i == 0 || !route[i - 1].spray)) pass++;
  }
  return pass < 0 ? 0 : pass;
}

/** Last point of the run the rover is currently driving, i.e. the next cusp
 *  or the end of the plan. */
inline int segmentEndIndex(const RoutePoint *route, int count, int fromIndex) {
  bool rev = route[fromIndex].reverse;
  int i = fromIndex;
  while (i + 1 < count && route[i + 1].reverse == rev) i++;
  return i;
}

/** Direction of the straight run the rover is currently on, as a rover
 *  heading. Taken across several points so a single point's rounding does not
 *  swing the answer. */
inline float segmentHeadingDeg(const RoutePoint *route, int count, int idx) {
  int end = segmentEndIndex(route, count, idx);
  int a = idx, b = idx + 4;
  if (b > end) b = end;
  if (b <= a) {                      // at the very end of a run: look back
    b = idx;
    a = (idx >= 4) ? idx - 4 : 0;
  }
  if (b <= a) return 0.0f;
  return bearingToWaypointDeg(route[b].x - route[a].x, route[b].y - route[a].y);
}

/** Lookahead that never aims past a cusp. Beyond one the plan doubles back, so
 *  a target on the far side would steer for a leg the rover is not driving
 *  yet -- and on a three-point turn that is a target roughly behind it. */
inline int lookaheadWithinSegment(const RoutePoint *route, int count,
                                  int fromIndex, float x, float y,
                                  float lookaheadFt) {
  int end = segmentEndIndex(route, count, fromIndex);
  float need = lookaheadFt * lookaheadFt;
  for (int i = fromIndex; i <= end; i++) {
    float dx = route[i].x - x, dy = route[i].y - y;
    if (dx * dx + dy * dy >= need) return i;
  }
  return end;
}

/** Advance the tracked index along the plan. Progress stays inside the current
 *  run of same-direction points and steps across a cusp only once the rover
 *  has actually reached it, so a reverse leg doubling back alongside the
 *  forward leg it came from cannot make the tracker skip ahead. */
inline int advanceRouteIndex(const RoutePoint *route, int count, int fromIndex,
                             float x, float y, int searchWindow,
                             float cuspTolFt) {
  int end = segmentEndIndex(route, count, fromIndex);
  int window = end - fromIndex + 1;
  if (window > searchWindow) window = searchWindow;

  int idx = nearestRouteIndex(route, count, fromIndex, x, y, window);
  if (idx < end || end + 1 >= count) return idx;

  // Close enough to call the leg finished.
  float dx = route[end].x - x, dy = route[end].y - y;
  if (dx * dx + dy * dy <= cuspTolFt * cuspTolFt) return end + 1;

  // Or already past it. Without this the rover that overshoots a cusp keeps
  // driving away from it forever, because the index cannot advance and the
  // target it is steering at is now behind.
  if (end > 0) {
    float tx = route[end].x - route[end - 1].x;
    float ty = route[end].y - route[end - 1].y;
    if (tx * (x - route[end].x) + ty * (y - route[end].y) > 0.0f) return end + 1;
  }
  return idx;
}

/** Build a no-reverse route using the measured-radius far-lane visit order.
 *  Every transit is a shortest curvature-bounded Dubins path; exact clearance
 *  is inspected after generation instead of guessed before motion. */
inline int buildForwardOnlyRoute(float fieldPassFt, float fieldWidthFt,
                                 float barWidthFt, float overlapFraction,
                                 float rLeftFt, float rRightFt,
                                 RoutePoint *out, int maxOut) {
  const int totalLanes = laneCount(fieldWidthFt, barWidthFt, overlapFraction);
  if (totalLanes < 1 || totalLanes > MAX_FORWARD_LANES || maxOut < 2) return 0;

  const float radius = fmaxf(rLeftFt, rRightFt) * TURN_PLANNING_MARGIN;
  int order[MAX_FORWARD_LANES];
  const int orderCount = buildAlternatingFarLaneOrder(
      totalLanes, forwardLaneSkip(radius, barWidthFt, overlapFraction, totalLanes),
      order, MAX_FORWARD_LANES);
  if (orderCount != totalLanes) return 0;

  int count = 0;
  int completedLanes = 0;
  bool haveExit = false;
  float exitX = 0.0f, exitY = 0.0f, exitHeading = 0.0f;

  for (int visit = 0; visit < orderCount && count < maxOut; ++visit) {
    const float laneX = laneCenterX(order[visit], barWidthFt, overlapFraction);
    const bool goesUp = (visit % 2) == 0;
    const float startY = goesUp ? 0.0f : fieldPassFt;
    const float endY = goesUp ? fieldPassFt : 0.0f;
    const float direction = goesUp ? 1.0f : -1.0f;
    const float heading = goesUp ? 0.0f : 180.0f;

    if (haveExit) {
      const float approachY = startY - direction * HEADLAND_MARGIN_FT;
      DubinsPath transit = {};
      if (!dubinsCompute(exitX, exitY, routeHeadingToMath(exitHeading),
                         laneX, approachY, routeHeadingToMath(heading),
                         radius, transit)) {
        return count;
      }
      const float length = dubinsLength(transit);
      for (float distance = ROUTE_STEP_FT;
           distance < length && count < maxOut;
           distance += ROUTE_STEP_FT) {
        float x, y, theta;
        dubinsPoseAt(transit, distance, x, y, theta);
        out[count] = {x, y, false, false, true, false};
        count++;
      }
      if (count >= maxOut) break;
      out[count] = {laneX, approachY, false, false, true, false};
      count++;
      count += emitLineTo(out + count, maxOut - count,
                          laneX, approachY, laneX, startY, false, false);
      if (count <= 0 || fabsf(out[count - 1].x - laneX) > 1e-4f ||
          fabsf(out[count - 1].y - startY) > 1e-4f) {
        break;
      }
    }

    if (count >= maxOut) break;
    out[count] = {laneX, startY, true, false, false, false};
    count++;
    count += emitLineTo(out + count, maxOut - count,
                        laneX, startY, laneX, endY, true, false);
    if (count <= 0 || fabsf(out[count - 1].x - laneX) > 1e-4f ||
        fabsf(out[count - 1].y - endY) > 1e-4f || !out[count - 1].spray) {
      break;
    }
    completedLanes++;

    if (visit + 1 < orderCount) {
      const float runoutY = endY + direction * HEADLAND_MARGIN_FT;
      count += emitLineTo(out + count, maxOut - count,
                          laneX, endY, laneX, runoutY, false, false);
      if (count <= 0 || fabsf(out[count - 1].x - laneX) > 1e-4f ||
          fabsf(out[count - 1].y - runoutY) > 1e-4f) {
        break;
      }
      exitX = laneX;
      exitY = runoutY;
      exitHeading = heading;
      haveExit = true;
    }
  }

  if (count > 0 && completedLanes == totalLanes) out[count - 1].terminal = true;
  return count;
}

/** Emit the full route. `rLeftFt` and `rRightFt` are the rover's two turning
 *  radii, carried separately because they are not the same size. Returns the
 *  number of points written. */
inline int buildRoute(float fieldPassFt, float fieldWidthFt,
                      float barWidthFt, float overlapFraction,
                      float rLeftFt, float rRightFt,
                      RoutePoint *out, int maxOut) {
  int lanes = laneCount(fieldWidthFt, barWidthFt, overlapFraction);
  if (lanes < 1 || maxOut < 2) return 0;

  float planLeft  = rLeftFt * TURN_PLANNING_MARGIN;
  float planRight = rRightFt * TURN_PLANNING_MARGIN;

  // The route starts under the rover, spraying: the first pass runs straight
  // ahead from where it was placed, and it is the pass the whole mission is
  // lined up on.
  int n = 0;
  out[n].x = 0.0f; out[n].y = 0.0f; out[n].spray = true;
  out[n].reverse = false; out[n].turning = false; out[n].terminal = false;
  n++;
  int completedLanes = 0;

  for (int i = 0; i < lanes && n < maxOut; i++) {
    float laneX  = laneCenterX(i, barWidthFt, overlapFraction);
    bool  goesUp = (i % 2) == 0;
    float startY = goesUp ? 0.0f : fieldPassFt;
    float endY   = goesUp ? fieldPassFt : 0.0f;
    float dir    = goesUp ? 1.0f : -1.0f;
    float headingDeg = goesUp ? 0.0f : 180.0f;

    // Run in from the headland so the rover is straight and on the line
    // before any spray comes out. The first pass has no run-in: the rover is
    // already sitting at its start, which is how it gets aimed.
    if (i > 0) {
      n += emitLineTo(out + n, maxOut - n,
                      laneX, startY - dir * HEADLAND_MARGIN_FT,
                      laneX, startY, false, false);
    }

    n += emitLineTo(out + n, maxOut - n, laneX, startY, laneX, endY, true, false);
    if (n == 0 || fabsf(out[n - 1].x - laneX) > 1e-4f ||
        fabsf(out[n - 1].y - endY) > 1e-4f || !out[n - 1].spray) {
      break;
    }
    completedLanes++;

    if (i + 1 >= lanes) break;

    float headlandY = endY + dir * HEADLAND_MARGIN_FT;
    n += emitLineTo(out + n, maxOut - n, laneX, endY, laneX, headlandY, false, false);

    // The turn is a pure sideways shift measured in the rover's own frame,
    // so it flips sign on return passes: the next lane is to the rover's
    // right going up the field and to its left coming back down.
    float nextX = laneCenterX(i + 1, barWidthFt, overlapFraction);
    float shift = (nextX - laneX) * cosf(turnRad(headingDeg));

    TurnPlan p;
    if (!planHeadlandTurn(shift, planLeft, planRight, p)) return n;
    n += emitTurn(out + n, maxOut - n, p, laneX, headlandY, headingDeg);
  }

  if (n > 0 && completedLanes == lanes) out[n - 1].terminal = true;

  return n;
}
