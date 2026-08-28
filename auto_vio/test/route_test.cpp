#include <cassert>
#include <cstdio>
#include <cmath>
#include <cstring>
#include "../headland.h"
#include "../route.h"

static const float BAR     = 17.0f / 12.0f;
static const float OVERLAP = 0.15f;
// The rover's measured circles, which are not the same size.
static const float RL = 4.33f;
static const float RR = 2.92f;

static RoutePoint pts[6000];
static RoutePoint sim[6000];

static const float MAX_OFFSET   = 700.0f;
static const float PURSUIT_GAIN = 16.0f;

/** The curvature the rover actually ends up with for a given steering command,
 *  including saturation at each side's own lock. Modelling this rather than
 *  "turn as hard as needed" is what makes the simulation say the same thing
 *  the firmware will do -- an optimistic steering model here was reporting
 *  tracking a third better than the real law achieves. */
static float appliedKappa(float commandUs) {
  if (commandUs > MAX_OFFSET)  commandUs = MAX_OFFSET;
  if (commandUs < -MAX_OFFSET) commandUs = -MAX_OFFSET;
  float kMax = (commandUs >= 0.0f) ? 1.0f / RR : 1.0f / RL;
  return (commandUs / MAX_OFFSET) * kMax;
}

// Drive a simulated Ackermann rover, with the rover's real asymmetric radii,
// along the whole plan -- reversing legs included. This is the part that used
// to fail on grass with no way to see why.
static void simulatePrepared(float field, const char *label, int n) {
  const float STEP           = 0.15f;   // ft per tick
  const float LINE_T         = 1.5f;    // straight-run convergence constant
  const float LOOKAHEAD_TURN = 1.0f;    // shorter than the tightest radius
  const float CUSP_TOL       = 0.5f;
  const int   WINDOW         = 80;
  const float SPRAY_OFF      = 1.5f;
  const float SPRAY_OFF_1ST  = 3.0f;

  int lanes = laneCount(field, BAR, OVERLAP);
  assert(n > 20);

  int firstPassEnd = n;
  for (int i = 0; i < n; i++) if (!sim[i].spray) { firstPassEnd = i; break; }

  float x = sim[0].x, y = sim[0].y, heading = 0.0f;
  int idx = 0, ticks = 0, stuck = 0;
  float worstOff = 0.0f, worstStraight = 0.0f;
  int worstStraightIdx = 0;
  float worstStraightX = x, worstStraightY = y;
  bool firstPassGap = false;
  float sprayedFt = 0.0f;

  while (idx < n - 1 && ticks < 80000) {
    int prevIdx = idx;
    idx = advanceRouteIndex(sim, n, idx, x, y, WINDOW, CUSP_TOL);
    stuck = (idx == prevIdx) ? stuck + 1 : 0;
    assert(stuck < 400);            // never wedged against a cusp

    bool rev = sim[idx].reverse;
    float reference = rev ? fmodf(heading + 180.0f, 360.0f) : heading;
    float command, crossNow = 0.0f;

    if (sim[idx].turning) {
      int la = lookaheadWithinSegment(sim, n, idx, x, y, LOOKAHEAD_TURN);
      float want = bearingToWaypointDeg(sim[la].x - x, sim[la].y - y);
      command = angleDiffDeg(want, reference) * PURSUIT_GAIN;
    } else {
      // The same law the firmware uses on straight runs.
      float lineHeading = segmentHeadingDeg(sim, n, idx);
      crossNow = crossTrackFt(sim[idx].x, sim[idx].y, lineHeading, x, y);
      float headingErr = angleDiffDeg(lineHeading, reference);
      command = curvatureToCommand(lineFollowCurvature(crossNow, headingErr, LINE_T),
                                   RL, RR, MAX_OFFSET);
    }

    // Both driving forward and backing up, the travel direction turns the same
    // way for a given command: the firmware mirrors the steering in reverse and
    // the physics mirrors it back.
    float turn = appliedKappa(command) * STEP * 180.0f / (float)M_PI;
    heading = fmodf(heading + turn + 360.0f, 360.0f);
    float rad = heading * (float)M_PI / 180.0f;
    float sgn = rev ? -1.0f : 1.0f;
    x += sgn * STEP * sinf(rad);
    y += sgn * STEP * cosf(rad);

    float offX = sim[idx].x - x, offY = sim[idx].y - y;
    float off = sqrtf(offX * offX + offY * offY);
    if (off > worstOff) worstOff = off;

    // Sideways error while actually spraying is what decides whether the
    // strips land side by side or on top of each other.
    if (sim[idx].spray && fabsf(crossNow) > worstStraight) {
      worstStraight = fabsf(crossNow);
      worstStraightIdx = idx;
      worstStraightX = x;
      worstStraightY = y;
    }

    float limit = (idx < firstPassEnd) ? SPRAY_OFF_1ST : SPRAY_OFF;
    bool spraying = sim[idx].spray && off <= limit;
    if (spraying) sprayedFt += STEP;

    // The first pass must spray without interruption along its length.
    if (idx < firstPassEnd && !spraying && y > 0.5f && y < field - 0.5f) {
      firstPassGap = true;
    }
    ticks++;
  }

  assert(idx >= n - 2);          // reached the end of the plan
  assert(worstOff < 1.5f);       // never wandered far from it
  assert(!firstPassGap);         // first pass sprayed end to end

  if (worstStraight >= 0.5f * laneSpacing(BAR, OVERLAP)) {
    std::printf("route_test: %s excessive spray error %.2f ft at point %d "
                "(actual %.2f,%.2f target %.2f,%.2f)\n",
                label, worstStraight, worstStraightIdx,
                worstStraightX, worstStraightY,
                sim[worstStraightIdx].x, sim[worstStraightIdx].y);
  }

  // While spraying, the rover must stay inside half a lane width of its line.
  // Beyond that, neighbouring strips start landing on top of each other --
  // which is what "it drives back over what it just covered" looks like.
  assert(worstStraight < 0.5f * laneSpacing(BAR, OVERLAP));

  float wanted = lanes * field;
  assert(sprayedFt > 0.92f * wanted);

  printf("route_test: %s -> %d ticks, worst %.2f ft off plan, "
         "%.2f ft sideways while spraying, sprayed %.0f/%.0f ft\n",
         label, ticks, worstOff, worstStraight, sprayedFt, wanted);
}

static void simulateFollow(float field, const char *label) {
  int n = buildRoute(field, field, BAR, OVERLAP, RL, RR, sim, 6000);
  simulatePrepared(field, label, n);
}

int main() {
  // --- lane geometry ------------------------------------------------------
  {
    // The first lane sits under the rover, so placing the rover is how the
    // first pass gets aimed.
    assert(fabsf(laneCenterX(0, BAR, OVERLAP)) < 0.001f);

    assert(laneCount(10.0f, 2.0f, 0.0f) == 5);
    assert(fabsf(laneCenterX(4, 2.0f, 0.0f) - 8.0f) < 0.001f);

    // Lanes stop before the far edge: a width not divisible by the bar is
    // under-covered at the far side, never overrun.
    int m = laneCount(9.0f, 2.0f, 0.0f);
    float band = (laneCenterX(m - 1, 2.0f, 0.0f) + 1.0f) - (laneCenterX(0, 2.0f, 0.0f) - 1.0f);
    assert(band <= 9.0f + 0.001f);

    assert(laneCount(1.0f, 2.0f, 0.0f) == 1);
    printf("route_test: lane geometry ok (10ft/2ft bar = 5 lanes, real = %d)\n",
           laneCount(21.91f, BAR, OVERLAP));
  }

  // --- the real field -----------------------------------------------------
  int n = 0, lanes = 0;
  const float FIELD = 21.91f;
  {
    n = buildRoute(FIELD, FIELD, BAR, OVERLAP, RL, RR, pts, 6000);
    lanes = laneCount(FIELD, BAR, OVERLAP);
    assert(n > 100 && n < 6000);

    // Every lane must actually get sprayed somewhere along its length.
    for (int lane = 0; lane < lanes; lane++) {
      float laneX = laneCenterX(lane, BAR, OVERLAP);
      bool covered = false;
      for (int i = 0; i < n; i++) {
        if (pts[i].spray && fabsf(pts[i].x - laneX) < 0.05f) { covered = true; break; }
      }
      assert(covered);
    }

    // Spray must never be commanded outside the rectangle, and never while
    // backing up.
    for (int i = 0; i < n; i++) {
      if (!pts[i].spray) continue;
      assert(pts[i].y >= -0.01f && pts[i].y <= FIELD + 0.01f);
      assert(pts[i].x >= -0.01f && pts[i].x <= FIELD + 0.01f);
      assert(!pts[i].reverse);
    }

    // Consecutive points stay close enough to follow.
    for (int i = 1; i < n; i++) {
      float dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y;
      assert(sqrtf(dx * dx + dy * dy) <= ROUTE_STEP_FT * 2.5f);
    }

    // The route begins under the rover, already spraying.
    assert(fabsf(pts[0].x) < 0.01f && fabsf(pts[0].y) < 0.01f);
    assert(pts[0].spray && !pts[0].reverse);

    int reversals = 0;
    for (int i = 1; i < n; i++) if (pts[i].reverse != pts[i - 1].reverse) reversals++;
    assert(reversals > 0);   // it really is using three-point turns
    printf("route_test: real field -> %d points, %d lanes, %d direction changes\n",
           n, lanes, reversals);
  }

  // --- the first pass sprays from end to end ------------------------------
  // This is the pass the user cares most about, and the one most at risk:
  // it sits on x=0, so any test based on being inside the rectangle clips it.
  {
    int firstPassEnd = n;
    for (int i = 0; i < n; i++) if (!pts[i].spray) { firstPassEnd = i; break; }

    assert(firstPassEnd > 2);
    for (int i = 0; i < firstPassEnd; i++) {
      assert(pts[i].spray);
      assert(fabsf(pts[i].x) < 0.01f);          // dead straight
    }
    // and it runs the full length of the field
    assert(fabsf(pts[0].y) < 0.01f);
    assert(fabsf(pts[firstPassEnd - 1].y - FIELD) < 0.01f);
    printf("route_test: first pass sprays %d points, 0.0 -> %.2f ft, unbroken\n",
           firstPassEnd, pts[firstPassEnd - 1].y);
  }

  // --- how far outside the rectangle the route reaches --------------------
  {
    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    for (int i = 0; i < n; i++) {
      if (pts[i].x < minX) minX = pts[i].x;
      if (pts[i].x > maxX) maxX = pts[i].x;
      if (pts[i].y < minY) minY = pts[i].y;
      if (pts[i].y > maxY) maxY = pts[i].y;
    }
    // Turns must not swing wildly off the plot; anything much beyond a
    // margin plus a turning radius would need clearance the user may not have.
    assert(maxY <= FIELD + HEADLAND_MARGIN_FT + RL + 0.5f);
    assert(minY >= -(HEADLAND_MARGIN_FT + RL + 0.5f));
    printf("route_test: route spans x %.1f..%.1f, y %.1f..%.1f "
           "(needs %.1f ft clear beyond each end)\n",
           minX, maxX, minY, maxY, maxY - FIELD);
  }

  // --- a small plot still produces a complete route -----------------------
  {
    static RoutePoint small[6000];
    int sn = buildRoute(10.0f, 10.0f, BAR, OVERLAP, RL, RR, small, 6000);
    assert(sn > 50);
    int sl = laneCount(10.0f, BAR, OVERLAP);
    for (int lane = 0; lane < sl; lane++) {
      float laneX = laneCenterX(lane, BAR, OVERLAP);
      bool covered = false;
      for (int i = 0; i < sn; i++) {
        if (small[i].spray && fabsf(small[i].x - laneX) < 0.05f) { covered = true; break; }
      }
      assert(covered);
    }
    printf("route_test: 10x10 -> %d points, %d lanes\n", sn, sl);
  }

  // --- a one-lane area is a straight-line test ----------------------------
  // Setting the width to less than one bar gives a single pass and nothing
  // else, which isolates the tracker: no turns, no lane stepping, just "can
  // this rover hold a line". Worth guaranteeing, since it is the first thing
  // to check when coverage looks wrong.
  {
    static RoutePoint line[6000];
    int ln = buildRoute(20.0f, 1.0f, BAR, OVERLAP, RL, RR, line, 6000);
    assert(laneCount(1.0f, BAR, OVERLAP) == 1);
    assert(ln > 10);
    for (int i = 0; i < ln; i++) {
      assert(line[i].spray);            // sprays the whole way
      assert(!line[i].reverse);         // never backs up
      assert(!line[i].turning);         // and never turns
      assert(fabsf(line[i].x) < 0.01f); // dead straight along x=0
    }
    assert(fabsf(line[0].y) < 0.01f);
    assert(fabsf(line[ln - 1].y - 20.0f) < 0.01f);
    printf("route_test: 20x1 -> %d points, one straight sprayed pass\n", ln);
  }

  // --- following the route, reversing included ----------------------------
  simulateFollow(FIELD, "real field");
  simulateFollow(10.0f, "10x10");
  simulateFollow(6.0f, "6x6");

  // --- prefer a continuous forward route when the pavement fits -----------
  {
    static RoutePoint selected[6000];
    RouteSelection forward = selectRoute(
        FIELD, FIELD, BAR, OVERLAP, RL, RR,
        100.0f, 100.0f, true, selected, 6000);
    assert(forward.style == ROUTE_FORWARD_ONLY);
    assert(forward.count > 0 && !forward.requirements.truncated);
    assert(forward.requirements.reversals == 0);
    float forwardMinX = selected[0].x, forwardMaxX = selected[0].x;
    for (int i = 1; i < forward.count; ++i) {
      if (selected[i].x < forwardMinX) forwardMinX = selected[i].x;
      if (selected[i].x > forwardMaxX) forwardMaxX = selected[i].x;
    }
    std::printf("route_test: forward-only -> %d points, needs %.1f/%.1f ft headland, x %.1f..%.1f\n",
                forward.count, forward.requirements.beforeStartFt,
                forward.requirements.beyondEndFt, forwardMinX, forwardMaxX);
    for (int i = 0; i < forward.count; ++i) assert(!selected[i].reverse);

    // The far-lane ordering changes visit order, never lane placement: every
    // lane at the unchanged 15% overlap spacing still receives a full pass.
    for (int lane = 0; lane < lanes; ++lane) {
      const float laneX = laneCenterX(lane, BAR, OVERLAP);
      bool covered = false;
      for (int i = 0; i < forward.count; ++i) {
        if (selected[i].spray && std::fabs(selected[i].x - laneX) < 0.05f) {
          covered = true;
          break;
        }
      }
      assert(covered);
    }

    // Exercise the selected forward-only plan through the same tracker model
    // as the reversing route. Geometry alone cannot prove that its longer
    // Dubins transitions remain followable by the production control law.
    std::memcpy(sim, selected, forward.count * sizeof(RoutePoint));
    simulatePrepared(FIELD, "forward-only", forward.count);

    static RoutePoint kturn[6000];
    int kCount = buildRoute(FIELD, FIELD, BAR, OVERLAP, RL, RR, kturn, 6000);
    RouteRequirements kNeeds = inspectRoute(kturn, kCount, FIELD);
    assert(forward.requirements.beforeStartFt > kNeeds.beforeStartFt + 0.01f ||
           forward.requirements.beyondEndFt > kNeeds.beyondEndFt + 0.01f);

    RouteSelection fallback = selectRoute(
        FIELD, FIELD, BAR, OVERLAP, RL, RR,
        kNeeds.beforeStartFt, kNeeds.beyondEndFt,
        true, selected, 6000);
    assert(fallback.style == ROUTE_THREE_POINT);
    assert(fallback.count == kCount);
    assert(fallback.requirements.reversals > 0);
  }

  // Passes are addressable so a faulted mission can resume on the one it was
  // driving instead of re-covering the whole rectangle.
  {
    const float FIELD = 24.0f;
    static RoutePoint plan[6000];
    const int count = buildRoute(FIELD, FIELD, BAR, OVERLAP, RL, RR, plan, 6000);
    const int lanes = laneCount(FIELD, BAR, OVERLAP);
    assert(routePassCount(plan, count) == lanes);
    assert(routePassStartIndex(plan, count, 0) == 0);
    assert(routePassStartIndex(plan, count, lanes) == -1);
    assert(routePassStartIndex(plan, count, -1) == -1);
    for (int pass = 0; pass < lanes; ++pass) {
      const int start = routePassStartIndex(plan, count, pass);
      assert(start >= 0 && plan[start].spray);
      assert(start == 0 || !plan[start - 1].spray);
      if (pass > 0) assert(start > routePassStartIndex(plan, count, pass - 1));
      // Resuming lands the rover on the lane it is about to spray.
      assert(std::fabs(plan[start].x - laneCenterX(pass, BAR, OVERLAP)) < 0.05f);
      assert(routePassIndexAt(plan, count, start) == pass);
    }
  }

  printf("route_test: all assertions passed\n");
  return 0;
}
