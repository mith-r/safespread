#include <cassert>
#include <cstdio>
#include <cmath>
#include "../route.h"

// Following a planned arc. The question is not only "does it get round" -- the
// old law did get round -- but how much of the turn it spends with the steering
// pinned against a mechanical stop. A command that is saturated is a command
// carrying no information: it says "as hard as possible" whether the rover is
// an inch off the arc or a foot off it, and it cannot ask for less until the
// error changes sign. The 2026-08-28 mission held a lock for 56% of its control
// frames and swung the wheels stop to stop at every one of its eight direction
// changes.

static const float RL = 5.54f;          // steering map's turning circles
static const float RR = 5.05f;
static const float MAX_OFFSET = 700.0f;
static const float LOOKAHEAD_TURN_FT = 1.0f;
static const float CORRECTION_DISTANCE_FT = 3.0f;
static const float CORRECTION_LIMIT_FRACTION = 0.5f;

static const int MAX_ARC_POINTS = 256;

/** A right-hand arc of radius `radiusFt` starting at the origin pointing along
 *  +Y, sampled the way the planner samples one. */
static int buildArc(RoutePoint *out, int maxOut, float radiusFt, float sweepDeg) {
  const float sweep = sweepDeg * (float)M_PI / 180.0f;
  const float arcLen = radiusFt * sweep;
  int n = 0;
  for (float s = 0.0f; s <= arcLen + 1e-4f && n < maxOut; s += ROUTE_STEP_FT) {
    const float theta = s / radiusFt;
    out[n].x = radiusFt * (1.0f - cosf(theta));
    out[n].y = radiusFt * sinf(theta);
    out[n].spray = false;
    out[n].reverse = false;
    out[n].turning = true;
    out[n].terminal = false;
    n++;
  }
  return n;
}

struct Trace {
  float lockedFraction;   // share of control steps with the command saturated
  int   lockReversals;    // times the command crossed from one lock to the other
  float worstOffArcFt;    // furthest the rover ever sat from the planned arc
  bool  completed;        // reached the end of the plan
};

// Deterministic jitter standing in for what the phone actually delivers. The
// rover does not steer on where it is, it steers on where it was told it is,
// and on 2026-08-28 that difference was enough to move the lookahead point from
// one route sample to another between consecutive frames -- the reported
// heading error jumped from -15.9 to +0.2 degrees in one 60 Hz step without the
// rover having moved three inches.
struct Jitter {
  unsigned state = 12345u;
  float next() {                       // uniform in [-1, 1)
    state = state * 1103515245u + 12345u;
    return (float)((state >> 16) & 0x7fff) / 16384.0f - 1.0f;
  }
};

/** Drive the arc. `feedForward` selects the new law; without it this is the
 *  law the mission flew: pure pursuit at a one foot lookahead and nothing
 *  else. `noise` scales the pose error the controller sees. */
static Trace run(const RoutePoint *route, int count, float radiusFt,
                 bool feedForward, float startOffFt, float startAcrossDeg,
                 float noise = 1.0f) {
  const float STEP = 0.1f;
  const float POSITION_NOISE_FT = 0.05f * noise;
  const float HEADING_NOISE_DEG = 1.5f * noise;
  Jitter jitter;
  // Start off the arc and pointing across it, the way a rover actually enters
  // a headland after a pass.
  float x = startOffFt;
  float y = 0.0f;
  float heading = startAcrossDeg;

  Trace trace = { 0.0f, 0, 0.0f, false };
  int locked = 0;
  int steps = 0;
  int index = 0;
  int lockSide = 0;   // -1 left lock, +1 right lock, 0 not against a stop

  for (int i = 0; i < 6000; i++) {
    // Everything the controller sees is the reported pose, not the true one.
    const float mx = x + POSITION_NOISE_FT * jitter.next();
    const float my = y + POSITION_NOISE_FT * jitter.next();
    const float mheading = heading + HEADING_NOISE_DEG * jitter.next();

    index = nearestRouteIndex(route, count, index, mx, my, 40);
    if (index >= count - 1) { trace.completed = true; break; }

    const int la = lookaheadWithinSegment(route, count, index, mx, my,
                                          LOOKAHEAD_TURN_FT);
    const float want = bearingToWaypointDeg(route[la].x - mx, route[la].y - my);
    const float err = angleDiffDeg(want, mheading);
    const float targetDistance = sqrtf((route[la].x - mx) * (route[la].x - mx) +
                                       (route[la].y - my) * (route[la].y - my));

    float kappa;
    if (feedForward) {
      float correction = purePursuitCurvature(err, targetDistance,
                                              CORRECTION_DISTANCE_FT);
      const float limit = CORRECTION_LIMIT_FRACTION *
          ((correction >= 0.0f) ? 1.0f / RR : 1.0f / RL);
      if (correction > limit) correction = limit;
      if (correction < -limit) correction = -limit;
      kappa = plannedCurvature(route, count, index) + correction;
    } else {
      kappa = purePursuitCurvature(err, targetDistance);
    }

    // The steering map cannot express a curvature tighter than full lock, and
    // the vehicle could not drive it if it could.
    const float kMax = (kappa >= 0.0f) ? 1.0f / RR : 1.0f / RL;
    if (fabsf(kappa) >= fabsf(kMax)) {
      const int side = (kappa >= 0.0f) ? 1 : -1;
      if (lockSide != 0 && side != lockSide) trace.lockReversals++;
      lockSide = side;
      kappa = (kappa >= 0.0f) ? kMax : -kMax;
      locked++;
    }
    steps++;

    heading += kappa * STEP * 180.0f / (float)M_PI;
    const float rad = heading * (float)M_PI / 180.0f;
    x += STEP * sinf(rad);
    y += STEP * cosf(rad);

    // Distance from the planned circle, whose centre is at (radiusFt, 0).
    const float offArc = fabsf(sqrtf((x - radiusFt) * (x - radiusFt) + y * y) -
                               radiusFt);
    if (offArc > trace.worstOffArcFt) trace.worstOffArcFt = offArc;
  }

  trace.lockedFraction = steps > 0 ? (float)locked / (float)steps : 1.0f;
  return trace;
}

int main() {
  // --- plannedCurvature reads the arc out of the plan ----------------------
  {
    RoutePoint arc[MAX_ARC_POINTS];
    const int n = buildArc(arc, MAX_ARC_POINTS, 5.05f, 180.0f);
    assert(n > 3 * PLAN_CURVATURE_SPAN);

    // A right-hand arc bends toward increasing heading, so its curvature is
    // positive, and it is the reciprocal of the radius it was built from.
    for (int i = 0; i < n; i++) {
      const float k = plannedCurvature(arc, n, i);
      assert(k > 0.0f);
      assert(fabsf(k - 1.0f / 5.05f) < 0.01f);
    }

    // Mirroring the arc about x flips the sign and nothing else.
    RoutePoint mirrored[MAX_ARC_POINTS];
    for (int i = 0; i < n; i++) {
      mirrored[i] = arc[i];
      mirrored[i].x = -arc[i].x;
    }
    for (int i = 0; i < n; i++) {
      assert(fabsf(plannedCurvature(mirrored, n, i) +
                   plannedCurvature(arc, n, i)) < 1e-4f);
    }

    // A straight run is not bending at all.
    RoutePoint line[MAX_ARC_POINTS];
    for (int i = 0; i < 40; i++) {
      line[i] = {0.0f, i * ROUTE_STEP_FT, true, false, false, false};
    }
    for (int i = 0; i < 40; i++) assert(fabsf(plannedCurvature(line, 40, i)) < 1e-4f);

    // A leg too short for the full span still reports the arc, measured across
    // a shorter one -- less precisely, but a short arc is exactly where handing
    // the whole maneuver back to the error term hurts most.
    for (int shortCount = 3; shortCount <= 2 * PLAN_CURVATURE_SPAN; shortCount++) {
      const float k = plannedCurvature(arc, shortCount, 0);
      assert(k > 0.0f);
      assert(fabsf(k - 1.0f / 5.05f) < 0.05f);
    }
    // Two points describe a chord, not a curve, and get no answer at all.
    assert(fabsf(plannedCurvature(arc, 2, 0)) < 1e-6f);
    printf("turn_follow_test: plannedCurvature recovers the planned arc\n");
  }

  // --- the lookahead divisor has a floor -----------------------------------
  {
    // Ten degrees off, aiming at a point two inches away: without a floor this
    // asks for a curvature of 12 per foot -- a two-inch turning circle -- so
    // the command is pinned to full lock for the whole approach to the cusp.
    const float unfloored = purePursuitCurvature(10.0f, 0.17f);
    assert(unfloored > 2.0f);
    const float floored = purePursuitCurvature(10.0f, 0.17f, 3.0f);
    assert(fabsf(floored - 2.0f * sinf(10.0f * (float)M_PI / 180.0f) / 3.0f) < 1e-5f);
    assert(fabsf(floored) < 1.0f / 5.05f);   // no longer saturating on its own

    // Beyond the floor it is unchanged: this only bounds the divisor.
    assert(fabsf(purePursuitCurvature(10.0f, 6.0f, 3.0f) -
                 purePursuitCurvature(10.0f, 6.0f)) < 1e-6f);
    printf("turn_follow_test: pursuit divisor floor stops a near cusp saturating\n");
  }

  // --- an arc the rover has some steering left to correct with -------------
  // The planner is free to choose the radius; when it leaves any margin at all
  // over the minimum circle, feeding the arc forward takes the command off the
  // stops entirely.
  {
    RoutePoint arc[MAX_ARC_POINTS];
    const float radius = 1.3f * 5.05f;
    const int n = buildArc(arc, MAX_ARC_POINTS, radius, 170.0f);

    printf("turn_follow_test: entering a %.2f ft arc 0.3 ft off and 5 deg across\n",
           radius);
    const Trace before = run(arc, n, radius, false, 0.3f, 5.0f);
    const Trace after = run(arc, n, radius, true, 0.3f, 5.0f);
    printf("  pursuit only:      %3.0f%% at full lock, %d lock-to-lock swings, worst %.2f ft off arc\n",
           before.lockedFraction * 100.0f, before.lockReversals, before.worstOffArcFt);
    printf("  arc + correction:  %3.0f%% at full lock, %d lock-to-lock swings, worst %.2f ft off arc\n",
           after.lockedFraction * 100.0f, after.lockReversals, after.worstOffArcFt);

    assert(before.completed && after.completed);
    assert(before.lockedFraction > 0.4f);
    assert(after.lockedFraction < 0.5f * before.lockedFraction);
    assert(before.lockReversals > 0);
    assert(after.lockReversals == 0);
    // Tracking is no worse for it. Both laws hold the arc to within a few
    // inches here; the difference between them is what the command looks like,
    // not where the rover ends up on an arc it has room to drive.
    assert(after.worstOffArcFt < before.worstOffArcFt + 0.1f);
    printf("  -> saturation falls from %.0f%% to %.0f%% of the turn\n",
           before.lockedFraction * 100.0f, after.lockedFraction * 100.0f);
  }

  // --- an arc planned at the rover's minimum circle -------------------------
  // Here the plan itself asks for full lock, so the command is on the stop
  // whatever the law is -- there is no tighter answer to give. What must not
  // happen is the wheels swinging to the *other* stop, which is what a relay
  // driven by a one foot lookahead does and what cost the mission a second of
  // wrong-way steering at every direction change.
  {
    RoutePoint arc[MAX_ARC_POINTS];
    const float radius = 5.05f;
    const int n = buildArc(arc, MAX_ARC_POINTS, radius, 170.0f);
    const Trace before = run(arc, n, radius, false, 0.3f, 5.0f);
    const Trace after = run(arc, n, radius, true, 0.3f, 5.0f);
    printf("turn_follow_test: at the minimum circle, lock-to-lock swings %d -> %d, worst off arc %.2f -> %.2f ft\n",
           before.lockReversals, after.lockReversals,
           before.worstOffArcFt, after.worstOffArcFt);
    assert(before.completed && after.completed);
    assert(before.lockReversals > 0);
    assert(after.lockReversals == 0);
    assert(after.worstOffArcFt < before.worstOffArcFt);
  }

  // --- and from every entry a headland actually produces --------------------
  {
    RoutePoint arc[MAX_ARC_POINTS];
    const float radius = 1.3f * 5.05f;
    const int n = buildArc(arc, MAX_ARC_POINTS, radius, 170.0f);
    for (float off = -0.5f; off <= 0.5f; off += 0.25f) {
      for (float across = -10.0f; across <= 10.0f; across += 5.0f) {
        const Trace t = run(arc, n, radius, true, off, across);
        assert(t.completed);
        assert(t.lockReversals == 0);
        assert(t.worstOffArcFt < 1.0f);
      }
    }
    printf("turn_follow_test: holds the arc from every entry up to 0.5 ft and 10 deg off\n");
  }

  printf("turn_follow_test: all assertions passed\n");
  return 0;
}
