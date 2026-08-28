#include <cassert>
#include <cstdio>
#include <cmath>
#include "../route.h"
#include "../steering.h"

static const float BAR     = 21.0f / 12.0f;
static const float OVERLAP = 0.0f;
static const float RL = 4.33f;    // measured
static const float RR = 2.92f;    // measured
static const float LEFT_LOCK  = 2390.0f;
static const float RIGHT_LOCK = 700.0f;
static const float MAX_OFFSET = 700.0f;
static const float FIELD = 14.0f;

static RoutePoint rt[6000];

// The rover's real steering: a pulse either side of the true centre bends it,
// and full lock either way reproduces the measured circles by construction.
static const float TRUE_CENTRE = 1709.0f;
static float curvatureFor(float pulseUs) {
  // 1/ft, positive to the right. Right lock is below centre.
  static const float PER_US = (1.0f / RR) / (TRUE_CENTRE - RIGHT_LOCK);
  return (TRUE_CENTRE - pulseUs) * PER_US;
}

struct LaneStat { double sum; int n; };

/** Drive the plan with a given belief about where the steering centre is, and
 *  report the average sideways error on each lane. `learn` turns on the trim
 *  estimator. Returns the number of lanes measured. */
static int runField(float assumedCentre, bool learn, bool pursuitOnStraights,
                    LaneStat *out, int maxLanes, float *finalTrim) {
  const float STEP = 0.15f, LOOKAHEAD_TURN = 1.0f, LINE_T = 1.5f;
  const float CUSP_TOL = 0.1f, PURSUIT_GAIN = 16.0f;
  const int   WINDOW = 80;
  const float TRIM_RATE = 0.05f, TRIM_LIMIT = 250.0f, TRIM_NEAR = 0.4f;

  for (int i = 0; i < maxLanes; i++) { out[i].sum = 0; out[i].n = 0; }

  int n = buildRoute(FIELD, FIELD, BAR, OVERLAP, RL, RR, rt, 6000);
  float spacing = laneSpacing(BAR, OVERLAP);

  float x = rt[0].x, y = rt[0].y, heading = 0.0f;
  float trim = 0.0f;
  int idx = 0, ticks = 0;
  unsigned long sinceTrim = 0;

  while (idx < n - 1 && ticks < 80000) {
    idx = advanceRouteIndex(rt, n, idx, x, y, WINDOW, CUSP_TOL);
    bool rev = rt[idx].reverse;
    float reference = rev ? fmodf(heading + 180.0f, 360.0f) : heading;
    float command;

    // Same split the firmware uses: chase a point round a turn, steer onto the
    // line itself on a straight run.
    if (rt[idx].turning) {
      int la = lookaheadWithinSegment(rt, n, idx, x, y,
                                      LOOKAHEAD_TURN);
      float dx = rt[la].x - x, dy = rt[la].y - y;
      float want = bearingToWaypointDeg(dx, dy);
      command = curvatureToCommand(
          purePursuitCurvature(angleDiffDeg(want, reference),
                               std::sqrt(dx * dx + dy * dy)),
          RL, RR, MAX_OFFSET);
    } else if (pursuitOnStraights) {
      int la = lookaheadWithinSegment(rt, n, idx, x, y, 2.5f);
      float want = bearingToWaypointDeg(rt[la].x - x, rt[la].y - y);
      command = angleDiffDeg(want, reference) * PURSUIT_GAIN;
    } else {
      float lineHeading = segmentHeadingDeg(rt, n, idx);
      float cross = crossTrackFt(rt[idx].x, rt[idx].y, lineHeading, x, y);
      float headingErr = angleDiffDeg(lineHeading, reference);
      command = curvatureToCommand(lineFollowCurvature(cross, headingErr, LINE_T),
                                   RL, RR, MAX_OFFSET);
    }

    float centre = assumedCentre + trim;
    float pulse = steerPulseUs(rev ? -command : command, centre,
                               LEFT_LOCK, RIGHT_LOCK, MAX_OFFSET);

    float offX = rt[idx].x - x, offY = rt[idx].y - y;
    float off = sqrtf(offX * offX + offY * offY);

    if (learn && !rev && !rt[idx].turning && off <= TRIM_NEAR) {
      if (++sinceTrim >= 2) {          // ~50ms at this tick rate
        sinceTrim = 0;
        trim = updateSteeringTrim(trim, pulse, centre, TRIM_RATE, TRIM_LIMIT);
      }
    }

    // Physics: the wheels are where they are, whatever the code believes.
    float kappa = curvatureFor(pulse);
    float dh = kappa * STEP * 180.0f / (float)M_PI;
    heading = fmodf(heading + (rev ? -dh : dh) + 360.0f, 360.0f);

    float rad = heading * (float)M_PI / 180.0f;
    float sgn = rev ? -1.0f : 1.0f;
    x += sgn * STEP * sinf(rad);
    y += sgn * STEP * cosf(rad);

    // Record how far off its lane the rover sits, along the settled middle of
    // each pass where the answer means something.
    if (rt[idx].spray && y > FIELD * 0.2f && y < FIELD * 0.8f) {
      int lane = (int)lroundf(rt[idx].x / spacing);
      if (lane >= 0 && lane < maxLanes) {
        out[lane].sum += (x - rt[idx].x);
        out[lane].n++;
      }
    }
    ticks++;
  }

  if (finalTrim) *finalTrim = trim;
  return laneCount(FIELD, BAR, OVERLAP);
}

static void report(const char *label, LaneStat *s, int lanes) {
  printf("  %-22s", label);
  for (int i = 0; i < lanes && i < 8; i++) {
    printf(" %+5.2f", s[i].n ? s[i].sum / s[i].n : 0.0);
  }
  printf("\n");
}

int main() {
  // --- where the centre actually is --------------------------------------
  {
    float c = steeringCentreUs(RL, RR, LEFT_LOCK, RIGHT_LOCK);
    assert(fabsf(c - 1709.0f) < 2.0f);

    // Equal circles put it at mid-travel, as they must.
    float sym = steeringCentreUs(4.0f, 4.0f, LEFT_LOCK, RIGHT_LOCK);
    assert(fabsf(sym - 1545.0f) < 0.5f);

    // And the recovered centre reproduces the measured radii, which is the
    // consistency check that says the model is the right one.
    assert(fabsf(curvatureFor(RIGHT_LOCK) - 1.0f / RR) < 1e-4f);
    assert(fabsf(curvatureFor(LEFT_LOCK) + 1.0f / RL) < 1e-3f);
    printf("steering_test: centre from 4.33/2.92 ft circles = %.0fus "
           "(servo mid-travel is 1545)\n", c);
  }

  // --- command mapping ----------------------------------------------------
  {
    float c = steeringCentreUs(RL, RR, LEFT_LOCK, RIGHT_LOCK);
    assert(fabsf(steerPulseUs(0.0f, c, LEFT_LOCK, RIGHT_LOCK, MAX_OFFSET) - c) < 0.01f);
    assert(fabsf(steerPulseUs(MAX_OFFSET, c, LEFT_LOCK, RIGHT_LOCK, MAX_OFFSET)
                 - RIGHT_LOCK) < 0.01f);
    assert(fabsf(steerPulseUs(-MAX_OFFSET, c, LEFT_LOCK, RIGHT_LOCK, MAX_OFFSET)
                 - LEFT_LOCK) < 0.01f);
    // Over-commanding cannot push past the stops.
    assert(steerPulseUs(5000.0f, c, LEFT_LOCK, RIGHT_LOCK, MAX_OFFSET) >= RIGHT_LOCK);
  }

  const int MAXL = 32;
  static LaneStat wrong[MAXL], right[MAXL], learned[MAXL];
  int lanes = 0;
  float trimWrong = 0.0f, trimRight = 0.0f;

  printf("steering_test: mean sideways error per lane, first 8 lanes (ft)\n");

  // --- the bug ------------------------------------------------------------
  // Believing the centre is the servo's mid-travel. Every pass should sit off
  // to one side, and the side should alternate, which is what draws pairs of
  // overlapping lines.
  {
    lanes = runField(1500.0f, false, true, wrong, MAXL, NULL);
    report("pursuit, centre=1500:", wrong, lanes);

    float worst = 0.0f;
    for (int i = 0; i < lanes; i++) {
      if (!wrong[i].n) continue;
      float m = (float)(wrong[i].sum / wrong[i].n);
      if (fabsf(m) > worst) worst = fabsf(m);
    }
    // The wrong centre still produces a large pass error.
    assert(worst > 0.30f);
  }

  // --- the fix ------------------------------------------------------------
  {
    float c = steeringCentreUs(RL, RR, LEFT_LOCK, RIGHT_LOCK);
    lanes = runField(c, false, false, right, MAXL, &trimRight);
    report("line follow, centre ok:", right, lanes);

    float worst = 0.0f;
    for (int i = 0; i < lanes; i++) {
      if (!right[i].n) continue;
      float m = fabsf((float)(right[i].sum / right[i].n));
      if (m > worst) worst = m;
    }
    // Well inside a lane width, so neighbouring passes stay apart.
    assert(worst < 0.20f);
    printf("  worst lane error: %.2f ft -> %.2f ft (lane spacing %.2f ft)\n",
           0.0f, worst, laneSpacing(BAR, OVERLAP));
  }

  // --- the safety net -----------------------------------------------------
  // Even starting from the wrong belief, the trim estimator has to find the
  // centre on its own, so a rover whose steering shifts does not need
  // re-measuring to keep driving straight.
  {
    lanes = runField(1500.0f, true, false, learned, MAXL, &trimWrong);
    report("line follow, 1500+learn:", learned, lanes);

    // It should have learned most of the 209us it was out by.
    assert(trimWrong > 120.0f);

    // And the later passes, after it has converged, should be close in.
    float lateWorst = 0.0f;
    int counted = 0;
    for (int i = lanes / 2; i < lanes; i++) {
      if (!learned[i].n) continue;
      float m = fabsf((float)(learned[i].sum / learned[i].n));
      if (m > lateWorst) lateWorst = m;
      counted++;
    }
    assert(counted > 0);
    assert(lateWorst < 0.30f);
    printf("  learned trim %+.0fus of the %+.0fus it was out by; "
           "late-pass error %.2f ft\n", trimWrong, TRUE_CENTRE - 1500.0f, lateWorst);
  }

  // --- what actually ships ------------------------------------------------
  // Centre computed from the measured circles, with learning on top.
  {
    static LaneStat both[MAXL];
    float c = steeringCentreUs(RL, RR, LEFT_LOCK, RIGHT_LOCK);
    float trim = 0.0f;
    lanes = runField(c, true, false, both, MAXL, &trim);
    report("shipping config:", both, lanes);

    float worst = 0.0f;
    for (int i = 0; i < lanes; i++) {
      if (!both[i].n) continue;
      float m = fabsf((float)(both[i].sum / both[i].n));
      if (m > worst) worst = m;
    }
    // Every pass must sit well inside half a lane width of its line, or
    // neighbouring passes start closing on each other again.
    assert(worst < 0.5f * laneSpacing(BAR, OVERLAP));
    printf("  worst lane error %.2f ft against %.2f ft spacing, residual trim %+.0fus\n",
           worst, laneSpacing(BAR, OVERLAP), trim);
  }

  printf("steering_test: all assertions passed\n");
  return 0;
}
