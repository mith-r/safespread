#include <cassert>
#include <cstdio>
#include <cmath>
#include "../nav_math.h"

// How far does a rover have to drive to get onto a straight line, and does it
// cross the line on the way? That second question is the one that decides
// whether passes come out straight or wavy, because a pass is only about ten
// feet long -- there is no room to settle out an oscillation.

static const float RL = 5.54f;    // measured wet turning circles
static const float RR = 5.05f;
static const float MAX_OFFSET = 700.0f;

struct Result {
  float settleFt;     // distance to get within 0.1 ft and stay there
  float overshootFt;  // furthest past the line on the far side
  bool  settled;
};

// The line is x = 0, running along +Y. The rover starts `startOff` to the
// right of it, pointing along it.
static Result run(bool pursuit, float T, float lookahead,
                  float startOff, float startHeadingDeg) {
  const float STEP = 0.1f;
  float x = startOff, y = 0.0f, heading = startHeadingDeg;
  Result r = { 0.0f, 0.0f, false };
  float travelled = 0.0f;
  float lastBad = 0.0f;               // last point it was still off the line

  for (int i = 0; i < 4000; i++) {
    float kappa;

    if (pursuit) {
      // Aim at a point `lookahead` further along the line.
      float tx = 0.0f, ty = y + lookahead;
      float want = bearingToWaypointDeg(tx - x, ty - y);
      float err = angleDiffDeg(want, heading);
      // Same proportional law the firmware used, expressed as curvature.
      float cmd = err * 16.0f;
      if (cmd > MAX_OFFSET) cmd = MAX_OFFSET;
      if (cmd < -MAX_OFFSET) cmd = -MAX_OFFSET;
      kappa = (cmd / MAX_OFFSET) * ((cmd >= 0.0f) ? 1.0f / RR : 1.0f / RL);
    } else {
      float cross = crossTrackFt(0.0f, 0.0f, 0.0f, x, y);
      float headingErr = angleDiffDeg(0.0f, heading);
      kappa = lineFollowCurvature(cross, headingErr, T);
      float cmd = curvatureToCommand(kappa, RL, RR, MAX_OFFSET);
      kappa = (cmd / MAX_OFFSET) * ((cmd >= 0.0f) ? 1.0f / RR : 1.0f / RL);
    }

    // The rover cannot turn tighter than its circles, whatever is asked.
    float kMax = (kappa >= 0.0f) ? 1.0f / RR : 1.0f / RL;
    if (fabsf(kappa) > fabsf(kMax)) kappa = kMax;

    heading += kappa * STEP * 180.0f / (float)M_PI;
    float rad = heading * (float)M_PI / 180.0f;
    x += STEP * sinf(rad);
    y += STEP * cosf(rad);
    travelled += STEP;

    // Overshoot means ending up on the far side of the line from where it
    // started, which is a pass that visibly weaves.
    if (startOff > 0.0f && x < -r.overshootFt) r.overshootFt = -x;
    if (startOff < 0.0f && x > r.overshootFt)  r.overshootFt = x;

    if (fabsf(x) > 0.1f) lastBad = travelled;
    if (travelled - lastBad > 3.0f) { r.settled = true; r.settleFt = lastBad; break; }
  }
  return r;
}

int main() {
  // --- the geometry the law is built on ----------------------------------
  {
    // A rover a foot to the right of a line running north.
    assert(fabsf(crossTrackFt(0, 0, 0.0f, 1.0f, 5.0f) - 1.0f) < 1e-4f);
    // Same rover, line running south: it is now on the left.
    assert(fabsf(crossTrackFt(0, 0, 180.0f, 1.0f, 5.0f) + 1.0f) < 1e-4f);

    // Off to the right and parallel: steer left.
    assert(lineFollowCurvature(1.0f, 0.0f, 1.5f) < 0.0f);
    // On the line but pointing left of it: steer right.
    assert(lineFollowCurvature(0.0f, 10.0f, 1.5f) > 0.0f);
    // On the line and parallel: hold straight.
    assert(fabsf(lineFollowCurvature(0.0f, 0.0f, 1.5f)) < 1e-6f);

    // A command asking to bend right harder than the right circle allows is
    // capped at full lock, not at some average of the two circles.
    assert(fabsf(curvatureToCommand(1.0f / RR, RL, RR, MAX_OFFSET) - MAX_OFFSET) < 0.01f);
    assert(fabsf(curvatureToCommand(-1.0f / RL, RL, RR, MAX_OFFSET) + MAX_OFFSET) < 0.01f);
  }

  printf("line_test: recovering from 1.0 ft off the line, pointing along it\n");

  Result p = run(true, 0.0f, 2.5f, 1.0f, 0.0f);
  printf("  pure pursuit (2.5ft lookahead): settles in %.1f ft, crosses %.2f ft past\n",
         p.settleFt, p.overshootFt);

  Result best = { 0, 0, false };
  float bestT = 0.0f;
  for (float T = 1.0f; T <= 3.01f; T += 0.5f) {
    Result r = run(false, T, 0.0f, 1.0f, 0.0f);
    printf("  line follow T=%.1f ft:          settles in %.1f ft, crosses %.2f ft past\n",
           T, r.settleFt, r.overshootFt);
    if (r.settled && (!best.settled || r.settleFt < best.settleFt)) { best = r; bestT = T; }
  }

  assert(p.settled);
  assert(best.settled);

  // The point of the change: it must not cross the line on its way in, and it
  // must be onto it well inside the length of a pass.
  assert(best.overshootFt < 0.05f);
  assert(best.settleFt < p.settleFt);
  printf("  -> T=%.1f ft settles %.1f ft sooner and does not cross the line\n",
         bestT, p.settleFt - best.settleFt);

  // --- worse starting conditions -----------------------------------------
  // Coming out of a turn the rover is off the line AND pointing across it.
  {
    for (float off = -1.5f; off <= 1.5f; off += 0.5f) {
      for (float ang = -30.0f; ang <= 30.0f; ang += 15.0f) {
        if (fabsf(off) < 0.01f && fabsf(ang) < 0.01f) continue;
        Result r = run(false, 1.5f, 0.0f, off, ang);
        assert(r.settled);
        assert(r.settleFt < 12.0f);
      }
    }
    printf("line_test: settles from every entry up to 1.5 ft off and 30 deg across\n");
  }

  printf("line_test: all assertions passed\n");
  return 0;
}
