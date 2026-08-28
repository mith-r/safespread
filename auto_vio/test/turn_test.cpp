#include <cassert>
#include <cstdio>
#include <cmath>
#include "../turn.h"

// The rover's real, measured circles. They differ by 48% because the steering
// trim is off-centre, which is exactly the case a single averaged radius gets
// wrong.
static const float RL = 5.54f;
static const float RR = 5.05f;

// Walk the maneuver and confirm it does what it claims: arrives at the right
// sideways offset, back on the line it started from, pointing the opposite
// way, having never turned tighter than the rover's tightest circle.
static void checkTurn(float shift, float rLeft, float rRight) {
  TurnPlan p;
  bool ok = planHeadlandTurn(shift, rLeft, rRight, p);
  assert(ok && p.ok);
  assert(p.lengthFt > 0.0f && p.lengthFt < 500.0f);

  // Every arc must use the radius belonging to the side the wheels are on.
  // Getting this wrong is the whole failure mode we are guarding against.
  for (int i = 0; i < p.legCount; i++) {
    if (!p.leg[i].isArc) continue;
    float want = p.leg[i].steerLeft ? rLeft : rRight;
    assert(fabsf(p.leg[i].radius - want) < 1e-4f);
  }

  float x, y, h; bool rev;
  turnPoseAt(p, p.lengthFt, x, y, h, rev);

  if (fabsf(x - shift) > 0.02f || fabsf(y) > 0.02f || fabsf(fabsf(h) - 180.0f) > 0.1f) {
    printf("FAIL shift=%.2f -> end=(%.3f,%.3f,%.2f) len=%.2f legs=%d rev=%d\n",
           shift, x, y, h, p.lengthFt, p.legCount, p.reversals);
  }
  assert(fabsf(x - shift) < 0.02f);       // lands the requested distance over
  assert(fabsf(y) < 0.02f);               // back on the headland line
  assert(fabsf(fabsf(h) - 180.0f) < 0.1f);// facing back down the field
  assert(!rev);                           // and leaves the turn driving forward

  // No step may imply a circle tighter than the rover owns.
  const float step = 0.05f;
  float px, py, ph; bool pr;
  turnPoseAt(p, 0.0f, px, py, ph, pr);
  float tightest = (rLeft < rRight) ? rLeft : rRight;
  float walked = 0.0f;
  for (float s = step; s <= p.lengthFt; s += step) {
    float cx, cy, ch; bool cr;
    turnPoseAt(p, s, cx, cy, ch, cr);
    float moved = sqrtf((cx - px) * (cx - px) + (cy - py) * (cy - py));
    float turned = fabsf(ch - ph) * TURN_PI / 180.0f;
    // A three-point turn has genuine cusps: at the moment it changes from
    // forward to reverse the rover doubles back, so a step straddling one
    // covers almost no ground while still rotating. Near the family boundary
    // an entire reverse leg can be shorter than one sample, leaving both ends
    // marked forward, so also require meaningful displacement before treating
    // the sample as a single arc.
    if (cr == pr && moved > step * 0.5f) {
      assert(turned / moved <= (1.0f / tightest) + 0.05f);
    }
    walked += moved;
    px = cx; py = cy; ph = ch; pr = cr;
  }
  // Arc length sampled along the path must match what the plan reports.
  assert(fabsf(walked - p.lengthFt) < 0.05f * p.lengthFt + 0.1f);
}

int main() {
  // --- the shift that actually matters -----------------------------------
  // One lane over: 21in bar with zero overlap = 1.75 ft. A plain 180 cannot
  // do this at all; it always displaces a full turning diameter.
  {
    const float LANE = 21.0f / 12.0f;
    checkTurn(LANE, RL, RR);
    checkTurn(-LANE, RL, RR);

    TurnPlan p;
    planHeadlandTurn(LANE, RL, RR, p);
    assert(p.reversals == 2);   // forward, back, forward
    printf("turn_test: one lane over (%.2f ft) -> %.1f ft of travel, %d reversals\n",
           LANE, p.lengthFt, p.reversals);
  }

  // The production maneuver starts only after the complete rover footprint is
  // beyond the boundary, and chooses the smaller of the two exact K-turns.
  {
    const RoverFootprint footprint = {2.5f / 12.0f, 13.5f / 12.0f,
                                      (19.5f / 30.48f) * 0.5f};
    OutsideTurnPlan outside;
    assert(planOutsideHeadlandTurn(21.0f / 12.0f, RL, RR, footprint, outside));
    assert(outside.ok && outside.runoutFt > 0.0f);
    const TurnEnvelope envelope = turnFootprintEnvelope(outside.turn, footprint);
    assert(outside.runoutFt + envelope.minForwardFt >= -0.001f);
    assert(std::fabs(outside.headlandFt -
                     (outside.runoutFt + envelope.maxForwardFt +
                      TURN_ENVELOPE_ALLOWANCE_FT)) < 0.001f);
  }

  // --- the whole reachable range -----------------------------------------
  {
    for (float shift = -14.0f; shift <= 14.0f; shift += 0.13f) {
      checkTurn(shift, RL, RR);
    }
    printf("turn_test: swept shifts -14..+14 ft, all reachable and exact\n");
  }

  // --- the two families meet where they should ---------------------------
  {
    TurnPlan p;
    // Exactly one right-hand diameter over: drivable forward, no reversing.
    planHeadlandTurn(2.0f * RR, RL, RR, p);
    assert(p.reversals == 0);
    // A hair less, and it needs a three-point turn.
    planHeadlandTurn(2.0f * RR - 0.05f, RL, RR, p);
    assert(p.reversals == 2);
    // Same on the left, at the larger radius.
    planHeadlandTurn(-2.0f * RL, RL, RR, p);
    assert(p.reversals == 0);
    planHeadlandTurn(-2.0f * RL + 0.05f, RL, RR, p);
    assert(p.reversals == 2);
    printf("turn_test: forward turns take over beyond -%.1f / +%.1f ft\n",
           2.0f * RL, 2.0f * RR);
  }

  // --- turning on the spot ------------------------------------------------
  // Reversing direction with no sideways shift at all: a pure three-point
  // turn, which is only possible because forward and reverse arcs cancel.
  {
    checkTurn(0.0f, RL, RR);
    TurnPlan p;
    planHeadlandTurn(0.0f, RL, RR, p);
    assert(p.reversals == 2);
    printf("turn_test: reversing in place costs %.1f ft of travel\n", p.lengthFt);
  }

  // --- measured wet-ground asymmetry is actually used --------------------
  // Both solutions are exact on paper, but the latest wet run showed the
  // forward-left entry running far wider than its model while forward-right
  // tracked tightly. Prefer the right-led plan even when its modeled extent
  // is a little larger.
  {
    TurnPlan ccw, cw;
    solveKTurn(1.204f, RL, RR, true, ccw);
    solveKTurn(1.204f, RL, RR, false, cw);
    assert(ccw.ok && cw.ok);
    assert(fabsf(ccw.lengthFt - cw.lengthFt) > 0.5f);   // genuinely different

    TurnPlan chosen;
    planHeadlandTurn(1.204f, RL, RR, chosen);
    assert(!chosen.leg[0].steerLeft);
    assert(fabsf(chosen.lengthFt - cw.lengthFt) < 0.01f);
    printf("turn_test: same shift reaches %.1f/%.1f ft out; wet-proven "
           "right-led choice %.1f ft\n",
           turnForwardExtent(ccw), turnForwardExtent(cw),
           turnForwardExtent(chosen));
  }

  // --- a symmetric rover still works -------------------------------------
  // Nothing here may depend on the radii being different.
  {
    for (float shift = -9.0f; shift <= 9.0f; shift += 0.37f) checkTurn(shift, 4.0f, 4.0f);
    printf("turn_test: symmetric 4ft rover also exact across the range\n");
  }

  // --- headland required --------------------------------------------------
  // How far past the end of the field the turn reaches. If this were larger
  // than the space available the route would run off the plot, so it is worth
  // stating rather than discovering.
  {
    const float LANE = 21.0f / 12.0f;
    TurnPlan p;
    planHeadlandTurn(LANE, RL, RR, p);
    float maxY = 0.0f, minY = 0.0f;
    for (float s = 0.0f; s <= p.lengthFt; s += 0.05f) {
      float x, y, h; bool r;
      turnPoseAt(p, s, x, y, h, r);
      if (y > maxY) maxY = y;
      if (y < minY) minY = y;
    }
    assert(maxY < RL + 0.01f);   // never reaches further out than one radius
    printf("turn_test: lane-to-lane turn needs %.1f ft beyond the pass "
           "(and %.1f ft back inside)\n", maxY, -minY);
  }

  printf("turn_test: all assertions passed\n");
  return 0;
}
