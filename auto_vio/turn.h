#pragma once
#include <math.h>

// A headland turn: reverse the rover's direction of travel and shift it
// sideways by a given amount, finishing on the line it started from.
//
// That single maneuver is the only transit a back-and-forth coverage route
// needs, and solving exactly that problem beats a general pose-to-pose planner
// here: the answer is closed form, and it can carry the rover's two turning
// radii separately. The current wet response measures 5.54 ft to the left and
// 5.05 ft to the right, so planning on one averaged radius
// would ask for left turns tighter than the rover can drive and right turns
// wider than it needs.
//
// Two families cover every shift, and they meet exactly at the boundary:
//
//   Three-point turn, for shifts within [-2*rLeft, +2*rRight]: forward on one
//   lock, reverse on the other, forward again. This is what a driver does in a
//   narrow street, and it is the only thing that makes adjacent-lane passes
//   possible -- a car cannot slide sideways by one bar width, and a plain 180
//   always displaces it by a full turning diameter whether that is wanted or
//   not.
//
//   Arc-straight-arc, for shifts beyond that: quarter circle, straight run
//   across, quarter circle, all forward.
//
// Everything is worked in a local frame where the rover starts at the origin
// heading "up", with +X to its right, matching the rover's own convention of
// heading measured clockwise from +Y. Forward motion advances by
// (sin h, cos h).

const float TURN_PI = 3.14159265359f;

inline float turnRad(float deg) { return deg * TURN_PI / 180.0f; }

struct TurnLeg {
  bool  isArc;
  bool  reverse;
  bool  steerLeft;   // arcs only: which way the wheels point
  float radius;      // arcs only
  float h1, h2;      // arcs only: local heading, degrees, signed and unwrapped
  float lengthFt;    // straights only
};

struct TurnPlan {
  bool    ok;
  int     legCount;
  TurnLeg leg[3];
  float   lengthFt;
  int     reversals;   // how many times the rover changes direction of travel
};

inline float turnLegLength(const TurnLeg &l) {
  return l.isArc ? l.radius * fabsf(turnRad(l.h2 - l.h1)) : l.lengthFt;
}

inline void finaliseTurn(TurnPlan &p) {
  p.lengthFt = 0.0f;
  p.reversals = 0;
  bool moving = false;   // the rover arrives at the turn going forward
  for (int i = 0; i < p.legCount; i++) {
    p.lengthFt += turnLegLength(p.leg[i]);
    if (p.leg[i].reverse != moving) { p.reversals++; moving = p.leg[i].reverse; }
  }
  if (moving) p.reversals++;   // and must leave it going forward
  p.ok = true;
}

/** Three-point turn. `ccw` picks which way the rover rotates; both directions
 *  can reach the same shift, by different paths and different lengths, because
 *  the two turning circles are different sizes.
 *
 *  Geometry: with the first and last legs sweeping alpha and the middle leg
 *  sweeping 180-2*alpha, the longitudinal displacements cancel exactly, so the
 *  rover finishes on the line it started from for any alpha. The sideways
 *  shift is then 2*((rLeft+rRight)*cos(alpha) - rLeft) turning counter-
 *  clockwise, which sweeps the whole reachable range as alpha goes 0 to 90. */
inline bool solveKTurn(float shiftFt, float rLeft, float rRight, bool ccw,
                       TurnPlan &p) {
  float sum = rLeft + rRight;
  if (!(sum > 0.0f)) { p.ok = false; return false; }

  float c = ccw ? (shiftFt * 0.5f + rLeft) / sum
                : (rRight - shiftFt * 0.5f) / sum;
  if (!(c >= 0.0f) || !(c <= 1.0f)) { p.ok = false; return false; }

  float a = acosf(c) * 180.0f / TURN_PI;   // 0..90 degrees

  p.legCount = 3;
  if (ccw) {
    // Rotating counter-clockwise: forward on left lock, back on right lock,
    // forward on left lock again. Steering flips each leg; the rotation does
    // not, which is what a three-point turn is.
    p.leg[0] = { true, false, true,  rLeft,  0.0f,          -a,             0.0f };
    p.leg[1] = { true, true,  false, rRight, -a,            -(180.0f - a),  0.0f };
    p.leg[2] = { true, false, true,  rLeft,  -(180.0f - a), -180.0f,        0.0f };
  } else {
    p.leg[0] = { true, false, false, rRight, 0.0f,         a,            0.0f };
    p.leg[1] = { true, true,  true,  rLeft,  a,            180.0f - a,   0.0f };
    p.leg[2] = { true, false, false, rRight, 180.0f - a,   180.0f,       0.0f };
  }
  finaliseTurn(p);
  return true;
}

/** Quarter circle, straight, quarter circle -- all forward. Only reaches
 *  shifts of at least a full turning diameter, which is exactly where the
 *  three-point family runs out, so between them every shift has an answer. */
inline bool solveForwardUTurn(float shiftFt, float rLeft, float rRight,
                              TurnPlan &p) {
  if (shiftFt >= 2.0f * rRight) {
    p.legCount = 3;
    p.leg[0] = { true,  false, false, rRight, 0.0f,  90.0f,  0.0f };
    p.leg[1] = { false, false, false, 0.0f,   0.0f,  0.0f,   shiftFt - 2.0f * rRight };
    p.leg[2] = { true,  false, false, rRight, 90.0f, 180.0f, 0.0f };
    finaliseTurn(p);
    return true;
  }
  if (shiftFt <= -2.0f * rLeft) {
    p.legCount = 3;
    p.leg[0] = { true,  false, true, rLeft, 0.0f,   -90.0f,  0.0f };
    p.leg[1] = { false, false, true, 0.0f,  0.0f,   0.0f,    -shiftFt - 2.0f * rLeft };
    p.leg[2] = { true,  false, true, rLeft, -90.0f, -180.0f, 0.0f };
    finaliseTurn(p);
    return true;
  }
  p.ok = false;
  return false;
}

inline void turnPoseAt(const TurnPlan &p, float s,
                       float &x, float &y, float &headingDeg, bool &reverse);

/** Furthest distance the maneuver reaches straight ahead of the pass end.
 *  Turn planning runs only once, so dense sampling is cheap and lets an
 *  asymmetric rover choose the physically smallest of its two K-turns. */
inline float turnForwardExtent(const TurnPlan &p) {
  float extent = 0.0f;
  for (float s = 0.0f; s < p.lengthFt; s += 0.05f) {
    float x, y, heading; bool reverse;
    turnPoseAt(p, s, x, y, heading, reverse);
    if (y > extent) extent = y;
  }
  float x, y, heading; bool reverse;
  turnPoseAt(p, p.lengthFt, x, y, heading, reverse);
  return y > extent ? y : extent;
}

/** Best maneuver for the required sideways shift. A shift big enough to drive
 *  forward round is driven forward round. For an adjacent-lane shift choose
 *  the three-point turn with the smallest forward/headland extent; path length
 *  only breaks a tie. */
inline bool planHeadlandTurn(float shiftFt, float rLeft, float rRight,
                             TurnPlan &p) {
  if (solveForwardUTurn(shiftFt, rLeft, rRight, p)) return true;

  TurnPlan ccw, cw;
  bool okCcw = solveKTurn(shiftFt, rLeft, rRight, true, ccw);
  bool okCw  = solveKTurn(shiftFt, rLeft, rRight, false, cw);

  if (okCcw && okCw) {
    const float ccwExtent = turnForwardExtent(ccw);
    const float cwExtent = turnForwardExtent(cw);
    p = (ccwExtent < cwExtent ||
         (fabsf(ccwExtent - cwExtent) < 0.01f && ccw.lengthFt <= cw.lengthFt))
        ? ccw : cw;
    return true;
  }
  if (okCcw) { p = ccw; return true; }
  if (okCw)  { p = cw;  return true; }

  p.ok = false;
  return false;
}

/** Pose a given distance along the turn, in the local frame. `reverse` reports
 *  whether the rover is backing up at that point, which the tracker needs in
 *  order to invert its steering and run the motor the other way. */
inline void turnPoseAt(const TurnPlan &p, float s,
                       float &x, float &y, float &headingDeg, bool &reverse) {
  x = 0.0f; y = 0.0f; headingDeg = 0.0f;
  reverse = (p.legCount > 0) ? p.leg[0].reverse : false;

  float remaining = (s > 0.0f) ? s : 0.0f;

  for (int i = 0; i < p.legCount; i++) {
    const TurnLeg &l = p.leg[i];
    float len = turnLegLength(l);

    if (len <= 1e-6f) {
      if (l.isArc) headingDeg = l.h2;
      continue;
    }

    float use = (len < remaining) ? len : remaining;
    reverse = l.reverse;

    if (l.isArc) {
      float hB = l.h1 + (l.h2 - l.h1) * (use / len);
      float a = turnRad(l.h1), b = turnRad(hB);
      // Displacement along an arc depends only on the two headings and which
      // side the centre is on -- not on whether the rover drove it forwards
      // or backwards.
      if (l.steerLeft) {
        x += l.radius * (cosf(b) - cosf(a));
        y += l.radius * (sinf(a) - sinf(b));
      } else {
        x += l.radius * (cosf(a) - cosf(b));
        y += l.radius * (sinf(b) - sinf(a));
      }
      headingDeg = hB;
    } else {
      float h = turnRad(headingDeg);
      float sgn = l.reverse ? -1.0f : 1.0f;
      x += sgn * use * sinf(h);
      y += sgn * use * cosf(h);
    }

    remaining -= use;
    if (remaining <= 1e-6f) return;
  }
}
