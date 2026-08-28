import { SAFESPREAD_HARDWARE_GEOMETRY } from './hardwareGeometry';

/**
 * Port of the firmware's coverage route planner (auto_vio/route.h, turn.h,
 * headland.h): lanes driven in order, joined by car-style headland turns —
 * a three-point turn where the next lane is closer than a turning diameter,
 * an arc-straight-arc otherwise. The rover plans the real route itself when
 * the rectangle is configured; this copy exists so the app can show the
 * operator, before arming, how much clear pavement the turns will need.
 * Keep the constants in step with the firmware.
 */

export const BAR_WIDTH_FT = SAFESPREAD_HARDWARE_GEOMETRY.sprayWidthFt;
export const LANE_OVERLAP_FRACTION = 0.15;
export const ROUTE_STEP_FT = 0.5;
export const HEADLAND_MARGIN_FT = 3.5;
export const TURN_PLANNING_MARGIN = 1.3;
export const MAX_ROUTE_POINTS = 6000;
export const ROUTE_EXTREMA_ALLOWANCE_FT = 0.01;

export interface TurnRadii {
  turnRadiusLeftFt: number;
  turnRadiusRightFt: number;
}

export const MEASURED_TURN_RADII: TurnRadii = {
  turnRadiusLeftFt: SAFESPREAD_HARDWARE_GEOMETRY.turnRadiusLeftFt,
  turnRadiusRightFt: SAFESPREAD_HARDWARE_GEOMETRY.turnRadiusRightFt,
};

export interface RoutePoint {
  x: number;
  y: number;
  spray: boolean;
  reverse: boolean;
  turning: boolean;
  terminal: boolean;
}

export interface HeadlandRequirement {
  beforeStartFt: number;
  beyondEndFt: number;
}

// --- Lane geometry (nav_math.h) -------------------------------------------
// Math.fround keeps the lane count identical to the firmware's float32 result
// when the width is an exact multiple of the spacing.

export function laneSpacing(barWidthFt = BAR_WIDTH_FT, overlapFraction = LANE_OVERLAP_FRACTION): number {
  const spacing = Math.fround(Math.fround(barWidthFt) * Math.fround(1 - overlapFraction));
  return spacing > 0.01 ? spacing : 0.01;
}

export function laneCenterX(lane: number, barWidthFt = BAR_WIDTH_FT, overlapFraction = LANE_OVERLAP_FRACTION): number {
  return lane * laneSpacing(barWidthFt, overlapFraction);
}

export function laneCount(widthFt: number, barWidthFt = BAR_WIDTH_FT, overlapFraction = LANE_OVERLAP_FRACTION): number {
  if (widthFt <= barWidthFt) return 1;
  const usable = Math.fround(Math.fround(widthFt) - Math.fround(barWidthFt));
  const count = Math.floor(Math.fround(usable / laneSpacing(barWidthFt, overlapFraction))) + 1;
  return count < 1 ? 1 : count;
}

// --- Headland turns (turn.h) -----------------------------------------------
// Worked in a local frame: rover at the origin heading "up", +X to its right,
// heading measured clockwise from +Y.

interface TurnLeg {
  isArc: boolean;
  reverse: boolean;
  steerLeft: boolean;
  radius: number;
  h1: number;
  h2: number;
  lengthFt: number;
}

interface TurnPlan {
  legs: TurnLeg[];
  lengthFt: number;
  reversals: number;
}

function turnRad(deg: number): number {
  return (deg * Math.PI) / 180;
}

function arc(reverse: boolean, steerLeft: boolean, radius: number, h1: number, h2: number): TurnLeg {
  return { isArc: true, reverse, steerLeft, radius, h1, h2, lengthFt: 0 };
}

function straight(lengthFt: number, steerLeft: boolean): TurnLeg {
  return { isArc: false, reverse: false, steerLeft, radius: 0, h1: 0, h2: 0, lengthFt };
}

function turnLegLength(leg: TurnLeg): number {
  return leg.isArc ? leg.radius * Math.abs(turnRad(leg.h2 - leg.h1)) : leg.lengthFt;
}

function finaliseTurn(legs: TurnLeg[]): TurnPlan {
  let lengthFt = 0;
  let reversals = 0;
  let moving = false; // the rover arrives at the turn going forward
  for (const leg of legs) {
    lengthFt += turnLegLength(leg);
    if (leg.reverse !== moving) {
      reversals++;
      moving = leg.reverse;
    }
  }
  if (moving) reversals++; // and must leave it going forward
  return { legs, lengthFt, reversals };
}

/** Three-point turn: forward on one lock, reverse on the other, forward again.
 *  Reaches sideways shifts within [-2*rLeft, +2*rRight]. */
function solveKTurn(shiftFt: number, rLeft: number, rRight: number, ccw: boolean): TurnPlan | null {
  const sum = rLeft + rRight;
  if (!(sum > 0)) return null;
  const c = ccw ? (shiftFt * 0.5 + rLeft) / sum : (rRight - shiftFt * 0.5) / sum;
  if (!(c >= 0) || !(c <= 1)) return null;
  const a = (Math.acos(c) * 180) / Math.PI; // 0..90 degrees
  if (ccw) {
    return finaliseTurn([
      arc(false, true, rLeft, 0, -a),
      arc(true, false, rRight, -a, -(180 - a)),
      arc(false, true, rLeft, -(180 - a), -180),
    ]);
  }
  return finaliseTurn([
    arc(false, false, rRight, 0, a),
    arc(true, true, rLeft, a, 180 - a),
    arc(false, false, rRight, 180 - a, 180),
  ]);
}

/** Quarter circle, straight, quarter circle — all forward. Only for shifts of
 *  at least a full turning diameter. */
function solveForwardUTurn(shiftFt: number, rLeft: number, rRight: number): TurnPlan | null {
  if (shiftFt >= 2 * rRight) {
    return finaliseTurn([
      arc(false, false, rRight, 0, 90),
      straight(shiftFt - 2 * rRight, false),
      arc(false, false, rRight, 90, 180),
    ]);
  }
  if (shiftFt <= -2 * rLeft) {
    return finaliseTurn([
      arc(false, true, rLeft, 0, -90),
      straight(-shiftFt - 2 * rLeft, true),
      arc(false, true, rLeft, -90, -180),
    ]);
  }
  return null;
}

/** Best maneuver for the required sideways shift: drive round forward when
 *  the shift allows it, otherwise the shorter of the two three-point turns. */
function planHeadlandTurn(shiftFt: number, rLeft: number, rRight: number): TurnPlan | null {
  const forward = solveForwardUTurn(shiftFt, rLeft, rRight);
  if (forward) return forward;
  const ccw = solveKTurn(shiftFt, rLeft, rRight, true);
  const cw = solveKTurn(shiftFt, rLeft, rRight, false);
  if (ccw && cw) return ccw.lengthFt <= cw.lengthFt ? ccw : cw;
  return ccw ?? cw;
}

/** Pose a given distance along the turn, in the local frame. */
function turnPoseAt(plan: TurnPlan, s: number): { x: number; y: number; reverse: boolean } {
  let x = 0;
  let y = 0;
  let headingDeg = 0;
  let reverse = plan.legs.length > 0 ? plan.legs[0].reverse : false;
  let remaining = s > 0 ? s : 0;

  for (const leg of plan.legs) {
    const len = turnLegLength(leg);
    if (len <= 1e-6) {
      if (leg.isArc) headingDeg = leg.h2;
      continue;
    }
    const use = len < remaining ? len : remaining;
    reverse = leg.reverse;
    if (leg.isArc) {
      const hB = leg.h1 + (leg.h2 - leg.h1) * (use / len);
      const a = turnRad(leg.h1);
      const b = turnRad(hB);
      if (leg.steerLeft) {
        x += leg.radius * (Math.cos(b) - Math.cos(a));
        y += leg.radius * (Math.sin(a) - Math.sin(b));
      } else {
        x += leg.radius * (Math.cos(a) - Math.cos(b));
        y += leg.radius * (Math.sin(b) - Math.sin(a));
      }
      headingDeg = hB;
    } else {
      const h = turnRad(headingDeg);
      const sign = leg.reverse ? -1 : 1;
      x += sign * use * Math.sin(h);
      y += sign * use * Math.cos(h);
    }
    remaining -= use;
    if (remaining <= 1e-6) break;
  }
  return { x, y, reverse };
}

// --- Route construction (route.h) ------------------------------------------

/** Points along a straight line, excluding the start and landing exactly on
 *  the end. Returns false when the output limit cut the line short. */
function emitLineTo(
  out: RoutePoint[], x1: number, y1: number, x2: number, y2: number, spray: boolean,
): boolean {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const length = Math.hypot(dx, dy);
  if (length < 1e-6) return true;
  const steps = Math.floor(length / ROUTE_STEP_FT);
  for (let k = 1; k <= steps; k++) {
    if (out.length >= MAX_ROUTE_POINTS) return false;
    const t = (k * ROUTE_STEP_FT) / length;
    out.push({ x: x1 + dx * t, y: y1 + dy * t, spray, reverse: false, turning: false, terminal: false });
  }
  const last = out[out.length - 1];
  if (!last || Math.abs(last.x - x2) > 1e-4 || Math.abs(last.y - y2) > 1e-4) {
    if (out.length >= MAX_ROUTE_POINTS) return false;
    out.push({ x: x2, y: y2, spray, reverse: false, turning: false, terminal: false });
  }
  return true;
}

/** Points along a headland turn, transformed from the maneuver's own frame
 *  into the field. Returns false when the output limit cut the turn short. */
function emitTurn(out: RoutePoint[], plan: TurnPlan, x0: number, y0: number, h0Deg: number): boolean {
  const ch = Math.cos(turnRad(h0Deg));
  const sh = Math.sin(turnRad(h0Deg));
  const place = (s: number, finalPoint: boolean) => {
    if (out.length >= MAX_ROUTE_POINTS) return false;
    const local = turnPoseAt(plan, s);
    out.push({
      x: x0 + local.x * ch + local.y * sh,
      y: y0 - local.x * sh + local.y * ch,
      spray: false, // never spray through a turn
      reverse: finalPoint ? false : local.reverse, // the turn always finishes driving forward
      turning: true,
      terminal: false,
    });
    return true;
  };
  for (let s = ROUTE_STEP_FT; s < plan.lengthFt; s += ROUTE_STEP_FT) {
    if (!place(s, false)) return false;
  }
  return place(plan.lengthFt, true);
}

function requirePositive(value: number, name: string): number {
  if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${name} must be positive`);
  return value;
}

/** Build the route the rover drives: lanes in order, each with a run-in and
 *  run-out outside the rectangle, joined by headland turns. The result is
 *  truncated (last point not terminal) when it would exceed the firmware's
 *  point limit. */
export function buildRoute(
  mFt: number,
  nFt: number,
  radii: TurnRadii = MEASURED_TURN_RADII,
): RoutePoint[] {
  const fieldPassFt = requirePositive(mFt, 'M');
  const fieldWidthFt = requirePositive(nFt, 'N');
  const lanes = laneCount(fieldWidthFt);
  const planLeft = radii.turnRadiusLeftFt * TURN_PLANNING_MARGIN;
  const planRight = radii.turnRadiusRightFt * TURN_PLANNING_MARGIN;

  // The route starts under the rover, spraying: the first pass runs straight
  // ahead from where it was placed.
  const out: RoutePoint[] = [{ x: 0, y: 0, spray: true, reverse: false, turning: false, terminal: false }];
  let completedLanes = 0;

  for (let i = 0; i < lanes; i++) {
    const laneX = laneCenterX(i);
    const goesUp = i % 2 === 0;
    const startY = goesUp ? 0 : fieldPassFt;
    const endY = goesUp ? fieldPassFt : 0;
    const dir = goesUp ? 1 : -1;
    const headingDeg = goesUp ? 0 : 180;

    // Run in from the headland so the rover is straight and on the line
    // before any spray comes out. The first pass has no run-in.
    if (i > 0 && !emitLineTo(out, laneX, startY - dir * HEADLAND_MARGIN_FT, laneX, startY, false)) break;
    if (!emitLineTo(out, laneX, startY, laneX, endY, true)) break;
    completedLanes++;
    if (i + 1 >= lanes) break;

    const headlandY = endY + dir * HEADLAND_MARGIN_FT;
    if (!emitLineTo(out, laneX, endY, laneX, headlandY, false)) break;

    // The turn is a sideways shift in the rover's own frame, so it flips sign
    // on return passes.
    const shift = (laneCenterX(i + 1) - laneX) * Math.cos(turnRad(headingDeg));
    const plan = planHeadlandTurn(shift, planLeft, planRight);
    if (!plan) return out;
    if (!emitTurn(out, plan, laneX, headlandY, headingDeg)) break;
  }

  if (completedLanes === lanes) {
    out[out.length - 1] = { ...out[out.length - 1], terminal: true };
  }
  return out;
}

// --- Headland inspection (headland.h) --------------------------------------

export interface RouteRequirements extends HeadlandRequirement {
  truncated: boolean;
}

/** Clear pavement the sampled route needs outside the rectangle. Arcs are
 *  sampled every 0.5 ft, so a small allowance covers the extremum that can
 *  fall between two samples. */
export function inspectRoute(route: RoutePoint[], passLengthFt: number): RouteRequirements {
  if (route.length === 0 || !(passLengthFt > 0)) {
    return { beforeStartFt: 0, beyondEndFt: 0, truncated: true };
  }
  let minY = route[0].y;
  let maxY = route[0].y;
  for (const point of route) {
    if (point.y < minY) minY = point.y;
    if (point.y > maxY) maxY = point.y;
  }
  const sampledBefore = Math.max(0, -minY);
  const sampledBeyond = Math.max(0, maxY - passLengthFt);
  return {
    beforeStartFt: sampledBefore > 0 ? sampledBefore + ROUTE_EXTREMA_ALLOWANCE_FT : 0,
    beyondEndFt: sampledBeyond > 0 ? sampledBeyond + ROUTE_EXTREMA_ALLOWANCE_FT : 0,
    truncated: !route[route.length - 1].terminal,
  };
}

/** The minimum clear pavement before A and beyond M for the planned route, or
 *  null when the route cannot be fully planned (too many points) and the
 *  rover has to decide at Configure. */
export function estimateHeadland(
  mFt: number,
  nFt: number,
  radii: TurnRadii = MEASURED_TURN_RADII,
): HeadlandRequirement | null {
  const requirements = inspectRoute(buildRoute(mFt, nFt, radii), mFt);
  if (requirements.truncated) return null;
  return { beforeStartFt: requirements.beforeStartFt, beyondEndFt: requirements.beyondEndFt };
}

const ROVER_HEADLAND_LINE =
  /^Needs clear pavement (\d+(?:\.\d+)?) ft past the far end, (\d+(?:\.\d+)?) ft behind the start\./;

/** The rover logs its planned route's clearance right after Configure. */
export function parseRoverHeadlandLog(line: string): HeadlandRequirement | null {
  const match = ROVER_HEADLAND_LINE.exec(line);
  if (!match) return null;
  return { beforeStartFt: Number.parseFloat(match[2]), beyondEndFt: Number.parseFloat(match[1]) };
}
