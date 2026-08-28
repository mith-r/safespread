import { normalizeHeading, Pose } from './poseMath';
import { estimateHeadland, HeadlandRequirement } from './routePlan';

export type CoverageSide = 'right' | 'left';

/** Where the headland figures came from: the app's planner estimate, the
 *  rover's own plan after Configure, or nowhere yet because the route could
 *  not be planned within the firmware's point limit and the rover decides. */
export type HeadlandSource = 'estimated' | 'rover' | 'unknown';

export interface RectangleDefinition {
  originWorld: Pose;
  mAxisHeadingDeg: number;
  mFt: number;
  nFt: number;
  side: CoverageSide;
  /** Clear pavement the planned turns need behind A, in feet. */
  startClearFt: number;
  /** Clear pavement the planned turns need beyond the far M edge, in feet. */
  endClearFt: number;
  headlandSource: HeadlandSource;
  source: 'entered';
}

function requirePose(pose: Pose, name: string): Pose {
  if (![pose.x, pose.y, pose.heading].every(Number.isFinite)) {
    throw new TypeError(`${name} pose must be finite`);
  }
  return { x: pose.x, y: pose.y, heading: normalizeHeading(pose.heading) };
}

function requirePositive(value: number, name: string): number {
  if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${name} must be positive`);
  return value;
}

type PlannedHeadland = Pick<RectangleDefinition, 'startClearFt' | 'endClearFt' | 'headlandSource'>;

/** The minimum headland the planned route (car-style three-point turns)
 *  needs for this rectangle. The rover always runs at its own computed
 *  minimum; this is what the operator sees before arming. */
function plannedHeadland(mFt: number, nFt: number): PlannedHeadland {
  const estimate = estimateHeadland(mFt, nFt);
  if (!estimate) return { startClearFt: 0, endClearFt: 0, headlandSource: 'unknown' };
  return {
    startClearFt: estimate.beforeStartFt,
    endClearFt: estimate.beyondEndFt,
    headlandSource: 'estimated',
  };
}

/** Replace the estimate with what the rover reported after planning. */
export function withRoverHeadland(
  definition: RectangleDefinition,
  requirement: HeadlandRequirement,
): RectangleDefinition {
  if (![requirement.beforeStartFt, requirement.beyondEndFt].every((v) => Number.isFinite(v) && v >= 0)) {
    throw new RangeError('rover headland must be finite and non-negative');
  }
  return {
    ...definition,
    startClearFt: requirement.beforeStartFt,
    endClearFt: requirement.beyondEndFt,
    headlandSource: 'rover',
  };
}

function worldDeltaToAxes(dx: number, dy: number, headingDeg: number) {
  const theta = headingDeg * Math.PI / 180;
  return {
    right: dx * Math.cos(theta) - dy * Math.sin(theta),
    forward: dx * Math.sin(theta) + dy * Math.cos(theta),
  };
}

function axesDeltaToWorld(right: number, forward: number, headingDeg: number) {
  const theta = headingDeg * Math.PI / 180;
  return {
    x: right * Math.cos(theta) + forward * Math.sin(theta),
    y: -right * Math.sin(theta) + forward * Math.cos(theta),
  };
}

export function defineEnteredRectangle(
  stableRover: Pose,
  mFt: number,
  nFt: number,
  side: CoverageSide,
): RectangleDefinition {
  const originWorld = requirePose(stableRover, 'rover');
  if (side !== 'right' && side !== 'left') throw new RangeError('coverage side is invalid');
  const m = requirePositive(mFt, 'M');
  const n = requirePositive(nFt, 'N');
  return {
    originWorld,
    mAxisHeadingDeg: originWorld.heading,
    mFt: m,
    nFt: n,
    side,
    ...plannedHeadland(m, n),
    source: 'entered',
  };
}

export function worldToRectangle(world: Pose, definition: RectangleDefinition): Pose {
  const pose = requirePose(world, 'world');
  const dx = pose.x - definition.originWorld.x;
  const dy = pose.y - definition.originWorld.y;
  const projected = worldDeltaToAxes(dx, dy, definition.mAxisHeadingDeg);
  const sideSign = definition.side === 'right' ? 1 : -1;
  const relativeHeading = normalizeHeading(pose.heading - definition.mAxisHeadingDeg);
  return {
    x: projected.right * sideSign,
    y: projected.forward,
    heading: definition.side === 'right' ? relativeHeading : normalizeHeading(-relativeHeading),
  };
}

export function rectangleToWorld(rectangle: Pose, definition: RectangleDefinition): Pose {
  const pose = requirePose(rectangle, 'rectangle');
  const sideSign = definition.side === 'right' ? 1 : -1;
  const delta = axesDeltaToWorld(pose.x * sideSign, pose.y, definition.mAxisHeadingDeg);
  return {
    x: definition.originWorld.x + delta.x,
    y: definition.originWorld.y + delta.y,
    heading: normalizeHeading(
      definition.mAxisHeadingDeg + (definition.side === 'right' ? pose.heading : -pose.heading),
    ),
  };
}
