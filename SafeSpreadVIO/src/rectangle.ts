import { normalizeHeading, Pose, wrappedHeadingDelta } from './poseMath';

/** Rover staging distance behind boundary A before the mission's initial run-in. */
export const INITIAL_RUN_IN_FT = 1.0;

export type CoverageSide = 'right' | 'left';

export interface RectangleDefinition {
  originWorld: Pose;
  mAxisHeadingDeg: number;
  mFt: number;
  nFt: number;
  side: CoverageSide;
  startClearFt: number;
  endClearFt: number;
  source: 'entered' | 'walked';
}

export interface CornerA {
  poseWorld: Pose;
  mAxisHeadingDeg: number;
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

function requireClearance(value: number, name: string): number {
  if (!Number.isFinite(value) || value < 0) throw new RangeError(`${name} must not be negative`);
  return value;
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
  startClearFt: number,
  endClearFt: number,
): RectangleDefinition {
  const stagingWorld = requirePose(stableRover, 'rover');
  if (side !== 'right' && side !== 'left') throw new RangeError('coverage side is invalid');
  const runIn = axesDeltaToWorld(0, INITIAL_RUN_IN_FT, stagingWorld.heading);
  const originWorld = {
    x: stagingWorld.x + runIn.x,
    y: stagingWorld.y + runIn.y,
    heading: stagingWorld.heading,
  };
  return {
    originWorld,
    mAxisHeadingDeg: stagingWorld.heading,
    mFt: requirePositive(mFt, 'M'),
    nFt: requirePositive(nFt, 'N'),
    side,
    startClearFt: requireClearance(startClearFt, 'start clearance'),
    endClearFt: requireClearance(endClearFt, 'end clearance'),
    source: 'entered',
  };
}

export function isAtInitialStagingPose(
  rectanglePose: Pose | null,
  positionToleranceFt: number,
  headingToleranceDeg: number,
): boolean {
  if (!rectanglePose || ![rectanglePose.x, rectanglePose.y, rectanglePose.heading].every(Number.isFinite)) {
    return false;
  }
  if (!Number.isFinite(positionToleranceFt) || positionToleranceFt < 0 ||
      !Number.isFinite(headingToleranceDeg) || headingToleranceDeg < 0) {
    return false;
  }
  return Math.hypot(rectanglePose.x, rectanglePose.y + INITIAL_RUN_IN_FT) <= positionToleranceFt &&
    Math.abs(wrappedHeadingDelta(rectanglePose.heading, 0)) <= headingToleranceDeg;
}

export function captureCornerA(stableCamera: Pose, isStable: boolean): CornerA {
  if (!isStable) throw new Error('Corner A requires a stable normal pose');
  const poseWorld = requirePose(stableCamera, 'Corner A');
  return { poseWorld, mAxisHeadingDeg: poseWorld.heading };
}

export function defineWalkedRectangle(
  a: CornerA,
  bWorld: Pose,
  startClearFt: number,
  endClearFt: number,
  isBStable: boolean,
): RectangleDefinition {
  if (!isBStable) throw new Error('Corner B requires a stable normal pose');
  const aPose = requirePose(a.poseWorld, 'Corner A');
  const bPose = requirePose(bWorld, 'Corner B');
  const heading = normalizeHeading(a.mAxisHeadingDeg);
  const dx = bPose.x - aPose.x;
  const dy = bPose.y - aPose.y;
  if (Math.hypot(dx, dy) < 3) throw new RangeError('corner diagonal must be at least 3 ft');
  const projected = worldDeltaToAxes(dx, dy, heading);
  if (projected.forward < 0) throw new RangeError('Corner B must be ahead of Corner A');
  if (projected.forward < 1) throw new RangeError('forward projection must be at least 1 ft');
  if (Math.abs(projected.right) < 1) throw new RangeError('lateral projection must be at least 1 ft');
  return {
    originWorld: { ...aPose, heading },
    mAxisHeadingDeg: heading,
    mFt: projected.forward,
    nFt: Math.abs(projected.right),
    side: projected.right < 0 ? 'left' : 'right',
    startClearFt: requireClearance(startClearFt, 'start clearance'),
    endClearFt: requireClearance(endClearFt, 'end clearance'),
    source: 'walked',
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
