import { normalizeHeading, Pose } from './poseMath';

export type CoverageSide = 'right' | 'left';

export interface RectangleDefinition {
  originWorld: Pose;
  mAxisHeadingDeg: number;
  mFt: number;
  nFt: number;
  side: CoverageSide;
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
  return {
    originWorld,
    mAxisHeadingDeg: originWorld.heading,
    mFt: requirePositive(mFt, 'M'),
    nFt: requirePositive(nFt, 'N'),
    side,
    source: 'entered',
  };
}

export function captureCornerA(stableCamera: Pose, isStable: boolean): CornerA {
  if (!isStable) throw new Error('Corner A requires a stable normal pose');
  const poseWorld = requirePose(stableCamera, 'Corner A');
  return { poseWorld, mAxisHeadingDeg: poseWorld.heading };
}

export function defineWalkedRectangle(
  a: CornerA,
  bWorld: Pose,
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
