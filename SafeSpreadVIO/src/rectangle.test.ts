import {
  captureCornerA,
  defineEnteredRectangle,
  defineWalkedRectangle,
  INITIAL_RUN_IN_FT,
  isAtInitialStagingPose,
  rectangleToWorld,
  worldToRectangle,
} from './rectangle';
import { Pose } from './poseMath';

function expectPoseClose(actual: Pose, expected: Pose) {
  expect(actual.x).toBeCloseTo(expected.x, 8);
  expect(actual.y).toBeCloseTo(expected.y, 8);
  expect(actual.heading).toBeCloseTo(expected.heading, 8);
}

describe('entered rectangle', () => {
  it.each(['right', 'left'] as const)('uses the stable rover pose as the %s-side staging pose', (side) => {
    const staging = { x: 10, y: -4, heading: 37 };
    const definition = defineEnteredRectangle(staging, 20, 8, side, 4, 6);
    expect(definition).toMatchObject({
      mAxisHeadingDeg: 37,
      mFt: 20,
      nFt: 8,
      side,
      startClearFt: 4,
      endClearFt: 6,
      source: 'entered',
    });
    expect(definition.originWorld.x).toBeCloseTo(staging.x + Math.sin(37 * Math.PI / 180));
    expect(definition.originWorld.y).toBeCloseTo(staging.y + Math.cos(37 * Math.PI / 180));
    expectPoseClose(worldToRectangle(staging, definition), {
      x: 0,
      y: -INITIAL_RUN_IN_FT,
      heading: 0,
    });
  });

  it('round trips an arbitrary world heading for either coverage side', () => {
    for (const side of ['right', 'left'] as const) {
      const definition = defineEnteredRectangle({ x: 3, y: 8, heading: 123 }, 12, 5, side, 2, 3);
      const rectanglePose = { x: 2.5, y: 7.25, heading: 81 };
      expectPoseClose(
        worldToRectangle(rectangleToWorld(rectanglePose, definition), definition),
        rectanglePose,
      );
    }
  });
});

describe('walked opposite-corner rectangle', () => {
  it('projects B onto A forward/right axes when A points north', () => {
    const a = captureCornerA({ x: 0, y: 0, heading: 0 }, true);
    const definition = defineWalkedRectangle(a, { x: 4, y: 10, heading: 0 }, 3, 5, true);
    expect(definition).toMatchObject({
      originWorld: { x: 0, y: 0, heading: 0 },
      mAxisHeadingDeg: 0,
      mFt: 10,
      nFt: 4,
      side: 'right',
      source: 'walked',
    });
  });

  it('uses A heading at 90 degrees and reports a deliberate left side', () => {
    const a = captureCornerA({ x: 2, y: 3, heading: 90 }, true);
    const right = defineWalkedRectangle(a, { x: 12, y: -1, heading: 12 }, 0, 0, true);
    expect(right.mFt).toBeCloseTo(10);
    expect(right.nFt).toBeCloseTo(4);
    expect(right.side).toBe('right');

    const left = defineWalkedRectangle(a, { x: 12, y: 7, heading: 12 }, 0, 0, true);
    expect(left.mFt).toBeCloseTo(10);
    expect(left.nFt).toBeCloseTo(4);
    expect(left.side).toBe('left');
  });

  it('rejects unstable captures and degenerate diagonals/projections', () => {
    expect(() => captureCornerA({ x: 0, y: 0, heading: 0 }, false)).toThrow('stable');
    const a = captureCornerA({ x: 0, y: 0, heading: 0 }, true);
    expect(() => defineWalkedRectangle(a, { x: 1, y: 1, heading: 0 }, 0, 0, true)).toThrow(
      'diagonal',
    );
    expect(() => defineWalkedRectangle(a, { x: 3, y: 0.5, heading: 0 }, 0, 0, true)).toThrow(
      'forward',
    );
    expect(() => defineWalkedRectangle(a, { x: 0.5, y: 3, heading: 0 }, 0, 0, true)).toThrow(
      'lateral',
    );
    expect(() => defineWalkedRectangle(a, { x: 3, y: 3, heading: 0 }, 0, 0, false)).toThrow(
      'stable',
    );
  });

  it('does not silently accept B behind the direction indicated at A', () => {
    const a = captureCornerA({ x: 0, y: 0, heading: 0 }, true);
    expect(() => defineWalkedRectangle(a, { x: 4, y: -10, heading: 0 }, 0, 0, true)).toThrow(
      'ahead',
    );
  });
});

describe('initial run-in readiness', () => {
  it('accepts only a pose near the staging point and aligned along M', () => {
    expect(isAtInitialStagingPose({ x: 0, y: -1, heading: 0 }, 0.75, 5)).toBe(true);
    expect(isAtInitialStagingPose({ x: 0.3, y: -0.4, heading: 355 }, 0.75, 5)).toBe(true);
    expect(isAtInitialStagingPose({ x: 0, y: 0, heading: 0 }, 0.75, 5)).toBe(false);
    expect(isAtInitialStagingPose({ x: 0, y: -1, heading: 5.1 }, 0.75, 5)).toBe(false);
    expect(isAtInitialStagingPose(null, 0.75, 5)).toBe(false);
  });
});
