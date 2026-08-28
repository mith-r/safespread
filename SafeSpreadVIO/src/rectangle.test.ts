import {
  captureCornerA,
  defineEnteredRectangle,
  defineWalkedRectangle,
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
  it.each(['right', 'left'] as const)('uses the stable rover pose and explicit %s side', (side) => {
    const origin = { x: 10, y: -4, heading: 37 };
    const definition = defineEnteredRectangle(origin, 20, 8, side);
    expect(definition).toEqual({
      originWorld: origin,
      mAxisHeadingDeg: 37,
      mFt: 20,
      nFt: 8,
      side,
      source: 'entered',
    });
    expectPoseClose(worldToRectangle(origin, definition), { x: 0, y: 0, heading: 0 });
  });

  it('round trips an arbitrary world heading for either coverage side', () => {
    for (const side of ['right', 'left'] as const) {
      const definition = defineEnteredRectangle({ x: 3, y: 8, heading: 123 }, 12, 5, side);
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
    const definition = defineWalkedRectangle(a, { x: 4, y: 10, heading: 0 }, true);
    expect(definition).toMatchObject({
      mAxisHeadingDeg: 0,
      mFt: 10,
      nFt: 4,
      side: 'right',
      source: 'walked',
    });
  });

  it('uses A heading at 90 degrees and reports a deliberate left side', () => {
    const a = captureCornerA({ x: 2, y: 3, heading: 90 }, true);
    const right = defineWalkedRectangle(a, { x: 12, y: -1, heading: 12 }, true);
    expect(right.mFt).toBeCloseTo(10);
    expect(right.nFt).toBeCloseTo(4);
    expect(right.side).toBe('right');

    const left = defineWalkedRectangle(a, { x: 12, y: 7, heading: 12 }, true);
    expect(left.mFt).toBeCloseTo(10);
    expect(left.nFt).toBeCloseTo(4);
    expect(left.side).toBe('left');
  });

  it('rejects unstable captures and degenerate diagonals/projections', () => {
    expect(() => captureCornerA({ x: 0, y: 0, heading: 0 }, false)).toThrow('stable');
    const a = captureCornerA({ x: 0, y: 0, heading: 0 }, true);
    expect(() => defineWalkedRectangle(a, { x: 1, y: 1, heading: 0 }, true)).toThrow(
      'diagonal',
    );
    expect(() => defineWalkedRectangle(a, { x: 3, y: 0.5, heading: 0 }, true)).toThrow(
      'forward',
    );
    expect(() => defineWalkedRectangle(a, { x: 0.5, y: 3, heading: 0 }, true)).toThrow(
      'lateral',
    );
    expect(() => defineWalkedRectangle(a, { x: 3, y: 3, heading: 0 }, false)).toThrow(
      'stable',
    );
  });

  it('does not silently accept B behind the direction indicated at A', () => {
    const a = captureCornerA({ x: 0, y: 0, heading: 0 }, true);
    expect(() => defineWalkedRectangle(a, { x: 4, y: -10, heading: 0 }, true)).toThrow(
      'ahead',
    );
  });
});
