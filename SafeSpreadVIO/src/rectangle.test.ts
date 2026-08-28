import {
  defineEnteredRectangle,
  rectangleToWorld,
  withRoverHeadland,
  worldToRectangle,
} from './rectangle';
import { estimateHeadland } from './routePlan';
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
    const planned = estimateHeadland(20, 8);
    expect(planned).not.toBeNull();
    expect(definition).toEqual({
      originWorld: origin,
      mAxisHeadingDeg: 37,
      mFt: 20,
      nFt: 8,
      side,
      startClearFt: planned?.beforeStartFt,
      endClearFt: planned?.beyondEndFt,
      headlandSource: 'estimated',
      source: 'entered',
    });
    expectPoseClose(worldToRectangle(origin, definition), { x: 0, y: 0, heading: 0 });
  });

  it('plans the measured pavement rectangle at the same headland the rover needs', () => {
    const definition = defineEnteredRectangle({ x: 0, y: 0, heading: 0 }, 21.9, 21.9, 'right');
    expect(definition.startClearFt).toBeCloseTo(6.83, 2);
    expect(definition.endClearFt).toBeCloseTo(7.07, 2);
    expect(definition.headlandSource).toBe('estimated');
  });

  it('defers to the rover when the route cannot be planned within the point limit', () => {
    const definition = defineEnteredRectangle({ x: 0, y: 0, heading: 0 }, 300, 120, 'right');
    expect(definition).toMatchObject({ startClearFt: 0, endClearFt: 0, headlandSource: 'unknown' });
  });

  it('rejects non-positive dimensions', () => {
    expect(() => defineEnteredRectangle({ x: 0, y: 0, heading: 0 }, 0, 8, 'right')).toThrow('M');
    expect(() => defineEnteredRectangle({ x: 0, y: 0, heading: 0 }, 8, -1, 'right')).toThrow('N');
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

describe('withRoverHeadland', () => {
  it('replaces the estimate with the rover-reported requirement', () => {
    const definition = defineEnteredRectangle({ x: 0, y: 0, heading: 0 }, 21.9, 21.9, 'right');
    const confirmed = withRoverHeadland(definition, { beforeStartFt: 19, beyondEndFt: 12 });
    expect(confirmed).toMatchObject({ startClearFt: 19, endClearFt: 12, headlandSource: 'rover' });
    expect(confirmed.mFt).toBe(definition.mFt);
    expect(definition.headlandSource).toBe('estimated');
  });

  it('rejects malformed rover figures', () => {
    const definition = defineEnteredRectangle({ x: 0, y: 0, heading: 0 }, 21.9, 21.9, 'right');
    expect(() => withRoverHeadland(definition, { beforeStartFt: -1, beyondEndFt: 2 })).toThrow(RangeError);
    expect(() => withRoverHeadland(definition, { beforeStartFt: Number.NaN, beyondEndFt: 2 })).toThrow(RangeError);
  });
});
