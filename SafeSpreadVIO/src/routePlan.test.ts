import {
  buildRoute,
  estimateHeadland,
  inspectRoute,
  laneCenterX,
  laneCount,
  parseRoverHeadlandLog,
} from './routePlan';
import { SAFESPREAD_HARDWARE_GEOMETRY } from './hardwareGeometry';

const MEASURED = {
  turnRadiusLeftFt: SAFESPREAD_HARDWARE_GEOMETRY.turnRadiusLeftFt,
  turnRadiusRightFt: SAFESPREAD_HARDWARE_GEOMETRY.turnRadiusRightFt,
};

// Ground truth produced by compiling the firmware's own route.h / turn.h /
// headland.h (auto_vio/) into a host harness and running buildRoute +
// inspectRoute. Values are what the rover logs as "Needs clear pavement ..."
// for the same rectangle.
const FIRMWARE_TRUTH = [
  { name: 'default 21.9x21.9', mFt: 21.9, nFt: 21.9, radii: MEASURED, lanes: 18, count: 1498, reversals: 34, before: 6.8281, beyond: 7.0671, minX: -1.2654, maxX: 21.6359 },
  { name: 'single lane', mFt: 10, nFt: 1, radii: MEASURED, lanes: 1, count: 21, reversals: 0, before: 0, beyond: 0, minX: 0, maxX: 0 },
  { name: 'two lanes', mFt: 12, nFt: 3, radii: MEASURED, lanes: 2, count: 90, reversals: 2, before: 0, beyond: 7.0671, minX: -1.2654, maxX: 2.3692 },
  { name: 'narrow', mFt: 30, nFt: 5, radii: MEASURED, lanes: 3, count: 264, reversals: 4, before: 6.8281, beyond: 7.0671, minX: -1.2654, maxX: 4.3571 },
  { name: 'wide', mFt: 15, nFt: 40, radii: MEASURED, lanes: 33, count: 2319, reversals: 64, before: 6.8281, beyond: 7.0671, minX: -1.2654, maxX: 40.4821 },
  { name: 'small', mFt: 4, nFt: 4, radii: MEASURED, lanes: 3, count: 108, reversals: 4, before: 6.8281, beyond: 7.0671, minX: -1.2654, maxX: 4.3571 },
  { name: 'equal radii', mFt: 21.9, nFt: 21.9, radii: { turnRadiusLeftFt: 3, turnRadiusRightFt: 3 }, lanes: 18, count: 1456, reversals: 34, before: 7.0126, beyond: 6.68, minX: -1.4679, maxX: 22.0987 },
  { name: 'long', mFt: 100, nFt: 21.9, radii: MEASURED, lanes: 18, count: 4306, reversals: 34, before: 6.8281, beyond: 7.0671, minX: -1.2654, maxX: 21.6359 },
];

describe('lane geometry', () => {
  it('matches the firmware lane rules', () => {
    expect(laneCenterX(0)).toBeCloseTo(0, 3);
    expect(laneCount(10, 2, 0)).toBe(5);
    expect(laneCenterX(4, 2, 0)).toBeCloseTo(8, 3);
    expect(laneCount(1, 2, 0)).toBe(1);
    expect(laneCount(21.9)).toBe(18);
  });
});

describe('buildRoute', () => {
  it.each(FIRMWARE_TRUTH)('reproduces the firmware route for $name', (truth) => {
    const route = buildRoute(truth.mFt, truth.nFt, truth.radii);
    expect(laneCount(truth.nFt)).toBe(truth.lanes);
    expect(Math.abs(route.length - truth.count)).toBeLessThanOrEqual(3);
    expect(route[route.length - 1].terminal).toBe(true);
    let reversals = 0;
    for (let i = 1; i < route.length; i++) if (route[i].reverse !== route[i - 1].reverse) reversals++;
    expect(reversals).toBe(truth.reversals);
    const xs = route.map((p) => p.x);
    expect(Math.min(0, ...xs)).toBeCloseTo(truth.minX, 2);
    expect(Math.max(0, ...xs)).toBeCloseTo(truth.maxX, 2);
    const requirement = inspectRoute(route, truth.mFt);
    expect(requirement.truncated).toBe(false);
    expect(requirement.beforeStartFt).toBeCloseTo(truth.before, 2);
    expect(requirement.beyondEndFt).toBeCloseTo(truth.beyond, 2);
  });

  it('starts under the rover, spraying straight up the first lane', () => {
    const route = buildRoute(21.9, 21.9, MEASURED);
    expect(route[0]).toMatchObject({ x: 0, y: 0, spray: true, turning: false, reverse: false });
    const firstPass = route.slice(0, route.findIndex((p) => !p.spray));
    expect(firstPass.length).toBe(45);
    expect(firstPass.every((p) => Math.abs(p.x) < 1e-6 && p.spray)).toBe(true);
    expect(firstPass[firstPass.length - 1].y).toBeCloseTo(21.9, 4);
  });

  it('uses three-point turns between adjacent lanes and never sprays or reverses on a pass', () => {
    const route = buildRoute(21.9, 21.9, MEASURED);
    expect(route.some((p) => p.turning && p.reverse)).toBe(true);
    expect(route.every((p) => !(p.turning && p.spray))).toBe(true);
    expect(route.every((p) => !(p.spray && p.reverse))).toBe(true);
  });

  it('stays continuous at the route step', () => {
    const route = buildRoute(21.9, 21.9, MEASURED);
    for (let i = 1; i < route.length; i++) {
      const dx = route[i].x - route[i - 1].x;
      const dy = route[i].y - route[i - 1].y;
      expect(Math.hypot(dx, dy)).toBeLessThanOrEqual(0.5 * 2.5);
    }
  });
});

describe('estimateHeadland', () => {
  it('returns the three-point requirement for a plannable rectangle', () => {
    expect(estimateHeadland(21.9, 21.9)).toEqual({
      beforeStartFt: expect.closeTo(6.8281, 2),
      beyondEndFt: expect.closeTo(7.0671, 2),
    });
  });

  it('is null when the route would hit the firmware point limit', () => {
    expect(estimateHeadland(300, 120)).toBeNull();
    expect(estimateHeadland(20, 100)).toBeNull();
  });

  it('rejects non-positive dimensions', () => {
    expect(() => estimateHeadland(0, 5)).toThrow(RangeError);
    expect(() => estimateHeadland(5, Number.NaN)).toThrow(RangeError);
  });
});

describe('parseRoverHeadlandLog', () => {
  it('reads the firmware clearance line', () => {
    expect(parseRoverHeadlandLog('Needs clear pavement 7.1 ft past the far end, 6.8 ft behind the start.'))
      .toEqual({ beforeStartFt: 6.8, beyondEndFt: 7.1 });
  });

  it('ignores other lines', () => {
    expect(parseRoverHeadlandLog('Turn radii L=4.33 R=2.92 ft, straight-ahead at 1500us.')).toBeNull();
    expect(parseRoverHeadlandLog('')).toBeNull();
  });
});
