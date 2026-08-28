import {
  computeViewBox,
  createPathChunker,
  pathChunks,
  PathPoint,
  projector,
  shouldRecord,
  MIN_SAMPLE_STEP_FT,
} from './pathMath';

describe('shouldRecord', () => {
  it('records the first point', () => {
    expect(shouldRecord(undefined, { x: 0, y: 0, spraying: false })).toBe(true);
  });

  it('skips points that have barely moved', () => {
    const last = { x: 0, y: 0, spraying: false };
    expect(shouldRecord(last, { x: 0.01, y: 0, spraying: false })).toBe(false);
  });

  it('records once past the step threshold', () => {
    const last = { x: 0, y: 0, spraying: false };
    const far = { x: MIN_SAMPLE_STEP_FT + 0.01, y: 0, spraying: false };
    expect(shouldRecord(last, far)).toBe(true);
  });

  it('always records a spray transition, however small the move', () => {
    const last = { x: 0, y: 0, spraying: false };
    expect(shouldRecord(last, { x: 0.001, y: 0, spraying: true })).toBe(true);
  });
});

describe('computeViewBox', () => {
  it('covers the rectangle even with no path yet', () => {
    const box = computeViewBox(10, 20, [], 1);
    expect(box.minX).toBe(-1);
    expect(box.minY).toBe(-1);
    expect(box.spanX).toBeCloseTo(12);
    expect(box.spanY).toBeCloseTo(22);
  });

  it('expands to include headland excursions outside the rectangle', () => {
    const box = computeViewBox(10, 20, [{ x: -4, y: 25, spraying: false }], 1);
    expect(box.minX).toBeCloseTo(-5);
    expect(box.minY).toBeCloseTo(-1);
    expect(box.spanX).toBeCloseTo(16); // -5 .. 11
    expect(box.spanY).toBeCloseTo(27); // -1 .. 26
  });
});

describe('projector', () => {
  it('flips Y so the field origin is at the bottom of the canvas', () => {
    const box = computeViewBox(10, 10, [], 0);
    const p = projector(box, 100, 100);
    const origin = p.toPx(0, 0);
    const top = p.toPx(0, 10);
    expect(origin.top).toBeGreaterThan(top.top);
  });

  it('keeps aspect ratio square on a non-square canvas', () => {
    const box = computeViewBox(10, 10, [], 0);
    const p = projector(box, 200, 100);
    const a = p.toPx(0, 0);
    const b = p.toPx(10, 0);
    // 10ft across should map to the smaller dimension, not stretch to 200px
    expect(b.left - a.left).toBeCloseTo(100);
  });
});

describe('path chunking for the mission map', () => {
  const point = (i: number): PathPoint => ({ x: i, y: i, spraying: false });

  it('splits into fixed-size chunks that cover every point in order', () => {
    const points = Array.from({ length: 7 }, (_, i) => point(i));
    const chunks = pathChunks(points, 3);
    expect(chunks.map((c) => c.length)).toEqual([3, 3, 1]);
    expect(chunks.flat()).toEqual(points);
  });

  it('keeps settled chunks identical as the path grows, so the map can skip them', () => {
    const chunk = createPathChunker(3);
    const points: PathPoint[] = [];
    for (let i = 0; i < 6; i += 1) points.push(point(i));

    const first = chunk([...points]);
    expect(first.map((c) => c.length)).toEqual([3, 3]);

    // Append one point: the two full chunks must come back as the very same
    // arrays, because identity is what React.memo compares.
    points.push(point(6));
    const second = chunk([...points]);
    expect(second[0]).toBe(first[0]);
    expect(second[1]).toBe(first[1]);
    expect(second[2]).toEqual([point(6)]);
  });

  it('rebuilds a partial chunk until it fills up', () => {
    const chunk = createPathChunker(3);
    const a = chunk([point(0)]);
    const b = chunk([point(0), point(1)]);
    expect(b[0]).not.toBe(a[0]);
    expect(b[0]).toHaveLength(2);
  });

  it('drops the cache when the path stops being append-only', () => {
    const chunk = createPathChunker(3);
    const points = Array.from({ length: 6 }, (_, i) => point(i));
    const first = chunk(points);
    // The path has hit its cap and is now a sliding window: every chunk holds
    // different points than it did, so none of them may be reused.
    const slid = [...points.slice(1), point(6)];
    const second = chunk(slid);
    expect(second[0]).not.toBe(first[0]);
    expect(second.flat()).toEqual(slid);
  });

  it('handles an empty path', () => {
    expect(createPathChunker(3)([])).toEqual([]);
    expect(pathChunks([], 3)).toEqual([]);
  });
});
