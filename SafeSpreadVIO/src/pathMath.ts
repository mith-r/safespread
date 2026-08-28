export interface PathPoint {
  x: number;
  y: number;
  spraying: boolean;
  /** Recorded after ARKit moved the world, so it is not where it says it is. */
  suspect?: boolean;
}

// The map is on screen for the whole run now, and the path only ever grows, so
// it is drawn in fixed-size chunks: appending a point invalidates the last one
// instead of all three thousand.
export const PATH_CHUNK_SIZE = 200;

export function pathChunks(points: PathPoint[], size = PATH_CHUNK_SIZE): PathPoint[][] {
  if (size < 1) throw new RangeError('chunk size must be positive');
  const chunks: PathPoint[][] = [];
  for (let start = 0; start < points.length; start += size) {
    chunks.push(points.slice(start, start + size));
  }
  return chunks;
}

/** Chunk a growing path, handing back the *same array instance* for every chunk
 *  whose contents have not changed. Identity is the whole point: it is what lets
 *  the map skip redrawing a chunk, and slicing afresh each time would produce
 *  equal-but-new arrays that skip nothing.
 *
 *  A path is append-only until it reaches its cap and then becomes a sliding
 *  window. Once it slides, every chunk holds different points than it did, so
 *  the cache is dropped entirely -- detected by the first point no longer being
 *  the same object. */
export function createPathChunker(size = PATH_CHUNK_SIZE) {
  if (size < 1) throw new RangeError('chunk size must be positive');
  let firstPoint: PathPoint | undefined;
  let chunks: PathPoint[][] = [];

  return function chunk(points: PathPoint[]): PathPoint[][] {
    const previous = points.length > 0 && firstPoint === points[0] ? chunks : [];
    const next: PathPoint[][] = [];
    for (let start = 0, index = 0; start < points.length; start += size, index += 1) {
      const end = Math.min(start + size, points.length);
      const cached = previous[index];
      // Only a chunk that is already full is settled; a partial one is still
      // being appended to and has to be rebuilt.
      next.push(cached !== undefined && cached.length === size && end - start === size
        ? cached
        : points.slice(start, end));
    }
    firstPoint = points[0];
    chunks = next;
    return next;
  };
}

/** Only keep a sample once the rover has actually moved, so a mission of any
 *  length stays bounded, but always keep one where spray switches state so the
 *  sprayed segments are not smeared across the transition. */
export const MIN_SAMPLE_STEP_FT = 0.15;
export const MAX_PATH_POINTS = 3000;

export function shouldRecord(
  last: PathPoint | undefined,
  next: PathPoint
): boolean {
  if (!last) return true;
  if (last.spraying !== next.spraying) return true;
  const dx = next.x - last.x;
  const dy = next.y - last.y;
  return dx * dx + dy * dy >= MIN_SAMPLE_STEP_FT * MIN_SAMPLE_STEP_FT;
}

export interface ViewBox {
  minX: number;
  minY: number;
  spanX: number;
  spanY: number;
}

/** Fit the rectangle and every point travelled -- including excursions into
 *  the headland -- into one box, so nothing is drawn off-canvas. */
export function computeViewBox(
  widthFt: number,
  lengthFt: number,
  points: PathPoint[],
  marginFt = 1
): ViewBox {
  let minX = 0;
  let maxX = widthFt;
  let minY = 0;
  let maxY = lengthFt;

  for (const p of points) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
  }

  minX -= marginFt;
  maxX += marginFt;
  minY -= marginFt;
  maxY += marginFt;

  return {
    minX,
    minY,
    spanX: Math.max(maxX - minX, 0.001),
    spanY: Math.max(maxY - minY, 0.001),
  };
}

/** Map field feet to canvas pixels, preserving aspect ratio and flipping Y
 *  (field +Y is away from the operator; screen +Y goes down). */
export function projector(box: ViewBox, canvasW: number, canvasH: number) {
  const scale = Math.min(canvasW / box.spanX, canvasH / box.spanY);
  const offsetX = (canvasW - box.spanX * scale) / 2;
  const offsetY = (canvasH - box.spanY * scale) / 2;

  return {
    scale,
    toPx(xFt: number, yFt: number) {
      return {
        left: offsetX + (xFt - box.minX) * scale,
        top: offsetY + (box.spanY - (yFt - box.minY)) * scale,
      };
    },
  };
}
