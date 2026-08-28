import React, { useMemo, useRef } from 'react';
import { Modal, Pressable, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { createPathChunker, PathPoint, computeViewBox, projector } from './pathMath';
import { RectangleDefinition } from './rectangle';
import { RoutePlanV2 } from './protocolV2';

interface Props {
  visible: boolean;
  onClose: () => void;
  definition: RectangleDefinition;
  points: PathPoint[];
  routePlan: RoutePlanV2 | null;
  /** Shown across the top while the mission is live. */
  status?: string;
  spraying?: boolean;
  /** Present while the rover can still move. The map opens by itself when a run
   *  starts and covers the whole screen, so the stop control has to come with
   *  it -- otherwise the operator has to dismiss the map to reach it. */
  onStop?: () => void;
}

const DOT = 4;

type ToPx = (xFt: number, yFt: number) => { left: number; top: number };

// One slice of the path, redrawn only when its own points or the projection
// change. The map is now open for the entire run, and the rover appends a point
// every couple of inches -- without this, every one of those appends walks the
// whole path again, and the work grows for as long as the mission lasts.
const PathChunk = React.memo(function PathChunk(
  { points, toPx }: { points: PathPoint[]; toPx: ToPx },
) {
  return (
    <>
      {points.map((p, i) => {
        const { left, top } = toPx(p.x, p.y);
        return (
          <View
            key={i}
            style={[
              styles.dot,
              p.spraying ? styles.dotSpray : styles.dotTravel,
              p.suspect && styles.dotSuspect,
              { left: left - DOT / 2, top: top - DOT / 2 },
            ]}
          />
        );
      })}
    </>
  );
});

export default function PathMap({
  visible, onClose, definition, points, routePlan, status, spraying, onStop,
}: Props) {
  const { width: screenW, height: screenH } = useWindowDimensions();
  const canvasW = screenW - 40;
  const canvasH = screenH - 240;
  const beforeStartFt = routePlan?.beforeStartFt ?? 0;
  const beyondEndFt = routePlan?.beyondEndFt ?? 0;

  const extents: PathPoint[] = [
    { x: 0, y: -beforeStartFt, spraying: false },
    { x: definition.nFt, y: definition.mFt + beyondEndFt, spraying: false },
  ];
  const box = computeViewBox(definition.nFt, definition.mFt, [...points, ...extents]);
  // `toPx` is a dependency of every chunk. Rebuilding it each render would make
  // them all redraw whatever else was memoised, so it is held stable until the
  // view actually has to grow.
  const { toPx, scale } = useMemo(
    () => projector(box, canvasW, canvasH),
    [box.minX, box.minY, box.spanX, box.spanY, canvasW, canvasH],
  );

  const rectTopLeft = toPx(0, definition.mFt);
  const rectW = definition.nFt * scale;
  const rectH = definition.mFt * scale;
  const origin = toPx(0, 0);
  const farM = toPx(0, definition.mFt);
  const farN = toPx(definition.nFt, 0);
  const startHeadland = toPx(0, 0);
  const endHeadland = toPx(0, definition.mFt + beyondEndFt);

  const sprayed = points.filter((p) => p.spraying).length;
  const suspect = points.filter((p) => p.suspect).length;
  // Settled chunks keep their identity as the path grows, so only the chunk
  // still being appended to is re-rendered.
  const chunker = useRef(createPathChunker()).current;
  const chunks = chunker(points);

  return (
    <Modal visible={visible} animationType="slide" onRequestClose={onClose}>
      <View style={[styles.container, spraying && styles.spraying]}>
        {status ? <Text style={styles.status}>{status}</Text> : null}
        <Text style={styles.title}>
          M {definition.mFt.toFixed(1)} × N {definition.nFt.toFixed(1)} ft
        </Text>
        <Text style={styles.orientation}>
          N extends {definition.side}; {definition.source} rectangle
        </Text>
        <Text style={styles.legend}>
          <Text style={styles.spraySwatch}>■</Text> sprayed ({sprayed}) {'   '}
          <Text style={styles.travelSwatch}>■</Text> travelling ({points.length - sprayed})
          {suspect > 0 ? (
            <>
              {'   '}
              <Text style={styles.suspectSwatch}>■</Text> frame moved ({suspect})
            </>
          ) : null}
        </Text>

        <View style={[styles.canvas, { width: canvasW, height: canvasH }]}>
          {/* The rectangle asked for */}
          <View
            style={[
              styles.target,
              {
                left: rectTopLeft.left,
                top: rectTopLeft.top,
                width: rectW,
                height: rectH,
              },
            ]}
          />

          {beforeStartFt > 0 ? (
            <View style={[styles.headland, {
              left: startHeadland.left,
              top: startHeadland.top,
              width: rectW,
              height: beforeStartFt * scale,
            }]} />
          ) : null}
          {beyondEndFt > 0 ? (
            <View style={[styles.headland, {
              left: endHeadland.left,
              top: endHeadland.top,
              width: rectW,
              height: beyondEndFt * scale,
            }]} />
          ) : null}

          <View style={[styles.axis, {
            left: origin.left,
            top: farM.top,
            width: 2,
            height: origin.top - farM.top,
          }]} />
          <Text style={[styles.axisLabel, { left: farM.left + 5, top: farM.top }]}>M ↑</Text>
          <View style={[styles.axis, {
            left: origin.left,
            top: origin.top,
            width: farN.left - origin.left,
            height: 2,
          }]} />
          <Text style={[styles.axisLabel, { left: farN.left - 38, top: farN.top - 20 }]}>N →</Text>

          {chunks.map((chunk, index) => (
            <PathChunk key={index} points={chunk} toPx={toPx} />
          ))}

          {/* Where the rover started: the corner everything is measured from */}
          <View style={[styles.origin, { left: origin.left - 5, top: origin.top - 5 }]} />
        </View>

        {points.length === 0 ? (
          <Text style={styles.empty}>No path recorded yet — run a mission.</Text>
        ) : null}

        <View style={styles.actions}>
          {onStop ? (
            <Pressable
              accessibilityRole="button"
              accessibilityLabel="Stop the rover"
              style={({ pressed }) => [styles.stop, pressed && { opacity: 0.6 }]}
              onPress={onStop}
            >
              <Text style={styles.stopText}>STOP</Text>
            </Pressable>
          ) : null}
          <Pressable
            style={({ pressed }) => [styles.close, pressed && { opacity: 0.6 }]}
            onPress={onClose}
          >
            <Text style={styles.closeText}>Close</Text>
          </Pressable>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#111', paddingTop: 60, alignItems: 'center' },
  // Same signal as the mission screen: the operator can see from across the
  // field that the valve is open without reading anything.
  spraying: { backgroundColor: '#6f0b18' },
  title: { color: 'white', fontSize: 18, fontWeight: '700', marginBottom: 4 },
  orientation: { color: '#9ecbff', fontSize: 13, marginBottom: 4 },
  legend: { color: '#bbb', fontSize: 13, marginBottom: 12 },
  spraySwatch: { color: '#ff5252' },
  travelSwatch: { color: '#5a5a5a' },
  suspectSwatch: { color: '#ffb300' },
  canvas: { backgroundColor: '#000', borderRadius: 6, overflow: 'hidden' },
  target: {
    position: 'absolute',
    borderWidth: 2,
    borderColor: '#2e7d32',
    backgroundColor: 'rgba(46,125,50,0.12)',
  },
  headland: {
    position: 'absolute',
    borderWidth: 1,
    borderColor: '#8d6e63',
    backgroundColor: 'rgba(141,110,99,0.15)',
  },
  axis: { position: 'absolute', backgroundColor: '#ffd54f' },
  axisLabel: { position: 'absolute', color: '#ffd54f', fontSize: 12, fontWeight: '700' },
  dot: { position: 'absolute', width: DOT, height: DOT, borderRadius: DOT / 2 },
  dotSpray: { backgroundColor: '#ff5252' },
  dotTravel: { backgroundColor: '#5a5a5a' },
  // Drawn over the spray/travel colour: this ground is not where it says it is.
  dotSuspect: { borderWidth: 1, borderColor: '#ffb300' },
  origin: {
    position: 'absolute',
    width: 10,
    height: 10,
    borderRadius: 5,
    backgroundColor: '#ffd54f',
  },
  empty: { color: '#888', marginTop: 16 },
  status: { color: 'white', fontSize: 15, fontWeight: '700', marginBottom: 6 },
  actions: {
    marginTop: 'auto',
    marginBottom: 40,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 16,
  },
  stop: {
    backgroundColor: '#d50000',
    paddingVertical: 18,
    paddingHorizontal: 44,
    borderRadius: 8,
  },
  stopText: { color: 'white', fontSize: 20, fontWeight: '900' },
  close: {
    backgroundColor: '#1565c0',
    paddingVertical: 14,
    paddingHorizontal: 40,
    borderRadius: 8,
  },
  closeText: { color: 'white', fontSize: 16, fontWeight: '700' },
});
