import React from 'react';
import { Modal, Pressable, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { PathPoint, computeViewBox, projector } from './pathMath';
import { INITIAL_RUN_IN_FT, RectangleDefinition } from './rectangle';

interface Props {
  visible: boolean;
  onClose: () => void;
  definition: RectangleDefinition;
  points: PathPoint[];
}

const DOT = 4;

export default function PathMap({ visible, onClose, definition, points }: Props) {
  const { width: screenW, height: screenH } = useWindowDimensions();
  const canvasW = screenW - 40;
  const canvasH = screenH - 240;

  const extents: PathPoint[] = [
    { x: 0, y: -definition.startClearFt, spraying: false },
    { x: 0, y: -INITIAL_RUN_IN_FT, spraying: false },
    { x: definition.nFt, y: definition.mFt + definition.endClearFt, spraying: false },
  ];
  const box = computeViewBox(definition.nFt, definition.mFt, [...points, ...extents]);
  const { toPx, scale } = projector(box, canvasW, canvasH);

  const rectTopLeft = toPx(0, definition.mFt);
  const rectW = definition.nFt * scale;
  const rectH = definition.mFt * scale;
  const origin = toPx(0, 0);
  const staging = toPx(0, -INITIAL_RUN_IN_FT);
  const farM = toPx(0, definition.mFt);
  const farN = toPx(definition.nFt, 0);
  const startHeadland = toPx(0, 0);
  const endHeadland = toPx(0, definition.mFt + definition.endClearFt);

  const sprayed = points.filter((p) => p.spraying).length;

  return (
    <Modal visible={visible} animationType="slide" onRequestClose={onClose}>
      <View style={styles.container}>
        <Text style={styles.title}>
          M {definition.mFt.toFixed(1)} × N {definition.nFt.toFixed(1)} ft
        </Text>
        <Text style={styles.orientation}>
          N extends {definition.side}; {definition.source} rectangle
        </Text>
        <Text style={styles.stagingHelp}>
          Start at the blue staging dot, {INITIAL_RUN_IN_FT.toFixed(1)} ft before boundary A, facing M ↑
        </Text>
        <Text style={styles.legend}>
          <Text style={styles.spraySwatch}>■</Text> sprayed ({sprayed}) {'   '}
          <Text style={styles.travelSwatch}>■</Text> travelling ({points.length - sprayed})
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

          {definition.startClearFt > 0 ? (
            <View style={[styles.headland, {
              left: startHeadland.left,
              top: startHeadland.top,
              width: rectW,
              height: definition.startClearFt * scale,
            }]} />
          ) : null}
          {definition.endClearFt > 0 ? (
            <View style={[styles.headland, {
              left: endHeadland.left,
              top: endHeadland.top,
              width: rectW,
              height: definition.endClearFt * scale,
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

          {points.map((p, i) => {
            const { left, top } = toPx(p.x, p.y);
            return (
              <View
                key={i}
                style={[
                  styles.dot,
                  p.spraying ? styles.dotSpray : styles.dotTravel,
                  { left: left - DOT / 2, top: top - DOT / 2 },
                ]}
              />
            );
          })}

          {/* Boundary A is the rectangle origin; the rover stages behind it for the run-in. */}
          <View style={[styles.origin, { left: origin.left - 5, top: origin.top - 5 }]} />
          <Text style={[styles.pointLabel, { left: origin.left + 7, top: origin.top - 8 }]}>A boundary</Text>
          <View style={[styles.staging, { left: staging.left - 6, top: staging.top - 6 }]} />
          <Text style={[styles.pointLabel, { left: staging.left + 8, top: staging.top - 7 }]}>START −{INITIAL_RUN_IN_FT.toFixed(1)} ft</Text>
        </View>

        {points.length === 0 ? (
          <Text style={styles.empty}>No path recorded yet — run a mission.</Text>
        ) : null}

        <Pressable
          accessibilityRole="button"
          accessibilityLabel="Close path map"
          style={({ pressed }) => [styles.close, pressed && { opacity: 0.6 }]}
          onPress={onClose}
        >
          <Text style={styles.closeText}>Close</Text>
        </Pressable>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#111', paddingTop: 60, alignItems: 'center' },
  title: { color: 'white', fontSize: 18, fontWeight: '700', marginBottom: 4 },
  orientation: { color: '#9ecbff', fontSize: 13, marginBottom: 4 },
  stagingHelp: { color: '#80d8ff', fontSize: 13, marginBottom: 4, textAlign: 'center' },
  legend: { color: '#bbb', fontSize: 13, marginBottom: 12 },
  spraySwatch: { color: '#ff5252' },
  travelSwatch: { color: '#5a5a5a' },
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
  origin: {
    position: 'absolute',
    width: 10,
    height: 10,
    borderRadius: 5,
    backgroundColor: '#ffd54f',
  },
  staging: {
    position: 'absolute',
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: '#29b6f6',
    borderColor: 'white',
    borderWidth: 1,
  },
  pointLabel: { position: 'absolute', color: 'white', fontSize: 10, fontWeight: '700' },
  empty: { color: '#888', marginTop: 16 },
  close: {
    marginTop: 'auto',
    marginBottom: 40,
    backgroundColor: '#1565c0',
    paddingVertical: 14,
    paddingHorizontal: 40,
    borderRadius: 8,
  },
  closeText: { color: 'white', fontSize: 16, fontWeight: '700' },
});
