import React, { useState } from 'react';
import { LayoutChangeEvent, StyleSheet, Text, View } from 'react-native';
import { computeViewBox, projector } from './pathMath';
import { RectangleDefinition } from './rectangle';
import { laneCenterX, laneCount } from './routePlan';

interface Props {
  definition: RectangleDefinition;
  /** Tallest the figure may grow; width follows the container. */
  maxHeight?: number;
}

const MIN_HEIGHT = 170;
const LABEL_H = 16;
// Smallest band (px) that can hold a caption plus a corner label without them touching.
const BAND_WITH_CORNERS_PX = 44;
const MIN_EDGE_FOR_LABEL_PX = 90;

/** Describe the headland the way the operator should read it. */
export function headlandSummary(definition: RectangleDefinition): string {
  const { startClearFt, endClearFt, headlandSource } = definition;
  if (headlandSource === 'unknown') {
    return 'Clear pavement: rover decides at Arm (area too large for the app to plan)';
  }
  const who = headlandSource === 'rover' ? 'confirmed by rover' : 'estimated';
  return `Clear pavement needed (${who}): ${startClearFt.toFixed(1)} ft behind A · ${endClearFt.toFixed(1)} ft beyond M`;
}

function bandCaption(ft: number, where: string, widthPx: number): string | null {
  if (widthPx >= 170) return `${ft.toFixed(1)} ft clear ${where}`;
  if (widthPx >= 48) return `${ft.toFixed(1)} ft`;
  return null;
}

/**
 * The rectangle as the operator sees it standing at A and looking along M:
 * M runs up the screen, N runs to the coverage side, and the clear pavement
 * the turns need is shaded behind A and beyond the far edge. Labels that
 * would not fit are dropped; the caller's summary line carries the numbers.
 */
export default function RectangleFigure({ definition, maxHeight = 320 }: Props) {
  const [width, setWidth] = useState(0);
  const onLayout = (event: LayoutChangeEvent) => {
    const next = Math.floor(event.nativeEvent.layout.width);
    if (next !== width) setWidth(next);
  };

  const { mFt, nFt, startClearFt, endClearFt, side } = definition;
  const mirror = side === 'left';

  // Field feet including both headlands, plus a 1 ft margin from computeViewBox.
  const box = computeViewBox(nFt, mFt, [
    { x: 0, y: -startClearFt, spraying: false },
    { x: nFt, y: mFt + endClearFt, spraying: false },
  ]);
  const height = width > 0
    ? Math.round(Math.min(maxHeight, Math.max(MIN_HEIGHT, (width * box.spanY) / box.spanX)))
    : MIN_HEIGHT;

  let content: React.ReactNode = null;
  if (width > 0) {
    const { toPx, scale } = projector(box, width, height);
    const px = (xFt: number, yFt: number) => {
      const p = toPx(xFt, yFt);
      return { left: mirror ? width - p.left : p.left, top: p.top };
    };

    const rectLeft = Math.min(px(0, 0).left, px(nFt, 0).left);
    const rectW = nFt * scale;
    const rectH = mFt * scale;
    const rectTop = toPx(0, mFt).top;
    const originPx = px(0, 0);
    const farCorner = px(nFt, mFt);
    const startBand = { top: originPx.top, height: startClearFt * scale };
    const endBand = { top: toPx(0, mFt + endClearFt).top, height: endClearFt * scale };
    const lanes = laneCount(nFt);
    const laneXs = Array.from({ length: lanes }, (_, i) => laneCenterX(i)).filter((x) => x <= nFt);

    // Corner labels and the N dimension sit just outside the rectangle, in the
    // headland bands, when those are tall enough; otherwise just inside it.
    const nearOutside = startBand.height >= BAND_WITH_CORNERS_PX;
    const farOutside = endBand.height >= BAND_WITH_CORNERS_PX;
    const nLabelTop = nearOutside ? originPx.top + 4 : originPx.top - LABEL_H - 5;
    const aLabelTop = nearOutside ? originPx.top + 2 : originPx.top - 22;
    const bLabelTop = farOutside ? farCorner.top - 22 : farCorner.top + 3;
    const showN = rectW - 44 >= MIN_EDGE_FOR_LABEL_PX;
    const showM = rectH >= MIN_EDGE_FOR_LABEL_PX;
    const startCaption = bandCaption(startClearFt, 'behind A', rectW);
    const endCaption = bandCaption(endClearFt, 'beyond M', rectW);

    content = (
      <>
        {startClearFt > 0 ? (
          <View style={[styles.headland, styles.startBand, { left: rectLeft, top: startBand.top, width: rectW, height: startBand.height }]}>
            {startCaption && startBand.height >= LABEL_H + 6 ? (
              <Text style={styles.headlandLabel} numberOfLines={1}>{startCaption}</Text>
            ) : null}
          </View>
        ) : null}
        {endClearFt > 0 ? (
          <View style={[styles.headland, styles.endBand, { left: rectLeft, top: endBand.top, width: rectW, height: endBand.height }]}>
            {endCaption && endBand.height >= LABEL_H + 6 ? (
              <Text style={styles.headlandLabel} numberOfLines={1}>{endCaption}</Text>
            ) : null}
          </View>
        ) : null}

        <View style={[styles.rectangle, { left: rectLeft, top: rectTop, width: rectW, height: rectH }]} />

        {laneXs.map((x) => (
          <View
            key={x}
            style={[styles.lane, { left: px(x, 0).left - 0.5, top: rectTop, height: rectH }]}
          />
        ))}

        {/* M dimension: highlighted A edge, label rotated to run up it, inside the rectangle */}
        <View style={[styles.axis, { left: originPx.left - 1, top: rectTop, width: 2, height: rectH }]} />
        {showM ? (
          <Text
            style={[
              styles.axisLabel,
              {
                width: rectH,
                left: originPx.left + (mirror ? -1 : 1) * (LABEL_H / 2 + 5) - rectH / 2,
                top: rectTop + rectH / 2 - LABEL_H / 2,
                textAlign: 'center',
                transform: [{ rotate: '-90deg' }],
              },
            ]}
            numberOfLines={1}
          >
            M {mFt.toFixed(1)} ft →
          </Text>
        ) : null}

        {/* N dimension: highlighted near edge, label centred along it */}
        <View style={[styles.axis, { left: rectLeft, top: originPx.top - 1, width: rectW, height: 2 }]} />
        {showN ? (
          <Text
            style={[styles.axisLabel, { top: nLabelTop, left: rectLeft + 22, width: Math.max(0, rectW - 44), textAlign: 'center' }]}
            numberOfLines={1}
          >
            {mirror ? '← ' : ''}N {nFt.toFixed(1)} ft{mirror ? '' : ' →'}
          </Text>
        ) : null}

        {/* Corners: A is the rover's start, B the opposite corner */}
        <View style={[styles.cornerDot, { left: originPx.left - 6, top: originPx.top - 6 }]} />
        <Text style={[styles.cornerLabel, { left: mirror ? originPx.left - 20 : originPx.left + 8, top: aLabelTop }]}>A</Text>
        <View style={[styles.cornerDot, styles.farDot, { left: farCorner.left - 5, top: farCorner.top - 5 }]} />
        <Text style={[styles.cornerLabel, { left: mirror ? farCorner.left + 8 : farCorner.left - 20, top: bLabelTop }]}>B</Text>
      </>
    );
  }

  return (
    <View onLayout={onLayout} style={[styles.canvas, { height }]}>
      {content}
    </View>
  );
}

const styles = StyleSheet.create({
  canvas: { width: '100%', backgroundColor: '#000', borderRadius: 6, overflow: 'hidden' },
  rectangle: {
    position: 'absolute',
    borderWidth: 2,
    borderColor: '#2e7d32',
    backgroundColor: 'rgba(46,125,50,0.16)',
  },
  headland: {
    position: 'absolute',
    borderWidth: 1,
    borderColor: '#8d6e63',
    backgroundColor: 'rgba(141,110,99,0.22)',
    alignItems: 'center',
    paddingVertical: 3,
  },
  // Captions keep clear of the corner labels: the band behind A is labelled
  // at its bottom, the band beyond M at its top.
  startBand: { justifyContent: 'flex-end' },
  endBand: { justifyContent: 'flex-start' },
  headlandLabel: { color: '#d7ccc8', fontSize: 11, fontWeight: '600' },
  lane: { position: 'absolute', width: 1, backgroundColor: 'rgba(255,255,255,0.16)' },
  axis: { position: 'absolute', backgroundColor: '#ffd54f' },
  axisLabel: { position: 'absolute', color: '#ffd54f', fontSize: 12, fontWeight: '700', height: LABEL_H },
  cornerDot: { position: 'absolute', width: 12, height: 12, borderRadius: 6, backgroundColor: '#ffd54f' },
  farDot: { width: 10, height: 10, borderRadius: 5, backgroundColor: '#9ecbff' },
  cornerLabel: { position: 'absolute', color: 'white', fontSize: 14, fontWeight: '800' },
});
