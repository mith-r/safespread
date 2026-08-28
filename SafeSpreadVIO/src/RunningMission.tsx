import React, { useState } from 'react';
import { Pressable, StyleSheet, Text, View } from 'react-native';
import PathMap from './PathMap';
import { PathPoint } from './pathMath';
import { Pose } from './poseMath';
import { RectangleDefinition } from './rectangle';
import { SetupPhase } from './setupMachine';
import { TelemetryV2 } from './protocolV2';

interface RunningMissionProps {
  phase: SetupPhase;
  pose: Pose | null;
  trackingDetail: string;
  telemetry: TelemetryV2 | null;
  rectangle: RectangleDefinition;
  path: PathPoint[];
  fault: string | null;
  logName: string | null;
  faultDumpReady: boolean;
  busy: boolean;
  /** Sprayed pass the rover was driving, counted in route order from zero. */
  currentPassIndex: number;
  onStop(): Promise<void>;
  onResume(passIndex: number): Promise<void>;
  onDownloadFault(): Promise<void>;
}

function stateLabel(phase: SetupPhase): string {
  if (phase === 'running') return 'RUNNING';
  if (phase === 'complete') return 'COMPLETE';
  if (phase === 'fault') return 'FAULT';
  return phase.toUpperCase();
}

export default function RunningMission(props: RunningMissionProps) {
  const [mapOpen, setMapOpen] = useState(false);
  const [resumePass, setResumePass] = useState<number | null>(null);
  const telemetry = props.telemetry;
  // Default to the pass that was interrupted: it was cut short, so it is the
  // one that still needs covering.
  const pass = resumePass ?? props.currentPassIndex;
  const stopped = props.phase === 'fault' || props.phase === 'complete';
  const spraying = Boolean(telemetry && (telemetry.flags & 1));
  return (
    <View style={[styles.screen, spraying && styles.spraying]}>
      <View style={styles.topRow}>
        <View>
          <Text style={styles.state}>{stateLabel(props.phase)}</Text>
          <Text style={styles.detail}>{props.trackingDetail}</Text>
        </View>
        <Pressable
          accessibilityRole="button"
          onPress={() => void props.onStop()}
          style={({ pressed }) => [styles.stop, pressed && styles.pressed]}
        >
          <Text style={styles.stopText}>STOP</Text>
        </Pressable>
      </View>

      <View style={styles.card}>
        <Text style={styles.cardTitle}>Pose</Text>
        <Text style={styles.metric}>X {props.pose?.x.toFixed(2) ?? '—'} ft</Text>
        <Text style={styles.metric}>Y {props.pose?.y.toFixed(2) ?? '—'} ft</Text>
        <Text style={styles.metric}>Heading {props.pose?.heading.toFixed(1) ?? '—'}°</Text>
      </View>

      <View style={styles.card}>
        <Text style={styles.cardTitle}>Rover telemetry</Text>
        <Text style={styles.metric}>Route {telemetry ? `${telemetry.routeIndex}/${telemetry.routeCount}` : '—'}</Text>
        <Text style={styles.metric}>Cross-track {telemetry?.crossTrackFt.toFixed(2) ?? '—'} ft</Text>
        <Text style={styles.metric}>Heading error {telemetry?.headingErrorDeg.toFixed(1) ?? '—'}°</Text>
        <Text style={styles.metric}>Speed {telemetry?.speedFps.toFixed(2) ?? '—'} ft/s</Text>
        <Text style={styles.metric}>Steering / throttle {telemetry ? `${telemetry.steeringUs} / ${telemetry.throttleUs} µs` : '—'}</Text>
        <Text style={styles.metric}>Pose age {telemetry?.poseAgeMs ?? '—'} ms · dropped {telemetry?.droppedPackets ?? '—'}</Text>
      </View>

      {props.fault ? (
        <View style={styles.faultCard}>
          <Text style={styles.faultTitle}>Primary fault</Text>
          <Text style={styles.faultText}>{props.fault}</Text>
          <Text style={styles.detail}>Mission log: {props.logName ?? 'unavailable'}</Text>
          <Pressable
            accessibilityRole="button"
            disabled={!props.faultDumpReady}
            onPress={() => void props.onDownloadFault()}
            style={({ pressed }) => [styles.download, !props.faultDumpReady && styles.disabled, pressed && styles.pressed]}
          >
            <Text style={styles.buttonText}>{props.faultDumpReady ? 'Share ESP32 fault buffer' : 'Collecting fault buffer…'}</Text>
          </Pressable>
        </View>
      ) : null}

      {stopped ? (
        <View style={styles.card}>
          <Text style={styles.cardTitle}>Resume</Text>
          <Text style={styles.detail}>
            Keeps this rectangle and starts again on the pass you choose, so the passes
            already laid down are not re-sprayed. The rover refuses to arm until it is
            standing on that pass.
          </Text>
          <View style={styles.resumeRow}>
            <Pressable
              accessibilityRole="button"
              accessibilityLabel="Earlier pass"
              disabled={props.busy || pass <= 0}
              onPress={() => setResumePass(Math.max(0, pass - 1))}
              style={({ pressed }) => [styles.step, (props.busy || pass <= 0) && styles.disabled, pressed && styles.pressed]}
            >
              <Text style={styles.buttonText}>−</Text>
            </Pressable>
            <Text style={styles.metric}>Pass {pass + 1}</Text>
            <Pressable
              accessibilityRole="button"
              accessibilityLabel="Later pass"
              disabled={props.busy}
              onPress={() => setResumePass(pass + 1)}
              style={({ pressed }) => [styles.step, props.busy && styles.disabled, pressed && styles.pressed]}
            >
              <Text style={styles.buttonText}>+</Text>
            </Pressable>
          </View>
          <Pressable
            accessibilityRole="button"
            disabled={props.busy}
            onPress={() => void props.onResume(pass)}
            style={({ pressed }) => [styles.mapButton, props.busy && styles.disabled, pressed && styles.pressed]}
          >
            <Text style={styles.buttonText}>Stop and resume from pass {pass + 1}</Text>
          </Pressable>
        </View>
      ) : null}

      <View style={styles.bottomRow}>
        <Pressable onPress={() => setMapOpen(true)} style={({ pressed }) => [styles.mapButton, pressed && styles.pressed]}>
          <Text style={styles.buttonText}>Mission map</Text>
        </Pressable>
        <Text style={styles.logName}>{props.logName ?? 'Mission log unavailable'}</Text>
      </View>

      <PathMap
        visible={mapOpen}
        onClose={() => setMapOpen(false)}
        definition={props.rectangle}
        points={props.path}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  screen: { flex: 1, backgroundColor: '#050505', paddingTop: 58, paddingHorizontal: 18, paddingBottom: 34, gap: 12 },
  spraying: { backgroundColor: '#6f0b18' },
  topRow: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' },
  state: { color: 'white', fontSize: 28, fontWeight: '900' },
  detail: { color: '#bbb', marginTop: 3 },
  stop: { backgroundColor: '#d50000', minWidth: 132, paddingVertical: 20, paddingHorizontal: 24, borderRadius: 12, alignItems: 'center' },
  stopText: { color: 'white', fontWeight: '900', fontSize: 23 },
  card: { backgroundColor: 'rgba(25,25,25,0.94)', borderRadius: 10, padding: 14, gap: 5 },
  cardTitle: { color: '#9ecbff', fontWeight: '800', fontSize: 17, marginBottom: 3 },
  metric: { color: 'white', fontFamily: 'Menlo', fontSize: 14 },
  faultCard: { backgroundColor: '#3d1010', borderColor: '#ff5252', borderWidth: 1, borderRadius: 10, padding: 14, gap: 8 },
  faultTitle: { color: '#ff8a80', fontSize: 19, fontWeight: '900' },
  faultText: { color: 'white', fontSize: 16, lineHeight: 22 },
  download: { backgroundColor: '#455a64', borderRadius: 8, padding: 12, alignItems: 'center' },
  resumeRow: { flexDirection: 'row', alignItems: 'center', gap: 16 },
  step: { backgroundColor: '#37474f', borderRadius: 8, paddingVertical: 10, paddingHorizontal: 20 },
  bottomRow: { marginTop: 'auto', gap: 8 },
  mapButton: { backgroundColor: '#1565c0', borderRadius: 8, padding: 13, alignItems: 'center' },
  buttonText: { color: 'white', fontWeight: '700' },
  logName: { color: '#aaa', textAlign: 'center', fontSize: 12 },
  pressed: { opacity: 0.62 },
  disabled: { opacity: 0.35 },
});
