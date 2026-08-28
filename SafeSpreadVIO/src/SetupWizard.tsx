import React, { useState } from 'react';
import {
  Modal,
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native';
import { MissionLogFile } from './missionLog';
import { Pose } from './poseMath';
import { RoutePlanV2 } from './protocolV2';
import { SetupAction, SetupState } from './setupMachine';

interface SetupWizardProps {
  state: SetupState;
  roverPose: Pose | null;
  trackingDetail: string;
  readinessReason: string;
  turnRadii: { leftFt: number; rightFt: number; measured: boolean } | null;
  routePlan: RoutePlanV2 | null;
  recentLogs: MissionLogFile[];
  busy: boolean;
  diagnosticProgress: string;
  dispatch(action: SetupAction): void;
  onMeasureTurningRadii(): Promise<void>;
  onSelfTest(): Promise<void>;
  onArm(): Promise<void>;
  onStart(): Promise<void>;
  onStop(): Promise<void>;
  onExport(log: MissionLogFile, format: 'jsonl' | 'csv'): Promise<void>;
}

function numberValue(value: string): number {
  return Number.parseFloat(value);
}

function Button(props: {
  label: string;
  onPress(): void;
  disabled?: boolean;
  tone?: 'primary' | 'danger' | 'secondary';
}) {
  const tone = props.tone ?? 'primary';
  return (
    <Pressable
      accessibilityRole="button"
      disabled={props.disabled}
      onPress={props.onPress}
      style={({ pressed }) => [
        styles.button,
        tone === 'danger' && styles.dangerButton,
        tone === 'secondary' && styles.secondaryButton,
        props.disabled && styles.disabled,
        pressed && styles.pressed,
      ]}
    >
      <Text style={styles.buttonText}>{props.label}</Text>
    </Pressable>
  );
}

function Choice(props: { active: boolean; label: string; onPress(): void; disabled?: boolean }) {
  return (
    <Pressable
      disabled={props.disabled}
      onPress={props.onPress}
      style={({ pressed }) => [
        styles.choice,
        props.active && styles.choiceActive,
        props.disabled && styles.disabled,
        pressed && styles.pressed,
      ]}
    >
      <Text style={styles.choiceText}>{props.label}</Text>
    </Pressable>
  );
}

function Field(props: { label: string; value: string; onChange(value: string): void }) {
  return (
    <View style={styles.field}>
      <Text style={styles.fieldLabel}>{props.label}</Text>
      <TextInput
        style={styles.input}
        value={props.value}
        onChangeText={props.onChange}
        keyboardType="numbers-and-punctuation"
        placeholderTextColor="#777"
        selectTextOnFocus
      />
    </View>
  );
}

function RectanglePreview({ state, routePlan }: {
  state: SetupState;
  routePlan: RoutePlanV2 | null;
}) {
  if (!state.rectangle) return null;
  const rectangle = state.rectangle;
  return (
    <View style={styles.preview}>
      <Text style={styles.previewTitle}>Rectangle preview</Text>
      <Text style={styles.body}>
        {rectangle.mFt.toFixed(1)} ft along M × {rectangle.nFt.toFixed(1)} ft across N
      </Text>
      <Text style={styles.body}>
        Coverage: {rectangle.side.toUpperCase()} · {routePlan
          ? `Calculated headland: ${routePlan.beforeStartFt.toFixed(2)} ft before / ${routePlan.beyondEndFt.toFixed(2)} ft beyond`
          : 'Headland will be calculated by the rover before Start'}
      </Text>
      <View style={styles.rectangleGlyph}>
        <Text style={styles.glyphText}>A  ───── M ─────  far edge{`\n`}│{`\n`}N       opposite corner B{`\n`}│</Text>
      </View>
    </View>
  );
}

export default function SetupWizard(props: SetupWizardProps) {
  const { state } = props;
  const [mText, setMText] = useState('40');
  const [nText, setNText] = useState('12');
  const [side, setSide] = useState<'right' | 'left'>('right');
  const [showDiagnostics, setShowDiagnostics] = useState(false);

  const setEntered = () => {
    if (!props.roverPose) return;
    props.dispatch({
      type: 'SET_ENTERED_RECTANGLE',
      pose: props.roverPose,
      mFt: numberValue(mText),
      nFt: numberValue(nText),
      side,
    });
  };

  const activeMissionPhase = ['arming', 'armed', 'starting', 'running', 'fault'].includes(state.phase);

  return (
    <View style={styles.screen}>
      <View style={styles.header}>
        <Pressable
          accessibilityRole="button"
          onPress={() => void props.onStop()}
          style={({ pressed }) => [styles.stop, pressed && styles.pressed]}
        >
          <Text style={styles.stopText}>STOP</Text>
        </Pressable>
        <View style={styles.headerRow}>
          <View style={styles.headerText}>
            <Text style={styles.title}>SafeSpread setup</Text>
            <Text style={styles.status} numberOfLines={2}>
              BLE {state.connectionStatus} · {props.trackingDetail}
            </Text>
          </View>
          {!activeMissionPhase ? (
            <Pressable
              accessibilityRole="button"
              onPress={() => setShowDiagnostics(true)}
              style={({ pressed }) => [styles.diagButton, pressed && styles.pressed]}
            >
              <Text style={styles.diagButtonText}>Diagnostics</Text>
            </Pressable>
          ) : null}
        </View>
      </View>

      <ScrollView style={styles.scroll} contentContainerStyle={styles.content}>
        {state.phase === 'connection' ? (
          <View style={styles.card}>
            <Text style={styles.cardTitle}>1. Connection</Text>
            <Text style={styles.body}>The app will only proceed after the rover acknowledges protocol v2.</Text>
            <Text style={styles.check}>{state.connectionStatus === 'connected' ? '✓ Connected and compatible' : '○ Waiting for compatible firmware'}</Text>
            <Button label="Continue" disabled={!state.compatible || props.busy} onPress={() => props.dispatch({ type: 'CONTINUE' })} />
          </View>
        ) : null}

        {['connection', 'rectangle'].includes(state.phase) ? (
          <View style={styles.card}>
            <Text style={styles.cardTitle}>Turning radius</Text>
            <Text style={styles.body}>
              {props.turnRadii
                ? `${props.turnRadii.measured ? 'Measured' : 'Built-in'}: left ${props.turnRadii.leftFt.toFixed(2)} ft · right ${props.turnRadii.rightFt.toFixed(2)} ft`
                : 'Waiting for the rover to report its current left and right radii.'}
            </Text>
            <Text style={styles.help}>
              Do this dry, before setting the rectangle, on a large clear paved area. One press drives a left sweep and a right sweep using the existing steering signals. Keep at least 30 ft clear and press STOP if anything is unexpected.
            </Text>
            <Button
              label="Measure both turning radii"
              disabled={props.busy || state.connectionStatus !== 'connected' || !props.roverPose}
              onPress={() => void props.onMeasureTurningRadii()}
            />
            {props.diagnosticProgress ? <Text style={styles.check}>{props.diagnosticProgress}</Text> : null}
          </View>
        ) : null}

        {state.phase === 'rectangle' ? (
          <View style={styles.card}>
            <Text style={styles.cardTitle}>2. Rectangle</Text>
            <Text style={styles.help}>At the start corner, point the mounted phone top along M. Passes run along M.</Text>
            <View style={styles.row}>
              <Field label="M length (ft)" value={mText} onChange={setMText} />
              <Field label="N width (ft)" value={nText} onChange={setNText} />
            </View>
            <View style={styles.row}>
              <Choice active={side === 'right'} label="Cover right" onPress={() => setSide('right')} />
              <Choice active={side === 'left'} label="Cover left" onPress={() => setSide('left')} />
            </View>
            <Button label="Set rectangle at rover" disabled={!props.roverPose || !state.readiness.poseStable} onPress={setEntered} />

            {state.rectangle?.side === 'left' && !state.coverageSideConfirmed ? (
              <Button label="Use LEFT coverage (flip)" tone="secondary" onPress={() => props.dispatch({ type: 'CONFIRM_COVERAGE_SIDE' })} />
            ) : null}

            <RectanglePreview state={state} routePlan={props.routePlan} />
            <Button label="Confirm rectangle" disabled={!state.rectangle} onPress={() => props.dispatch({ type: 'CONTINUE' })} />
          </View>
        ) : null}

        {['readiness', 'arming', 'armed', 'starting'].includes(state.phase) ? (
          <View style={styles.card}>
            <Text style={styles.cardTitle}>3. Readiness and acknowledgements</Text>
            <View style={styles.row}>
              <Choice active={!state.wet} label="Dry diagnostic" disabled={activeMissionPhase} onPress={() => props.dispatch({ type: 'SET_WET_MODE', wet: false })} />
              <Choice active={state.wet} label="Wet brine" disabled={activeMissionPhase} onPress={() => props.dispatch({ type: 'SET_WET_MODE', wet: true })} />
            </View>
            <Text style={styles.check}>{state.readiness.trackingNormal ? '✓' : '○'} Tracking normal</Text>
            <Text style={styles.check}>{state.readiness.poseStable ? '✓' : '○'} Stable 2-second pose window — {props.readinessReason}</Text>
            {state.resumePassIndex > 0 ? (
              <Text style={styles.check}>
                ↻ Resuming on pass {state.resumePassIndex + 1}. Put the rover on that lane facing
                along it — the rover checks the spot itself and refuses to arm until it is there.
              </Text>
            ) : (
              <Text style={styles.check}>{state.readiness.atStart ? '✓' : '○'} Rover at rectangle start</Text>
            )}
            <Text style={styles.check}>
              {props.turnRadii
                ? `✓ Turning radii: left ${props.turnRadii.leftFt.toFixed(2)} ft · right ${props.turnRadii.rightFt.toFixed(2)} ft`
                : '△ Waiting for turning-radius status'}
            </Text>
            {props.routePlan ? (
              <Text style={styles.check}>
                ✓ Required headland: {props.routePlan.beforeStartFt.toFixed(2)} ft before A / {props.routePlan.beyondEndFt.toFixed(2)} ft beyond M
              </Text>
            ) : (
              <Text style={styles.check}>○ Required headland is calculated when the rover is configured.</Text>
            )}
            <Text style={styles.check}>{state.loggingReady ? '✓ Mission log ready' : '△ Mission log unavailable; operation may continue'}</Text>
            {state.phase === 'readiness' ? <Button label="Calculate headland and Arm" disabled={props.busy} onPress={() => void props.onArm()} /> : null}
            {state.phase === 'arming' ? <Text style={styles.waiting}>Waiting for Arm acknowledgement…</Text> : null}
            {state.phase === 'armed' ? <Button label="Start mission" disabled={props.busy} onPress={() => void props.onStart()} /> : null}
            {state.phase === 'starting' ? <Text style={styles.waiting}>Waiting for Running acknowledgement…</Text> : null}
          </View>
        ) : null}

        {state.validationError ? <Text style={styles.error}>{state.validationError}</Text> : null}
        {state.warning ? <Text style={styles.warning}>{state.warning}</Text> : null}

        {!activeMissionPhase ? (
          <View style={styles.card}>
            <Text style={styles.cardTitle}>Recent mission logs</Text>
            {props.recentLogs.length === 0 ? <Text style={styles.body}>No saved missions yet.</Text> : null}
            {props.recentLogs.slice(0, 8).map((log) => (
              <View key={log.uri} style={styles.logRow}>
                <Text numberOfLines={1} style={styles.logName}>{log.name}</Text>
                <Button label="JSONL" tone="secondary" onPress={() => void props.onExport(log, 'jsonl')} />
                <Button label="CSV" tone="secondary" onPress={() => void props.onExport(log, 'csv')} />
              </View>
            ))}
          </View>
        ) : null}
      </ScrollView>

      <Modal visible={showDiagnostics} animationType="slide" onRequestClose={() => setShowDiagnostics(false)}>
        <View style={styles.screen}>
          <View style={styles.header}>
            <Text style={styles.title}>Diagnostics</Text>
            <Text style={styles.status}>The retained self-test checks wiring and moves the rover briefly.</Text>
          </View>
          <ScrollView style={styles.scroll} contentContainerStyle={styles.content}>
            <View style={styles.card}>
              <Text style={styles.cardTitle}>Run self-test</Text>
              <Text style={styles.help}>
                Use Dry diagnostic and clear the area immediately around the rover. The test checks the PWM controller, steering, valve, forward motion, and reverse motion.
              </Text>
              <Button label="Run self-test" tone="secondary" disabled={props.busy || !props.roverPose} onPress={() => void props.onSelfTest()} />
              {props.diagnosticProgress ? <Text style={styles.check}>{props.diagnosticProgress}</Text> : null}
            </View>
          </ScrollView>
          <View style={styles.modalFooter}>
            <Button label="Close" tone="secondary" onPress={() => setShowDiagnostics(false)} />
          </View>
        </View>
      </Modal>
    </View>
  );
}

const styles = StyleSheet.create({
  screen: { flex: 1, backgroundColor: '#050505' },
  header: {
    paddingTop: 56,
    paddingHorizontal: 18,
    paddingBottom: 10,
    gap: 8,
    backgroundColor: '#050505',
    borderBottomWidth: 1,
    borderBottomColor: '#262626',
  },
  stop: { backgroundColor: '#d50000', paddingVertical: 16, borderRadius: 12, alignItems: 'center' },
  stopText: { color: 'white', fontWeight: '900', fontSize: 22 },
  headerRow: { flexDirection: 'row', alignItems: 'flex-start', justifyContent: 'space-between', gap: 12 },
  headerText: { flex: 1 },
  title: { color: 'white', fontSize: 22, fontWeight: '800', marginTop: 4 },
  status: { color: '#aaa', fontSize: 13, lineHeight: 18 },
  diagButton: { backgroundColor: '#37474f', paddingVertical: 10, paddingHorizontal: 14, borderRadius: 8, marginTop: 4 },
  diagButtonText: { color: 'white', fontWeight: '700', fontSize: 13 },
  scroll: { flex: 1 },
  content: { paddingTop: 12, paddingHorizontal: 18, paddingBottom: 48, gap: 14 },
  card: { backgroundColor: '#171717', padding: 14, borderRadius: 10, gap: 10 },
  cardTitle: { color: 'white', fontSize: 19, fontWeight: '700' },
  body: { color: '#ddd', fontSize: 14, lineHeight: 20 },
  help: { color: '#a8c9ff', fontSize: 13, lineHeight: 18 },
  row: { flexDirection: 'row', alignItems: 'flex-end', gap: 8 },
  rowWrap: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  field: { flex: 1, minWidth: 90 },
  fieldLabel: { color: '#bbb', fontSize: 12, marginBottom: 4 },
  input: { backgroundColor: '#292929', color: 'white', borderRadius: 7, paddingHorizontal: 10, paddingVertical: 9, fontSize: 15 },
  button: { backgroundColor: '#2e7d32', paddingVertical: 11, paddingHorizontal: 14, borderRadius: 8, alignItems: 'center', minWidth: 78 },
  dangerButton: { backgroundColor: '#b71c1c' },
  secondaryButton: { backgroundColor: '#37474f' },
  buttonText: { color: 'white', fontSize: 14, fontWeight: '700' },
  choice: { flex: 1, minWidth: 90, backgroundColor: '#292929', padding: 10, borderRadius: 8, alignItems: 'center' },
  choiceActive: { backgroundColor: '#1565c0' },
  choiceText: { color: 'white', fontWeight: '600', textTransform: 'capitalize' },
  pressed: { opacity: 0.62 },
  disabled: { opacity: 0.35 },
  preview: { borderColor: '#496274', borderWidth: 1, borderRadius: 8, padding: 10, gap: 5 },
  previewTitle: { color: '#9ecbff', fontWeight: '700' },
  rectangleGlyph: { borderColor: '#6aa5cc', borderWidth: 2, marginTop: 5, padding: 12, minHeight: 90 },
  glyphText: { color: '#c9e8ff', fontFamily: 'Menlo', fontSize: 12 },
  check: { color: '#c9f7d1', fontSize: 14 },
  waiting: { color: '#ffd180', fontSize: 15, fontWeight: '700' },
  error: { color: '#ff8a80', backgroundColor: '#3b1111', padding: 12, borderRadius: 8 },
  warning: { color: '#ffe082', backgroundColor: '#3b3011', padding: 12, borderRadius: 8 },
  logRow: { flexDirection: 'row', gap: 6, alignItems: 'center' },
  logName: { color: '#ddd', flex: 1, fontSize: 12 },
  modalFooter: { paddingHorizontal: 18, paddingBottom: 34, paddingTop: 8, backgroundColor: '#050505', borderTopWidth: 1, borderTopColor: '#262626' },
});
