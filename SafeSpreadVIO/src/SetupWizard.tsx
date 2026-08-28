import React, { useEffect, useState } from 'react';
import {
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native';
import { CalibrationRecord, PavementSurface } from './calibration';
import { DEFAULT_MOUNT_CALIBRATION } from './hardwareGeometry';
import { MissionLogFile } from './missionLog';
import { Pose } from './poseMath';
import { SetupAction, SetupState } from './setupMachine';

export interface CalibrationFormValue {
  cameraForwardFt: number;
  cameraRightFt: number;
  cameraYawDeg: number;
  sprayForwardFt: number;
  sprayRightFt: number;
  surface: PavementSurface;
}

interface SetupWizardProps {
  state: SetupState;
  roverPose: Pose | null;
  cameraPose: Pose | null;
  trackingDetail: string;
  readinessReason: string;
  calibration: CalibrationRecord | null;
  recentLogs: MissionLogFile[];
  busy: boolean;
  calibrationProgress: string;
  dispatch(action: SetupAction): void;
  onSaveCalibration(value: CalibrationFormValue): Promise<void>;
  onRunCalibration(opcode: 5 | 6 | 7): Promise<void>;
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
      />
    </View>
  );
}

function RectanglePreview({ state }: { state: SetupState }) {
  if (!state.rectangle) return null;
  const rectangle = state.rectangle;
  return (
    <View style={styles.preview}>
      <Text style={styles.previewTitle}>Rectangle preview</Text>
      <Text style={styles.body}>
        {rectangle.mFt.toFixed(1)} ft along M × {rectangle.nFt.toFixed(1)} ft across N
      </Text>
      <Text style={styles.body}>
        Coverage: {rectangle.side.toUpperCase()} · Headland: {rectangle.startClearFt.toFixed(1)} ft before / {rectangle.endClearFt.toFixed(1)} ft beyond
      </Text>
      <View style={styles.rectangleGlyph}>
        <Text style={styles.glyphText}>A  ───── M ─────  far edge{`\n`}│{`\n`}N       opposite corner B{`\n`}│</Text>
      </View>
    </View>
  );
}

export default function SetupWizard(props: SetupWizardProps) {
  const { state } = props;
  const [mText, setMText] = useState('21.9');
  const [nText, setNText] = useState('21.9');
  const [startClearText, setStartClearText] = useState('18.5');
  const [endClearText, setEndClearText] = useState('11.3');
  const [side, setSide] = useState<'right' | 'left'>('right');
  const [cameraForward, setCameraForward] = useState(String(DEFAULT_MOUNT_CALIBRATION.cameraForwardFt));
  const [cameraRight, setCameraRight] = useState(String(DEFAULT_MOUNT_CALIBRATION.cameraRightFt));
  const [cameraYaw, setCameraYaw] = useState(String(DEFAULT_MOUNT_CALIBRATION.cameraYawDeg));
  const [sprayForward, setSprayForward] = useState(String(DEFAULT_MOUNT_CALIBRATION.sprayForwardFt));
  const [sprayRight, setSprayRight] = useState(String(DEFAULT_MOUNT_CALIBRATION.sprayRightFt));
  const [surface, setSurface] = useState<PavementSurface>('concrete');

  useEffect(() => {
    if (!props.calibration) return;
    setCameraForward(String(props.calibration.cameraForwardFt));
    setCameraRight(String(props.calibration.cameraRightFt));
    setCameraYaw(String(props.calibration.cameraYawDeg));
    setSprayForward(String(props.calibration.sprayForwardFt));
    setSprayRight(String(props.calibration.sprayRightFt));
    setSurface(props.calibration.surface);
  }, [props.calibration?.id]);

  const setEntered = () => {
    if (!props.roverPose) return;
    props.dispatch({
      type: 'SET_ENTERED_RECTANGLE',
      pose: props.roverPose,
      mFt: numberValue(mText),
      nFt: numberValue(nText),
      side,
      startClearFt: numberValue(startClearText),
      endClearFt: numberValue(endClearText),
    });
  };

  const captureB = () => {
    if (!props.cameraPose) return;
    props.dispatch({
      type: 'CAPTURE_CORNER_B',
      pose: props.cameraPose,
      stable: state.readiness.poseStable && state.readiness.trackingNormal,
      startClearFt: numberValue(startClearText),
      endClearFt: numberValue(endClearText),
    });
  };

  const saveCalibration = () => {
    void props.onSaveCalibration({
      cameraForwardFt: numberValue(cameraForward),
      cameraRightFt: numberValue(cameraRight),
      cameraYawDeg: numberValue(cameraYaw),
      sprayForwardFt: numberValue(sprayForward),
      sprayRightFt: numberValue(sprayRight),
      surface,
    });
  };

  const activeMissionPhase = ['arming', 'armed', 'starting', 'running', 'fault'].includes(state.phase);

  return (
    <ScrollView style={styles.screen} contentContainerStyle={styles.content}>
      <View style={styles.headerRow}>
        <View>
          <Text style={styles.title}>SafeSpread setup</Text>
          <Text style={styles.status}>
            BLE {state.connectionStatus} · {props.trackingDetail}
          </Text>
        </View>
        <Button label="STOP" tone="danger" onPress={() => void props.onStop()} />
      </View>

      {state.phase === 'connection' ? (
        <View style={styles.card}>
          <Text style={styles.cardTitle}>1. Connection</Text>
          <Text style={styles.body}>The app will only proceed after the rover acknowledges protocol v2.</Text>
          <Text style={styles.check}>{state.connectionStatus === 'connected' ? '✓ Connected and compatible' : '○ Waiting for compatible firmware'}</Text>
          <Button label="Continue" disabled={!state.compatible || props.busy} onPress={() => props.dispatch({ type: 'CONTINUE' })} />
        </View>
      ) : null}

      {state.phase === 'rectangle' ? (
        <View style={styles.card}>
          <Text style={styles.cardTitle}>2. Rectangle</Text>
          <View style={styles.row}>
            <Choice active={state.rectangleMode === 'entered'} label="Enter M × N" onPress={() => props.dispatch({ type: 'SELECT_RECTANGLE_MODE', mode: 'entered' })} />
            <Choice active={state.rectangleMode === 'walked'} label="Walk corners" onPress={() => props.dispatch({ type: 'SELECT_RECTANGLE_MODE', mode: 'walked' })} />
          </View>

          {state.rectangleMode === 'entered' ? (
            <>
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
            </>
          ) : null}

          {state.rectangleMode === 'walked' ? (
            <>
              <Text style={styles.help}>Remove the phone, stand at A, and point its top along M. Then walk directly to opposite corner B.</Text>
              <View style={styles.row}>
                <Button
                  label={state.cornerA ? 'Corner A set' : 'Set Corner A'}
                  disabled={!props.cameraPose || !state.readiness.poseStable}
                  onPress={() => props.cameraPose && props.dispatch({
                    type: 'CAPTURE_CORNER_A',
                    pose: props.cameraPose,
                    stable: state.readiness.poseStable && state.readiness.trackingNormal,
                  })}
                />
                <Button label="Set opposite B" disabled={!state.cornerA || !props.cameraPose || !state.readiness.poseStable} onPress={captureB} />
              </View>
            </>
          ) : null}

          {state.rectangle?.side === 'left' && !state.coverageSideConfirmed ? (
            <Button label="Use LEFT coverage (flip)" tone="secondary" onPress={() => props.dispatch({ type: 'CONFIRM_COVERAGE_SIDE' })} />
          ) : null}

          <View style={styles.row}>
            <Field label="Clear before A (ft)" value={startClearText} onChange={setStartClearText} />
            <Field label="Clear beyond M (ft)" value={endClearText} onChange={setEndClearText} />
          </View>
          <RectanglePreview state={state} />
          <Button label="Confirm rectangle" disabled={!state.rectangle} onPress={() => props.dispatch({ type: 'CONTINUE' })} />
        </View>
      ) : null}

      {state.phase === 'calibration' ? (
        <View style={styles.card}>
          <Text style={styles.cardTitle}>3. Pavement calibration</Text>
          <Text style={styles.body}>Status: {state.calibrationStatus.toUpperCase()}. Recalibrate after changing the mount, steering linkage, tires, load, phone orientation, or firmware calibration schema.</Text>
          <View style={styles.rowWrap}>
            {(['asphalt', 'concrete', 'pavers', 'other'] as PavementSurface[]).map((value) => (
              <Choice key={value} active={surface === value} label={value} onPress={() => setSurface(value)} />
            ))}
          </View>
          <View style={styles.row}>
            <Choice active={!state.wet} label="Dry diagnostic" onPress={() => props.dispatch({ type: 'SET_WET_MODE', wet: false })} />
            <Choice active={state.wet} label="Wet brine" onPress={() => props.dispatch({ type: 'SET_WET_MODE', wet: true })} />
          </View>
          <View style={styles.row}>
            <Field label="Camera forward ft" value={cameraForward} onChange={setCameraForward} />
            <Field label="Camera right ft" value={cameraRight} onChange={setCameraRight} />
          </View>
          <View style={styles.row}>
            <Field label="Camera yaw °" value={cameraYaw} onChange={setCameraYaw} />
            <Field label="Spray forward ft" value={sprayForward} onChange={setSprayForward} />
            <Field label="Spray right ft" value={sprayRight} onChange={setSprayRight} />
          </View>
          <Button label="Save mount and pavement calibration" disabled={props.busy} onPress={saveCalibration} />
          <Text style={styles.help}>Motion calibration is dry and moves the rover. Clear the stated pavement area and press each step only once per completed movement. Steering requires seven presses; speed requires six. The retained self-test also moves forward and reverse.</Text>
          <View style={styles.rowWrap}>
            <Button label="Next steering step" disabled={props.busy || !props.roverPose} onPress={() => void props.onRunCalibration(5)} />
            <Button label="Next speed step" disabled={props.busy || !props.roverPose} onPress={() => void props.onRunCalibration(6)} />
            <Button label="Verify reverse" disabled={props.busy || !props.roverPose} onPress={() => void props.onRunCalibration(7)} />
            <Button label="Run self-test" tone="secondary" disabled={props.busy || !props.roverPose} onPress={() => void props.onSelfTest()} />
          </View>
          {props.calibrationProgress ? <Text style={styles.check}>{props.calibrationProgress}</Text> : null}
          <Button label="Continue to readiness" disabled={state.wet && state.calibrationStatus !== 'ready'} onPress={() => props.dispatch({ type: 'CONTINUE' })} />
        </View>
      ) : null}

      {['readiness', 'arming', 'armed', 'starting'].includes(state.phase) ? (
        <View style={styles.card}>
          <Text style={styles.cardTitle}>4. Readiness and acknowledgements</Text>
          <Text style={styles.check}>{state.readiness.trackingNormal ? '✓' : '○'} Tracking normal</Text>
          <Text style={styles.check}>{state.readiness.poseStable ? '✓' : '○'} Stable 2-second pose window — {props.readinessReason}</Text>
          {state.resumePassIndex > 0 ? (
            <Text style={styles.check}>
              ↻ Resuming on pass {state.resumePassIndex + 1}. Put the rover on that lane facing
              along it — the rover checks the spot itself and refuses to arm until it is there.
            </Text>
          ) : (
            <Text style={styles.check}>{state.readiness.atStart ? '✓' : '○'} Rover at rectangle start{state.rectangleMode === 'walked' ? ' / Corner A' : ''}</Text>
          )}
          <Text style={styles.check}>{state.calibrationStatus === 'ready' ? '✓' : state.wet ? '✕' : '△'} Calibration {state.calibrationStatus}</Text>
          <Text style={styles.check}>{state.loggingReady ? '✓ Mission log ready' : state.wet ? '✕ Mission log required for wet use' : '△ Dry run may continue without a log'}</Text>
          {state.phase === 'readiness' ? <Button label="Configure and Arm" disabled={props.busy} onPress={() => void props.onArm()} /> : null}
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
  );
}

const styles = StyleSheet.create({
  screen: { flex: 1, backgroundColor: '#050505' },
  content: { paddingTop: 56, paddingHorizontal: 18, paddingBottom: 48, gap: 14 },
  headerRow: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', gap: 12 },
  title: { color: 'white', fontSize: 26, fontWeight: '800' },
  status: { color: '#aaa', marginTop: 4 },
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
});
