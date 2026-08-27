import React, { useEffect, useReducer, useRef, useState } from 'react';
import { File, Paths } from 'expo-file-system';
import { useKeepAwake } from 'expo-keep-awake';
import { setAudioModeAsync, useAudioPlayer } from 'expo-audio';
import { SafeSpreadBLE } from './src/ble';
import {
  CalibrationRecord,
  createCalibration,
  markMotionCalibrationVerified,
  motionCalibrationIdFromLog,
} from './src/calibration';
import {
  appendDiagnosticLine,
  calibrationLineResult,
  CalibrationOpcode,
  calibrationStepName,
  calibrationStepTimeoutMs,
  selfTestLineResult,
} from './src/calibrationWorkflow';
import { loadCalibration, saveCalibration } from './src/calibrationStore';
import { LatestPoseSender } from './src/latestPoseSender';
import { MissionControl } from './src/missionControl';
import { missionJsonlToCsv } from './src/missionCsv';
import {
  createFileLogSink,
  exportMissionLog,
  listMissionLogs,
  MissionLogFile,
  MissionLogger,
  MissionRecord,
} from './src/missionLog';
import { MAX_PATH_POINTS, PathPoint, shouldRecord } from './src/pathMath';
import {
  buildPoseV2,
  FaultSampleV2,
  parseFaultSampleV2,
  TelemetryV2,
} from './src/protocolV2';
import { isAtInitialStagingPose, worldToRectangle } from './src/rectangle';
import RunningMission from './src/RunningMission';
import SetupWizard, { CalibrationFormValue } from './src/SetupWizard';
import {
  assembleFaultPackets,
  initialSetupState,
  isAuthoritativeLogReady,
  MissionOperationGate,
  setupReducer,
} from './src/setupMachine';
import { DEFAULT_MOUNT_CALIBRATION, useVIOPose } from './src/useVIOPose';

const HARDWARE_TAG = 'safespread-rover-a';
const APP_VERSION = '1.0.0';
const FIRMWARE_VERSION = 'protocol-v2-hardened-0x0202';
const START_POSITION_TOLERANCE_FT = 0.75;
const START_HEADING_TOLERANCE_DEG = 5;
const POST_CONFIG_POSE_TIMEOUT_MS = 1500;
const SELF_TEST_RESULT_TIMEOUT_MS = 45000;
const ble = new SafeSpreadBLE();

function faultName(code: number): string {
  const names = [
    'none', 'BLE disconnected', 'pose timeout', 'invalid pose', 'pose jump',
    'PWM controller', 'I2C controller', 'no-motion stall', 'wrong direction',
    'tracking degraded', 'route invalid', 'calibration mismatch', 'headland insufficient',
  ];
  return names[code] ?? `firmware fault ${code}`;
}

async function nextMissionEpoch(): Promise<number> {
  const file = new File(Paths.document, 'SafeSpread', 'mission-epoch.txt');
  let previous = Math.floor(Date.now() / 1000) & 0xffff;
  if (file.exists) {
    const decoded = Number.parseInt(await file.text(), 10);
    if (Number.isInteger(decoded) && decoded >= 0 && decoded <= 0xffff) previous = decoded;
  }
  const next = (previous + 1) & 0xffff;
  file.create({ intermediates: true, overwrite: true });
  file.write(String(next));
  return next;
}

export default function App() {
  useKeepAwake();
  const beep = useAudioPlayer(require('./assets/spray-beep.wav'));
  const [setup, dispatch] = useReducer(setupReducer, undefined, initialSetupState);
  const setupRef = useRef(setup);
  setupRef.current = setup;
  const [calibration, setCalibration] = useState<CalibrationRecord | null>(null);
  const calibrationRef = useRef<CalibrationRecord | null>(calibration);
  calibrationRef.current = calibration;
  const mountCalibration = calibration ?? DEFAULT_MOUNT_CALIBRATION;
  const vio = useVIOPose(mountCalibration);
  const trackingOkRef = useRef(vio.trackingOk);
  trackingOkRef.current = vio.trackingOk;
  const [busy, setBusy] = useState(false);
  const [operationError, setOperationError] = useState<string | null>(null);
  const [calibrationProgress, setCalibrationProgress] = useState('');
  const [recentLogs, setRecentLogs] = useState<MissionLogFile[]>([]);
  const [telemetry, setTelemetry] = useState<TelemetryV2 | null>(null);
  const telemetryRef = useRef<TelemetryV2 | null>(null);
  const [path, setPath] = useState<PathPoint[]>([]);
  const [logName, setLogName] = useState<string | null>(null);
  const [faultDumpUri, setFaultDumpUri] = useState<string | null>(null);
  const [faultDumpError, setFaultDumpError] = useState<string | null>(null);

  const controlRef = useRef<MissionControl | null>(null);
  const senderRef = useRef<LatestPoseSender | null>(null);
  const loggerRef = useRef<MissionLogger | null>(null);
  const epochRef = useRef<number | null>(null);
  const calibrationWireRef = useRef<CalibrationRecord | typeof DEFAULT_MOUNT_CALIBRATION | null>(null);
  const poseStreamingRef = useRef(false);
  const rectangleConfiguredRef = useRef(false);
  const calibrationPreparedRef = useRef(false);
  const resourceWetRef = useRef<boolean | null>(null);
  const lastPoseOfferedSequenceRef = useRef(0);
  const firstRectanglePoseOfferIdRef = useRef<number | null>(null);
  const poseBySequenceRef = useRef(new Map<number, MissionRecord>());
  const faultPacketsRef = useRef<Uint8Array[]>([]);
  const faultHandledRef = useRef(false);
  const bootFaultSummaryRef = useRef<string | null>(null);
  const diagnosticModeRef = useRef<'calibration' | 'self-test' | null>(null);
  const pendingMotionCalibrationProofRef = useRef<string | null>(null);
  const calibrationWaiterRef = useRef<{
    resolve(message: string): void;
    reject(error: Error): void;
    hardTimer: ReturnType<typeof setTimeout>;
  } | null>(null);
  const operationGateRef = useRef(new MissionOperationGate());
  const activeOperationSettledRef = useRef<Promise<void> | null>(null);
  const selfTestWaiterRef = useRef<{
    resolve(message: string): void;
    reject(error: Error): void;
    hardTimer: ReturnType<typeof setTimeout>;
  } | null>(null);

  function refreshLogs() {
    try {
      setRecentLogs(listMissionLogs());
    } catch (error) {
      setOperationError(error instanceof Error ? error.message : String(error));
    }
  }

  function beginMissionOperation() {
    const generation = operationGateRef.current.begin();
    let settle!: () => void;
    const settled = new Promise<void>((resolve) => { settle = resolve; });
    activeOperationSettledRef.current = settled;
    return { generation, settle };
  }

  function finishMissionOperation(generation: number, settle: () => void) {
    settle();
    if (operationGateRef.current.isCurrent(generation)) {
      activeOperationSettledRef.current = null;
      setBusy(false);
    }
  }

  function cancelActiveMissionOperation(): Promise<void> | null {
    const pending = activeOperationSettledRef.current;
    operationGateRef.current.cancel();
    const calibrationWaiter = calibrationWaiterRef.current;
    if (calibrationWaiter) {
      clearTimeout(calibrationWaiter.hardTimer);
      calibrationWaiterRef.current = null;
      pendingMotionCalibrationProofRef.current = null;
      calibrationWaiter.resolve('Calibration cancelled by Stop');
    }
    const selfTestWaiter = selfTestWaiterRef.current;
    if (selfTestWaiter) {
      clearTimeout(selfTestWaiter.hardTimer);
      selfTestWaiterRef.current = null;
      selfTestWaiter.resolve('Self-test cancelled by Stop');
    }
    return pending;
  }

  function recordLog(record: MissionRecord) {
    const logger = loggerRef.current;
    if (!logger) return;
    void logger.record(record).catch((error) => {
      dispatch({ type: 'SET_LOGGING_READY', ready: false });
      const current = setupRef.current;
      if (current.wet && ['arming', 'armed', 'starting', 'running'].includes(current.phase)) {
        void handleMissionFault(`authoritative mission log failed: ${error.message}`);
      } else {
        setOperationError(`Mission log failed: ${error.message}`);
      }
    });
  }

  async function closeLogger() {
    const logger = loggerRef.current;
    loggerRef.current = null;
    if (logger) await logger.close();
    refreshLogs();
  }

  async function releaseMissionResources(closeLog = true) {
    poseStreamingRef.current = false;
    const sender = senderRef.current;
    senderRef.current = null;
    if (sender) await sender.stop().catch(() => {});
    controlRef.current?.dispose();
    controlRef.current = null;
    epochRef.current = null;
    calibrationWireRef.current = null;
    rectangleConfiguredRef.current = false;
    firstRectanglePoseOfferIdRef.current = null;
    calibrationPreparedRef.current = false;
    resourceWetRef.current = null;
    poseBySequenceRef.current.clear();
    if (closeLog) await closeLogger().catch((error) => {
      setOperationError(error instanceof Error ? error.message : String(error));
    });
  }

  async function ensureMissionResources() {
    if (!setup.rectangle) throw new Error('Define the rectangle before starting a mission.');
    if (controlRef.current && senderRef.current && epochRef.current !== null &&
        resourceWetRef.current === setup.wet &&
        (!setup.wet || isAuthoritativeLogReady(loggerRef.current))) {
      return { control: controlRef.current, epoch: epochRef.current };
    }
    if (controlRef.current || senderRef.current || loggerRef.current) {
      await controlRef.current?.stop().catch(() => {});
      await releaseMissionResources(true);
      dispatch({ type: 'SET_LOGGING_READY', ready: false });
    }
    const epoch = await nextMissionEpoch();
    const wire = calibration ?? DEFAULT_MOUNT_CALIBRATION;
    const missionId = `${new Date().toISOString().replace(/[:.]/g, '-')}-e${epoch}`;
    try {
      const fileLog = await createFileLogSink(missionId);
      const logger = await MissionLogger.create({
        missionId,
        createdAtIso: new Date().toISOString(),
        appVersion: APP_VERSION,
        firmwareVersion: FIRMWARE_VERSION,
        protocolVersion: 2,
        epoch,
        calibrationId: wire.id,
        calibrationSchemaVersion: wire.schemaVersion,
        pavement: {
          surface: calibration?.surface ?? 'other',
          condition: setup.wet ? 'wet' : 'dry',
          operatingLoadLb: calibration?.operatingLoadLb ?? null,
        },
        rectangle: {
          source: setup.rectangle.source,
          mFt: setup.rectangle.mFt,
          nFt: setup.rectangle.nFt,
          side: setup.rectangle.side,
        },
      }, fileLog.sink);
      loggerRef.current = logger;
      setLogName(fileLog.uri.split('/').pop() ?? missionId);
      dispatch({ type: 'SET_LOGGING_READY', ready: true });
    } catch (error) {
      dispatch({ type: 'SET_LOGGING_READY', ready: false });
      if (setup.wet) throw new Error(`Wet operation requires a mission log: ${error instanceof Error ? error.message : String(error)}`);
      setOperationError(`Dry diagnostic log unavailable: ${error instanceof Error ? error.message : String(error)}`);
    }
    const control = new MissionControl(ble, epoch, () => trackingOkRef.current, {
      dryMode: !setup.wet,
      preferForwardOnly: true,
    });
    const sender = new LatestPoseSender(ble, (error) => {
      void handleMissionFault(`pose transport failed: ${error.message}`);
    });
    controlRef.current = control;
    senderRef.current = sender;
    epochRef.current = epoch;
    calibrationWireRef.current = wire;
    poseStreamingRef.current = true;
    rectangleConfiguredRef.current = false;
    firstRectanglePoseOfferIdRef.current = null;
    calibrationPreparedRef.current = false;
    resourceWetRef.current = setup.wet;
    faultHandledRef.current = false;
    faultPacketsRef.current = [];
    setFaultDumpUri(null);
    setFaultDumpError(null);
    setPath([]);
    return { control, epoch };
  }

  async function persistFaultDump(samples: FaultSampleV2[], epoch: number): Promise<string> {
    const file = new File(Paths.document, 'SafeSpread', 'faults', `fault-e${epoch}-${Date.now()}.json`);
    file.create({ intermediates: true, overwrite: true });
    file.write(JSON.stringify({
      schemaVersion: 1,
      epoch,
      persistedSummary: bootFaultSummaryRef.current,
      samples,
    }, null, 2));
    return file.uri;
  }

  async function handleMissionFault(cause: string) {
    if (faultHandledRef.current) return;
    faultHandledRef.current = true;
    const pendingOperation = cancelActiveMissionOperation();
    dispatch({ type: 'MISSION_FAULT', cause });
    setBusy(true);
    recordLog({ type: 'fault', phoneMs: Date.now(), fault: cause, state: 'FAULT' });
    const control = controlRef.current;
    const epoch = epochRef.current;
    try {
      if (control) await control.stop();
      else await ble.emergencyStop();
    } catch (error) {
      setOperationError(`Stop acknowledgement failed: ${error instanceof Error ? error.message : String(error)}`);
    }
    await pendingOperation;
    activeOperationSettledRef.current = null;
    if (control && epoch !== null) {
      faultPacketsRef.current = [];
      try {
        await control.dumpFault();
        const samples = assembleFaultPackets(faultPacketsRef.current, epoch);
        for (const sample of samples) {
          const { state: firmwareState, ...fields } = sample;
          recordLog({
            type: 'fault_buffer',
            phoneMs: Date.now(),
            ...fields,
            firmwareState,
          });
        }
        if (bootFaultSummaryRef.current) {
          recordLog({ type: 'persisted_fault_summary', phoneMs: Date.now(), summary: bootFaultSummaryRef.current });
        }
        setFaultDumpUri(await persistFaultDump(samples, epoch));
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        recordLog({
          type: 'fault_buffer_error',
          phoneMs: Date.now(),
          fault: message,
        });
        setFaultDumpError(`ESP32 fault buffer unavailable: ${message}`);
      }
    }
    poseStreamingRef.current = false;
    await senderRef.current?.stop().catch(() => {});
    await closeLogger().catch((error) => {
      setOperationError(error instanceof Error ? error.message : String(error));
    });
    setBusy(false);
  }

  function settleCalibrationResult(line: string) {
    const result = calibrationLineResult(line);
    const waiter = calibrationWaiterRef.current;
    if (!result || !waiter) return;
    clearTimeout(waiter.hardTimer);
    calibrationWaiterRef.current = null;
    const proofLine = pendingMotionCalibrationProofRef.current;
    pendingMotionCalibrationProofRef.current = null;
    if (result === 'success') {
      if (proofLine) void persistMotionCalibrationProof(proofLine);
      waiter.resolve(line);
    } else {
      waiter.reject(new Error(line));
    }
  }

  function settleSelfTestResult(line: string) {
    const result = selfTestLineResult(line);
    const waiter = selfTestWaiterRef.current;
    if (!result || !waiter) return;
    clearTimeout(waiter.hardTimer);
    selfTestWaiterRef.current = null;
    if (result === 'success') waiter.resolve(line);
    else waiter.reject(new Error(line));
  }

  async function persistMotionCalibrationProof(line: string) {
    const completedId = motionCalibrationIdFromLog(line);
    if (completedId === null) return;
    const current = calibrationRef.current;
    if (!current || current.id !== completedId) {
      dispatch({ type: 'SET_CALIBRATION_STATUS', status: 'stale' });
      setOperationError(
        `Rover completed calibration ID ${completedId}, but the phone currently has ` +
        `${current ? `ID ${current.id}` : 'no saved calibration'}. Save and repeat the loaded calibration.`,
      );
      return;
    }
    try {
      const verified = markMotionCalibrationVerified(current, new Date().toISOString());
      await saveCalibration(verified, HARDWARE_TAG);
      if (calibrationRef.current?.id !== completedId) return;
      calibrationRef.current = verified;
      setCalibration(verified);
      dispatch({ type: 'SET_CALIBRATION_STATUS', status: 'ready' });
    } catch (error) {
      dispatch({ type: 'SET_CALIBRATION_STATUS', status: 'stale' });
      setOperationError(`Could not save motion-calibration proof: ${error instanceof Error ? error.message : String(error)}`);
    }
  }

  function handleFirmwareLog(line: string) {
    if (line.startsWith('[BOOT FAULT]') || line.startsWith('[FAULT SUMMARY]')) {
      bootFaultSummaryRef.current = line;
    }
    if (diagnosticModeRef.current || line.startsWith('[CAL')) {
      setCalibrationProgress((previous) => appendDiagnosticLine(previous, line));
    }
    settleCalibrationResult(line);
    settleSelfTestResult(line);
    if (motionCalibrationIdFromLog(line) !== null && calibrationWaiterRef.current) {
      pendingMotionCalibrationProofRef.current = line;
    }
    recordLog({ type: 'firmware_log', phoneMs: Date.now(), message: line });
  }

  useEffect(() => {
    setAudioModeAsync({ playsInSilentMode: true }).catch(() => {});
    beep.loop = true;
  }, []);

  useEffect(() => {
    const spraying = Boolean(telemetry && (telemetry.flags & 1));
    if (!setup.wet && spraying) {
      beep.seekTo(0);
      beep.play();
    } else {
      beep.pause();
    }
  }, [setup.wet, telemetry?.flags]);

  useEffect(() => {
    void loadCalibration(HARDWARE_TAG).then((result) => {
      calibrationRef.current = result.calibration;
      setCalibration(result.calibration);
      dispatch({
        type: 'SET_CALIBRATION_STATUS',
        status: result.reason === 'ready' ? 'ready' : result.reason === 'missing' ? 'missing' : 'stale',
      });
    }).catch((error) => {
      setOperationError(`Calibration load failed: ${error instanceof Error ? error.message : String(error)}`);
      dispatch({ type: 'SET_CALIBRATION_STATUS', status: 'stale' });
    });
    refreshLogs();
  }, []);

  async function connectToRover() {
    setOperationError(null);
    try {
      await ble.connect(
        (status) => dispatch({
          type: 'CONNECTION_CHANGED',
          status,
          compatible: status === 'connected',
        }),
        handleFirmwareLog,
      );
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      if (!/connection cancelled/i.test(message)) {
        setOperationError(`BLE connection failed: ${message}`);
      }
    }
  }

  useEffect(() => {
    const removeTelemetry = ble.subscribeTelemetry((next) => {
      const activeEpoch = epochRef.current;
      if (activeEpoch === null || next.epoch !== activeEpoch) return;
      telemetryRef.current = next;
      setTelemetry(next);
      const poseRecord = poseBySequenceRef.current.get(next.consumedPoseSequence) ?? {};
      recordLog({
        ...poseRecord,
        type: 'control',
        phoneMs: Date.now(),
        sequence: next.consumedPoseSequence,
        epoch: next.epoch,
        routeIndex: next.routeIndex,
        crossTrackFt: next.crossTrackFt,
        headingErrorDeg: next.headingErrorDeg,
        speedFps: next.speedFps,
        steeringUs: next.steeringUs,
        throttleUs: next.throttleUs,
        fault: next.faultCode,
        spray: Boolean(next.flags & 1),
        reverse: Boolean(next.flags & 2),
        pwmReady: Boolean(next.flags & 4),
        poseAgeMs: next.poseAgeMs,
        droppedPackets: next.droppedPackets,
      });
      if (next.state === 4 && setupRef.current.phase === 'running') {
        controlRef.current?.notifyComplete();
        dispatch({ type: 'MISSION_COMPLETE' });
        recordLog({ type: 'state', phoneMs: Date.now(), state: 'COMPLETE' });
        poseStreamingRef.current = false;
        void senderRef.current?.stop().catch(() => {});
        void closeLogger().catch((error) => setOperationError(error.message));
      } else if (next.state === 5) {
        void handleMissionFault(faultName(next.faultCode));
      }
    });
    const removeFaultPackets = ble.subscribeFaultPackets((packet) => {
      const sample = parseFaultSampleV2(packet);
      if (sample && sample.epoch === epochRef.current) faultPacketsRef.current.push(packet);
    });
    const removeDisconnect = ble.subscribeDisconnect(() => {
      controlRef.current?.notifyDisconnect();
      const phase = setupRef.current.phase;
      if (['arming', 'armed', 'starting', 'running'].includes(phase)) {
        void handleMissionFault('BLE disconnected');
      } else if (phase !== 'fault') {
        const pendingOperation = cancelActiveMissionOperation();
        setBusy(true);
        void (async () => {
          await pendingOperation;
          activeOperationSettledRef.current = null;
          await releaseMissionResources(true);
          dispatch({ type: 'SET_LOGGING_READY', ready: false });
          setOperationError('BLE disconnected. Reconnect before continuing setup.');
          setBusy(false);
        })();
      }
    });
    void connectToRover();
    return () => {
      cancelActiveMissionOperation();
      removeTelemetry();
      removeFaultPackets();
      removeDisconnect();
      controlRef.current?.dispose();
      void ble.disconnect();
    };
  }, []);

  const rectanglePose = vio.validatedPose && setup.rectangle
    ? worldToRectangle(vio.validatedPose.rover, setup.rectangle)
    : null;
  const atStart = isAtInitialStagingPose(
    rectanglePose,
    START_POSITION_TOLERANCE_FT,
    START_HEADING_TOLERANCE_DEG,
  );

  useEffect(() => {
    dispatch({
      type: 'SET_READINESS',
      trackingNormal: vio.trackingOk,
      poseStable: vio.readiness.ready,
      atStart,
    });
  }, [vio.trackingOk, vio.readiness.ready, atStart]);

  useEffect(() => {
    const validated = vio.validatedPose;
    const sender = senderRef.current;
    const epoch = epochRef.current;
    const wire = calibrationWireRef.current;
    const rectangle = setupRef.current.rectangle;
    if (!validated || !sender || epoch === null || !wire || !rectangle ||
        !poseStreamingRef.current || !vio.trackingOk) return;
    const rectangleFrame = rectangleConfiguredRef.current;
    const roverPose = rectangleFrame
      ? worldToRectangle(validated.rover, rectangle)
      : validated.rover;
    const ageMs = Math.max(0, validated.captureAgeMs +
      (globalThis.performance?.now() ?? Date.now()) - validated.receivedAtMs);
    const yawRate = rectangleFrame && rectangle.side === 'left'
      ? -validated.yawRateDps
      : validated.yawRateDps;
    try {
      const offerId = sender.offer(buildPoseV2({
        flags: 1 | (validated.courseDeg === null ? 0 : 2) | 4,
        epoch,
        sequence: validated.sequence,
        ageMs,
        x: roverPose.x,
        y: roverPose.y,
        heading: roverPose.heading,
        speedFps: validated.speedFps,
        yawRateDps: yawRate,
        calibrationId: wire.id,
      }));
      if (rectangleFrame && firstRectanglePoseOfferIdRef.current === null) {
        firstRectanglePoseOfferIdRef.current = offerId;
      }
      lastPoseOfferedSequenceRef.current = validated.sequence;
      const poseRecord: MissionRecord = {
        type: 'pose',
        phoneMs: Date.now(),
        sequence: validated.sequence,
        epoch,
        xFt: roverPose.x,
        yFt: roverPose.y,
        headingDeg: roverPose.heading,
        speedFps: validated.speedFps,
        yawRateDps: yawRate,
        trackingValid: true,
        captureAgeMs: ageMs,
        cameraXFt: validated.camera.x,
        cameraYFt: validated.camera.y,
        cameraHeadingDeg: validated.camera.heading,
        roverWorldXFt: validated.rover.x,
        roverWorldYFt: validated.rover.y,
        sprayWorldXFt: validated.sprayBar.x,
        sprayWorldYFt: validated.sprayBar.y,
        trackingState: vio.trackingState,
        trackingReason: vio.trackingReason,
        mappingStatus: vio.mappingStatus,
        senderDropped: sender.dropped,
      };
      poseBySequenceRef.current.set(validated.sequence, poseRecord);
      while (poseBySequenceRef.current.size > 256) {
        const oldest = poseBySequenceRef.current.keys().next().value as number | undefined;
        if (oldest === undefined) break;
        poseBySequenceRef.current.delete(oldest);
      }
      recordLog(poseRecord);
      if (setupRef.current.phase === 'running') {
        const nextPoint: PathPoint = {
          x: roverPose.x,
          y: roverPose.y,
          spraying: Boolean(telemetryRef.current && (telemetryRef.current.flags & 1)),
        };
        setPath((previous) => shouldRecord(previous.at(-1), nextPoint)
          ? [...previous.slice(-(MAX_PATH_POINTS - 1)), nextPoint]
          : previous);
      }
    } catch (error) {
      void handleMissionFault(`pose packet failed: ${error instanceof Error ? error.message : String(error)}`);
    }
  }, [vio.validatedPose, vio.trackingOk]);

  async function onSaveCalibration(value: CalibrationFormValue) {
    setBusy(true);
    setOperationError(null);
    try {
      if (controlRef.current) {
        await controlRef.current.stop().catch(() => {});
        await releaseMissionResources(true);
      }
      const record = createCalibration({
        schemaVersion: 1,
        hardwareTag: HARDWARE_TAG,
        createdAtIso: new Date().toISOString(),
        ...value,
        condition: setup.wet ? 'wet' : 'dry',
      });
      await saveCalibration(record, HARDWARE_TAG);
      calibrationRef.current = record;
      setCalibration(record);
      dispatch({ type: 'SET_CALIBRATION_STATUS', status: 'stale' });
      setCalibrationProgress(
        `Phone calibration ID ${record.id} saved. Wet mode remains blocked until the rover ` +
        'reports that steering, loaded speed, and reverse calibration were saved for this same ID.',
      );
    } catch (error) {
      setOperationError(error instanceof Error ? error.message : String(error));
      dispatch({ type: 'SET_CALIBRATION_STATUS', status: 'stale' });
    } finally {
      setBusy(false);
    }
  }

  function awaitCalibrationResult(opcode: CalibrationOpcode): Promise<string> {
    return new Promise((resolve, reject) => {
      const hardTimer = setTimeout(() => {
        calibrationWaiterRef.current = null;
        pendingMotionCalibrationProofRef.current = null;
        reject(new Error(
          `${calibrationStepName(opcode)} result timeout; the rover was given ` +
          `${calibrationStepTimeoutMs(opcode) / 1000} seconds. Press Stop and inspect the visible rover transcript.`,
        ));
      }, calibrationStepTimeoutMs(opcode));
      calibrationWaiterRef.current = {
        resolve,
        reject,
        hardTimer,
      };
    });
  }

  async function onRunCalibration(opcode: 5 | 6 | 7) {
    const operation = beginMissionOperation();
    setBusy(true);
    setOperationError(null);
    diagnosticModeRef.current = 'calibration';
    pendingMotionCalibrationProofRef.current = null;
    setCalibrationProgress(`Starting ${calibrationStepName(opcode)}…`);
    try {
      if (setup.wet) throw new Error('Select Dry diagnostic before any calibration movement.');
      if (!calibration) throw new Error('Save mount and pavement calibration before motion calibration.');
      const { control } = await ensureMissionResources();
      operationGateRef.current.assertCurrent(operation.generation);
      if (!calibrationPreparedRef.current) {
        await control.prepareCalibration(calibration);
        operationGateRef.current.assertCurrent(operation.generation);
        calibrationPreparedRef.current = true;
      }
      const beforeSequence = lastPoseOfferedSequenceRef.current;
      const deadline = Date.now() + 750;
      while (lastPoseOfferedSequenceRef.current <= beforeSequence && Date.now() < deadline) {
        await new Promise((resolve) => setTimeout(resolve, 20));
        operationGateRef.current.assertCurrent(operation.generation);
      }
      operationGateRef.current.assertCurrent(operation.generation);
      if (lastPoseOfferedSequenceRef.current <= beforeSequence) {
        throw new Error('No new validated pose reached the rover. Restore normal tracking before moving.');
      }
      const result = awaitCalibrationResult(opcode);
      try {
        await control.runCalibrationStep(opcode);
        operationGateRef.current.assertCurrent(operation.generation);
        const terminalLine = await result;
        setCalibrationProgress((previous) => appendDiagnosticLine(previous, terminalLine));
        operationGateRef.current.assertCurrent(operation.generation);
      } catch (error) {
        const waiter = calibrationWaiterRef.current;
        if (waiter) {
          clearTimeout(waiter.hardTimer);
          calibrationWaiterRef.current = null;
          pendingMotionCalibrationProofRef.current = null;
          waiter.resolve('Calibration command did not start.');
        }
        throw error;
      }
    } catch (error) {
      if (operationGateRef.current.isCurrent(operation.generation)) {
        setOperationError(error instanceof Error ? error.message : String(error));
      }
    } finally {
      diagnosticModeRef.current = null;
      finishMissionOperation(operation.generation, operation.settle);
    }
  }

  async function onSelfTest() {
    const operation = beginMissionOperation();
    setBusy(true);
    setOperationError(null);
    diagnosticModeRef.current = 'self-test';
    setCalibrationProgress('Starting dry self-test; the rover will steer and verify both drive directions while physical spray output remains off…');
    try {
      if (setup.wet) throw new Error('Select Dry diagnostic before the self-test.');
      if (!calibration) throw new Error('Save mount and pavement calibration before the self-test.');
      const { control } = await ensureMissionResources();
      operationGateRef.current.assertCurrent(operation.generation);
      if (!calibrationPreparedRef.current) {
        await control.prepareCalibration(calibration);
        operationGateRef.current.assertCurrent(operation.generation);
        calibrationPreparedRef.current = true;
      }
      const result = new Promise<string>((resolve, reject) => {
        const hardTimer = setTimeout(() => {
          selfTestWaiterRef.current = null;
          reject(new Error('Self-test result timeout; press Stop and inspect rover logs.'));
        }, SELF_TEST_RESULT_TIMEOUT_MS);
        selfTestWaiterRef.current = { resolve, reject, hardTimer };
      });
      try {
        await control.selfTest();
        operationGateRef.current.assertCurrent(operation.generation);
        const terminalLine = await result;
        setCalibrationProgress((previous) => appendDiagnosticLine(previous, terminalLine));
        operationGateRef.current.assertCurrent(operation.generation);
      } catch (error) {
        const waiter = selfTestWaiterRef.current;
        if (waiter) {
          clearTimeout(waiter.hardTimer);
          selfTestWaiterRef.current = null;
          waiter.resolve('Self-test command did not start.');
        }
        throw error;
      }
    } catch (error) {
      if (operationGateRef.current.isCurrent(operation.generation)) {
        setOperationError(error instanceof Error ? error.message : String(error));
      }
    } finally {
      diagnosticModeRef.current = null;
      finishMissionOperation(operation.generation, operation.settle);
    }
  }

  async function onArm() {
    const operation = beginMissionOperation();
    setBusy(true);
    setOperationError(null);
    try {
      const { control } = await ensureMissionResources();
      operationGateRef.current.assertCurrent(operation.generation);
      const candidate = setupReducer({
        ...setup,
        loggingReady: isAuthoritativeLogReady(loggerRef.current) || !setup.wet,
      }, { type: 'REQUEST_ARM' });
      if (candidate.phase !== 'arming') {
        throw new Error(candidate.validationError ?? 'Readiness checks did not pass.');
      }
      dispatch({ type: 'REQUEST_ARM' });
      if (!setup.rectangle) throw new Error('Rectangle is missing.');
      const wire = calibration ?? DEFAULT_MOUNT_CALIBRATION;
      await control.configure(setup.rectangle, wire);
      operationGateRef.current.assertCurrent(operation.generation);
      firstRectanglePoseOfferIdRef.current = null;
      rectangleConfiguredRef.current = true;
      const sender = senderRef.current;
      if (!sender) throw new Error('Pose transport stopped during Configure.');
      const poseDeadline = Date.now() + POST_CONFIG_POSE_TIMEOUT_MS;
      let deliveredRectanglePose = false;
      while (Date.now() < poseDeadline) {
        operationGateRef.current.assertCurrent(operation.generation);
        const firstRectangleOffer = firstRectanglePoseOfferIdRef.current;
        if (firstRectangleOffer !== null && sender.lastSentOfferId >= firstRectangleOffer) {
          deliveredRectanglePose = true;
          break;
        }
        await new Promise<void>((resolve) => setTimeout(resolve, 25));
      }
      if (!deliveredRectanglePose) {
        throw new Error(
          'No fresh rectangle-frame pose reached the rover after Configure. Stop, restore normal tracking, and try again.',
        );
      }
      const currentReadiness = setupRef.current.readiness;
      if (!trackingOkRef.current || !currentReadiness.poseStable || !currentReadiness.atStart) {
        throw new Error('Readiness changed during Configure; Stop and return to staging 1.0 ft before boundary A.');
      }
      await control.arm();
      operationGateRef.current.assertCurrent(operation.generation);
      dispatch({ type: 'ARM_ACKNOWLEDGED' });
      recordLog({ type: 'state', phoneMs: Date.now(), state: 'ARMED' });
    } catch (error) {
      if (!operationGateRef.current.isCurrent(operation.generation)) return;
      const message = error instanceof Error ? error.message : String(error);
      const cause = /ACK timeout/i.test(message)
        ? 'Arm acknowledgement timeout; the rover was stopped before motion could start.'
        : message;
      void handleMissionFault(cause);
    } finally {
      finishMissionOperation(operation.generation, operation.settle);
    }
  }

  async function onStart() {
    const control = controlRef.current;
    if (!control) return;
    const operation = beginMissionOperation();
    setBusy(true);
    setOperationError(null);
    dispatch({ type: 'REQUEST_START' });
    try {
      await control.start();
      operationGateRef.current.assertCurrent(operation.generation);
      dispatch({ type: 'START_ACKNOWLEDGED' });
      recordLog({ type: 'state', phoneMs: Date.now(), state: 'RUNNING' });
    } catch (error) {
      if (!operationGateRef.current.isCurrent(operation.generation)) return;
      const message = error instanceof Error ? error.message : String(error);
      const cause = /ACK timeout/i.test(message)
        ? 'Start acknowledgement timeout; the rover was stopped.'
        : message;
      void handleMissionFault(cause);
    } finally {
      finishMissionOperation(operation.generation, operation.settle);
    }
  }

  async function onStop() {
    const pendingOperation = cancelActiveMissionOperation();
    setBusy(true);
    setOperationError(null);
    try {
      if (controlRef.current) await controlRef.current.stop();
      else if (setup.connectionStatus === 'connected') await ble.emergencyStop();
      else {
        await ble.disconnect();
        dispatch({ type: 'CONNECTION_CHANGED', status: 'disconnected', compatible: false });
      }
      recordLog({ type: 'state', phoneMs: Date.now(), state: 'STOPPED' });
    } catch (error) {
      setOperationError(`Stop acknowledgement failed: ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      await pendingOperation;
      activeOperationSettledRef.current = null;
      dispatch({ type: 'STOP' });
      await releaseMissionResources(true);
      setTelemetry(null);
      telemetryRef.current = null;
      setPath([]);
      setBusy(false);
    }
  }

  async function onExport(log: MissionLogFile, format: 'jsonl' | 'csv') {
    setOperationError(null);
    try {
      if (format === 'jsonl') {
        await exportMissionLog(log.uri);
        return;
      }
      const source = new File(log.uri);
      const csv = missionJsonlToCsv(await source.text());
      const output = new File(Paths.document, 'SafeSpread', 'exports', log.name.replace(/\.jsonl$/i, '.csv'));
      output.create({ intermediates: true, overwrite: true });
      output.write(csv);
      await exportMissionLog(output.uri);
    } catch (error) {
      setOperationError(error instanceof Error ? error.message : String(error));
    }
  }

  const trackingDetail = vio.trackingState === 'limited'
    ? `tracking limited: ${vio.trackingReason}`
    : `tracking ${vio.trackingState}`;
  const showMission = Boolean(setup.rectangle && ['running', 'complete', 'fault'].includes(setup.phase));

  if (showMission && setup.rectangle) {
    return (
      <RunningMission
        phase={setup.phase}
        pose={rectanglePose}
        trackingDetail={trackingDetail}
        telemetry={telemetry}
        rectangle={setup.rectangle}
        path={path}
        fault={setup.fault ?? operationError}
        logName={logName}
        faultDumpReady={Boolean(faultDumpUri)}
        faultDumpError={faultDumpError}
        busy={busy}
        onStop={onStop}
        onDownloadFault={async () => {
          if (faultDumpUri) await exportMissionLog(faultDumpUri);
        }}
      />
    );
  }

  return (
    <SetupWizard
      state={setup}
      roverPose={vio.pose}
      cameraPose={vio.validatedPose?.camera ?? null}
      trackingDetail={trackingDetail}
      readinessReason={vio.readiness.reason}
      calibration={calibration}
      recentLogs={recentLogs}
      busy={busy}
      calibrationProgress={calibrationProgress}
      operationError={operationError}
      dispatch={dispatch}
      onSaveCalibration={onSaveCalibration}
      onRunCalibration={onRunCalibration}
      onSelfTest={onSelfTest}
      onArm={onArm}
      onStart={onStart}
      onStop={onStop}
      onReconnect={connectToRover}
      onExport={onExport}
    />
  );
}
