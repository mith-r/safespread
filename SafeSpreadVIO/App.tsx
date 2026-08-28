import React, { useEffect, useReducer, useRef, useState } from 'react';
import { File, Paths } from 'expo-file-system';
import { useKeepAwake } from 'expo-keep-awake';
import { setAudioModeAsync, useAudioPlayer } from 'expo-audio';
import { SafeSpreadBLE } from './src/ble';
import { LatestPoseSender } from './src/latestPoseSender';
import { MissionCommandRefused, MissionControl } from './src/missionControl';
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
import { wrappedHeadingDelta } from './src/poseMath';
import { ValidatedPose } from './src/posePipeline';
import { buildPoseV2, FaultSampleV2, RoutePlanV2, TelemetryV2 } from './src/protocolV2';
import { worldToRectangle } from './src/rectangle';
import RunningMission from './src/RunningMission';
import SetupWizard from './src/SetupWizard';
import {
  assembleFaultPackets,
  initialSetupState,
  isAuthoritativeLogReady,
  MissionOperationGate,
  setupReducer,
} from './src/setupMachine';
import { DEFAULT_MOUNT_CALIBRATION, useVIOPose } from './src/useVIOPose';

const APP_VERSION = '1.0.0';
const FIRMWARE_VERSION = 'protocol-v2-acknowledged';
const START_POSITION_TOLERANCE_FT = 0.75;
const START_HEADING_TOLERANCE_DEG = 5;
// How far ARKit may move the world before it is worth telling the operator, and
// before continuing to spray is worse than stopping. The 2026-08-28 mission took
// a single 1.11 ft shift mid-headland; every lane after it was a foot off the
// ground it was supposed to cover, and no number on board said so.
const RELOCALIZATION_WARN_FT = 0.3;
const RELOCALIZATION_FAULT_FT = 1.0;
const ble = new SafeSpreadBLE();

function faultName(code: number): string {
  const names = [
    'none', 'BLE disconnected', 'pose timeout', 'invalid pose', 'pose jump',
    'PWM controller', 'I2C controller', 'no-motion stall', 'wrong direction',
    'tracking degraded', 'route invalid', 'calibration mismatch', 'headland insufficient',
    'not on the resumed pass',
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
  // Every refused pose is a pose the rover never sees. Record them -- the first
  // of each cause immediately, then once a second per cause so a long tracking
  // outage annotates the log instead of filling it.
  const rejectLoggedAtRef = useRef<Partial<Record<string, number>>>({});
  const vio = useVIOPose(DEFAULT_MOUNT_CALIBRATION, {
    onPose: (pose) => onValidatedPose(pose),
    onReject: (rejection) => {
      const now = Date.now();
      const previous = rejectLoggedAtRef.current[rejection.reason];
      if (previous !== undefined && now - previous < 1000) return;
      rejectLoggedAtRef.current[rejection.reason] = now;
      recordLog({
        type: 'pose_reject',
        phoneMs: now,
        sequence: rejection.sequence,
        reason: rejection.reason,
        frameTimestampMs: rejection.frameTimestampMs,
        trackingState: rejection.trackingState,
        trackingReason: rejection.trackingReason,
        reasonCount: rejection.count,
        rejectedTotal: rejection.total,
      });
    },
  });
  const trackingOkRef = useRef(vio.trackingOk);
  trackingOkRef.current = vio.trackingOk;
  const [busy, setBusy] = useState(false);
  const [operationError, setOperationError] = useState<string | null>(null);
  const [diagnosticProgress, setDiagnosticProgress] = useState('');
  const [turnRadii, setTurnRadii] = useState<{ leftFt: number; rightFt: number; measured: boolean } | null>(null);
  const [routePlan, setRoutePlan] = useState<RoutePlanV2 | null>(null);
  const [recentLogs, setRecentLogs] = useState<MissionLogFile[]>([]);
  const [telemetry, setTelemetry] = useState<TelemetryV2 | null>(null);
  const telemetryRef = useRef<TelemetryV2 | null>(null);
  // Which sprayed pass the rover is on, counted from the spray transitions in
  // its own telemetry. The rover reports route points, not passes, and the pass
  // is what an operator resumes from.
  const [passesStarted, setPassesStarted] = useState(0);
  const passesStartedRef = useRef(0);
  const sprayingRef = useRef(false);
  const [path, setPath] = useState<PathPoint[]>([]);
  const [logName, setLogName] = useState<string | null>(null);
  const [faultDumpUri, setFaultDumpUri] = useState<string | null>(null);
  const [relocalizationShiftFt, setRelocalizationShiftFt] = useState(0);
  const relocalizationShiftFtRef = useRef(0);

  const controlRef = useRef<MissionControl | null>(null);
  const senderRef = useRef<LatestPoseSender | null>(null);
  const loggerRef = useRef<MissionLogger | null>(null);
  const epochRef = useRef<number | null>(null);
  const geometryWireRef = useRef<typeof DEFAULT_MOUNT_CALIBRATION | null>(null);
  const poseStreamingRef = useRef(false);
  const rectangleConfiguredRef = useRef(false);
  const sessionPreparedRef = useRef(false);
  const resourceWetRef = useRef<boolean | null>(null);
  const resourceHasRectangleRef = useRef(false);
  const lastPoseOfferedSequenceRef = useRef(0);
  const poseBySequenceRef = useRef(new Map<number, MissionRecord>());
  const faultPacketsRef = useRef<Uint8Array[]>([]);
  const faultHandledRef = useRef(false);
  const bootFaultSummaryRef = useRef<string | null>(null);
  const radiusWaiterRef = useRef<{
    resolve(message: string): void;
    reject(error: Error): void;
    timer: ReturnType<typeof setTimeout>;
  } | null>(null);
  const operationGateRef = useRef(new MissionOperationGate());
  const activeOperationSettledRef = useRef<Promise<void> | null>(null);
  const selfTestWaiterRef = useRef<{
    resolve(message: string): void;
    reject(error: Error): void;
    timer: ReturnType<typeof setTimeout>;
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
    const radiusWaiter = radiusWaiterRef.current;
    if (radiusWaiter) {
      clearTimeout(radiusWaiter.timer);
      radiusWaiterRef.current = null;
      radiusWaiter.resolve('Radius measurement cancelled by Stop');
    }
    const selfTestWaiter = selfTestWaiterRef.current;
    if (selfTestWaiter) {
      clearTimeout(selfTestWaiter.timer);
      selfTestWaiterRef.current = null;
      selfTestWaiter.resolve('Self-test cancelled by Stop');
    }
    return pending;
  }

  function recordLog(record: MissionRecord) {
    const logger = loggerRef.current;
    if (!logger) return;
    void logger.record(record).catch(() => {
      if (loggerRef.current === logger) loggerRef.current = null;
      setLogName(null);
      dispatch({ type: 'SET_LOGGING_READY', ready: false });
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
    geometryWireRef.current = null;
    rectangleConfiguredRef.current = false;
    sessionPreparedRef.current = false;
    resourceWetRef.current = null;
    resourceHasRectangleRef.current = false;
    poseBySequenceRef.current.clear();
    if (closeLog) await closeLogger().catch((error) => {
      setOperationError(error instanceof Error ? error.message : String(error));
    });
  }

  // `requireRectangle` is false for diagnostics. A self-test needs a connection
  // and an epoch, nothing more -- the rectangle only ever fed the mission log
  // header. Gating it behind setup made the one command that reports why the
  // rover will not move unreachable on a rover that will not move.
  async function ensureMissionResources({ requireRectangle = true } = {}) {
    if (requireRectangle && !setup.rectangle) {
      throw new Error('Define the rectangle before starting a mission.');
    }
    // Resources built for a diagnostic have no rectangle in their log header,
    // so a real mission must rebuild rather than inherit a headerless log.
    if (controlRef.current && senderRef.current && epochRef.current !== null &&
        resourceWetRef.current === setup.wet &&
        (!requireRectangle || resourceHasRectangleRef.current)) {
      return { control: controlRef.current, epoch: epochRef.current };
    }
    if (controlRef.current || senderRef.current || loggerRef.current) {
      await controlRef.current?.stop().catch(() => {});
      await releaseMissionResources(true);
      dispatch({ type: 'SET_LOGGING_READY', ready: false });
    }
    const epoch = await nextMissionEpoch();
    const wire = DEFAULT_MOUNT_CALIBRATION;
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
          surface: 'other',
          condition: setup.wet ? 'wet' : 'dry',
        },
        rectangle: setup.rectangle ? {
          source: setup.rectangle.source,
          mFt: setup.rectangle.mFt,
          nFt: setup.rectangle.nFt,
          side: setup.rectangle.side,
        } : undefined,
      }, fileLog.sink);
      loggerRef.current = logger;
      setLogName(fileLog.uri.split('/').pop() ?? missionId);
      dispatch({ type: 'SET_LOGGING_READY', ready: true });
    } catch {
      loggerRef.current = null;
      setLogName(null);
      dispatch({ type: 'SET_LOGGING_READY', ready: false });
    }
    const control = new MissionControl(ble, epoch, () => trackingOkRef.current, {
      dryMode: !setup.wet,
    });
    const sender = new LatestPoseSender(ble, (error) => {
      void handleMissionFault(`pose transport failed: ${error.message}`);
    });
    controlRef.current = control;
    senderRef.current = sender;
    epochRef.current = epoch;
    geometryWireRef.current = wire;
    poseStreamingRef.current = true;
    // A mission's first pose must use the same frame as every later pose.
    // Otherwise Configure changes world coordinates into rectangle coordinates
    // and the rover correctly interprets the discontinuity as a pose jump.
    rectangleConfiguredRef.current = !!setup.rectangle;
    sessionPreparedRef.current = false;
    resourceWetRef.current = setup.wet;
    resourceHasRectangleRef.current = !!setup.rectangle;
    faultHandledRef.current = false;
    faultPacketsRef.current = [];
    setFaultDumpUri(null);
    setPath([]);
    relocalizationShiftFtRef.current = 0;
    setRelocalizationShiftFt(0);
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
        recordLog({
          type: 'fault_buffer_error',
          phoneMs: Date.now(),
          fault: error instanceof Error ? error.message : String(error),
        });
      }
    }
    poseStreamingRef.current = false;
    await senderRef.current?.stop().catch(() => {});
    await closeLogger().catch((error) => {
      setOperationError(error instanceof Error ? error.message : String(error));
    });
    setBusy(false);
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
    refreshLogs();
  }, []);

  useEffect(() => {
    const removeTelemetry = ble.subscribeTelemetry((next) => {
      telemetryRef.current = next;
      setTelemetry(next);
      const spraying = Boolean(next.flags & 1);
      if (spraying && !sprayingRef.current) {
        passesStartedRef.current += 1;
        setPassesStarted(passesStartedRef.current);
      }
      sprayingRef.current = spraying;
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
      faultPacketsRef.current.push(packet);
    });
    const removeDisconnect = ble.subscribeDisconnect(() => {
      controlRef.current?.notifyDisconnect();
      const phase = setupRef.current.phase;
      if (['arming', 'armed', 'starting', 'running'].includes(phase)) {
        void handleMissionFault('BLE disconnected');
      }
    });
    void ble.connect(
      (status) => dispatch({
        type: 'CONNECTION_CHANGED',
        status,
        compatible: status === 'connected',
      }),
      (line) => {
        if (line.startsWith('[BOOT FAULT]') || line.startsWith('[FAULT SUMMARY]')) {
          bootFaultSummaryRef.current = line;
        }
        if (line.startsWith('[RADIUS')) {
          setDiagnosticProgress(line);
          const measured = line.match(
            /^\[RADIUS (?:STATUS|PASS)\] left=([0-9.]+) right=([0-9.]+) ft(?: source=(measured|built-in))?/,
          );
          if (measured) {
            setTurnRadii({
              leftFt: Number.parseFloat(measured[1]),
              rightFt: Number.parseFloat(measured[2]),
              measured: line.startsWith('[RADIUS PASS]') || measured[3] === 'measured',
            });
          }
          const waiter = radiusWaiterRef.current;
          if (waiter && (line.startsWith('[RADIUS PASS]') || line.startsWith('[RADIUS FAIL]'))) {
            clearTimeout(waiter.timer);
            radiusWaiterRef.current = null;
            if (line.startsWith('[RADIUS FAIL]')) waiter.reject(new Error(line));
            else waiter.resolve(line);
          }
        }
        if (line.startsWith('=== SELF TEST')) {
          setDiagnosticProgress(line);
          const waiter = selfTestWaiterRef.current;
          if (waiter && (line.includes('COMPLETE') || line.includes('FAILED') ||
              line.includes('ABORTED') || line.includes('STOPPED'))) {
            clearTimeout(waiter.timer);
            selfTestWaiterRef.current = null;
            if (line.includes('COMPLETE')) waiter.resolve(line);
            else waiter.reject(new Error(line));
          }
        }
        recordLog({ type: 'firmware_log', phoneMs: Date.now(), message: line });
      },
    ).catch((error) => setOperationError(error instanceof Error ? error.message : String(error)));
    return () => {
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
  const atStart = Boolean(rectanglePose &&
    Math.hypot(rectanglePose.x, rectanglePose.y) <= START_POSITION_TOLERANCE_FT &&
    Math.abs(wrappedHeadingDelta(rectanglePose.heading, 0)) <= START_HEADING_TOLERANCE_DEG);

  useEffect(() => {
    dispatch({
      type: 'SET_READINESS',
      trackingNormal: vio.trackingOk,
      poseStable: vio.readiness.ready,
      atStart,
    });
  }, [vio.trackingOk, vio.readiness.ready, atStart]);

  /** Every accepted pose, on the ARKit frame that produced it. This runs
   *  outside React's render cycle on purpose: the rover's freshness budget is
   *  250 ms end to end, and waiting for the mission map to redraw before
   *  putting a pose on the wire is how that budget gets spent. */
  function onValidatedPose(validated: ValidatedPose) {
    const sender = senderRef.current;
    const epoch = epochRef.current;
    const wire = geometryWireRef.current;
    const rectangle = setupRef.current.rectangle;

    // ARKit moved the world out from under the rover. The pipeline has already
    // absorbed the jump so the pose stream stays continuous, which is right for
    // control and quietly wrong for coverage: every lane laid down afterwards
    // is offset by that much on real ground while cross-track error still reads
    // clean. Small shifts are worth knowing about; a large one means the rover
    // no longer knows where the field is and must not keep spraying.
    if (validated.relocalizationShiftFt >= RELOCALIZATION_WARN_FT) {
      relocalizationShiftFtRef.current += validated.relocalizationShiftFt;
      setRelocalizationShiftFt(relocalizationShiftFtRef.current);
      recordLog({
        type: 'relocalization',
        phoneMs: Date.now(),
        sequence: validated.sequence,
        shiftFt: validated.relocalizationShiftFt,
        accumulatedShiftFt: relocalizationShiftFtRef.current,
        accumulatedWorldOffsetXFt: validated.accumulatedWorldOffsetXFt,
        accumulatedWorldOffsetYFt: validated.accumulatedWorldOffsetYFt,
        mappingStatus: validated.mappingStatus,
      });
      if (validated.relocalizationShiftFt >= RELOCALIZATION_FAULT_FT &&
          setupRef.current.phase === 'running') {
        void handleMissionFault(
          `ARKit moved the world ${validated.relocalizationShiftFt.toFixed(2)} ft; ` +
          'the rover no longer knows where the rectangle is.');
        return;
      }
    }

    if (!sender || epoch === null || !wire || !poseStreamingRef.current) return;
    const roverPose = rectangleConfiguredRef.current && rectangle
      ? worldToRectangle(validated.rover, rectangle)
      : validated.rover;
    const ageMs = Math.max(0, validated.captureAgeMs +
      (globalThis.performance?.now() ?? Date.now()) - validated.receivedAtMs);
    const yawRate = rectangleConfiguredRef.current && rectangle?.side === 'left'
      ? -validated.yawRateDps
      : validated.yawRateDps;
    try {
      sender.offer(buildPoseV2({
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
        trackingState: validated.trackingState,
        trackingReason: validated.trackingReason,
        mappingStatus: validated.mappingStatus,
        senderDropped: sender.dropped,
        relocalizationShiftFt: validated.relocalizationShiftFt,
        accumulatedWorldOffsetXFt: validated.accumulatedWorldOffsetXFt,
        accumulatedWorldOffsetYFt: validated.accumulatedWorldOffsetYFt,
        // How fast ARKit is actually delivering, and how hot iOS thinks the
        // phone is. A pose stream that thins out over a long run ends as a
        // rover-side pose timeout, and these are what say why.
        frameIntervalMs: validated.frameIntervalMs,
        thermalState: validated.thermalState,
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
          // Everything laid down after the world moved sits somewhere other
          // than where this says it does. Mark it so the map does not present
          // suspect coverage in the same colour as coverage that can be trusted.
          suspect: relocalizationShiftFtRef.current >= RELOCALIZATION_WARN_FT,
        };
        setPath((previous) => shouldRecord(previous.at(-1), nextPoint)
          ? [...previous.slice(-(MAX_PATH_POINTS - 1)), nextPoint]
          : previous);
      }
    } catch (error) {
      void handleMissionFault(`pose packet failed: ${error instanceof Error ? error.message : String(error)}`);
    }
  }

  function awaitRadiusResult(): Promise<string> {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        radiusWaiterRef.current = null;
        reject(new Error('Radius measurement timed out; press Stop and inspect the rover.'));
      }, 120000);
      radiusWaiterRef.current = { resolve, reject, timer };
    });
  }

  async function onMeasureTurningRadii() {
    const operation = beginMissionOperation();
    setBusy(true);
    setOperationError(null);
    try {
      if (setup.wet) throw new Error('Select Dry diagnostic before measuring turning radii.');
      const { control } = await ensureMissionResources({ requireRectangle: false });
      operationGateRef.current.assertCurrent(operation.generation);
      if (!sessionPreparedRef.current) {
        await control.prepareDiagnostics(DEFAULT_MOUNT_CALIBRATION);
        operationGateRef.current.assertCurrent(operation.generation);
        sessionPreparedRef.current = true;
      }
      const beforeSequence = lastPoseOfferedSequenceRef.current;
      const deadline = Date.now() + 750;
      while (lastPoseOfferedSequenceRef.current <= beforeSequence && Date.now() < deadline) {
        await new Promise((resolve) => setTimeout(resolve, 20));
        operationGateRef.current.assertCurrent(operation.generation);
      }
      const result = awaitRadiusResult();
      try {
        await control.measureTurningRadii();
        operationGateRef.current.assertCurrent(operation.generation);
        setDiagnosticProgress(await result);
        operationGateRef.current.assertCurrent(operation.generation);
      } catch (error) {
        const waiter = radiusWaiterRef.current;
        if (waiter) {
          clearTimeout(waiter.timer);
          radiusWaiterRef.current = null;
        }
        throw error;
      }
    } catch (error) {
      if (operationGateRef.current.isCurrent(operation.generation)) {
        setOperationError(error instanceof Error ? error.message : String(error));
      }
    } finally {
      finishMissionOperation(operation.generation, operation.settle);
    }
  }

  async function onSelfTest() {
    const operation = beginMissionOperation();
    setBusy(true);
    setOperationError(null);
    try {
      if (setup.wet) throw new Error('Select Dry diagnostic before the self-test.');
      const { control } = await ensureMissionResources({ requireRectangle: false });
      operationGateRef.current.assertCurrent(operation.generation);
      if (!sessionPreparedRef.current) {
        await control.prepareDiagnostics(DEFAULT_MOUNT_CALIBRATION);
        operationGateRef.current.assertCurrent(operation.generation);
        sessionPreparedRef.current = true;
      }
      const result = new Promise<string>((resolve, reject) => {
        const timer = setTimeout(() => {
          selfTestWaiterRef.current = null;
          reject(new Error('Self-test result timeout; press Stop and inspect rover logs.'));
        }, 30000);
        selfTestWaiterRef.current = { resolve, reject, timer };
      });
      try {
        await control.selfTest();
        operationGateRef.current.assertCurrent(operation.generation);
        setDiagnosticProgress(await result);
        operationGateRef.current.assertCurrent(operation.generation);
      } catch (error) {
        const waiter = selfTestWaiterRef.current;
        if (waiter) {
          clearTimeout(waiter.timer);
          selfTestWaiterRef.current = null;
        }
        throw error;
      }
    } catch (error) {
      if (operationGateRef.current.isCurrent(operation.generation)) {
        setOperationError(error instanceof Error ? error.message : String(error));
      }
    } finally {
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
        loggingReady: isAuthoritativeLogReady(loggerRef.current),
      }, { type: 'REQUEST_ARM' });
      if (candidate.phase !== 'arming') {
        throw new Error(candidate.validationError ?? 'Readiness checks did not pass.');
      }
      dispatch({ type: 'REQUEST_ARM' });
      if (!setup.rectangle) throw new Error('Rectangle is missing.');
      const wire = DEFAULT_MOUNT_CALIBRATION;
      await control.configure(setup.rectangle, wire, setup.resumePassIndex);
      operationGateRef.current.assertCurrent(operation.generation);
      const planned = control.routePlan;
      if (!planned) throw new Error('The rover did not report its calculated headland.');
      setRoutePlan(planned);
      setTurnRadii((current) => ({
        leftFt: planned.leftRadiusFt,
        rightFt: planned.rightRadiusFt,
        measured: current?.measured ?? false,
      }));
      rectangleConfiguredRef.current = true;
      const currentReadiness = setupRef.current.readiness;
      // A resumed mission starts on a lane in the middle of the rectangle, so
      // the origin check does not apply; the rover refuses the Arm itself if
      // the operator has not put it on that pass.
      const startVerified = currentReadiness.atStart || setup.resumePassIndex > 0;
      if (!trackingOkRef.current || !currentReadiness.poseStable || !startVerified) {
        throw new Error('Readiness changed during Configure; Stop and return to the rectangle start.');
      }
      await control.arm();
      operationGateRef.current.assertCurrent(operation.generation);
      dispatch({ type: 'ARM_ACKNOWLEDGED' });
      recordLog({ type: 'state', phoneMs: Date.now(), state: 'ARMED' });
    } catch (error) {
      if (!operationGateRef.current.isCurrent(operation.generation)) return;
      const message = error instanceof Error ? error.message : String(error);
      // A refused Arm is not a fault: the rover stays configured and the
      // operator moves it onto the pass and tries again.
      if (error instanceof MissionCommandRefused) {
        dispatch({ type: 'ARM_REFUSED', reason: message });
        setOperationError(message);
        recordLog({ type: 'arm_refused', phoneMs: Date.now(), fault: error.faultCode, state: 'CONFIGURED' });
      } else if (/ACK timeout/i.test(message)) {
        dispatch({ type: 'ACK_TIMEOUT', operation: 'Arm' });
        void handleMissionFault(message);
      } else {
        dispatch({ type: 'MISSION_FAULT', cause: message });
        void handleMissionFault(message);
      }
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
      if (/ACK timeout/i.test(message)) dispatch({ type: 'ACK_TIMEOUT', operation: 'Start' });
      else dispatch({ type: 'MISSION_FAULT', cause: message });
      void handleMissionFault(message);
    } finally {
      finishMissionOperation(operation.generation, operation.settle);
    }
  }

  async function onStop() {
    await stopMission(null);
  }

  /** Stop the rover but keep the rectangle, so the remaining passes are laid
   *  down in the same place. Re-defining the rectangle would anchor a new
   *  origin wherever the rover happens to be standing now. */
  async function onResume(passIndex: number) {
    await stopMission(passIndex);
  }

  async function stopMission(resumePassIndex: number | null) {
    const pendingOperation = cancelActiveMissionOperation();
    setBusy(true);
    setOperationError(null);
    try {
      if (controlRef.current) await controlRef.current.stop();
      else if (setup.connectionStatus === 'connected') await ble.emergencyStop();
      recordLog({ type: 'state', phoneMs: Date.now(), state: 'STOPPED' });
    } catch (error) {
      setOperationError(`Stop acknowledgement failed: ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      await pendingOperation;
      activeOperationSettledRef.current = null;
      if (resumePassIndex === null) dispatch({ type: 'STOP' });
      else dispatch({ type: 'RESUME_MISSION', passIndex: resumePassIndex });
      await releaseMissionResources(true);
      setTelemetry(null);
      telemetryRef.current = null;
      setPath([]);
      passesStartedRef.current = 0;
      setPassesStarted(0);
      relocalizationShiftFtRef.current = 0;
      setRelocalizationShiftFt(0);
      setRoutePlan(null);
      sprayingRef.current = false;
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

  const trackingBase = vio.trackingState === 'limited'
    ? `tracking limited: ${vio.trackingReason}`
    : `tracking ${vio.trackingState}`;
  // Say when poses are being dropped and why. A silent pose stream and a
  // rejected pose stream both end as a rover-side pose timeout, and only this
  // line tells the operator which one they have.
  let trackingDetail = vio.rejectSummary.total === 0
    ? trackingBase
    : `${trackingBase} · ${vio.rejectSummary.total} poses dropped (last: ${vio.rejectSummary.lastReason})`;
  // Say it before it becomes a pose timeout. A frame gap approaching the
  // rover's 250 ms freshness budget is the whole failure in advance.
  if (vio.thermalState !== 'nominal' && vio.thermalState !== 'unknown') {
    trackingDetail += ` · phone ${vio.thermalState}`;
  }
  if (vio.worstFrameIntervalMs > 100) {
    trackingDetail += ` · ARKit gap ${vio.worstFrameIntervalMs.toFixed(0)} ms`;
  }
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
        routePlan={routePlan}
        fault={setup.fault ?? operationError}
        logName={logName}
        faultDumpReady={Boolean(faultDumpUri)}
        relocalizationShiftFt={relocalizationShiftFt}
        busy={busy}
        currentPassIndex={setup.resumePassIndex + Math.max(0, passesStarted - 1)}
        onStop={onStop}
        onResume={onResume}
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
      trackingDetail={trackingDetail}
      readinessReason={vio.readiness.reason}
      turnRadii={turnRadii}
      routePlan={routePlan}
      recentLogs={recentLogs}
      busy={busy}
      diagnosticProgress={operationError ?? diagnosticProgress}
      dispatch={dispatch}
      onMeasureTurningRadii={onMeasureTurningRadii}
      onSelfTest={onSelfTest}
      onArm={onArm}
      onStart={onStart}
      onStop={onStop}
      onExport={onExport}
    />
  );
}
