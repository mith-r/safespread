import { useEffect, useMemo, useRef, useState } from 'react';
import ArkitPoseModule from '../modules/arkit-pose/src/ArkitPoseModule';
import {
  isPoseUsable,
  MappingStatus,
  PoseUpdatePayload,
  ThermalState,
  TrackingReason,
  TrackingState,
} from '../modules/arkit-pose/src/ArkitPose.types';
import { DEFAULT_MOUNT_CALIBRATION } from './hardwareGeometry';
import { MountCalibration } from './poseMath';
import {
  PoseDecision,
  PosePipeline,
  PoseReadiness,
  PoseRejectReason,
  ValidatedPose,
} from './posePipeline';

export { DEFAULT_MOUNT_CALIBRATION } from './hardwareGeometry';

// A pose the pipeline refused never reaches the rover, which sees only silence
// and calls it a pose timeout 250 ms later. Report the refusal itself so the
// two are told apart.
export interface PoseRejection {
  reason: PoseRejectReason;
  sequence: number;
  frameTimestampMs: number;
  trackingState: TrackingState;
  trackingReason: TrackingReason;
  count: number;
  total: number;
}

export interface PoseRejectSummary {
  total: number;
  counts: Partial<Record<PoseRejectReason, number>>;
  lastReason: PoseRejectReason | null;
}

export interface VIOHandlers {
  /** Every pose that passed every gate, delivered synchronously on the ARKit
   *  frame that produced it. */
  onPose?: (pose: ValidatedPose) => void;
  onReject?: (rejection: PoseRejection) => void;
}

const EMPTY_REJECT_SUMMARY: PoseRejectSummary = { total: 0, counts: {}, lastReason: null };
const DISPLAY_INTERVAL_MS = 100;

function monotonicNow(): number {
  return globalThis.performance?.now() ?? Date.now();
}

// What the screen shows, refreshed on a timer rather than on every frame.
//
// ARKit delivers 60 frames a second. Publishing each one through React state
// re-rendered the whole app sixty times a second -- and with the mission map on
// screen that means re-walking every point the rover has driven, thousands of
// them by the end of a run. The work grows as the mission goes on, which is
// exactly the shape of the pose-rate decay in the 2026-08-28 log: 60 Hz for the
// first minute, 19 Hz by the end, and a rover-side pose timeout to finish.
//
// Nothing that drives the rover reads this. Poses reach the rover through
// `onPose`, on the frame they arrive; only the operator's display is throttled,
// and no operator can see 60 Hz anyway.
export interface VIOSnapshot {
  pose: ValidatedPose['rover'] | null;
  validatedPose: ValidatedPose | null;
  latestDecision: PoseDecision;
  trackingState: TrackingState;
  trackingReason: TrackingReason;
  mappingStatus: MappingStatus;
  thermalState: ThermalState;
  /** Longest gap between ARKit frames seen in the last display interval. */
  worstFrameIntervalMs: number;
  trackingOk: boolean;
  readiness: PoseReadiness;
  rejectSummary: PoseRejectSummary;
}

const INITIAL_SNAPSHOT: VIOSnapshot = {
  pose: null,
  validatedPose: null,
  latestDecision: { ok: false, reason: 'tracking' },
  trackingState: 'notAvailable',
  trackingReason: 'unknown',
  mappingStatus: 'notAvailable',
  thermalState: 'unknown',
  worstFrameIntervalMs: 0,
  trackingOk: false,
  readiness: { ready: false, reason: 'noSamples' },
  rejectSummary: EMPTY_REJECT_SUMMARY,
};

export function useVIOPose(
  calibration: MountCalibration = DEFAULT_MOUNT_CALIBRATION,
  handlers: VIOHandlers = {},
) {
  const pipeline = useMemo(
    () => new PosePipeline(calibration),
    [
      calibration.id,
      calibration.schemaVersion,
      calibration.cameraForwardFt,
      calibration.cameraRightFt,
      calibration.cameraYawDeg,
      calibration.sprayForwardFt,
      calibration.sprayRightFt,
    ],
  );
  const [snapshot, setSnapshot] = useState<VIOSnapshot>(INITIAL_SNAPSHOT);

  const latestPoseRef = useRef<ValidatedPose | null>(null);
  const latestEventRef = useRef<PoseUpdatePayload | null>(null);
  const latestDecisionRef = useRef<PoseDecision>(INITIAL_SNAPSHOT.latestDecision);
  const worstFrameIntervalRef = useRef(0);
  const rejectCountsRef = useRef<Partial<Record<PoseRejectReason, number>>>({});
  const rejectTotalRef = useRef(0);
  const lastRejectReasonRef = useRef<PoseRejectReason | null>(null);
  const handlersRef = useRef(handlers);
  handlersRef.current = handlers;

  useEffect(() => {
    ArkitPoseModule.start();
    const subscription = ArkitPoseModule.addListener('onPoseUpdate', (event) => {
      const receivedAtMs = monotonicNow();
      const decision = pipeline.ingest(event, receivedAtMs);
      latestEventRef.current = event;
      latestDecisionRef.current = decision;
      if (event.frameIntervalMs > worstFrameIntervalRef.current) {
        worstFrameIntervalRef.current = event.frameIntervalMs;
      }
      if (decision.ok) {
        latestPoseRef.current = decision.pose;
        // Straight to the rover. Anything that waited for a re-render here
        // would be waiting behind the map redraw.
        handlersRef.current.onPose?.(decision.pose);
        return;
      }
      const count = (rejectCountsRef.current[decision.reason] ?? 0) + 1;
      rejectCountsRef.current = { ...rejectCountsRef.current, [decision.reason]: count };
      rejectTotalRef.current += 1;
      lastRejectReasonRef.current = decision.reason;
      handlersRef.current.onReject?.({
        reason: decision.reason,
        sequence: event.sequence,
        frameTimestampMs: event.frameTimestampMs,
        trackingState: event.trackingState,
        trackingReason: event.trackingReason,
        count,
        total: rejectTotalRef.current,
      });
    });
    return () => {
      subscription.remove();
      ArkitPoseModule.stop();
      pipeline.reset();
      latestPoseRef.current = null;
      latestEventRef.current = null;
      latestDecisionRef.current = INITIAL_SNAPSHOT.latestDecision;
      worstFrameIntervalRef.current = 0;
      rejectCountsRef.current = {};
      rejectTotalRef.current = 0;
      lastRejectReasonRef.current = null;
      setSnapshot(INITIAL_SNAPSHOT);
    };
  }, [pipeline]);

  useEffect(() => {
    const timer = setInterval(() => {
      const nowMs = monotonicNow();
      const latest = latestPoseRef.current;
      const event = latestEventRef.current;
      const decision = latestDecisionRef.current;
      const fresh = Boolean(latest && latest.captureAgeMs + nowMs - latest.receivedAtMs <= 250);
      const trackingState = event?.trackingState ?? 'notAvailable';
      setSnapshot({
        pose: latest?.rover ?? null,
        validatedPose: latest,
        latestDecision: decision,
        trackingState,
        trackingReason: event?.trackingReason ?? 'unknown',
        mappingStatus: event?.mappingStatus ?? 'notAvailable',
        thermalState: event?.thermalState ?? 'unknown',
        worstFrameIntervalMs: worstFrameIntervalRef.current,
        trackingOk: fresh && isPoseUsable(trackingState) && decision.ok,
        readiness: pipeline.readiness(nowMs),
        rejectSummary: {
          total: rejectTotalRef.current,
          counts: rejectCountsRef.current,
          lastReason: lastRejectReasonRef.current,
        },
      });
      worstFrameIntervalRef.current = 0;
    }, DISPLAY_INTERVAL_MS);
    return () => clearInterval(timer);
  }, [pipeline]);

  return snapshot;
}
