import { useEffect, useMemo, useRef, useState } from 'react';
import ArkitPoseModule from '../modules/arkit-pose/src/ArkitPoseModule';
import {
  isPoseUsable,
  MappingStatus,
  PoseUpdatePayload,
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

const EMPTY_REJECT_SUMMARY: PoseRejectSummary = { total: 0, counts: {}, lastReason: null };

function monotonicNow(): number {
  return globalThis.performance?.now() ?? Date.now();
}

export function useVIOPose(
  calibration: MountCalibration = DEFAULT_MOUNT_CALIBRATION,
  onReject?: (rejection: PoseRejection) => void,
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
  const [rawEvent, setRawEvent] = useState<PoseUpdatePayload | null>(null);
  const [validatedPose, setValidatedPose] = useState<ValidatedPose | null>(null);
  const [latestDecision, setLatestDecision] = useState<PoseDecision>({
    ok: false,
    reason: 'tracking',
  });
  const [trackingState, setTrackingState] = useState<TrackingState>('notAvailable');
  const [trackingReason, setTrackingReason] = useState<TrackingReason>('unknown');
  const [mappingStatus, setMappingStatus] = useState<MappingStatus>('notAvailable');
  const [readiness, setReadiness] = useState<PoseReadiness>({ ready: false, reason: 'noSamples' });
  const [fresh, setFresh] = useState(false);
  const [rejectSummary, setRejectSummary] = useState<PoseRejectSummary>(EMPTY_REJECT_SUMMARY);
  const latestPoseRef = useRef<ValidatedPose | null>(null);
  const rejectCountsRef = useRef<Partial<Record<PoseRejectReason, number>>>({});
  const rejectTotalRef = useRef(0);
  const onRejectRef = useRef(onReject);
  onRejectRef.current = onReject;

  useEffect(() => {
    ArkitPoseModule.start();
    const subscription = ArkitPoseModule.addListener('onPoseUpdate', (event) => {
      const receivedAtMs = monotonicNow();
      const decision = pipeline.ingest(event, receivedAtMs);
      setRawEvent(event);
      setTrackingState(event.trackingState);
      setTrackingReason(event.trackingReason);
      setMappingStatus(event.mappingStatus);
      setLatestDecision(decision);
      if (decision.ok) {
        latestPoseRef.current = decision.pose;
        setValidatedPose(decision.pose);
        setFresh(true);
      } else {
        setFresh(false);
        const count = (rejectCountsRef.current[decision.reason] ?? 0) + 1;
        rejectCountsRef.current = { ...rejectCountsRef.current, [decision.reason]: count };
        rejectTotalRef.current += 1;
        setRejectSummary({
          total: rejectTotalRef.current,
          counts: rejectCountsRef.current,
          lastReason: decision.reason,
        });
        onRejectRef.current?.({
          reason: decision.reason,
          sequence: event.sequence,
          frameTimestampMs: event.frameTimestampMs,
          trackingState: event.trackingState,
          trackingReason: event.trackingReason,
          count,
          total: rejectTotalRef.current,
        });
      }
      setReadiness(pipeline.readiness(receivedAtMs));
    });
    return () => {
      subscription.remove();
      ArkitPoseModule.stop();
      pipeline.reset();
      latestPoseRef.current = null;
      rejectCountsRef.current = {};
      rejectTotalRef.current = 0;
      setRejectSummary(EMPTY_REJECT_SUMMARY);
    };
  }, [pipeline]);

  useEffect(() => {
    const timer = setInterval(() => {
      const nowMs = monotonicNow();
      const latest = latestPoseRef.current;
      setFresh(Boolean(latest && latest.captureAgeMs + nowMs - latest.receivedAtMs <= 250));
      setReadiness(pipeline.readiness(nowMs));
    }, 50);
    return () => clearInterval(timer);
  }, [pipeline]);

  return {
    pose: validatedPose?.rover ?? null,
    validatedPose,
    latestDecision,
    rawEvent,
    trackingState,
    trackingReason,
    mappingStatus,
    trackingOk: fresh && isPoseUsable(trackingState) && latestDecision.ok,
    readiness,
    rejectSummary,
  };
}
