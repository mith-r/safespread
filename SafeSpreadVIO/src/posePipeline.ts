import { PoseUpdatePayload } from '../modules/arkit-pose/src/ArkitPose.types';
import {
  cameraToRover,
  MountCalibration,
  normalizeHeading,
  Pose,
  roverToSprayBar,
  wrappedHeadingDelta,
} from './poseMath';

export type { MountCalibration } from './poseMath';

export type PoseRejectReason =
  | 'tracking'
  | 'calibration'
  | 'nonFinite'
  | 'sequence'
  | 'timestamp'
  | 'age'
  | 'speed'
  | 'acceleration'
  | 'yawRate'
  | 'innovation';

export interface ValidatedPose {
  sequence: number;
  frameTimestampMs: number;
  receivedAtMs: number;
  captureAgeMs: number;
  camera: Pose;
  rover: Pose;
  sprayBar: Pose;
  speedFps: number;
  yawRateDps: number;
  courseDeg: number | null;
}

export type PoseDecision =
  | { ok: true; pose: ValidatedPose }
  | { ok: false; reason: PoseRejectReason };

export interface PoseReadiness {
  ready: boolean;
  reason: 'ready' | 'noSamples' | 'samples' | 'duration' | 'stale' | 'positionSpread' | 'headingSpread';
}

interface MotionSample {
  frameTimestampMs: number;
  rover: Pose;
}

const MAX_CAPTURE_AGE_MS = 250;
const MAX_SPEED_FPS = 8;
const MAX_ACCELERATION_FPS2 = 15;
const MAX_YAW_RATE_DPS = 180;
const INNOVATION_ALLOWANCE_FT = 0.75;
// The position and innovation gates below already carry an additive floor; the
// acceleration gate needs one too. Standing still, the velocity estimate is
// noise, and dividing noise by a 16 ms frame interval clears 15 ft/s^2 without
// the rover having moved at all.
const SPEED_NOISE_FPS = 0.5;
const MIN_FRAME_INTERVAL_S = 0.005;
// Velocity from two frames 16 ms apart is mostly position noise amplified 60x.
// Only measure across a baseline long enough to contain real motion, and call
// anything under the deadband a standstill.
const MOTION_SAMPLE_LIMIT = 10;
const MIN_VELOCITY_BASELINE_S = 0.05;
const SPEED_DEADBAND_FPS = 0.1;
// A frame the motion gates refuse is a hiccup, not a loss of tracking, so it
// must not throw away the readiness history the operator is waiting on.
const HARD_REJECTS: ReadonlySet<PoseRejectReason> = new Set<PoseRejectReason>([
  'tracking', 'calibration', 'nonFinite', 'sequence', 'timestamp',
]);
const READY_DURATION_MS = 2000;
const READY_MIN_SAMPLES = 30;
const READY_MAX_AGE_MS = 150;
const READY_POSITION_RADIUS_FT = 0.1;
const READY_HEADING_DEVIATION_DEG = 1;

function median(values: number[]): number {
  if (values.length === 0) return 0;
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2;
}

function finitePose(pose: Pose): boolean {
  return Number.isFinite(pose.x) && Number.isFinite(pose.y) &&
    Number.isFinite(pose.heading) && pose.heading >= 0 && pose.heading < 360;
}

function validCalibration(calibration: MountCalibration): boolean {
  return Number.isInteger(calibration.id) && calibration.id >= 0 && calibration.id <= 0xffff &&
    Number.isInteger(calibration.schemaVersion) && calibration.schemaVersion > 0 &&
    [
      calibration.cameraForwardFt,
      calibration.cameraRightFt,
      calibration.cameraYawDeg,
      calibration.sprayForwardFt,
      calibration.sprayRightFt,
    ].every(Number.isFinite);
}

function estimateMotion(samples: MotionSample[]) {
  if (samples.length < 2) return { vx: 0, vy: 0, yawRateDps: 0 };
  const vx: number[] = [];
  const vy: number[] = [];
  const yaw: number[] = [];
  const add = (from: MotionSample, to: MotionSample) => {
    const seconds = (to.frameTimestampMs - from.frameTimestampMs) / 1000;
    if (seconds <= 0) return;
    vx.push((to.rover.x - from.rover.x) / seconds);
    vy.push((to.rover.y - from.rover.y) / seconds);
    yaw.push(wrappedHeadingDelta(to.rover.heading, from.rover.heading) / seconds);
  };
  for (let from = 0; from < samples.length - 1; from += 1) {
    for (let to = from + 1; to < samples.length; to += 1) {
      const seconds = (samples[to].frameTimestampMs - samples[from].frameTimestampMs) / 1000;
      if (seconds < MIN_VELOCITY_BASELINE_S) continue;
      add(samples[from], samples[to]);
    }
  }
  // For the first frames after a reset nothing spans the baseline yet. Report a
  // standstill rather than a velocity made entirely of position noise; the
  // arming window demands two still seconds before any of this drives a wheel.
  return { vx: median(vx), vy: median(vy), yawRateDps: median(yaw) };
}

export class PosePipeline {
  private readonly calibration: MountCalibration;
  private motionSamples: MotionSample[] = [];
  private readySamples: ValidatedPose[] = [];
  private lastSequence: number | null = null;
  private lastFrameTimestampMs: number | null = null;
  private lastReceivedAtMs: number | null = null;
  private lastMotionMagnitudeFps = 0;
  private lastAccepted: ValidatedPose | null = null;

  constructor(calibration: MountCalibration) {
    this.calibration = { ...calibration };
  }

  reset(): void {
    this.motionSamples = [];
    this.readySamples = [];
    this.lastSequence = null;
    this.lastFrameTimestampMs = null;
    this.lastReceivedAtMs = null;
    this.lastMotionMagnitudeFps = 0;
    this.lastAccepted = null;
  }

  ingest(event: PoseUpdatePayload, receivedAtMs: number): PoseDecision {
    if (event.kind !== 'pose' || event.trackingState !== 'normal') return this.reject('tracking');
    if (!validCalibration(this.calibration)) return this.reject('calibration');
    if (![event.frameTimestampMs, event.emittedTimestampMs, receivedAtMs].every(Number.isFinite) ||
        !finitePose(event)) return this.reject('nonFinite');
    if (!Number.isInteger(event.sequence) || event.sequence < 0 ||
        (this.lastSequence !== null && event.sequence <= this.lastSequence)) {
      return this.reject('sequence');
    }
    if (event.emittedTimestampMs < event.frameTimestampMs ||
        (this.lastFrameTimestampMs !== null && event.frameTimestampMs <= this.lastFrameTimestampMs) ||
        (this.lastReceivedAtMs !== null && receivedAtMs < this.lastReceivedAtMs)) {
      return this.reject('timestamp');
    }

    this.lastSequence = event.sequence;
    this.lastFrameTimestampMs = event.frameTimestampMs;
    this.lastReceivedAtMs = receivedAtMs;

    const captureAgeMs = event.emittedTimestampMs - event.frameTimestampMs;
    if (captureAgeMs > MAX_CAPTURE_AGE_MS) return this.reject('age');

    const camera = { x: event.x, y: event.y, heading: normalizeHeading(event.heading) };
    const rover = cameraToRover(camera, this.calibration);
    const sprayBar = roverToSprayBar(rover, this.calibration);
    const candidates = [...this.motionSamples, { frameTimestampMs: event.frameTimestampMs, rover }]
      .slice(-MOTION_SAMPLE_LIMIT);
    const motion = estimateMotion(candidates);
    const rawMagnitudeFps = Math.hypot(motion.vx, motion.vy);
    const standstill = rawMagnitudeFps < SPEED_DEADBAND_FPS;
    const motionMagnitudeFps = standstill ? 0 : rawMagnitudeFps;
    const velocityX = standstill ? 0 : motion.vx;
    const velocityY = standstill ? 0 : motion.vy;
    if (motionMagnitudeFps > MAX_SPEED_FPS) return this.reject('speed');
    if (Math.abs(motion.yawRateDps) > MAX_YAW_RATE_DPS) return this.reject('yawRate');

    if (this.lastAccepted) {
      const seconds = Math.max(
        (event.frameTimestampMs - this.lastAccepted.frameTimestampMs) / 1000,
        MIN_FRAME_INTERVAL_S,
      );
      const speedStep = Math.abs(motionMagnitudeFps - this.lastMotionMagnitudeFps);
      if (speedStep > MAX_ACCELERATION_FPS2 * seconds + SPEED_NOISE_FPS) {
        return this.reject('acceleration');
      }

      const displacement = Math.hypot(rover.x - this.lastAccepted.rover.x, rover.y - this.lastAccepted.rover.y);
      const allowed = INNOVATION_ALLOWANCE_FT + this.lastMotionMagnitudeFps * seconds;
      if (displacement > allowed) return this.reject('innovation');
    }

    const forwardX = Math.sin((rover.heading * Math.PI) / 180);
    const forwardY = Math.cos((rover.heading * Math.PI) / 180);
    const speedFps = velocityX * forwardX + velocityY * forwardY;
    const courseDeg = standstill
      ? null
      : normalizeHeading(Math.atan2(velocityX, velocityY) * 180 / Math.PI);
    const pose: ValidatedPose = {
      sequence: event.sequence,
      frameTimestampMs: event.frameTimestampMs,
      receivedAtMs,
      captureAgeMs,
      camera,
      rover,
      sprayBar,
      speedFps,
      yawRateDps: motion.yawRateDps,
      courseDeg,
    };

    this.motionSamples = candidates;
    this.lastMotionMagnitudeFps = motionMagnitudeFps;
    this.lastAccepted = pose;
    this.readySamples.push(pose);
    const cutoff = receivedAtMs - (READY_DURATION_MS + 100);
    this.readySamples = this.readySamples.filter((sample) => sample.receivedAtMs >= cutoff);
    return { ok: true, pose };
  }

  readiness(nowMs: number): PoseReadiness {
    if (this.readySamples.length === 0) return { ready: false, reason: 'noSamples' };
    if (this.readySamples.length < READY_MIN_SAMPLES) return { ready: false, reason: 'samples' };
    const first = this.readySamples[0];
    const last = this.readySamples[this.readySamples.length - 1];
    if (last.receivedAtMs - first.receivedAtMs < READY_DURATION_MS) {
      return { ready: false, reason: 'duration' };
    }
    if (last.captureAgeMs + nowMs - last.receivedAtMs > READY_MAX_AGE_MS) {
      return { ready: false, reason: 'stale' };
    }

    const centerX = this.readySamples.reduce((sum, sample) => sum + sample.rover.x, 0) / this.readySamples.length;
    const centerY = this.readySamples.reduce((sum, sample) => sum + sample.rover.y, 0) / this.readySamples.length;
    const maxRadius = Math.max(...this.readySamples.map((sample) =>
      Math.hypot(sample.rover.x - centerX, sample.rover.y - centerY)));
    if (maxRadius > READY_POSITION_RADIUS_FT) return { ready: false, reason: 'positionSpread' };

    const sin = this.readySamples.reduce((sum, sample) => sum + Math.sin(sample.rover.heading * Math.PI / 180), 0);
    const cos = this.readySamples.reduce((sum, sample) => sum + Math.cos(sample.rover.heading * Math.PI / 180), 0);
    const meanHeading = normalizeHeading(Math.atan2(sin, cos) * 180 / Math.PI);
    const maxHeading = Math.max(...this.readySamples.map((sample) =>
      Math.abs(wrappedHeadingDelta(sample.rover.heading, meanHeading))));
    if (maxHeading > READY_HEADING_DEVIATION_DEG) return { ready: false, reason: 'headingSpread' };
    return { ready: true, reason: 'ready' };
  }

  private reject(reason: PoseRejectReason): PoseDecision {
    if (HARD_REJECTS.has(reason)) this.readySamples = [];
    return { ok: false, reason };
  }
}
