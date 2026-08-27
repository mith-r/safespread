import { MountCalibration, normalizeHeading, Pose, wrappedHeadingDelta } from './poseMath';

export type PavementSurface = 'asphalt' | 'concrete' | 'pavers' | 'other';
export type PavementCondition = 'dry' | 'wet';
export type YawCalibrationSample = Pose;

export interface CalibrationInput {
  schemaVersion: 1;
  hardwareTag: string;
  createdAtIso: string;
  cameraForwardFt: number;
  cameraRightFt: number;
  cameraYawDeg: number;
  sprayForwardFt: number;
  sprayRightFt: number;
  operatingLoadLb: number;
  surface: PavementSurface;
  condition: PavementCondition;
}

export interface CalibrationRecord extends MountCalibration {
  schemaVersion: 1;
  hardwareTag: string;
  createdAtIso: string;
  surface: PavementSurface;
  condition: PavementCondition;
  operatingLoadLb: number;
  /** App-observed proof that the firmware persisted the complete steering,
   * speed, and reverse calibration for this exact ID. Older records omit it
   * and remain safe for dry diagnostics only. */
  motionCalibrationVerifiedAtIso?: string;
}

export type CalibrationDraft = Pick<CalibrationInput,
  'cameraForwardFt' | 'cameraRightFt' | 'cameraYawDeg' |
  'sprayForwardFt' | 'sprayRightFt' | 'operatingLoadLb' | 'surface'>;

export interface MountYawFit {
  cameraYawDeg: number;
  forwardCourseDeg: number;
  returnCourseDeg: number;
}

const MIN_PATH_FT = 6;
const MAX_PATH_BEND_DEG = 5;
const MAX_ROBUST_CROSS_TRACK_FT = 0.35;
const MAX_DIRECTION_DISAGREEMENT_DEG = 2;

function median(values: number[]): number {
  if (values.length === 0) throw new Error('calibration sample set is empty');
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 === 1
    ? sorted[middle]
    : (sorted[middle - 1] + sorted[middle]) / 2;
}

function signedAngle(value: number): number {
  return wrappedHeadingDelta(value, 0);
}

function bearing(dx: number, dy: number): number {
  return normalizeHeading(Math.atan2(dx, dy) * 180 / Math.PI);
}

function requireSamples(samples: YawCalibrationSample[], label: string): YawCalibrationSample[] {
  if (samples.length < 5) throw new Error(`${label} requires at least five samples`);
  return samples.map((sample) => {
    if (![sample.x, sample.y, sample.heading].every(Number.isFinite)) {
      throw new TypeError(`${label} samples must be finite`);
    }
    return { ...sample, heading: normalizeHeading(sample.heading) };
  });
}

function robustCourse(samples: YawCalibrationSample[]): number {
  const first = samples[0];
  const last = samples[samples.length - 1];
  const reference = bearing(last.x - first.x, last.y - first.y);
  const minimumGap = Math.max(2, Math.floor(samples.length / 4));
  const deltas: number[] = [];
  for (let from = 0; from < samples.length - minimumGap; from += 1) {
    for (let to = from + minimumGap; to < samples.length; to += 1) {
      const dx = samples[to].x - samples[from].x;
      const dy = samples[to].y - samples[from].y;
      if (Math.hypot(dx, dy) < 0.5) continue;
      deltas.push(wrappedHeadingDelta(bearing(dx, dy), reference));
    }
  }
  if (deltas.length === 0) throw new Error('calibration path has no usable displacement');
  return normalizeHeading(reference + median(deltas));
}

function fitPath(raw: YawCalibrationSample[], label: string) {
  const samples = requireSamples(raw, label);
  const courseDeg = robustCourse(samples);
  const theta = courseDeg * Math.PI / 180;
  const forwardX = Math.sin(theta);
  const forwardY = Math.cos(theta);
  const rightX = Math.cos(theta);
  const rightY = -Math.sin(theta);
  const first = samples[0];
  const last = samples[samples.length - 1];
  const lengthFt = (last.x - first.x) * forwardX + (last.y - first.y) * forwardY;
  if (lengthFt < MIN_PATH_FT) throw new Error(`${label} path must cover at least 6 ft`);

  const middle = Math.floor(samples.length / 2);
  const earlyCourse = robustCourse(samples.slice(0, middle + 1));
  const lateCourse = robustCourse(samples.slice(middle));
  if (Math.abs(wrappedHeadingDelta(lateCourse, earlyCourse)) > MAX_PATH_BEND_DEG) {
    throw new Error(`${label} calibration path must be straight`);
  }

  const crossTrack = samples
    .map((sample) => Math.abs(
      (sample.x - first.x) * rightX + (sample.y - first.y) * rightY,
    ))
    .sort((a, b) => a - b);
  const robustIndex = Math.min(crossTrack.length - 1, Math.floor(crossTrack.length * 0.8));
  if (crossTrack[robustIndex] > MAX_ROBUST_CROSS_TRACK_FT) {
    throw new Error(`${label} calibration path must be straight`);
  }

  const yawDeg = median(samples.map((sample) =>
    wrappedHeadingDelta(sample.heading, courseDeg)));
  return { courseDeg, yawDeg: signedAngle(yawDeg) };
}

export function fitMountYaw(
  forwardSamples: YawCalibrationSample[],
  returnSamples: YawCalibrationSample[],
): MountYawFit {
  const forward = fitPath(forwardSamples, 'forward');
  const returning = fitPath(returnSamples, 'return');
  const disagreement = wrappedHeadingDelta(returning.yawDeg, forward.yawDeg);
  if (Math.abs(disagreement) > MAX_DIRECTION_DISAGREEMENT_DEG) {
    throw new Error('forward and return mount-yaw estimates disagree by more than 2 degrees');
  }
  return {
    cameraYawDeg: signedAngle(forward.yawDeg + disagreement / 2),
    forwardCourseDeg: forward.courseDeg,
    returnCourseDeg: returning.courseDeg,
  };
}

function canonicalNumber(value: number): number {
  if (!Number.isFinite(value)) throw new TypeError('calibration numbers must be finite');
  const rounded = Math.round(value * 100000) / 100000;
  return Object.is(rounded, -0) ? 0 : rounded;
}

function requireInput(input: CalibrationInput): CalibrationInput {
  if (input.schemaVersion !== 1) throw new Error('unsupported calibration schema');
  if (!input.hardwareTag.trim()) throw new Error('hardware tag is required');
  if (!Number.isFinite(Date.parse(input.createdAtIso))) throw new Error('calibration timestamp is invalid');
  if (!['asphalt', 'concrete', 'pavers', 'other'].includes(input.surface)) {
    throw new Error('pavement surface is invalid');
  }
  if (input.condition !== 'dry' && input.condition !== 'wet') {
    throw new Error('pavement condition is invalid');
  }
  [input.cameraForwardFt, input.cameraRightFt, input.cameraYawDeg,
    input.sprayForwardFt, input.sprayRightFt, input.operatingLoadLb].forEach(canonicalNumber);
  if (input.operatingLoadLb < 0) throw new Error('operating load must not be negative');
  return input;
}

function canonicalFields(input: CalibrationInput): string {
  return JSON.stringify([
    input.schemaVersion,
    input.hardwareTag,
    input.createdAtIso,
    canonicalNumber(input.cameraForwardFt),
    canonicalNumber(input.cameraRightFt),
    canonicalNumber(signedAngle(input.cameraYawDeg)),
    canonicalNumber(input.sprayForwardFt),
    canonicalNumber(input.sprayRightFt),
    canonicalNumber(input.operatingLoadLb),
    input.surface,
    input.condition,
  ]);
}

function crc16String(value: string): number {
  let crc = 0xffff;
  for (let index = 0; index < value.length; index += 1) {
    const codeUnit = value.charCodeAt(index);
    for (const byte of [codeUnit >>> 8, codeUnit & 0xff]) {
      crc ^= byte << 8;
      for (let bit = 0; bit < 8; bit += 1) {
        crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
      }
    }
  }
  return crc;
}

export function calibrationId(input: CalibrationInput): number {
  requireInput(input);
  return crc16String(canonicalFields(input));
}

export function createCalibration(input: CalibrationInput): CalibrationRecord {
  requireInput(input);
  return {
    id: calibrationId(input),
    schemaVersion: 1,
    hardwareTag: input.hardwareTag,
    createdAtIso: input.createdAtIso,
    cameraForwardFt: canonicalNumber(input.cameraForwardFt),
    cameraRightFt: canonicalNumber(input.cameraRightFt),
    cameraYawDeg: canonicalNumber(signedAngle(input.cameraYawDeg)),
    sprayForwardFt: canonicalNumber(input.sprayForwardFt),
    sprayRightFt: canonicalNumber(input.sprayRightFt),
    operatingLoadLb: canonicalNumber(input.operatingLoadLb),
    surface: input.surface,
    condition: input.condition,
  };
}

export function calibrationDraftMatches(
  calibration: CalibrationRecord | null,
  draft: CalibrationDraft,
): boolean {
  if (!calibration) return false;
  try {
    return canonicalNumber(draft.cameraForwardFt) === calibration.cameraForwardFt &&
      canonicalNumber(draft.cameraRightFt) === calibration.cameraRightFt &&
      canonicalNumber(signedAngle(draft.cameraYawDeg)) === calibration.cameraYawDeg &&
      canonicalNumber(draft.sprayForwardFt) === calibration.sprayForwardFt &&
      canonicalNumber(draft.sprayRightFt) === calibration.sprayRightFt &&
      canonicalNumber(draft.operatingLoadLb) === calibration.operatingLoadLb &&
      draft.surface === calibration.surface;
  } catch {
    return false;
  }
}

export function hasMotionCalibrationProof(calibration: CalibrationRecord): boolean {
  return typeof calibration.motionCalibrationVerifiedAtIso === 'string' &&
    Number.isFinite(Date.parse(calibration.motionCalibrationVerifiedAtIso));
}

export function markMotionCalibrationVerified(
  calibration: CalibrationRecord,
  verifiedAtIso: string,
): CalibrationRecord {
  if (!isCalibrationRecord(calibration)) throw new Error('calibration record is invalid');
  if (!Number.isFinite(Date.parse(verifiedAtIso))) {
    throw new Error('motion calibration verification timestamp is invalid');
  }
  return { ...calibration, motionCalibrationVerifiedAtIso: verifiedAtIso };
}

export function motionCalibrationIdFromLog(line: string): number | null {
  const match = /^\[CAL PASS\] Motion calibration saved for ID (\d+)\.$/.exec(line.trim());
  if (!match) return null;
  const id = Number(match[1]);
  return Number.isInteger(id) && id >= 0 && id <= 0xffff ? id : null;
}

export function isCalibrationRecord(value: unknown): value is CalibrationRecord {
  if (typeof value !== 'object' || value === null) return false;
  const candidate = value as Partial<CalibrationRecord>;
  if (candidate.schemaVersion !== 1 || typeof candidate.hardwareTag !== 'string' ||
      typeof candidate.createdAtIso !== 'string' || typeof candidate.id !== 'number') return false;
  if (candidate.motionCalibrationVerifiedAtIso !== undefined &&
      (typeof candidate.motionCalibrationVerifiedAtIso !== 'string' ||
       !Number.isFinite(Date.parse(candidate.motionCalibrationVerifiedAtIso)))) return false;
  try {
    const rebuilt = createCalibration({
      schemaVersion: 1,
      hardwareTag: candidate.hardwareTag,
      createdAtIso: candidate.createdAtIso,
      cameraForwardFt: candidate.cameraForwardFt as number,
      cameraRightFt: candidate.cameraRightFt as number,
      cameraYawDeg: candidate.cameraYawDeg as number,
      sprayForwardFt: candidate.sprayForwardFt as number,
      sprayRightFt: candidate.sprayRightFt as number,
      operatingLoadLb: candidate.operatingLoadLb as number,
      surface: candidate.surface as PavementSurface,
      condition: candidate.condition as PavementCondition,
    });
    return Number.isInteger(candidate.id) && candidate.id === rebuilt.id;
  } catch {
    return false;
  }
}
