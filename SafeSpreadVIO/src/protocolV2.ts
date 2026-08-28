const VERSION = 2;
const MAGIC = 0x21;

export interface PoseV2 {
  flags: number;
  epoch: number;
  sequence: number;
  ageMs: number;
  x: number;
  y: number;
  heading: number;
  speedFps: number;
  yawRateDps: number;
  calibrationId: number;
}

export interface RectangleV2 {
  flags: number;
  epoch: number;
  commandId: number;
  mFt: number;
  nFt: number;
  startClearFt: number;
  endClearFt: number;
  calibrationId: number;
  /** Sprayed pass to begin on, counted in route order from zero. Omit or pass
   *  zero to drive the whole rectangle; anything else resumes a mission that
   *  faulted part-way through. */
  startPassIndex?: number;
}

export interface CalibrationV2 {
  flags: number;
  epoch: number;
  commandId: number;
  calibrationId: number;
  sprayForwardFt: number;
  sprayRightFt: number;
  schemaVersion: number;
}

export interface CommandV2 {
  opcode: number;
  epoch: number;
  commandId: number;
}

export interface AckV2 {
  state: number;
  epoch: number;
  commandId: number;
  faultCode: number;
  calibrationId: number;
}

export interface TelemetryV2 {
  state: number;
  epoch: number;
  consumedPoseSequence: number;
  routeIndex: number;
  routeCount: number;
  crossTrackFt: number;
  headingErrorDeg: number;
  speedFps: number;
  steeringUs: number;
  throttleUs: number;
  flags: number;
  faultCode: number;
  droppedPackets: number;
  poseAgeMs: number;
}

export interface RoutePlanV2 {
  style: 1 | 2;
  epoch: number;
  routeCount: number;
  passCount: number;
  leftRadiusFt: number;
  rightRadiusFt: number;
  beforeStartFt: number;
  beyondEndFt: number;
}

export interface FaultSampleV2 {
  flags: number;
  epoch: number;
  sequence: number;
  sampleIndex: number;
  sampleCount: number;
  routeIndex: number;
  crossTrackFt: number;
  headingErrorDeg: number;
  speedFps: number;
  steeringUs: number;
  throttleUs: number;
  state: number;
  faultCode: number;
  droppedPackets: number;
}

export function crc16Ccitt(bytes: Uint8Array): number {
  let crc = 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

function viewOf(bytes: Uint8Array): DataView {
  return new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
}

function uint(value: number, max: number, name: string): number {
  if (!Number.isInteger(value) || value < 0 || value > max) {
    throw new RangeError(`${name} must be an integer from 0 to ${max}`);
  }
  return value;
}

function finite(value: number, name: string): number {
  if (!Number.isFinite(value)) {
    throw new TypeError(`${name} must be finite`);
  }
  return value;
}

function positive(value: number, name: string): number {
  finite(value, name);
  if (value <= 0) throw new RangeError(`${name} must be positive`);
  return value;
}

function nonnegative(value: number, name: string): number {
  finite(value, name);
  if (value < 0) throw new RangeError(`${name} must not be negative`);
  return value;
}

function heading(value: number): number {
  finite(value, 'heading');
  if (value < 0 || value >= 360) throw new RangeError('heading must be in [0, 360)');
  return value;
}

function saturatedInteger(value: number, scale: number, min: number, max: number, name: string): number {
  finite(value, name);
  return Math.min(max, Math.max(min, Math.round(value * scale)));
}

function packet(length: number, type: number, flags: number): Uint8Array {
  const bytes = new Uint8Array(length);
  bytes[0] = MAGIC;
  bytes[1] = type;
  bytes[2] = VERSION;
  bytes[3] = uint(flags, 0xff, 'flags');
  return bytes;
}

function finalizePacket(bytes: Uint8Array): Uint8Array {
  viewOf(bytes).setUint16(bytes.length - 2, crc16Ccitt(bytes.subarray(0, -2)), true);
  return bytes;
}

function isPacket(bytes: Uint8Array, length: number, type: number): boolean {
  if (bytes.length !== length || bytes[0] !== MAGIC || bytes[1] !== type || bytes[2] !== VERSION) {
    return false;
  }
  const expected = viewOf(bytes).getUint16(length - 2, true);
  return crc16Ccitt(bytes.subarray(0, length - 2)) === expected;
}

export function buildPoseV2(value: PoseV2): Uint8Array {
  const bytes = packet(32, 0x56, uint(value.flags, 0b111, 'flags'));
  const view = viewOf(bytes);
  view.setUint16(4, uint(value.epoch, 0xffff, 'epoch'), true);
  view.setUint32(6, uint(value.sequence, 0xffffffff, 'sequence'), true);
  view.setUint16(10, saturatedInteger(value.ageMs, 1, 0, 0xffff, 'ageMs'), true);
  view.setFloat32(12, finite(value.x, 'x'), true);
  view.setFloat32(16, finite(value.y, 'y'), true);
  view.setFloat32(20, heading(value.heading), true);
  view.setInt16(24, saturatedInteger(value.speedFps, 100, -32768, 32767, 'speedFps'), true);
  view.setInt16(26, saturatedInteger(value.yawRateDps, 100, -32768, 32767, 'yawRateDps'), true);
  view.setUint16(28, uint(value.calibrationId, 0xffff, 'calibrationId'), true);
  return finalizePacket(bytes);
}

export function parsePoseV2(bytes: Uint8Array): PoseV2 | null {
  if (!isPacket(bytes, 32, 0x56)) return null;
  const view = viewOf(bytes);
  const x = view.getFloat32(12, true);
  const y = view.getFloat32(16, true);
  const wireHeading = view.getFloat32(20, true);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(wireHeading)) return null;
  if (wireHeading < 0 || wireHeading >= 360) return null;
  return {
    flags: bytes[3],
    epoch: view.getUint16(4, true),
    sequence: view.getUint32(6, true),
    ageMs: view.getUint16(10, true),
    x,
    y,
    heading: wireHeading,
    speedFps: view.getInt16(24, true) / 100,
    yawRateDps: view.getInt16(26, true) / 100,
    calibrationId: view.getUint16(28, true),
  };
}

export function buildRectangleV2(value: RectangleV2): Uint8Array {
  const bytes = packet(32, 0x44, uint(value.flags, 0b111, 'flags'));
  const view = viewOf(bytes);
  view.setUint16(4, uint(value.epoch, 0xffff, 'epoch'), true);
  view.setUint32(6, uint(value.commandId, 0xffffffff, 'commandId'), true);
  view.setFloat32(10, positive(value.mFt, 'mFt'), true);
  view.setFloat32(14, positive(value.nFt, 'nFt'), true);
  view.setFloat32(18, nonnegative(value.startClearFt, 'startClearFt'), true);
  view.setFloat32(22, nonnegative(value.endClearFt, 'endClearFt'), true);
  view.setUint16(26, uint(value.calibrationId, 0xffff, 'calibrationId'), true);
  view.setUint16(28, uint(value.startPassIndex ?? 0, 0xffff, 'startPassIndex'), true);
  return finalizePacket(bytes);
}

export function buildCalibrationV2(value: CalibrationV2): Uint8Array {
  if (value.flags !== 0) throw new RangeError('calibration flags are reserved');
  const bytes = packet(24, 0x4b, value.flags);
  const view = viewOf(bytes);
  view.setUint16(4, uint(value.epoch, 0xffff, 'epoch'), true);
  view.setUint32(6, uint(value.commandId, 0xffffffff, 'commandId'), true);
  view.setUint16(10, uint(value.calibrationId, 0xffff, 'calibrationId'), true);
  view.setFloat32(12, finite(value.sprayForwardFt, 'sprayForwardFt'), true);
  view.setFloat32(16, finite(value.sprayRightFt, 'sprayRightFt'), true);
  view.setUint16(20, uint(value.schemaVersion, 0xffff, 'schemaVersion'), true);
  return finalizePacket(bytes);
}

export function buildCommandV2(value: CommandV2): Uint8Array {
  const opcode = uint(value.opcode, 8, 'opcode');
  if (opcode === 0) throw new RangeError('opcode must be in [1, 8]');
  const bytes = packet(12, 0x43, opcode);
  const view = viewOf(bytes);
  view.setUint16(4, uint(value.epoch, 0xffff, 'epoch'), true);
  view.setUint32(6, uint(value.commandId, 0xffffffff, 'commandId'), true);
  return finalizePacket(bytes);
}

export function parseAckV2(bytes: Uint8Array): AckV2 | null {
  if (!isPacket(bytes, 16, 0x41)) return null;
  const view = viewOf(bytes);
  return {
    state: bytes[3],
    epoch: view.getUint16(4, true),
    commandId: view.getUint32(6, true),
    faultCode: view.getUint16(10, true),
    calibrationId: view.getUint16(12, true),
  };
}

export function parseTelemetryV2(bytes: Uint8Array): TelemetryV2 | null {
  if (!isPacket(bytes, 32, 0x54)) return null;
  const view = viewOf(bytes);
  return {
    state: bytes[3],
    epoch: view.getUint16(4, true),
    consumedPoseSequence: view.getUint32(6, true),
    routeIndex: view.getUint16(10, true),
    routeCount: view.getUint16(12, true),
    crossTrackFt: view.getInt16(14, true) / 100,
    headingErrorDeg: view.getInt16(16, true) / 100,
    speedFps: view.getInt16(18, true) / 100,
    steeringUs: view.getUint16(20, true),
    throttleUs: view.getUint16(22, true),
    flags: bytes[24],
    faultCode: bytes[25],
    droppedPackets: view.getUint16(26, true),
    poseAgeMs: view.getUint16(28, true),
  };
}

export function parseRoutePlanV2(bytes: Uint8Array): RoutePlanV2 | null {
  if (!isPacket(bytes, 28, 0x52) || bytes[3] < 1 || bytes[3] > 2) return null;
  const view = viewOf(bytes);
  const parsed: RoutePlanV2 = {
    style: bytes[3] as 1 | 2,
    epoch: view.getUint16(4, true),
    routeCount: view.getUint16(6, true),
    passCount: view.getUint16(8, true),
    leftRadiusFt: view.getFloat32(10, true),
    rightRadiusFt: view.getFloat32(14, true),
    beforeStartFt: view.getFloat32(18, true),
    beyondEndFt: view.getFloat32(22, true),
  };
  if (parsed.routeCount === 0 || parsed.passCount === 0 ||
      !Number.isFinite(parsed.leftRadiusFt) || parsed.leftRadiusFt <= 0 ||
      !Number.isFinite(parsed.rightRadiusFt) || parsed.rightRadiusFt <= 0 ||
      !Number.isFinite(parsed.beforeStartFt) || parsed.beforeStartFt < 0 ||
      !Number.isFinite(parsed.beyondEndFt) || parsed.beyondEndFt < 0) return null;
  return parsed;
}

export function parseFaultSampleV2(bytes: Uint8Array): FaultSampleV2 | null {
  if (!isPacket(bytes, 32, 0x42)) return null;
  const view = viewOf(bytes);
  return {
    flags: bytes[3],
    epoch: view.getUint16(4, true),
    sequence: view.getUint32(6, true),
    sampleIndex: view.getUint16(10, true),
    sampleCount: view.getUint16(12, true),
    routeIndex: view.getUint16(14, true),
    crossTrackFt: view.getInt16(16, true) / 100,
    headingErrorDeg: view.getInt16(18, true) / 100,
    speedFps: view.getInt16(20, true) / 100,
    steeringUs: view.getUint16(22, true),
    throttleUs: view.getUint16(24, true),
    state: bytes[26],
    faultCode: bytes[27],
    droppedPackets: view.getUint16(28, true),
  };
}
