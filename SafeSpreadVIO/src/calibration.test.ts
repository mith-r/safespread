import {
  calibrationDraftMatches,
  CalibrationInput,
  createCalibration,
  fitMountYaw,
  hasMotionCalibrationProof,
  markMotionCalibrationVerified,
  motionCalibrationIdFromLog,
  YawCalibrationSample,
} from './calibration';

function path(
  courseDeg: number,
  cameraYawDeg: number,
  lengthFt = 8,
  noiseFt = 0,
): YawCalibrationSample[] {
  const theta = courseDeg * Math.PI / 180;
  const rightX = Math.cos(theta);
  const rightY = -Math.sin(theta);
  return Array.from({ length: 17 }, (_, index) => {
    const along = lengthFt * index / 16;
    const noise = noiseFt * ((index % 3) - 1);
    return {
      x: along * Math.sin(theta) + noise * rightX,
      y: along * Math.cos(theta) + noise * rightY,
      heading: (courseDeg + cameraYawDeg + 360) % 360,
    };
  });
}

describe('fitMountYaw', () => {
  it('fits forward and return paths across angular wrap', () => {
    const result = fitMountYaw(path(350, 15, 8, 0.03), path(170, 15, 8, 0.03));
    expect(result.cameraYawDeg).toBeCloseTo(15, 1);
    expect(result.forwardCourseDeg).toBeCloseTo(350, 1);
    expect(result.returnCourseDeg).toBeCloseTo(170, 1);
  });

  it('uses a robust line fit despite a position outlier', () => {
    const forward = path(42, -7, 10, 0.04);
    forward[8] = { ...forward[8], x: forward[8].x + 1.2, y: forward[8].y - 0.7 };
    const result = fitMountYaw(forward, path(222, -7, 10, 0.04));
    expect(result.cameraYawDeg).toBeCloseTo(-7, 1);
  });

  it('requires at least six feet in each direction', () => {
    expect(() => fitMountYaw(path(0, 5, 5.9), path(180, 5, 8))).toThrow(/6 ft/i);
    expect(() => fitMountYaw(path(0, 5, 8), path(180, 5, 5.9))).toThrow(/6 ft/i);
  });

  it('rejects a curved calibration run', () => {
    const curved = [
      ...path(0, 4, 4).slice(0, 9),
      ...path(14, 4, 4).slice(1).map((sample) => ({ ...sample, y: sample.y + 4 })),
    ];
    expect(() => fitMountYaw(curved, path(180, 4, 8))).toThrow(/straight/i);
  });

  it('rejects forward/return yaw estimates over two degrees apart', () => {
    expect(() => fitMountYaw(path(20, 3, 8), path(200, 5.1, 8))).toThrow(/disagree/i);
  });
});

describe('createCalibration', () => {
  const input: CalibrationInput = {
    schemaVersion: 1,
    hardwareTag: 'safespread-rover-a',
    createdAtIso: '2026-08-26T16:00:00.000Z',
    cameraForwardFt: 0.42,
    cameraRightFt: -0.08,
    cameraYawDeg: 12.34567,
    sprayForwardFt: -1.2,
    sprayRightFt: 0.1,
    operatingLoadLb: 42.5,
    surface: 'concrete',
    condition: 'dry',
  };

  it('computes a repeatable uint16 ID from canonical fields', () => {
    const first = createCalibration(input);
    const second = createCalibration({ ...input });
    expect(first).toEqual(second);
    expect(first.id).toBeGreaterThanOrEqual(0);
    expect(first.id).toBeLessThanOrEqual(0xffff);
  });

  it('changes the ID when a canonical field changes', () => {
    expect(createCalibration(input).id).not.toBe(
      createCalibration({ ...input, sprayForwardFt: -1.1 }).id,
    );
  });

  it('matches only saved canonical draft values', () => {
    const calibration = createCalibration(input);
    const draft = {
      cameraForwardFt: input.cameraForwardFt,
      cameraRightFt: input.cameraRightFt,
      cameraYawDeg: input.cameraYawDeg,
      sprayForwardFt: input.sprayForwardFt,
      sprayRightFt: input.sprayRightFt,
      operatingLoadLb: input.operatingLoadLb,
      surface: input.surface,
    };
    expect(calibrationDraftMatches(calibration, draft)).toBe(true);
    expect(calibrationDraftMatches(calibration, { ...draft, surface: 'asphalt' })).toBe(false);
    expect(calibrationDraftMatches(calibration, { ...draft, operatingLoadLb: 43 })).toBe(false);
    expect(calibrationDraftMatches(calibration, { ...draft, cameraForwardFt: Number.NaN })).toBe(false);
  });

  it('includes operating load in identity and rejects invalid load values', () => {
    expect(createCalibration(input).id).not.toBe(
      createCalibration({ ...input, operatingLoadLb: 50 }).id,
    );
    expect(() => createCalibration({ ...input, operatingLoadLb: -1 })).toThrow(/load/i);
    expect(() => createCalibration({ ...input, operatingLoadLb: Number.NaN })).toThrow(/finite/i);
  });

  it('records motion proof without changing calibration identity', () => {
    const calibration = createCalibration(input);
    expect(hasMotionCalibrationProof(calibration)).toBe(false);
    const verified = markMotionCalibrationVerified(calibration, '2026-08-27T12:00:00.000Z');
    expect(verified.id).toBe(calibration.id);
    expect(hasMotionCalibrationProof(verified)).toBe(true);
  });

  it('accepts only the exact bounded firmware motion-calibration completion line', () => {
    expect(motionCalibrationIdFromLog('[CAL PASS] Motion calibration saved for ID 42.')).toBe(42);
    expect(motionCalibrationIdFromLog('[CAL SAMPLE] throttle=1620')).toBeNull();
    expect(motionCalibrationIdFromLog('[CAL PASS] Motion calibration saved for ID 70000.')).toBeNull();
  });
});
