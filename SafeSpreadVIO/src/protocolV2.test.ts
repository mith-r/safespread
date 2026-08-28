import {
  AckV2,
  buildCalibrationV2,
  buildCommandV2,
  buildPoseV2,
  buildRectangleV2,
  crc16Ccitt,
  parseAckV2,
  parseFaultSampleV2,
  parsePoseV2,
  parseTelemetryV2,
  PoseV2,
} from './protocolV2';

function hex(bytes: Uint8Array): string {
  return Array.from(bytes, (byte) => byte.toString(16).padStart(2, '0')).join('');
}

function finish(bytes: Uint8Array): Uint8Array {
  const crc = crc16Ccitt(bytes.subarray(0, bytes.length - 2));
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  view.setUint16(bytes.length - 2, crc, true);
  return bytes;
}

describe('protocol v2 builders', () => {
  const pose: PoseV2 = {
    flags: 0b111,
    epoch: 0x1234,
    sequence: 0x01020304,
    ageMs: 125,
    x: 1.5,
    y: -2.25,
    heading: 359.5,
    speedFps: -1.23,
    yawRateDps: 45.67,
    calibrationId: 0xbeef,
  };

  it('matches the canonical pose fixture and round trips', () => {
    const packet = buildPoseV2(pose);
    expect(packet).toHaveLength(32);
    expect(hex(packet)).toBe(
      '215602073412040302017d000000c03f000010c000c0b34385ffd711efbeb7e9',
    );
    expect(parsePoseV2(packet)).toEqual(pose);
  });

  it('saturates only age and signed fixed-point motion fields', () => {
    const parsed = parsePoseV2(
      buildPoseV2({ ...pose, ageMs: 1e9, speedFps: 1e9, yawRateDps: -1e9 }),
    );
    expect(parsed).toMatchObject({
      ageMs: 65535,
      speedFps: 327.67,
      yawRateDps: -327.68,
    });
  });

  it('builds every command/configuration packet with the frozen size', () => {
    const rectangle = buildRectangleV2({
        flags: 0b101,
        epoch: 4,
        commandId: 9,
        mFt: 20,
        nFt: 8,
        startClearFt: 4,
        endClearFt: 6,
        calibrationId: 3,
      });
    expect(hex(rectangle)).toBe(
      '214402050400090000000000a04100000041000080400000c04003000000e9c0',
    );

    const calibration = buildCalibrationV2({
        flags: 0,
        epoch: 4,
        commandId: 10,
        calibrationId: 3,
        sprayForwardFt: -0.5,
        sprayRightFt: 0.25,
        schemaVersion: 1,
      });
    expect(hex(calibration)).toBe('214b020004000a0000000300000000bf0000803e010078a5');

    const command = buildCommandV2({ opcode: 8, epoch: 4, commandId: 11 });
    expect(hex(command)).toBe('2143020804000b00000059c1');
  });

  it('carries the resume pass in the rectangle packet without moving anything else', () => {
    const base = {
      flags: 0b101,
      epoch: 4,
      commandId: 9,
      mFt: 20,
      nFt: 8,
      startClearFt: 4,
      endClearFt: 6,
      calibrationId: 3,
    };
    const fromStart = buildRectangleV2(base);
    const resumed = buildRectangleV2({ ...base, startPassIndex: 4 });
    expect(resumed.length).toBe(fromStart.length);
    // Everything before the formerly reserved field is byte-identical, so a
    // rover that ignores the field still reads the same rectangle.
    expect(hex(resumed.subarray(0, 28))).toBe(hex(fromStart.subarray(0, 28)));
    expect(Array.from(resumed.subarray(28, 30))).toEqual([4, 0]);
    const view = new DataView(resumed.buffer, resumed.byteOffset, resumed.byteLength);
    expect(view.getUint16(30, true)).toBe(crc16Ccitt(resumed.subarray(0, 30)));
    // An omitted field is the old wire format exactly.
    expect(hex(buildRectangleV2({ ...base, startPassIndex: 0 }))).toBe(hex(fromStart));
    expect(() => buildRectangleV2({ ...base, startPassIndex: -1 })).toThrow(RangeError);
    expect(() => buildRectangleV2({ ...base, startPassIndex: 0x10000 })).toThrow(RangeError);
  });

  it('rejects invalid identifiers, geometry, headings, and non-finite values', () => {
    expect(() => buildPoseV2({ ...pose, epoch: -1 })).toThrow(RangeError);
    expect(() => buildPoseV2({ ...pose, flags: 8 })).toThrow(RangeError);
    expect(() => buildPoseV2({ ...pose, x: Number.NaN })).toThrow(TypeError);
    expect(() => buildPoseV2({ ...pose, heading: 360 })).toThrow(RangeError);
    expect(() =>
      buildRectangleV2({
        flags: 0,
        epoch: 1,
        commandId: 1,
        mFt: 0,
        nFt: 4,
        startClearFt: 0,
        endClearFt: 0,
        calibrationId: 1,
      }),
    ).toThrow(RangeError);
    expect(() =>
      buildCalibrationV2({
        flags: 1,
        epoch: 1,
        commandId: 1,
        calibrationId: 1,
        sprayForwardFt: 0,
        sprayRightFt: 0,
        schemaVersion: 1,
      }),
    ).toThrow(RangeError);
    expect(() => buildCommandV2({ opcode: 9, epoch: 1, commandId: 1 })).toThrow(
      RangeError,
    );
  });

  it('rejects a bad CRC, version, magic, size, or non-finite wire float', () => {
    const badCrc = buildPoseV2(pose).slice();
    badCrc[12] ^= 1;
    expect(parsePoseV2(badCrc)).toBeNull();

    const badVersion = buildPoseV2(pose).slice();
    badVersion[2] = 1;
    expect(parsePoseV2(badVersion)).toBeNull();

    const badMagic = buildPoseV2(pose).slice();
    badMagic[1] = 0x50;
    expect(parsePoseV2(badMagic)).toBeNull();
    expect(parsePoseV2(badMagic.subarray(0, 31))).toBeNull();

    const nan = buildPoseV2(pose).slice();
    new DataView(nan.buffer).setUint32(12, 0x7fc00000, true);
    finish(nan);
    expect(parsePoseV2(nan)).toBeNull();
  });
});

describe('protocol v2 inbound packets', () => {
  it('parses ACK fields', () => {
    const bytes = new Uint8Array(16);
    const view = new DataView(bytes.buffer);
    bytes.set([0x21, 0x41, 2, 5]);
    view.setUint16(4, 0x1234, true);
    view.setUint32(6, 0x01020304, true);
    view.setUint16(10, 9, true);
    view.setUint16(12, 0xbeef, true);
    const expected: AckV2 = {
      state: 5,
      epoch: 0x1234,
      commandId: 0x01020304,
      faultCode: 9,
      calibrationId: 0xbeef,
    };
    expect(parseAckV2(finish(bytes))).toEqual(expected);
  });

  it('parses signed telemetry scaling and rejects corrupt packets', () => {
    const bytes = new Uint8Array(32);
    const view = new DataView(bytes.buffer);
    bytes.set([0x21, 0x54, 2, 3]);
    view.setUint16(4, 7, true);
    view.setUint32(6, 99, true);
    view.setUint16(10, 4, true);
    view.setUint16(12, 18, true);
    view.setInt16(14, -25, true);
    view.setInt16(16, 1250, true);
    view.setInt16(18, -75, true);
    view.setUint16(20, 1510, true);
    view.setUint16(22, 1420, true);
    bytes[24] = 0b101;
    bytes[25] = 0;
    view.setUint16(26, 12, true);
    view.setUint16(28, 80, true);
    finish(bytes);

    expect(parseTelemetryV2(bytes)).toEqual({
      state: 3,
      epoch: 7,
      consumedPoseSequence: 99,
      routeIndex: 4,
      routeCount: 18,
      crossTrackFt: -0.25,
      headingErrorDeg: 12.5,
      speedFps: -0.75,
      steeringUs: 1510,
      throttleUs: 1420,
      flags: 0b101,
      faultCode: 0,
      droppedPackets: 12,
      poseAgeMs: 80,
    });
    bytes[29] ^= 1;
    expect(parseTelemetryV2(bytes)).toBeNull();
  });

  it('parses a frozen fault-buffer sample', () => {
    const bytes = new Uint8Array(32);
    const view = new DataView(bytes.buffer);
    bytes.set([0x21, 0x42, 2, 0b11]);
    view.setUint16(4, 7, true);
    view.setUint32(6, 1234, true);
    view.setUint16(10, 4, true);
    view.setUint16(12, 5, true);
    view.setUint16(14, 8, true);
    view.setInt16(16, -10, true);
    view.setInt16(18, 225, true);
    view.setInt16(20, 80, true);
    view.setUint16(22, 1490, true);
    view.setUint16(24, 1540, true);
    bytes[26] = 5;
    bytes[27] = 2;
    view.setUint16(28, 3, true);

    expect(parseFaultSampleV2(finish(bytes))).toEqual({
      flags: 0b11,
      epoch: 7,
      sequence: 1234,
      sampleIndex: 4,
      sampleCount: 5,
      routeIndex: 8,
      crossTrackFt: -0.1,
      headingErrorDeg: 2.25,
      speedFps: 0.8,
      steeringUs: 1490,
      throttleUs: 1540,
      state: 5,
      faultCode: 2,
      droppedPackets: 3,
    });
  });
});
