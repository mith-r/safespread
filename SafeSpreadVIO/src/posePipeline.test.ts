import { PoseUpdatePayload } from '../modules/arkit-pose/src/ArkitPose.types';
import { MountCalibration } from './poseMath';
import { PosePipeline } from './posePipeline';

const calibration: MountCalibration = {
  id: 7,
  schemaVersion: 1,
  cameraForwardFt: 1,
  cameraRightFt: 0,
  cameraYawDeg: 0,
  sprayForwardFt: -0.5,
  sprayRightFt: 0,
};

function poseEvent(
  sequence: number,
  frameTimestampMs: number,
  values: Partial<Extract<PoseUpdatePayload, { kind: 'pose' }>> = {},
): Extract<PoseUpdatePayload, { kind: 'pose' }> {
  return {
    kind: 'pose',
    x: 0,
    y: 1,
    heading: 0,
    trackingState: 'normal',
    trackingReason: 'none',
    mappingStatus: 'mapped',
    frameTimestampMs,
    emittedTimestampMs: frameTimestampMs + 5,
    frameIntervalMs: 1000 / 60,
    thermalState: 'nominal',
    sequence,
    ...values,
  };
}

function accepted(pipeline: PosePipeline, event: PoseUpdatePayload, receivedAtMs: number) {
  const result = pipeline.ingest(event, receivedAtMs);
  expect(result.ok).toBe(true);
  if (!result.ok) throw new Error(`expected accepted pose, got ${result.reason}`);
  return result.pose;
}

function fillStationaryWindow(
  pipeline: PosePipeline,
  overrides: (index: number) => Partial<Extract<PoseUpdatePayload, { kind: 'pose' }>> = () => ({}),
) {
  let receivedAtMs = 10_000;
  for (let index = 0; index <= 30; index += 1) {
    const timestamp = index * (2000 / 30);
    receivedAtMs = 10_000 + timestamp;
    accepted(pipeline, poseEvent(index + 1, timestamp, overrides(index)), receivedAtMs);
  }
  return receivedAtMs;
}

describe('PosePipeline transforms and robust motion estimates', () => {
  it('converts camera pose to rear axle and spray-bar pose', () => {
    const pipeline = new PosePipeline(calibration);
    const pose = accepted(pipeline, poseEvent(1, 100), 10_000);
    expect(pose.camera).toEqual({ x: 0, y: 1, heading: 0 });
    expect(pose.rover).toEqual({ x: 0, y: 0, heading: 0 });
    expect(pose.sprayBar).toEqual({ x: 0, y: -0.5, heading: 0 });
    expect(pose.speedFps).toBe(0);
    expect(pose.courseDeg).toBeNull();
  });

  // An accepted pose goes to the rover on the frame it arrives, without waiting
  // for a re-render, so everything describing that frame has to travel with it
  // rather than being read back out of component state showing an older one.
  it('carries its own frame metadata, timing, and thermal state', () => {
    const pipeline = new PosePipeline(calibration);
    const pose = accepted(pipeline, poseEvent(1, 100, {
      frameIntervalMs: 52.4,
      thermalState: 'serious',
      trackingState: 'normal',
      trackingReason: 'none',
      mappingStatus: 'limited',
    }), 10_000);
    expect(pose.frameIntervalMs).toBe(52.4);
    expect(pose.thermalState).toBe('serious');
    expect(pose.trackingState).toBe('normal');
    expect(pose.trackingReason).toBe('none');
    expect(pose.mappingStatus).toBe('limited');
  });

  it('uses a robust five-sample estimate for velocity, course, and wrapped yaw', () => {
    const pipeline = new PosePipeline({ ...calibration, cameraForwardFt: 0, sprayForwardFt: 0 });
    const ys = [0, 0.1, 0.4, 0.3, 0.4];
    let latest;
    for (let index = 0; index < ys.length; index += 1) {
      latest = accepted(
        pipeline,
        poseEvent(index + 1, index * 100, { y: ys[index], heading: (359 + index) % 360 }),
        10_000 + index * 100,
      );
    }
    expect(latest!.speedFps).toBeCloseTo(1, 2);
    expect(latest!.courseDeg).toBeCloseTo(0, 3);
    expect(latest!.yawRateDps).toBeCloseTo(10, 3);
  });
});

describe('PosePipeline drive rejection', () => {
  it('rejects status-only and degraded-tracking events and resets readiness', () => {
    const pipeline = new PosePipeline(calibration);
    fillStationaryWindow(pipeline);
    expect(pipeline.readiness(12_000).ready).toBe(true);

    const limited = pipeline.ingest(
      poseEvent(32, 2100, {
        trackingState: 'limited',
        trackingReason: 'insufficientFeatures',
      }),
      12_100,
    );
    expect(limited).toEqual({ ok: false, reason: 'tracking' });
    expect(pipeline.readiness(12_100).ready).toBe(false);

    const status: PoseUpdatePayload = {
      kind: 'status',
      trackingState: 'notAvailable',
      trackingReason: 'interrupted',
      mappingStatus: 'notAvailable',
      frameTimestampMs: 2200,
      emittedTimestampMs: 2200,
      frameIntervalMs: 0,
      thermalState: 'nominal',
      sequence: 32,
    };
    expect(pipeline.ingest(status, 12_200)).toEqual({ ok: false, reason: 'tracking' });
  });

  it('rejects duplicate/backward sequence or timestamp and non-finite poses', () => {
    const pipeline = new PosePipeline(calibration);
    accepted(pipeline, poseEvent(2, 100), 1000);
    expect(pipeline.ingest(poseEvent(2, 110), 1010)).toEqual({ ok: false, reason: 'sequence' });
    expect(pipeline.ingest(poseEvent(3, 90), 1020)).toEqual({ ok: false, reason: 'timestamp' });
    expect(pipeline.ingest(poseEvent(4, 120, { x: Number.NaN }), 1030)).toEqual({
      ok: false,
      reason: 'nonFinite',
    });
  });

  it('rejects excessive capture age', () => {
    const pipeline = new PosePipeline(calibration);
    expect(
      pipeline.ingest(poseEvent(1, 100, { emittedTimestampMs: 351 }), 10_000),
    ).toEqual({ ok: false, reason: 'age' });
  });

  it('rejects implausible speed, acceleration, and yaw rate', () => {
    const speed = new PosePipeline({ ...calibration, cameraForwardFt: 0 });
    accepted(speed, poseEvent(1, 0, { y: 0 }), 1000);
    expect(speed.ingest(poseEvent(2, 50, { y: 0.5 }), 1050)).toEqual({
      ok: false,
      reason: 'speed',
    });

    const acceleration = new PosePipeline({ ...calibration, cameraForwardFt: 0 });
    accepted(acceleration, poseEvent(1, 0, { y: 0 }), 1000);
    accepted(acceleration, poseEvent(2, 100, { y: 0.1 }), 1100);
    expect(acceleration.ingest(poseEvent(3, 200, { y: 0.8 }), 1200)).toEqual({
      ok: false,
      reason: 'acceleration',
    });

    const yaw = new PosePipeline(calibration);
    accepted(yaw, poseEvent(1, 0, { heading: 0 }), 1000);
    expect(yaw.ingest(poseEvent(2, 100, { heading: 30 }), 1100)).toEqual({
      ok: false,
      reason: 'yawRate',
    });
  });

  it('absorbs an ARKit world translation and preserves subsequent motion', () => {
    const pipeline = new PosePipeline({ ...calibration, cameraForwardFt: 0 });
    accepted(pipeline, poseEvent(1, 0, { x: 0, y: 0 }), 1000);

    const corrected = accepted(
      pipeline,
      poseEvent(2, 16, { x: 1.33, y: 0.05 }),
      1016,
    );
    expect(corrected.rover.x).toBeCloseTo(0, 6);
    expect(corrected.rover.y).toBeCloseTo(0, 6);
    expect(corrected.relocalizationShiftFt).toBeCloseTo(Math.hypot(1.33, 0.05), 6);

    const continued = accepted(
      pipeline,
      poseEvent(3, 32, { x: 1.34, y: 0.05 }),
      1032,
    );
    expect(continued.rover.x).toBeCloseTo(0.01, 6);
    expect(continued.rover.y).toBeCloseTo(0, 6);
    expect(continued.relocalizationShiftFt).toBe(0);
    expect(continued.accumulatedWorldOffsetXFt).toBeCloseTo(-1.33, 6);
    expect(continued.accumulatedWorldOffsetYFt).toBeCloseTo(-0.05, 6);
  });
});

describe('PosePipeline standstill noise', () => {
  // A parked rover still produces a jittering velocity estimate. Dividing that
  // jitter by a 16 ms frame interval used to clear the acceleration limit and
  // stop the pose stream, which the rover then reported as a pose timeout.
  it('accepts a stationary stream carrying position noise at frame rate', () => {
    const pipeline = new PosePipeline({ ...calibration, cameraForwardFt: 0 });
    for (let index = 0; index < 200; index += 1) {
      const timestamp = index * 16;
      const result = pipeline.ingest(
        poseEvent(index + 1, timestamp, {
          x: index % 2 ? 0.008 : -0.008,
          y: index % 2 ? 0.994 : 1.006,
          heading: index % 2 ? 0.3 : 359.7,
        }),
        10_000 + timestamp,
      );
      expect(result).toEqual(expect.objectContaining({ ok: true }));
      if (result.ok) expect(result.pose.speedFps).toBe(0);
    }
  });

  it('keeps the arming window through an automatic world-frame correction', () => {
    const pipeline = new PosePipeline(calibration);
    const lastReceived = fillStationaryWindow(pipeline);
    expect(pipeline.readiness(lastReceived).ready).toBe(true);

    const corrected = accepted(
      pipeline,
      poseEvent(40, 2100, { x: 6 }),
      lastReceived + 34,
    );
    expect(corrected.relocalizationShiftFt).toBeGreaterThan(5);
    expect(pipeline.readiness(lastReceived + 34).ready).toBe(true);
  });
});

describe('PosePipeline stable arming window', () => {
  it('requires 2 seconds, 30 samples, stable position/heading, and a fresh newest pose', () => {
    const pipeline = new PosePipeline(calibration);
    const lastReceived = fillStationaryWindow(pipeline, (index) => ({
      x: index % 2 ? 0.01 : -0.01,
      heading: index % 2 ? 359.5 : 0.5,
    }));
    expect(pipeline.readiness(lastReceived)).toEqual({ ready: true, reason: 'ready' });
    expect(pipeline.readiness(lastReceived + 145).ready).toBe(true);
    expect(pipeline.readiness(lastReceived + 146)).toEqual({ ready: false, reason: 'stale' });
  });

  it('rejects insufficient duration/sample count and excessive stationary spread', () => {
    const tooShort = new PosePipeline(calibration);
    for (let index = 0; index < 30; index += 1) {
      accepted(tooShort, poseEvent(index + 1, index * 50), 10_000 + index * 50);
    }
    expect(tooShort.readiness(11_450).reason).toBe('duration');

    const position = new PosePipeline(calibration);
    const positionLast = fillStationaryWindow(position, (index) => ({ x: index === 30 ? 0.11 : 0 }));
    expect(position.readiness(positionLast).reason).toBe('positionSpread');

    const heading = new PosePipeline(calibration);
    const headingLast = fillStationaryWindow(heading, (index) => ({ heading: index === 30 ? 1.1 : 0 }));
    expect(heading.readiness(headingLast).reason).toBe('headingSpread');
  });
});
