import { crc16Ccitt, FaultSampleV2 } from './protocolV2';
import {
  assembleFaultPackets,
  initialSetupState,
  isAuthoritativeLogReady,
  MissionOperationGate,
  setupReducer,
  SetupState,
} from './setupMachine';

const atA = { x: 10, y: 20, heading: 0 };

function reduce(state: SetupState, ...actions: Parameters<typeof setupReducer>[1][]) {
  return actions.reduce(setupReducer, state);
}

function connectedState(): SetupState {
  return reduce(
    initialSetupState(),
    { type: 'CONNECTION_CHANGED', status: 'connected', compatible: true },
    { type: 'CONTINUE' },
  );
}

function enteredReadyState(options: { wet?: boolean; loggingReady?: boolean } = {}): SetupState {
  return reduce(
    connectedState(),
    { type: 'SELECT_RECTANGLE_MODE', mode: 'entered' },
    {
      type: 'SET_ENTERED_RECTANGLE',
      pose: atA,
      mFt: 20,
      nFt: 8,
      side: 'right',
      startClearFt: 19,
      endClearFt: 12,
    },
    { type: 'CONTINUE' },
    { type: 'SET_CALIBRATION_STATUS', status: 'ready' },
    { type: 'SET_WET_MODE', wet: options.wet ?? false },
    { type: 'SET_LOGGING_READY', ready: options.loggingReady ?? true },
    {
      type: 'SET_READINESS',
      trackingNormal: true,
      poseStable: true,
      atStart: true,
    },
  );
}

describe('setupReducer', () => {
  it('completes the entered M by N workflow only through acknowledged arm and start', () => {
    let state = enteredReadyState();
    expect(state.phase).toBe('readiness');
    expect(state.rectangle).toMatchObject({ mFt: 20, nFt: 8, side: 'right' });

    state = setupReducer(state, { type: 'REQUEST_ARM' });
    expect(state.phase).toBe('arming');
    state = setupReducer(state, { type: 'ARM_ACKNOWLEDGED' });
    expect(state.phase).toBe('armed');
    state = setupReducer(state, { type: 'REQUEST_START' });
    expect(state.phase).toBe('starting');
    state = setupReducer(state, { type: 'START_ACKNOWLEDGED' });
    expect(state.phase).toBe('running');
  });

  it('builds a walked opposite-corner rectangle and requires explicit left-side confirmation', () => {
    let state = reduce(
      connectedState(),
      { type: 'SELECT_RECTANGLE_MODE', mode: 'walked' },
      { type: 'CAPTURE_CORNER_A', pose: atA, stable: true },
      {
        type: 'CAPTURE_CORNER_B',
        pose: { x: 2, y: 40, heading: 90 },
        stable: true,
        startClearFt: 8,
        endClearFt: 9,
      },
    );
    expect(state.rectangle).toMatchObject({ source: 'walked', mFt: 20, nFt: 8, side: 'left' });
    state = setupReducer(state, { type: 'CONTINUE' });
    expect(state.phase).toBe('rectangle');
    expect(state.validationError).toMatch(/left.*confirm/i);
    state = reduce(state, { type: 'CONFIRM_COVERAGE_SIDE' }, { type: 'CONTINUE' });
    expect(state.phase).toBe('readiness');
  });

  it('rejects unstable corner captures and requires return to A before walked arming', () => {
    let state = reduce(
      connectedState(),
      { type: 'SELECT_RECTANGLE_MODE', mode: 'walked' },
      { type: 'CAPTURE_CORNER_A', pose: atA, stable: false },
    );
    expect(state.cornerA).toBeNull();
    expect(state.validationError).toMatch(/stable/i);

    state = reduce(
      state,
      { type: 'CAPTURE_CORNER_A', pose: atA, stable: true },
      {
        type: 'CAPTURE_CORNER_B',
        pose: { x: 8, y: 40, heading: 0 },
        stable: false,
        startClearFt: 8,
        endClearFt: 9,
      },
    );
    expect(state.rectangle).toBeNull();

    state = reduce(
      state,
      {
        type: 'CAPTURE_CORNER_B',
        pose: { x: 8, y: 40, heading: 0 },
        stable: true,
        startClearFt: 8,
        endClearFt: 9,
      },
      { type: 'CONFIRM_COVERAGE_SIDE' },
      { type: 'CONTINUE' },
      { type: 'SET_CALIBRATION_STATUS', status: 'ready' },
      { type: 'SET_LOGGING_READY', ready: true },
      {
        type: 'SET_READINESS',
        trackingNormal: true,
        poseStable: true,
        atStart: false,
      },
      { type: 'REQUEST_ARM' },
    );
    expect(state.phase).toBe('readiness');
    expect(state.validationError).toMatch(/corner A/i);
  });

  it('rejects invalid dimensions and negative headland before leaving rectangle setup', () => {
    let state = reduce(
      connectedState(),
      { type: 'SELECT_RECTANGLE_MODE', mode: 'entered' },
      {
        type: 'SET_ENTERED_RECTANGLE',
        pose: atA,
        mFt: 0,
        nFt: 8,
        side: 'right',
        startClearFt: -1,
        endClearFt: 4,
      },
      { type: 'CONTINUE' },
    );
    expect(state.phase).toBe('rectangle');
    expect(state.rectangle).toBeNull();
    expect(state.validationError).toBeTruthy();
  });

  it('requires explicit confirmation for an entered LEFT coverage side', () => {
    let state = reduce(
      connectedState(),
      { type: 'SELECT_RECTANGLE_MODE', mode: 'entered' },
      {
        type: 'SET_ENTERED_RECTANGLE',
        pose: atA,
        mFt: 20,
        nFt: 8,
        side: 'left',
        startClearFt: 8,
        endClearFt: 9,
      },
      { type: 'CONTINUE' },
    );
    expect(state.phase).toBe('rectangle');
    expect(state.validationError).toMatch(/left.*confirm/i);
    state = reduce(state, { type: 'CONFIRM_COVERAGE_SIDE' }, { type: 'CONTINUE' });
    expect(state.phase).toBe('readiness');
  });

  it('does not advance for incompatible firmware', () => {
    const state = reduce(
      initialSetupState(),
      { type: 'CONNECTION_CHANGED', status: 'incompatible', compatible: false },
      { type: 'CONTINUE' },
    );
    expect(state.phase).toBe('connection');
    expect(state.validationError).toMatch(/protocol v2/i);
  });

  it('does not erase a user-facing validation error on the next pose update', () => {
    const invalid = setupReducer(connectedState(), { type: 'CONTINUE' });
    const updated = setupReducer(invalid, {
      type: 'SET_READINESS',
      trackingNormal: true,
      poseStable: true,
      atStart: false,
    });
    expect(updated.validationError).toBe(invalid.validationError);
  });

  it.each(['missing', 'stale'] as const)(
    'permits wet and dry arming with %s calibration',
    (status) => {
      let wet = enteredReadyState({ wet: true });
      wet = reduce(
        wet,
        { type: 'SET_CALIBRATION_STATUS', status },
        { type: 'REQUEST_ARM' },
      );
      expect(wet.phase).toBe('arming');

      let dry = enteredReadyState();
      dry = reduce(
        dry,
        { type: 'SET_CALIBRATION_STATUS', status },
        { type: 'REQUEST_ARM' },
      );
      expect(dry.phase).toBe('arming');
    },
  );

  it('permits wet and dry arming when logging is unavailable', () => {
    const wet = setupReducer(
      enteredReadyState({ wet: true, loggingReady: false }),
      { type: 'REQUEST_ARM' },
    );
    expect(wet.phase).toBe('arming');
    expect(wet.warning).toMatch(/continues without a log/i);

    const dry = setupReducer(
      enteredReadyState({ wet: false, loggingReady: false }),
      { type: 'REQUEST_ARM' },
    );
    expect(dry.phase).toBe('arming');
    expect(dry.warning).toMatch(/continues without a log/i);
  });

  it('turns an acknowledgement timeout into a visible fault', () => {
    const state = setupReducer(
      setupReducer(enteredReadyState(), { type: 'REQUEST_ARM' }),
      { type: 'ACK_TIMEOUT', operation: 'Arm' },
    );
    expect(state.phase).toBe('fault');
    expect(state.fault).toMatch(/Arm.*timeout/i);
  });

  it.each([
    'connection', 'rectangle', 'calibration', 'readiness',
    'arming', 'armed', 'starting', 'running', 'complete', 'fault',
  ] as const)('accepts Stop from %s', (phase) => {
    const state = setupReducer({ ...initialSetupState(), phase }, { type: 'STOP' });
    expect(state.phase).toBe('connection');
    expect(state.fault).toBeNull();
  });

  it('keeps a compatible BLE connection after Stop and returns to rectangle setup', () => {
    const state = setupReducer({ ...connectedState(), phase: 'running' }, { type: 'STOP' });
    expect(state.phase).toBe('rectangle');
    expect(state.connectionStatus).toBe('connected');
    expect(state.compatible).toBe(true);
  });
});

describe('resuming a faulted mission', () => {
  function faultedState() {
    return reduce(
      enteredReadyState(),
      { type: 'REQUEST_ARM' },
      { type: 'ARM_ACKNOWLEDGED' },
      { type: 'REQUEST_START' },
      { type: 'START_ACKNOWLEDGED' },
      { type: 'MISSION_FAULT', cause: 'pose timeout' },
    );
  }

  it('keeps the rectangle and returns to readiness on the chosen pass', () => {
    const faulted = faultedState();
    const resumed = setupReducer(faulted, { type: 'RESUME_MISSION', passIndex: 3 });
    expect(resumed.phase).toBe('readiness');
    expect(resumed.resumePassIndex).toBe(3);
    expect(resumed.rectangle).toBe(faulted.rectangle);
    expect(resumed.fault).toBeNull();
    expect(resumed.readiness).toEqual({ trackingNormal: false, poseStable: false, atStart: false });
  });

  it('lets the rover own the start-position check while resuming', () => {
    const resumed = reduce(
      faultedState(),
      { type: 'RESUME_MISSION', passIndex: 2 },
      { type: 'SET_READINESS', trackingNormal: true, poseStable: true, atStart: false },
      { type: 'REQUEST_ARM' },
    );
    expect(resumed.phase).toBe('arming');

    const fromStart = reduce(
      faultedState(),
      { type: 'RESUME_MISSION', passIndex: 0 },
      { type: 'SET_READINESS', trackingNormal: true, poseStable: true, atStart: false },
      { type: 'REQUEST_ARM' },
    );
    expect(fromStart.phase).toBe('readiness');
    expect(fromStart.validationError).toMatch(/rectangle start/i);
  });

  it('refuses to resume mid-mission or without a rectangle', () => {
    const running = setupReducer(
      { ...enteredReadyState(), phase: 'running' },
      { type: 'RESUME_MISSION', passIndex: 1 },
    );
    expect(running.phase).toBe('running');
    expect(running.validationError).toMatch(/only available/i);

    const noRectangle = setupReducer(
      { ...initialSetupState(), phase: 'fault' },
      { type: 'RESUME_MISSION', passIndex: 1 },
    );
    expect(noRectangle.phase).toBe('fault');
    expect(noRectangle.validationError).toMatch(/rectangle/i);
  });

  it('returns to readiness when the rover refuses the Arm', () => {
    const refused = reduce(
      faultedState(),
      { type: 'RESUME_MISSION', passIndex: 2 },
      { type: 'SET_READINESS', trackingNormal: true, poseStable: true, atStart: false },
      { type: 'REQUEST_ARM' },
      { type: 'ARM_REFUSED', reason: 'not on that pass' },
    );
    expect(refused.phase).toBe('readiness');
    expect(refused.validationError).toBe('not on that pass');
    expect(refused.resumePassIndex).toBe(2);
  });

  it('drops the resume pass on a full Stop so the next mission starts over', () => {
    const resumed = setupReducer(faultedState(), { type: 'RESUME_MISSION', passIndex: 4 });
    expect(setupReducer(resumed, { type: 'STOP' }).resumePassIndex).toBe(0);
  });
});

describe('MissionOperationGate', () => {
  it('invalidates stale async workflows when Stop begins', () => {
    const gate = new MissionOperationGate();
    const stale = gate.begin();
    expect(gate.isCurrent(stale)).toBe(true);
    gate.cancel();
    expect(gate.isCurrent(stale)).toBe(false);
    expect(() => gate.assertCurrent(stale)).toThrow(/cancelled/i);
    expect(gate.isCurrent(gate.begin())).toBe(true);
  });
});

describe('isAuthoritativeLogReady', () => {
  it('rejects a retained logger object after its append pipeline has failed', () => {
    expect(isAuthoritativeLogReady({ failed: false })).toBe(true);
    expect(isAuthoritativeLogReady({ failed: true })).toBe(false);
    expect(isAuthoritativeLogReady(null)).toBe(false);
  });
});

function faultPacket(sample: Partial<FaultSampleV2> & Pick<FaultSampleV2, 'sampleIndex' | 'sampleCount'>) {
  const bytes = new Uint8Array(32);
  const view = new DataView(bytes.buffer);
  const index = sample.sampleIndex;
  const count = sample.sampleCount;
  bytes.set([0x21, 0x42, 2, sample.flags ?? ((index === 0 ? 1 : 0) | (index + 1 === count ? 2 : 0))]);
  view.setUint16(4, sample.epoch ?? 7, true);
  view.setUint32(6, sample.sequence ?? 100 + index, true);
  view.setUint16(10, index, true);
  view.setUint16(12, count, true);
  view.setUint16(14, sample.routeIndex ?? index, true);
  view.setInt16(16, Math.round((sample.crossTrackFt ?? 0.1) * 100), true);
  view.setInt16(18, Math.round((sample.headingErrorDeg ?? 1) * 100), true);
  view.setInt16(20, Math.round((sample.speedFps ?? 0.5) * 100), true);
  view.setUint16(22, sample.steeringUs ?? 1709, true);
  view.setUint16(24, sample.throttleUs ?? 1620, true);
  bytes[26] = sample.state ?? 5;
  bytes[27] = sample.faultCode ?? 2;
  view.setUint16(28, sample.droppedPackets ?? 0, true);
  view.setUint16(30, crc16Ccitt(bytes.subarray(0, 30)), true);
  return bytes;
}

describe('assembleFaultPackets', () => {
  it('validates and reorders a complete out-of-order dump', () => {
    const assembled = assembleFaultPackets([
      faultPacket({ sampleIndex: 2, sampleCount: 3 }),
      faultPacket({ sampleIndex: 0, sampleCount: 3 }),
      faultPacket({ sampleIndex: 1, sampleCount: 3 }),
    ], 7);
    expect(assembled.map((sample) => sample.sampleIndex)).toEqual([0, 1, 2]);
    expect(assembled[0].flags & 1).toBe(1);
    expect(assembled[2].flags & 2).toBe(2);
  });

  it('rejects missing chunks', () => {
    expect(() => assembleFaultPackets([
      faultPacket({ sampleIndex: 0, sampleCount: 3 }),
      faultPacket({ sampleIndex: 2, sampleCount: 3 }),
    ], 7)).toThrow(/incomplete/i);
  });

  it('rejects a packet whose CRC is corrupt', () => {
    const packet = faultPacket({ sampleIndex: 0, sampleCount: 1 });
    packet[18] ^= 1;
    expect(() => assembleFaultPackets([packet], 7)).toThrow(/invalid/i);
  });
});
