import { crc16Ccitt, FaultSampleV2 } from './protocolV2';
import { estimateHeadland } from './routePlan';
import {
  assembleFaultPackets,
  initialSetupState,
  isAuthoritativeLogReady,
  MissionOperationGate,
  ROVER_HEADLAND_WARN_FT,
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
    {
      type: 'SET_ENTERED_RECTANGLE',
      pose: atA,
      mFt: 20,
      nFt: 8,
      side: 'right',
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

  it('requires the rover be at the rectangle start before arming', () => {
    const state = reduce(
      enteredReadyState(),
      {
        type: 'SET_READINESS',
        trackingNormal: true,
        poseStable: true,
        atStart: false,
      },
      { type: 'REQUEST_ARM' },
    );
    expect(state.phase).toBe('readiness');
    expect(state.validationError).toMatch(/rectangle start/i);
  });

  describe('MOUNT_CALIBRATION_CHANGED', () => {
    it('drops the rectangle and returns to rectangle setup from readiness', () => {
      const state = setupReducer(enteredReadyState(), { type: 'MOUNT_CALIBRATION_CHANGED' });
      expect(state.phase).toBe('rectangle');
      expect(state.rectangle).toBeNull();
      expect(state.warning).toMatch(/set the rectangle again/i);
    });

    it('is a no-op without a rectangle or during a mission', () => {
      const idle = connectedState();
      expect(setupReducer(idle, { type: 'MOUNT_CALIBRATION_CHANGED' })).toBe(idle);
      const running = { ...enteredReadyState(), phase: 'running' as const };
      expect(setupReducer(running, { type: 'MOUNT_CALIBRATION_CHANGED' })).toBe(running);
    });
  });

  it('rejects invalid dimensions before leaving rectangle setup', () => {
    let state = reduce(
      connectedState(),
      {
        type: 'SET_ENTERED_RECTANGLE',
        pose: atA,
        mFt: 0,
        nFt: 8,
        side: 'right',
      },
      { type: 'CONTINUE' },
    );
    expect(state.phase).toBe('rectangle');
    expect(state.rectangle).toBeNull();
    expect(state.validationError).toBeTruthy();
  });

  it('plans the headland itself instead of asking the operator', () => {
    const state = enteredReadyState();
    const planned = estimateHeadland(20, 8);
    expect(state.rectangle).toMatchObject({
      startClearFt: planned?.beforeStartFt,
      endClearFt: planned?.beyondEndFt,
      headlandSource: 'estimated',
    });
  });

  describe('ROVER_HEADLAND', () => {
    it('adopts the rover-reported requirement after configure without a warning when it matches', () => {
      const armed = setupReducer(enteredReadyState(), { type: 'REQUEST_ARM' });
      const estimate = armed.rectangle!;
      const state = setupReducer(armed, {
        type: 'ROVER_HEADLAND',
        beforeStartFt: estimate.startClearFt + 0.1,
        beyondEndFt: estimate.endClearFt,
      });
      expect(state.phase).toBe('arming');
      expect(state.rectangle).toMatchObject({
        startClearFt: estimate.startClearFt + 0.1,
        endClearFt: estimate.endClearFt,
        headlandSource: 'rover',
      });
      expect(state.warning).toBeNull();
    });

    it('warns when the rover needs more room than the preview estimated', () => {
      const armed = setupReducer(enteredReadyState({ loggingReady: false }), { type: 'REQUEST_ARM' });
      const estimate = armed.rectangle!;
      const state = setupReducer(armed, {
        type: 'ROVER_HEADLAND',
        beforeStartFt: estimate.startClearFt + ROVER_HEADLAND_WARN_FT + 1,
        beyondEndFt: estimate.endClearFt,
      });
      expect(state.rectangle?.headlandSource).toBe('rover');
      expect(state.warning).toMatch(/re-check/i);
      expect(state.warning).toMatch(/log/i);
    });

    it('ignores a rover headland line when no rectangle is defined', () => {
      const before = connectedState();
      const after = setupReducer(before, { type: 'ROVER_HEADLAND', beforeStartFt: 5, beyondEndFt: 5 });
      expect(after).toBe(before);
    });

    it('reports malformed rover figures as a validation error', () => {
      const state = setupReducer(enteredReadyState(), {
        type: 'ROVER_HEADLAND',
        beforeStartFt: -2,
        beyondEndFt: 5,
      });
      expect(state.rectangle?.headlandSource).toBe('estimated');
      expect(state.validationError).toMatch(/rover headland/i);
    });
  });

  it('requires explicit confirmation for an entered LEFT coverage side', () => {
    let state = reduce(
      connectedState(),
      {
        type: 'SET_ENTERED_RECTANGLE',
        pose: atA,
        mFt: 20,
        nFt: 8,
        side: 'left',
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
    'blocks wet arming for %s calibration but permits dry diagnostics',
    (status) => {
      let wet = enteredReadyState({ wet: true });
      wet = reduce(
        wet,
        { type: 'SET_CALIBRATION_STATUS', status },
        { type: 'REQUEST_ARM' },
      );
      expect(wet.phase).toBe('readiness');
      expect(wet.validationError).toMatch(/calibration/i);

      let dry = enteredReadyState();
      dry = reduce(
        dry,
        { type: 'SET_CALIBRATION_STATUS', status },
        { type: 'REQUEST_ARM' },
      );
      expect(dry.phase).toBe('arming');
    },
  );

  it('blocks wet arming when logging fails but permits a warned dry diagnostic', () => {
    const wet = setupReducer(
      enteredReadyState({ wet: true, loggingReady: false }),
      { type: 'REQUEST_ARM' },
    );
    expect(wet.phase).toBe('readiness');
    expect(wet.validationError).toMatch(/log/i);

    const dry = setupReducer(
      enteredReadyState({ wet: false, loggingReady: false }),
      { type: 'REQUEST_ARM' },
    );
    expect(dry.phase).toBe('arming');
    expect(dry.warning).toMatch(/log/i);
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
    'connection', 'rectangle', 'readiness',
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
