import { ConnectionStatus } from './ble';
import { FaultSampleV2, parseFaultSampleV2 } from './protocolV2';
import {
  captureCornerA,
  CornerA,
  CoverageSide,
  defineEnteredRectangle,
  defineWalkedRectangle,
  RectangleDefinition,
} from './rectangle';
import { Pose } from './poseMath';

export type SetupPhase =
  | 'connection'
  | 'rectangle'
  | 'readiness'
  | 'arming'
  | 'armed'
  | 'starting'
  | 'running'
  | 'complete'
  | 'fault';

export type RectangleMode = 'entered' | 'walked';

export interface SetupReadiness {
  trackingNormal: boolean;
  poseStable: boolean;
  atStart: boolean;
}

export interface SetupState {
  phase: SetupPhase;
  connectionStatus: ConnectionStatus;
  compatible: boolean;
  rectangleMode: RectangleMode | null;
  rectangle: RectangleDefinition | null;
  cornerA: CornerA | null;
  coverageSideConfirmed: boolean;
  wet: boolean;
  loggingReady: boolean;
  readiness: SetupReadiness;
  // Sprayed pass this mission begins on, counted in route order from zero. A
  // fault used to cost the whole rectangle: the operator can now re-drive only
  // the passes that are left. The rover checks the rover is actually standing
  // there before it will arm, because only the rover knows the route order.
  resumePassIndex: number;
  validationError: string | null;
  warning: string | null;
  fault: string | null;
}

export type SetupAction =
  | { type: 'CONNECTION_CHANGED'; status: ConnectionStatus; compatible: boolean }
  | { type: 'SELECT_RECTANGLE_MODE'; mode: RectangleMode }
  | {
      type: 'SET_ENTERED_RECTANGLE';
      pose: Pose;
      mFt: number;
      nFt: number;
      side: CoverageSide;
    }
  | { type: 'CAPTURE_CORNER_A'; pose: Pose; stable: boolean }
  | {
      type: 'CAPTURE_CORNER_B';
      pose: Pose;
      stable: boolean;
    }
  | { type: 'CONFIRM_COVERAGE_SIDE' }
  | { type: 'SET_WET_MODE'; wet: boolean }
  | { type: 'SET_LOGGING_READY'; ready: boolean }
  | { type: 'SET_READINESS'; trackingNormal: boolean; poseStable: boolean; atStart: boolean }
  | { type: 'CONTINUE' }
  | { type: 'REQUEST_ARM' }
  | { type: 'ARM_ACKNOWLEDGED' }
  | { type: 'REQUEST_START' }
  | { type: 'START_ACKNOWLEDGED' }
  | { type: 'MISSION_COMPLETE' }
  | { type: 'MISSION_FAULT'; cause: string }
  | { type: 'RESUME_MISSION'; passIndex: number }
  | { type: 'ARM_REFUSED'; reason: string }
  | { type: 'ACK_TIMEOUT'; operation: string }
  | { type: 'STOP' };

export class MissionOperationGate {
  private generation = 0;

  begin(): number {
    this.generation += 1;
    return this.generation;
  }

  cancel(): void {
    this.generation += 1;
  }

  isCurrent(generation: number): boolean {
    return generation === this.generation;
  }

  assertCurrent(generation: number): void {
    if (!this.isCurrent(generation)) throw new Error('mission operation was cancelled by Stop');
  }
}

export function isAuthoritativeLogReady(logger: { failed: boolean } | null): boolean {
  return Boolean(logger && !logger.failed);
}

const NOT_READY: SetupReadiness = {
  trackingNormal: false,
  poseStable: false,
  atStart: false,
};

export function initialSetupState(): SetupState {
  return {
    phase: 'connection',
    connectionStatus: 'disconnected',
    compatible: false,
    rectangleMode: 'entered',
    rectangle: null,
    cornerA: null,
    coverageSideConfirmed: false,
    wet: false,
    loggingReady: false,
    readiness: { ...NOT_READY },
    resumePassIndex: 0,
    validationError: null,
    warning: null,
    fault: null,
  };
}

function fail(state: SetupState, message: string): SetupState {
  return { ...state, validationError: message };
}

function stoppedState(state: SetupState): SetupState {
  const reset = initialSetupState();
  const connected = state.connectionStatus === 'connected' && state.compatible;
  return {
    ...reset,
    phase: connected ? 'rectangle' : 'connection',
    connectionStatus: state.connectionStatus,
    compatible: connected,
  };
}

function canArm(state: SetupState): string | null {
  if (!state.rectangle) return 'Define and confirm the rectangle before arming.';
  if (!state.readiness.trackingNormal) return 'ARKit tracking must be normal before arming.';
  if (!state.readiness.poseStable) return 'Wait for a stable pose before arming.';
  // A resumed mission starts on a lane in the middle of the rectangle, not at
  // the origin this check knows about; the rover refuses the Arm itself if the
  // operator has not put it on that lane.
  if (!state.readiness.atStart && state.resumePassIndex === 0) {
    return state.rectangleMode === 'walked'
      ? 'Return the rover to Corner A before arming.'
      : 'Move the rover to the rectangle start before arming.';
  }
  return null;
}

export function setupReducer(state: SetupState, action: SetupAction): SetupState {
  if (action.type === 'STOP') return stoppedState(state);

  switch (action.type) {
    case 'CONNECTION_CHANGED':
      return {
        ...state,
        connectionStatus: action.status,
        compatible: action.status === 'connected' && action.compatible,
        validationError: null,
      };

    case 'SELECT_RECTANGLE_MODE':
      if (state.phase !== 'rectangle') return fail(state, 'Rectangle mode can only change during setup.');
      return {
        ...state,
        rectangleMode: action.mode,
        rectangle: null,
        cornerA: null,
        coverageSideConfirmed: false,
        validationError: null,
      };

    case 'SET_ENTERED_RECTANGLE':
      if (state.phase !== 'rectangle' || state.rectangleMode !== 'entered') {
        return fail(state, 'Select entered-dimensions mode first.');
      }
      try {
        return {
          ...state,
          rectangle: defineEnteredRectangle(
            action.pose,
            action.mFt,
            action.nFt,
            action.side,
          ),
          coverageSideConfirmed: action.side === 'right',
          validationError: null,
        };
      } catch (error) {
        return fail(state, error instanceof Error ? error.message : String(error));
      }

    case 'CAPTURE_CORNER_A':
      if (state.phase !== 'rectangle' || state.rectangleMode !== 'walked') {
        return fail(state, 'Select walked-corners mode first.');
      }
      try {
        return {
          ...state,
          cornerA: captureCornerA(action.pose, action.stable),
          rectangle: null,
          coverageSideConfirmed: false,
          validationError: null,
        };
      } catch (error) {
        return fail(state, error instanceof Error ? error.message : String(error));
      }

    case 'CAPTURE_CORNER_B':
      if (state.phase !== 'rectangle' || state.rectangleMode !== 'walked' || !state.cornerA) {
        return fail(state, 'Capture stable Corner A before Corner B.');
      }
      try {
        const rectangle = defineWalkedRectangle(
          state.cornerA,
          action.pose,
          action.stable,
        );
        return {
          ...state,
          rectangle,
          coverageSideConfirmed: rectangle.side === 'right',
          validationError: null,
        };
      } catch (error) {
        return fail(state, error instanceof Error ? error.message : String(error));
      }

    case 'CONFIRM_COVERAGE_SIDE':
      if (!state.rectangle) return fail(state, 'Define the rectangle before confirming its side.');
      return { ...state, coverageSideConfirmed: true, validationError: null };

    case 'SET_WET_MODE':
      return { ...state, wet: action.wet, validationError: null };

    case 'SET_LOGGING_READY':
      return {
        ...state,
        loggingReady: action.ready,
        validationError: null,
        warning: action.ready ? null : 'Mission log unavailable; operation continues without a log.',
      };

    case 'SET_READINESS':
      return {
        ...state,
        readiness: {
          trackingNormal: action.trackingNormal,
          poseStable: action.poseStable,
          atStart: action.atStart,
        },
      };

    case 'CONTINUE':
      if (state.phase === 'connection') {
        if (state.connectionStatus !== 'connected' || !state.compatible) {
          return fail(state, 'Protocol v2 compatible firmware must be connected.');
        }
        return { ...state, phase: 'rectangle', validationError: null };
      }
      if (state.phase === 'rectangle') {
        if (!state.rectangle) return fail(state, 'Define a valid rectangle before continuing.');
        if (state.rectangle.side === 'left' && !state.coverageSideConfirmed) {
          return fail(state, 'Left coverage side must be explicitly confirmed.');
        }
        return { ...state, phase: 'readiness', validationError: null };
      }
      return fail(state, 'Continue is not available in the current phase.');

    case 'REQUEST_ARM': {
      if (state.phase !== 'readiness') return fail(state, 'Arm is only available after readiness checks.');
      const reason = canArm(state);
      if (reason) return fail(state, reason);
      return {
        ...state,
        phase: 'arming',
        validationError: null,
        warning: state.loggingReady ? null : 'Mission log unavailable; operation continues without a log.',
      };
    }

    case 'ARM_ACKNOWLEDGED':
      return state.phase === 'arming'
        ? { ...state, phase: 'armed', validationError: null }
        : fail(state, 'Unexpected Arm acknowledgement.');

    case 'REQUEST_START':
      return state.phase === 'armed'
        ? { ...state, phase: 'starting', validationError: null }
        : fail(state, 'Start requires an Armed acknowledgement.');

    case 'START_ACKNOWLEDGED':
      return state.phase === 'starting'
        ? { ...state, phase: 'running', validationError: null }
        : fail(state, 'Unexpected Start acknowledgement.');

    case 'MISSION_COMPLETE':
      return state.phase === 'running'
        ? { ...state, phase: 'complete', validationError: null }
        : fail(state, 'Completion is only valid for a running mission.');

    case 'MISSION_FAULT':
      return { ...state, phase: 'fault', fault: action.cause, validationError: null };

    case 'RESUME_MISSION': {
      if (state.phase !== 'fault' && state.phase !== 'complete') {
        return fail(state, 'Resume is only available after a fault or completion.');
      }
      if (!state.rectangle) return fail(state, 'The rectangle is no longer defined; start over.');
      if (!Number.isInteger(action.passIndex) || action.passIndex < 0) {
        return fail(state, 'Choose which pass to resume from.');
      }
      // The rectangle is kept exactly as it was. Re-entering it would anchor a
      // new origin on wherever the rover now sits, and every remaining pass
      // would be laid down in the wrong place.
      return {
        ...state,
        phase: 'readiness',
        resumePassIndex: action.passIndex,
        readiness: { ...NOT_READY },
        fault: null,
        validationError: null,
      };
    }

    // The rover can refuse an Arm without faulting -- it stays configured, so
    // the operator fixes what it complained about and arms again.
    case 'ARM_REFUSED':
      return state.phase === 'arming'
        ? { ...state, phase: 'readiness', validationError: action.reason }
        : fail(state, action.reason);

    case 'ACK_TIMEOUT':
      return {
        ...state,
        phase: 'fault',
        fault: `${action.operation} acknowledgement timeout`,
        validationError: null,
      };
  }
}

export function assembleFaultPackets(
  packets: Uint8Array[],
  expectedEpoch: number,
): FaultSampleV2[] {
  if (packets.length === 0) throw new Error('fault dump is incomplete');
  const samples = packets.map((packet) => {
    const parsed = parseFaultSampleV2(packet);
    if (!parsed) throw new Error('invalid fault packet');
    if (parsed.epoch !== expectedEpoch) throw new Error('fault packet epoch does not match mission');
    return parsed;
  });
  const expectedCount = samples[0].sampleCount;
  if (expectedCount < 1 || expectedCount > 4096 ||
      samples.some((sample) => sample.sampleCount !== expectedCount) ||
      samples.length !== expectedCount) {
    throw new Error('fault dump is incomplete');
  }
  const byIndex = new Map<number, FaultSampleV2>();
  for (const sample of samples) {
    if (sample.sampleIndex >= expectedCount || byIndex.has(sample.sampleIndex)) {
      throw new Error('fault dump has an invalid or duplicate sample index');
    }
    byIndex.set(sample.sampleIndex, sample);
  }
  const ordered = Array.from({ length: expectedCount }, (_, index) => byIndex.get(index));
  if (ordered.some((sample) => !sample)) throw new Error('fault dump is incomplete');
  const complete = ordered as FaultSampleV2[];
  if ((complete[0].flags & 1) === 0 || (complete[expectedCount - 1].flags & 2) === 0) {
    throw new Error('fault dump boundary flags are invalid');
  }
  return complete;
}
