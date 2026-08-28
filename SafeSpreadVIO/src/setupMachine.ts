import { ConnectionStatus } from './ble';
import { FaultSampleV2, parseFaultSampleV2 } from './protocolV2';
import {
  CoverageSide,
  defineEnteredRectangle,
  RectangleDefinition,
  withRoverHeadland,
} from './rectangle';
import { Pose } from './poseMath';

/** How much more than the preview estimate the rover may ask for before the
 *  operator is told to re-check the coned area. */
export const ROVER_HEADLAND_WARN_FT = 0.25;

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

export type CalibrationStatus = 'missing' | 'stale' | 'ready';

export interface SetupReadiness {
  trackingNormal: boolean;
  poseStable: boolean;
  atStart: boolean;
}

export interface SetupState {
  phase: SetupPhase;
  connectionStatus: ConnectionStatus;
  compatible: boolean;
  rectangle: RectangleDefinition | null;
  coverageSideConfirmed: boolean;
  calibrationStatus: CalibrationStatus;
  wet: boolean;
  loggingReady: boolean;
  readiness: SetupReadiness;
  validationError: string | null;
  warning: string | null;
  fault: string | null;
}

export type SetupAction =
  | { type: 'CONNECTION_CHANGED'; status: ConnectionStatus; compatible: boolean }
  | {
      type: 'SET_ENTERED_RECTANGLE';
      pose: Pose;
      mFt: number;
      nFt: number;
      side: CoverageSide;
    }
  | { type: 'CONFIRM_COVERAGE_SIDE' }
  /** The camera-mount numbers changed, so poses now come through a different
   *  transform and any stored rectangle origin no longer matches. */
  | { type: 'MOUNT_CALIBRATION_CHANGED' }
  /** The rover's own headland requirement, logged after it plans the route. */
  | { type: 'ROVER_HEADLAND'; beforeStartFt: number; beyondEndFt: number }
  | { type: 'SET_CALIBRATION_STATUS'; status: CalibrationStatus }
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
    rectangle: null,
    coverageSideConfirmed: false,
    calibrationStatus: 'missing',
    wet: false,
    loggingReady: false,
    readiness: { ...NOT_READY },
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
    calibrationStatus: state.calibrationStatus,
  };
}

function canArm(state: SetupState): string | null {
  if (!state.rectangle) return 'Define and confirm the rectangle before arming.';
  if (!state.readiness.trackingNormal) return 'ARKit tracking must be normal before arming.';
  if (!state.readiness.poseStable) return 'Wait for a stable pose before arming.';
  if (!state.readiness.atStart) {
    return 'Move the rover to the rectangle start before arming.';
  }
  if (state.wet && state.calibrationStatus !== 'ready') {
    return 'Wet operation requires a current matching calibration.';
  }
  if (state.wet && !state.loggingReady) {
    return 'Wet operation requires a writable mission log.';
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

    case 'SET_ENTERED_RECTANGLE':
      if (state.phase !== 'rectangle') {
        return fail(state, 'Define the rectangle during setup.');
      }
      try {
        return {
          ...state,
          rectangle: defineEnteredRectangle(action.pose, action.mFt, action.nFt, action.side),
          coverageSideConfirmed: action.side === 'right',
          validationError: null,
        };
      } catch (error) {
        return fail(state, error instanceof Error ? error.message : String(error));
      }

    case 'CONFIRM_COVERAGE_SIDE':
      if (!state.rectangle) return fail(state, 'Define the rectangle before confirming its side.');
      return { ...state, coverageSideConfirmed: true, validationError: null };

    case 'MOUNT_CALIBRATION_CHANGED':
      // The rectangle origin was captured through the old mount transform;
      // with new numbers the computed rover pose shifts, so "at start" would
      // silently disagree with the stored origin (seen in the field as a
      // spurious "move the rover to the rectangle start"). Re-set it.
      if (!state.rectangle || !['rectangle', 'readiness'].includes(state.phase)) return state;
      return {
        ...state,
        phase: 'rectangle',
        rectangle: null,
        coverageSideConfirmed: false,
        warning: 'Mount calibration changed — set the rectangle again so its origin uses the new numbers.',
        validationError: null,
      };

    case 'ROVER_HEADLAND': {
      // A stray line with no rectangle in play carries nothing to update.
      if (!state.rectangle) return state;
      let rectangle: RectangleDefinition;
      try {
        rectangle = withRoverHeadland(state.rectangle, action);
      } catch (error) {
        return fail(state, error instanceof Error ? error.message : String(error));
      }
      const previous = state.rectangle;
      const needsMore = previous.headlandSource === 'estimated' && (
        action.beforeStartFt > previous.startClearFt + ROVER_HEADLAND_WARN_FT ||
        action.beyondEndFt > previous.endClearFt + ROVER_HEADLAND_WARN_FT);
      const headlandWarning = needsMore
        ? `Rover needs ${action.beforeStartFt.toFixed(1)} ft behind A and ${action.beyondEndFt.toFixed(1)} ft beyond M — more than the preview estimated (${previous.startClearFt.toFixed(1)} / ${previous.endClearFt.toFixed(1)} ft). Re-check the clear pavement before Start.`
        : null;
      return {
        ...state,
        rectangle,
        warning: headlandWarning && state.warning
          ? `${state.warning} ${headlandWarning}`
          : headlandWarning ?? state.warning,
      };
    }

    case 'SET_CALIBRATION_STATUS':
      return { ...state, calibrationStatus: action.status, validationError: null };

    case 'SET_WET_MODE':
      return { ...state, wet: action.wet, validationError: null };

    case 'SET_LOGGING_READY':
      return { ...state, loggingReady: action.ready, validationError: null };

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
        // Calibration is no longer a forced step: the stored mount/pavement
        // calibration carries between runs and is exercised from the
        // Diagnostics panel only when something changes. Wet operation still
        // requires a current calibration, enforced at Arm (see canArm).
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
        warning: state.loggingReady ? null : 'Mission log unavailable; dry diagnostic only.',
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
