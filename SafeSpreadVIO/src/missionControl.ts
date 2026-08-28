import {
  AckV2,
  buildCalibrationV2,
  buildCommandV2,
  buildRectangleV2,
  parseAckV2,
  RoutePlanV2,
} from './protocolV2';
import { RectangleDefinition } from './rectangle';

export interface CalibrationWire {
  id: number;
  schemaVersion: number;
  sprayForwardFt: number;
  sprayRightFt: number;
}

export interface MissionTransport {
  writeWithResponse(packet: Uint8Array): Promise<void>;
  writeCompatibilityStop(): Promise<void>;
  subscribeAck(listener: (packet: Uint8Array) => void): () => void;
  subscribeRoutePlan(listener: (plan: RoutePlanV2) => void): () => void;
}

export type MissionClientState = 'idle' | 'configured' | 'armed' | 'running' | 'complete' | 'fault';

interface MissionControlOptions {
  timeoutMs?: number;
  retries?: number;
  dryMode?: boolean;
}

interface PendingAck {
  commandId: number;
  received: AckV2 | null;
  listeners: Set<(ack: AckV2) => void>;
}

class MissionOperationCancelled extends Error {
  constructor() {
    super('mission operation cancelled by Stop');
  }
}

/** Firmware fault 13: the rover is not standing on the pass it was asked to
 *  resume from. The mission stays configured, so the operator moves the rover
 *  and arms again rather than starting over. */
export const FAULT_START_POINT = 13;

/** A refusal the operator can act on, as opposed to a mission fault. The
 *  client keeps its state so the same command can simply be retried. */
export class MissionCommandRefused extends Error {
  constructor(readonly faultCode: number, message: string) {
    super(message);
  }
}

export class MissionControl {
  private currentState: MissionClientState = 'idle';
  private nextCommandId = 1;
  private pending: PendingAck | null = null;
  private busy = false;
  private readonly unsubscribe: () => void;
  private readonly unsubscribeRoutePlan: () => void;
  private readonly timeoutMs: number;
  private readonly retries: number;
  private readonly dryMode: boolean;
  private calibrationId = 0;
  private preparedCalibration: CalibrationWire | null = null;
  private receivedRoutePlan: RoutePlanV2 | null = null;
  private operationGeneration = 0;

  constructor(
    private readonly transport: MissionTransport,
    private readonly epoch: number,
    private readonly hasFreshPose: () => boolean,
    options: MissionControlOptions = {},
  ) {
    if (!Number.isInteger(epoch) || epoch < 0 || epoch > 0xffff) {
      throw new RangeError('mission epoch must be uint16');
    }
    this.timeoutMs = options.timeoutMs ?? 750;
    this.retries = options.retries ?? 2;
    this.dryMode = options.dryMode ?? true;
    this.unsubscribe = transport.subscribeAck((packet) => this.receiveAck(packet));
    this.unsubscribeRoutePlan = transport.subscribeRoutePlan((plan) => {
      if (plan.epoch === this.epoch) this.receivedRoutePlan = plan;
    });
  }

  get state(): MissionClientState {
    return this.currentState;
  }

  get routePlan(): RoutePlanV2 | null {
    return this.receivedRoutePlan;
  }

  /** `startPassIndex` resumes a faulted mission on a later pass; zero drives
   *  the whole rectangle. The rover validates it against the plan it builds. */
  async configure(
    definition: RectangleDefinition,
    calibration: CalibrationWire,
    startPassIndex = 0,
  ): Promise<AckV2> {
    if (this.currentState !== 'idle') throw new Error('mission must be idle before configure');
    return this.exclusive(async (generation) => {
      this.calibrationId = calibration.id;
      if (!this.samePreparedCalibration(calibration)) {
        const calibrationCommandId = this.takeCommandId();
        const calibrationAck = await this.sendAndWait(buildCalibrationV2({
          flags: 0,
          epoch: this.epoch,
          commandId: calibrationCommandId,
          calibrationId: calibration.id,
          sprayForwardFt: calibration.sprayForwardFt,
          sprayRightFt: calibration.sprayRightFt,
          schemaVersion: calibration.schemaVersion,
        }), calibrationCommandId, generation);
        this.requireAck(calibrationAck, [0, 1], calibration.id);
        this.preparedCalibration = { ...calibration };
      }

      const rectangleCommandId = this.takeCommandId();
      this.receivedRoutePlan = null;
      const flags = (definition.side === 'left' ? 1 : 0) |
        (this.dryMode ? 2 : 0);
      const rectangleAck = await this.sendAndWait(buildRectangleV2({
        flags,
        epoch: this.epoch,
        commandId: rectangleCommandId,
        mFt: definition.mFt,
        nFt: definition.nFt,
        // Protocol-v2 retains these two fields for wire compatibility. The
        // rover now computes the exact required headland from its measured
        // radii and footprint; the operator no longer supplies either value.
        startClearFt: 0,
        endClearFt: 0,
        calibrationId: calibration.id,
        startPassIndex,
      }), rectangleCommandId, generation);
      this.requireAck(rectangleAck, [1], calibration.id);
      await this.waitForRoutePlan(generation);
      this.currentState = 'configured';
      return rectangleAck;
    });
  }

  async prepareDiagnostics(calibration: CalibrationWire): Promise<AckV2> {
    if (this.currentState !== 'idle') throw new Error('mission must be idle before calibration setup');
    return this.exclusive(async (generation) => {
      this.calibrationId = calibration.id;
      const commandId = this.takeCommandId();
      const acknowledgement = await this.sendAndWait(buildCalibrationV2({
        flags: 0,
        epoch: this.epoch,
        commandId,
        calibrationId: calibration.id,
        sprayForwardFt: calibration.sprayForwardFt,
        sprayRightFt: calibration.sprayRightFt,
        schemaVersion: calibration.schemaVersion,
      }), commandId, generation);
      this.requireAck(acknowledgement, [0], calibration.id);
      this.preparedCalibration = { ...calibration };
      return acknowledgement;
    });
  }

  async measureTurningRadii(): Promise<AckV2> {
    if (!this.dryMode) throw new Error('turn-radius measurement is dry-only');
    if (this.currentState !== 'idle' || !this.preparedCalibration) {
      throw new Error('diagnostics must be prepared while idle');
    }
    if (!this.hasFreshPose()) throw new Error('a fresh valid pose is required for radius measurement');
    return this.command(5, [0], 'idle');
  }

  async selfTest(): Promise<AckV2> {
    if (this.currentState !== 'idle' || !this.preparedCalibration) {
      throw new Error('calibration epoch must be prepared before self-test');
    }
    return this.command(4, [0], 'idle');
  }

  async dumpFault(): Promise<AckV2> {
    if (this.currentState !== 'idle' && this.currentState !== 'fault') {
      throw new Error('Stop before requesting the fault dump');
    }
    return this.command(8, [0, 5], this.currentState, false);
  }

  async arm(): Promise<AckV2> {
    if (this.currentState !== 'configured') throw new Error('mission must be configured before arm');
    if (!this.hasFreshPose()) throw new Error('a fresh valid pose is required before arm');
    return this.command(1, [2], 'armed', true, [FAULT_START_POINT]);
  }

  async start(): Promise<AckV2> {
    if (this.currentState !== 'armed') throw new Error('mission must be armed before start');
    return this.command(2, [3], 'running');
  }

  async stop(): Promise<AckV2> {
    if (this.busy) return this.priorityStop();
    let acknowledgement: AckV2 | null = null;
    try {
      acknowledgement = await this.command(3, [0], 'idle', false);
      this.preparedCalibration = null;
      this.calibrationId = 0;
      return acknowledgement;
    } finally {
      try {
        await this.transport.writeCompatibilityStop();
      } catch {
        // The v2 Stop result remains authoritative; this is a best-effort belt-and-suspenders Stop.
      }
    }
  }

  notifyDisconnect(): void {
    if (this.currentState !== 'idle') this.currentState = 'fault';
  }

  dispose(): void {
    this.unsubscribe();
    this.unsubscribeRoutePlan();
  }

  private waitForRoutePlan(generation: number): Promise<RoutePlanV2> {
    if (this.receivedRoutePlan) return Promise.resolve(this.receivedRoutePlan);
    return new Promise((resolve, reject) => {
      const startedAt = Date.now();
      const poll = () => {
        try {
          this.ensureCurrent(generation);
        } catch (error) {
          reject(error);
          return;
        }
        if (this.receivedRoutePlan) {
          resolve(this.receivedRoutePlan);
          return;
        }
        if (Date.now() - startedAt >= this.timeoutMs) {
          reject(new Error('route plan report timeout'));
          return;
        }
        setTimeout(poll, 10);
      };
      poll();
    });
  }

  private async command(
    opcode: number,
    expectedStates: number[],
    nextState: MissionClientState,
    validateCalibration = true,
    retryableFaults: number[] = [],
  ): Promise<AckV2> {
    return this.exclusive(async (generation) => {
      const commandId = this.takeCommandId();
      const ack = await this.sendAndWait(
        buildCommandV2({ opcode, epoch: this.epoch, commandId }),
        commandId,
        generation,
      );
      this.requireAck(ack, expectedStates, validateCalibration ? this.calibrationId : null,
                      retryableFaults);
      this.currentState = nextState;
      return ack;
    });
  }

  private async exclusive<T>(operation: (generation: number) => Promise<T>): Promise<T> {
    if (this.busy) throw new Error('another mission command is pending');
    this.busy = true;
    const generation = this.operationGeneration;
    try {
      const result = await operation(generation);
      this.ensureCurrent(generation);
      return result;
    } catch (error) {
      if (!(error instanceof MissionOperationCancelled) &&
          !(error instanceof MissionCommandRefused)) {
        this.currentState = 'fault';
      }
      throw error;
    } finally {
      this.busy = false;
    }
  }

  private async priorityStop(): Promise<AckV2> {
    this.operationGeneration += 1;
    const commandId = this.takeCommandId();
    this.currentState = 'idle';
    this.preparedCalibration = null;
    this.calibrationId = 0;
    try {
      await this.transport.writeWithResponse(buildCommandV2({
        opcode: 3,
        epoch: this.epoch,
        commandId,
      }));
    } finally {
      await this.transport.writeCompatibilityStop();
    }
    return {
      state: 0,
      epoch: this.epoch,
      commandId,
      faultCode: 0,
      calibrationId: 0,
    };
  }

  private ensureCurrent(generation: number): void {
    if (generation !== this.operationGeneration) throw new MissionOperationCancelled();
  }

  private takeCommandId(): number {
    const id = this.nextCommandId;
    this.nextCommandId = this.nextCommandId === 0xffffffff ? 1 : this.nextCommandId + 1;
    return id;
  }

  private samePreparedCalibration(calibration: CalibrationWire): boolean {
    const prepared = this.preparedCalibration;
    return Boolean(prepared && prepared.id === calibration.id &&
      prepared.schemaVersion === calibration.schemaVersion &&
      prepared.sprayForwardFt === calibration.sprayForwardFt &&
      prepared.sprayRightFt === calibration.sprayRightFt);
  }

  private receiveAck(packet: Uint8Array): void {
    const ack = parseAckV2(packet);
    if (!ack || !this.pending || ack.epoch !== this.epoch || ack.commandId !== this.pending.commandId) return;
    if (this.pending.received) return;
    this.pending.received = ack;
    for (const listener of this.pending.listeners) listener(ack);
    this.pending.listeners.clear();
  }

  private async sendAndWait(
    packet: Uint8Array,
    commandId: number,
    generation: number,
  ): Promise<AckV2> {
    this.ensureCurrent(generation);
    this.pending = { commandId, received: null, listeners: new Set() };
    try {
      for (let attempt = 0; attempt <= this.retries; attempt += 1) {
        await this.transport.writeWithResponse(packet);
        this.ensureCurrent(generation);
        const ack = await this.waitForAck();
        this.ensureCurrent(generation);
        if (ack) return ack;
      }
      throw new Error(`ACK timeout for command ${commandId}`);
    } finally {
      this.pending = null;
    }
  }

  private waitForAck(): Promise<AckV2 | null> {
    if (!this.pending) return Promise.resolve(null);
    if (this.pending.received) return Promise.resolve(this.pending.received);
    return new Promise((resolve) => {
      const listener = (ack: AckV2) => {
        clearTimeout(timer);
        resolve(ack);
      };
      const timer = setTimeout(() => {
        this.pending?.listeners.delete(listener);
        resolve(null);
      }, this.timeoutMs);
      this.pending!.listeners.add(listener);
    });
  }

  private requireAck(
    ack: AckV2,
    states: number[],
    calibrationId: number | null,
    retryableFaults: number[] = [],
  ): void {
    if (ack.state !== 5 && retryableFaults.includes(ack.faultCode)) {
      throw new MissionCommandRefused(
        ack.faultCode,
        'The rover is not on the pass you asked to resume from. Move it onto that lane, facing along it, and arm again.',
      );
    }
    if (ack.state === 5 || ack.faultCode !== 0) throw new Error(`firmware fault ${ack.faultCode}`);
    if (!states.includes(ack.state)) throw new Error(`unexpected acknowledgement state ${ack.state}`);
    if (calibrationId !== null && ack.calibrationId !== calibrationId) {
      throw new Error('acknowledgement calibration is stale');
    }
  }
}
