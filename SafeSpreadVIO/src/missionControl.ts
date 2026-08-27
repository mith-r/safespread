import {
  AckV2,
  buildCalibrationV2,
  buildCommandV2,
  buildRectangleV2,
  parseAckV2,
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
}

export type MissionClientState = 'idle' | 'configured' | 'armed' | 'running' | 'complete' | 'fault';

interface MissionControlOptions {
  timeoutMs?: number;
  retries?: number;
  faultDumpTimeoutMs?: number;
  dryMode?: boolean;
  preferForwardOnly?: boolean;
}

interface PendingAck {
  commandId: number;
  received: AckV2 | null;
  listeners: Set<(ack: AckV2 | null) => void>;
}

class MissionOperationCancelled extends Error {
  constructor() {
    super('mission operation cancelled by Stop');
  }
}

export class MissionControl {
  private currentState: MissionClientState = 'idle';
  private nextCommandId = 1;
  private pending: PendingAck | null = null;
  private busy = false;
  private readonly unsubscribe: () => void;
  private readonly timeoutMs: number;
  private readonly retries: number;
  private readonly faultDumpTimeoutMs: number;
  private readonly dryMode: boolean;
  private readonly preferForwardOnly: boolean;
  private calibrationId = 0;
  private preparedCalibration: CalibrationWire | null = null;
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
    // The firmware intentionally sends the complete frozen fault buffer before
    // its ACK. At the maximum sample count that takes substantially longer than
    // an ordinary command acknowledgement.
    this.faultDumpTimeoutMs = options.faultDumpTimeoutMs ?? Math.max(this.timeoutMs, 5000);
    this.dryMode = options.dryMode ?? true;
    this.preferForwardOnly = options.preferForwardOnly ?? true;
    this.unsubscribe = transport.subscribeAck((packet) => this.receiveAck(packet));
  }

  get state(): MissionClientState {
    return this.currentState;
  }

  async configure(definition: RectangleDefinition, calibration: CalibrationWire): Promise<AckV2> {
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
      const flags = (definition.side === 'left' ? 1 : 0) |
        (this.dryMode ? 2 : 0) |
        (this.preferForwardOnly ? 4 : 0);
      const rectangleAck = await this.sendAndWait(buildRectangleV2({
        flags,
        epoch: this.epoch,
        commandId: rectangleCommandId,
        mFt: definition.mFt,
        nFt: definition.nFt,
        startClearFt: definition.startClearFt,
        endClearFt: definition.endClearFt,
        calibrationId: calibration.id,
      }), rectangleCommandId, generation);
      this.requireAck(rectangleAck, [1], calibration.id);
      this.currentState = 'configured';
      return rectangleAck;
    });
  }

  async prepareCalibration(calibration: CalibrationWire): Promise<AckV2> {
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

  async runCalibrationStep(opcode: 5 | 6 | 7): Promise<AckV2> {
    if (!this.dryMode) throw new Error('calibration motion is dry-only');
    if (this.currentState !== 'idle' || !this.preparedCalibration) {
      throw new Error('calibration must be prepared while idle');
    }
    if (!this.hasFreshPose()) throw new Error('a fresh valid pose is required for calibration');
    return this.command(opcode, [0], 'idle');
  }

  async selfTest(): Promise<AckV2> {
    if (!this.dryMode) throw new Error('self-test motion is dry-only');
    if (this.currentState !== 'idle' || !this.preparedCalibration) {
      throw new Error('calibration epoch must be prepared before self-test');
    }
    if (!this.hasFreshPose()) throw new Error('a fresh valid pose is required for self-test');
    return this.command(4, [0], 'idle');
  }

  async dumpFault(): Promise<AckV2> {
    if (this.currentState !== 'idle' && this.currentState !== 'fault') {
      throw new Error('Stop before requesting the fault dump');
    }
    // Opcode 8 is not safe to retry while its response is in flight: current
    // firmware replays every sample for a duplicate request. Wait once for the
    // bounded dump instead of mixing duplicate samples in the app collector.
    return this.command(8, [0, 5], this.currentState, false, {
      timeoutMs: this.faultDumpTimeoutMs,
      retries: 0,
    });
  }

  async arm(): Promise<AckV2> {
    if (this.currentState !== 'configured') throw new Error('mission must be configured before arm');
    if (!this.hasFreshPose()) throw new Error('a fresh valid pose is required before arm');
    return this.command(1, [2], 'armed');
  }

  async start(): Promise<AckV2> {
    if (this.currentState !== 'armed') throw new Error('mission must be armed before start');
    if (!this.hasFreshPose()) throw new Error('a fresh valid pose is required before start');
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
    this.operationGeneration += 1;
    if (['configured', 'armed', 'running'].includes(this.currentState)) {
      this.currentState = 'fault';
    }
    this.cancelPendingAck();
  }

  notifyComplete(): void {
    if (this.currentState === 'running') this.currentState = 'complete';
  }

  dispose(): void {
    this.operationGeneration += 1;
    this.cancelPendingAck();
    this.unsubscribe();
  }

  private async command(
    opcode: number,
    expectedStates: number[],
    nextState: MissionClientState,
    validateCalibration = true,
    waitOptions?: { timeoutMs: number; retries: number },
  ): Promise<AckV2> {
    return this.exclusive(async (generation) => {
      const commandId = this.takeCommandId();
      const ack = await this.sendAndWait(
        buildCommandV2({ opcode, epoch: this.epoch, commandId }),
        commandId,
        generation,
        waitOptions,
      );
      this.requireAck(ack, expectedStates, validateCalibration ? this.calibrationId : null);
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
      if (!(error instanceof MissionOperationCancelled)) this.currentState = 'fault';
      throw error;
    } finally {
      this.busy = false;
    }
  }

  private async priorityStop(): Promise<AckV2> {
    this.operationGeneration += 1;
    this.cancelPendingAck();
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
    waitOptions?: { timeoutMs: number; retries: number },
  ): Promise<AckV2> {
    this.ensureCurrent(generation);
    this.pending = { commandId, received: null, listeners: new Set() };
    const timeoutMs = waitOptions?.timeoutMs ?? this.timeoutMs;
    const retries = waitOptions?.retries ?? this.retries;
    try {
      for (let attempt = 0; attempt <= retries; attempt += 1) {
        await this.transport.writeWithResponse(packet);
        this.ensureCurrent(generation);
        const ack = await this.waitForAck(timeoutMs);
        this.ensureCurrent(generation);
        if (ack) return ack;
      }
      throw new Error(`ACK timeout for command ${commandId}`);
    } finally {
      this.pending = null;
    }
  }

  private waitForAck(timeoutMs: number): Promise<AckV2 | null> {
    if (!this.pending) return Promise.resolve(null);
    if (this.pending.received) return Promise.resolve(this.pending.received);
    return new Promise((resolve) => {
      const listener = (ack: AckV2 | null) => {
        clearTimeout(timer);
        resolve(ack);
      };
      const timer = setTimeout(() => {
        this.pending?.listeners.delete(listener);
        resolve(null);
      }, timeoutMs);
      this.pending!.listeners.add(listener);
    });
  }

  private cancelPendingAck(): void {
    if (!this.pending) return;
    for (const listener of this.pending.listeners) listener(null);
    this.pending.listeners.clear();
  }

  private requireAck(ack: AckV2, states: number[], calibrationId: number | null): void {
    if (ack.state === 5 || ack.faultCode !== 0) throw new Error(`firmware fault ${ack.faultCode}`);
    if (!states.includes(ack.state)) throw new Error(`unexpected acknowledgement state ${ack.state}`);
    if (calibrationId !== null && ack.calibrationId !== calibrationId) {
      throw new Error('acknowledgement calibration is stale');
    }
  }
}
