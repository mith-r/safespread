import { addPoseQueueAgeV2, parsePoseV2 } from './protocolV2';

export interface PoseTransport {
  writePose(packet: Uint8Array): Promise<void>;
}

interface QueuedPose {
  packet: Uint8Array;
  offeredAtMs: number;
  offerId: number;
}

function monotonicNow(): number {
  return globalThis.performance?.now() ?? Date.now();
}

function isProtocolV2Pose(packet: Uint8Array): boolean {
  return packet.length === 32 && packet[0] === 0x21 && packet[1] === 0x56 && packet[2] === 2;
}

export class LatestPoseSender {
  private active: Promise<void> | null = null;
  private pending: QueuedPose | null = null;
  private stopping = false;
  private failure: Error | null = null;
  private droppedCount = 0;
  private sentPoseSequence: number | null = null;
  private nextOfferId = 1;
  private sentOfferId = 0;

  constructor(
    private readonly transport: PoseTransport,
    private readonly onError: (error: Error) => void = () => {},
    private readonly now: () => number = monotonicNow,
  ) {}

  get dropped(): number {
    return this.droppedCount;
  }

  /** Sequence of the most recent protocol-v2 pose whose BLE write completed. */
  get lastSentPoseSequence(): number | null {
    return this.sentPoseSequence;
  }

  /** Monotonic local ID of the latest offer whose BLE write completed. */
  get lastSentOfferId(): number {
    return this.sentOfferId;
  }

  offer(packet: Uint8Array): number {
    if (this.stopping) throw new Error('pose sender is stopped');
    if (this.failure) throw this.failure;
    const owned = {
      packet: packet.slice(),
      offeredAtMs: this.now(),
      offerId: this.nextOfferId,
    };
    this.nextOfferId += 1;
    if (this.active) {
      if (this.pending) this.droppedCount += 1;
      this.pending = owned;
      return owned.offerId;
    }
    this.start(owned);
    return owned.offerId;
  }

  async stop(): Promise<void> {
    this.stopping = true;
    while (this.active) await this.active;
    if (this.failure) throw this.failure;
  }

  private start(packet: QueuedPose): void {
    const task = this.drain(packet);
    this.active = task;
    void task.then(() => {
      if (this.active === task) this.active = null;
    });
  }

  private async drain(first: QueuedPose): Promise<void> {
    let current: QueuedPose | null = first;
    while (current) {
      try {
        const elapsedMs = Math.max(0, this.now() - current.offeredAtMs);
        const packet = isProtocolV2Pose(current.packet)
          ? addPoseQueueAgeV2(current.packet, elapsedMs)
          : current.packet;
        await this.transport.writePose(packet);
        this.sentOfferId = current.offerId;
        const writtenPose = parsePoseV2(packet);
        if (writtenPose) this.sentPoseSequence = writtenPose.sequence;
      } catch (value) {
        this.failure = value instanceof Error ? value : new Error(String(value));
        if (this.pending) {
          this.droppedCount += 1;
          this.pending = null;
        }
        this.onError(this.failure);
        return;
      }
      current = this.pending;
      this.pending = null;
    }
  }
}
