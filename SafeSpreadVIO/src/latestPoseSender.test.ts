import { LatestPoseSender, PoseTransport } from './latestPoseSender';
import { buildPoseV2, parsePoseV2 } from './protocolV2';

function deferred() {
  let resolve!: () => void;
  let reject!: (error: Error) => void;
  const promise = new Promise<void>((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

describe('LatestPoseSender', () => {
  it('keeps one write in flight and only the newest pending pose', async () => {
    const writes: number[] = [];
    const gates = [deferred(), deferred()];
    const transport: PoseTransport = {
      writePose: async (packet) => {
        writes.push(packet[0]);
        await gates[writes.length - 1].promise;
      },
    };
    const sender = new LatestPoseSender(transport);
    sender.offer(new Uint8Array([1]));
    sender.offer(new Uint8Array([2]));
    sender.offer(new Uint8Array([3]));
    expect(writes).toEqual([1]);
    expect(sender.dropped).toBe(1);
    gates[0].resolve();
    await Promise.resolve();
    await Promise.resolve();
    expect(writes).toEqual([1, 3]);
    gates[1].resolve();
    await sender.stop();
    expect(sender.lastSentOfferId).toBe(3);
  });

  it('drains the pending newest pose before stop resolves', async () => {
    const gate = deferred();
    const writes: number[] = [];
    const sender = new LatestPoseSender({
      async writePose(packet) {
        writes.push(packet[0]);
        if (packet[0] === 1) await gate.promise;
      },
    });
    sender.offer(new Uint8Array([1]));
    sender.offer(new Uint8Array([2]));
    let stopped = false;
    const stop = sender.stop().then(() => { stopped = true; });
    await Promise.resolve();
    expect(stopped).toBe(false);
    gate.resolve();
    await stop;
    expect(writes).toEqual([1, 2]);
    expect(() => sender.offer(new Uint8Array([3]))).toThrow('stopped');
  });

  it('reports a terminal write rejection and discards pending work', async () => {
    const errors: Error[] = [];
    const sender = new LatestPoseSender(
      { writePose: async () => { throw new Error('BLE lost'); } },
      (error) => errors.push(error),
    );
    sender.offer(new Uint8Array([1]));
    sender.offer(new Uint8Array([2]));
    await expect(sender.stop()).rejects.toThrow('BLE lost');
    expect(errors.map((error) => error.message)).toEqual(['BLE lost']);
    expect(sender.dropped).toBe(1);
  });

  it('adds BLE queue delay to a pending protocol-v2 pose age', async () => {
    const gate = deferred();
    const written: Uint8Array[] = [];
    let nowMs = 100;
    const sender = new LatestPoseSender({
      async writePose(packet) {
        written.push(packet);
        if (written.length === 1) await gate.promise;
      },
    }, undefined, () => nowMs);
    const first = buildPoseV2({
      flags: 7,
      epoch: 1,
      sequence: 1,
      ageMs: 20,
      x: 0,
      y: 0,
      heading: 0,
      speedFps: 0,
      yawRateDps: 0,
      calibrationId: 2,
    });
    const second = buildPoseV2({ ...parsePoseV2(first)!, sequence: 2 });

    sender.offer(first);
    sender.offer(second);
    nowMs = 425;
    gate.resolve();
    await sender.stop();

    expect(parsePoseV2(written[0])?.ageMs).toBe(20);
    expect(parsePoseV2(written[1])?.ageMs).toBe(345);
    expect(sender.lastSentPoseSequence).toBe(2);
  });

  it('reports a pose sequence only after that BLE write completes', async () => {
    const gate = deferred();
    const pose = buildPoseV2({
      flags: 7,
      epoch: 1,
      sequence: 42,
      ageMs: 0,
      x: 0,
      y: -1,
      heading: 0,
      speedFps: 0,
      yawRateDps: 0,
      calibrationId: 2,
    });
    const sender = new LatestPoseSender({
      async writePose() { await gate.promise; },
    });

    const offerId = sender.offer(pose);
    expect(offerId).toBe(1);
    expect(sender.lastSentOfferId).toBe(0);
    expect(sender.lastSentPoseSequence).toBeNull();
    gate.resolve();
    await sender.stop();
    expect(sender.lastSentPoseSequence).toBe(42);
    expect(sender.lastSentOfferId).toBe(offerId);
  });
});
