import {
  exportMissionLog,
  listMissionLogs,
  LogSink,
  MissionLogger,
  MissionMetadata,
} from './missionLog';

const metadata: MissionMetadata = {
  missionId: 'mission-7',
  createdAtIso: '2026-08-26T12:00:00.000Z',
  appVersion: '1.0.0',
  firmwareVersion: 'vio-v2',
  protocolVersion: 2,
  epoch: 7,
  calibrationId: 3,
  calibrationSchemaVersion: 1,
  pavement: { surface: 'concrete', condition: 'dry', operatingLoadLb: 42.5 },
  rectangle: { source: 'entered', mFt: 20, nFt: 8, side: 'right' },
};

function memorySink(): LogSink & { lines: string[]; closed: boolean } {
  return {
    lines: [],
    closed: false,
    async append(line) { this.lines.push(line); },
    async close() { this.closed = true; },
  };
}

describe('MissionLogger', () => {
  it('writes metadata first and one JSON object per line', async () => {
    const sink = memorySink();
    const logger = await MissionLogger.create(metadata, sink);
    await logger.record({ type: 'state', phoneMs: 10, state: 'ARMED' });
    await logger.close();

    expect(sink.lines).toHaveLength(2);
    expect(sink.lines.every((line) => line.endsWith('\n'))).toBe(true);
    expect(JSON.parse(sink.lines[0])).toEqual({ type: 'metadata', ...metadata });
    expect(JSON.parse(sink.lines[1])).toEqual({ type: 'state', phoneMs: 10, state: 'ARMED' });
    expect(sink.closed).toBe(true);
  });

  it('preserves invocation order across concurrent records', async () => {
    let releaseFirst!: () => void;
    const firstBlocked = new Promise<void>((resolve) => { releaseFirst = resolve; });
    const lines: string[] = [];
    const sink: LogSink = {
      async append(line) {
        if (JSON.parse(line).sequence === 1) await firstBlocked;
        lines.push(line);
      },
      async close() {},
    };
    const logger = await MissionLogger.create(metadata, sink);
    const first = logger.record({ type: 'pose', phoneMs: 1, sequence: 1 });
    const second = logger.record({ type: 'pose', phoneMs: 2, sequence: 2 });
    await Promise.resolve();
    expect(lines).toHaveLength(1); // metadata only
    releaseFirst();
    await Promise.all([first, second]);
    expect(lines.slice(1).map((line) => JSON.parse(line).sequence)).toEqual([1, 2]);
  });

  it('permanently fails after an append error', async () => {
    let appends = 0;
    const sink: LogSink = {
      async append() {
        appends += 1;
        if (appends === 2) throw new Error('disk full');
      },
      async close() {},
    };
    const logger = await MissionLogger.create(metadata, sink);
    await expect(logger.record({ type: 'pose', phoneMs: 1 })).rejects.toThrow('disk full');
    await expect(logger.record({ type: 'pose', phoneMs: 2 })).rejects.toThrow('disk full');
    expect(appends).toBe(2);
    expect(logger.failed).toBe(true);
  });

  it('flushes queued records before closing and rejects records after close', async () => {
    let release!: () => void;
    const blocked = new Promise<void>((resolve) => { release = resolve; });
    let closed = false;
    const sink: LogSink = {
      async append(line) { if (JSON.parse(line).type === 'pose') await blocked; },
      async close() { closed = true; },
    };
    const logger = await MissionLogger.create(metadata, sink);
    const write = logger.record({ type: 'pose', phoneMs: 1 });
    const close = logger.close();
    await Promise.resolve();
    expect(closed).toBe(false);
    release();
    await Promise.all([write, close]);
    expect(closed).toBe(true);
    await expect(logger.record({ type: 'pose', phoneMs: 2 })).rejects.toThrow('closed');
  });
});

describe('recent mission files and export', () => {
  it('lists only JSONL files newest first', () => {
    const logs = listMissionLogs({
      list: () => [
        { name: 'old.jsonl', uri: 'file:///old.jsonl', modificationTime: 10, size: 20 },
        { name: 'notes.txt', uri: 'file:///notes.txt', modificationTime: 30, size: 4 },
        { name: 'new.jsonl', uri: 'file:///new.jsonl', modificationTime: 20, size: 40 },
      ],
    });
    expect(logs.map((log) => log.name)).toEqual(['new.jsonl', 'old.jsonl']);
  });

  it('shares the exact selected URI and reports an unavailable share sheet', async () => {
    const shared: string[] = [];
    await exportMissionLog('file:///mission.jsonl', {
      isAvailableAsync: async () => true,
      shareAsync: async (uri) => { shared.push(uri); },
    });
    expect(shared).toEqual(['file:///mission.jsonl']);
    await expect(exportMissionLog('file:///kept.jsonl', {
      isAvailableAsync: async () => false,
      shareAsync: async () => { throw new Error('must not share'); },
    })).rejects.toThrow('unavailable');
  });
});
