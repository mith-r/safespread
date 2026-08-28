import { Directory, File, FileMode, Paths } from 'expo-file-system';
import * as Sharing from 'expo-sharing';

export interface LogSink {
  append(line: string): Promise<void>;
  close(): Promise<void>;
}

export interface MissionMetadata {
  missionId: string;
  createdAtIso: string;
  appVersion: string;
  firmwareVersion: string;
  protocolVersion: 2;
  epoch: number;
  calibrationId: number;
  calibrationSchemaVersion: number;
  pavement: {
    surface: 'asphalt' | 'concrete' | 'pavers' | 'other';
    condition: 'dry' | 'wet';
  };
  // Absent for a diagnostic session. The self-test needs a connection and an
  // epoch but no rectangle, and requiring one here is what kept it from running
  // on a rover that could not be set up in the first place.
  rectangle?: {
    source: 'entered' | 'walked';
    mFt: number;
    nFt: number;
    side: 'right' | 'left';
  };
}

export interface MissionRecord {
  type: string;
  phoneMs: number;
  sequence?: number;
  epoch?: number;
  xFt?: number;
  yFt?: number;
  headingDeg?: number;
  speedFps?: number;
  yawRateDps?: number;
  trackingValid?: boolean;
  routeIndex?: number;
  crossTrackFt?: number;
  headingErrorDeg?: number;
  steeringUs?: number;
  throttleUs?: number;
  fault?: string | number;
  state?: string;
  [key: string]: string | number | boolean | null | undefined;
}

export interface MissionLogFile {
  name: string;
  uri: string;
  modificationTime: number;
  size: number;
}

export interface MissionDirectoryAdapter {
  list(): Array<{
    name: string;
    uri: string;
    modificationTime?: number | null;
    size?: number | null;
  }>;
}

export interface ShareAdapter {
  isAvailableAsync(): Promise<boolean>;
  shareAsync(uri: string): Promise<void>;
}

function errorFrom(value: unknown): Error {
  return value instanceof Error ? value : new Error(String(value));
}

function jsonLine(value: object): string {
  const encoded = JSON.stringify(value);
  if (encoded === undefined) throw new TypeError('mission record is not JSON serializable');
  return `${encoded}\n`;
}

export class MissionLogger {
  private tail: Promise<void> = Promise.resolve();
  private failure: Error | null = null;
  private closed = false;

  private constructor(private readonly sink: LogSink) {}

  static async create(meta: MissionMetadata, sink: LogSink): Promise<MissionLogger> {
    const logger = new MissionLogger(sink);
    await logger.enqueue(jsonLine({ type: 'metadata', ...meta }));
    return logger;
  }

  get failed(): boolean {
    return this.failure !== null;
  }

  async record(record: MissionRecord): Promise<void> {
    if (this.closed) throw new Error('mission logger is closed');
    if (this.failure) throw this.failure;
    return this.enqueue(jsonLine(record));
  }

  async close(): Promise<void> {
    if (this.closed) {
      if (this.failure) throw this.failure;
      return;
    }
    this.closed = true;
    await this.tail;
    try {
      await this.sink.close();
    } catch (error) {
      this.failure = errorFrom(error);
    }
    if (this.failure) throw this.failure;
  }

  private enqueue(line: string): Promise<void> {
    const operation = this.tail.then(async () => {
      if (this.failure) throw this.failure;
      await this.sink.append(line);
    });
    this.tail = operation.catch((error) => {
      this.failure = errorFrom(error);
    });
    return operation;
  }
}

function logsDirectory(): Directory {
  return new Directory(Paths.document, 'SafeSpread', 'logs');
}

export async function createFileLogSink(
  missionId: string,
): Promise<{ sink: LogSink; uri: string }> {
  const safeId = missionId.replace(/[^A-Za-z0-9._-]/g, '_');
  if (!safeId || safeId === '.' || safeId === '..') throw new Error('mission ID is invalid');
  const directory = logsDirectory();
  directory.create({ idempotent: true, intermediates: true });
  const file = new File(directory, `${safeId}.jsonl`);
  file.create({ intermediates: true });
  const handle = file.open(FileMode.Append);
  const encoder = new TextEncoder();
  let closed = false;
  return {
    uri: file.uri,
    sink: {
      async append(line: string) {
        if (closed) throw new Error('file log sink is closed');
        handle.writeBytes(encoder.encode(line));
      },
      async close() {
        if (!closed) {
          closed = true;
          handle.close();
        }
      },
    },
  };
}

const expoDirectoryAdapter: MissionDirectoryAdapter = {
  list() {
    const directory = logsDirectory();
    if (!directory.exists) return [];
    return directory.list()
      .filter((entry): entry is File => entry instanceof File)
      .map((file) => {
        const info = file.info();
        return {
          name: file.name,
          uri: file.uri,
          modificationTime: info.modificationTime,
          size: info.size,
        };
      });
  },
};

export function listMissionLogs(
  adapter: MissionDirectoryAdapter = expoDirectoryAdapter,
): MissionLogFile[] {
  return adapter.list()
    .filter((entry) => entry.name.toLowerCase().endsWith('.jsonl'))
    .map((entry) => ({
      name: entry.name,
      uri: entry.uri,
      modificationTime: entry.modificationTime ?? 0,
      size: entry.size ?? 0,
    }))
    .sort((a, b) => b.modificationTime - a.modificationTime || a.name.localeCompare(b.name));
}

export async function exportMissionLog(
  uri: string,
  adapter: ShareAdapter = Sharing,
): Promise<void> {
  if (!await adapter.isAvailableAsync()) throw new Error('mission log sharing is unavailable');
  await adapter.shareAsync(uri);
}
