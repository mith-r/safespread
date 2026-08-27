import { File, Paths } from 'expo-file-system';
import {
  CalibrationRecord,
  hasMotionCalibrationProof,
  isCalibrationRecord,
} from './calibration';

export interface CalibrationStorageAdapter {
  read(): Promise<string | null>;
  write(value: string): Promise<void>;
}

export type CalibrationLoadReason =
  | 'ready'
  | 'motion'
  | 'load'
  | 'missing'
  | 'schema'
  | 'hardware'
  | 'corrupt';

export interface CalibrationLoadResult {
  calibration: CalibrationRecord | null;
  wetAllowed: boolean;
  reason: CalibrationLoadReason;
}

function calibrationFile(): File {
  return new File(Paths.document, 'SafeSpread', 'calibration.json');
}

const expoCalibrationStorage: CalibrationStorageAdapter = {
  async read() {
    const file = calibrationFile();
    return file.exists ? file.text() : null;
  },
  async write(value: string) {
    const file = calibrationFile();
    file.create({ intermediates: true, overwrite: true });
    file.write(value);
  },
};

function dryOnly(reason: Exclude<CalibrationLoadReason, 'ready'>): CalibrationLoadResult {
  return { calibration: null, wetAllowed: false, reason };
}

export async function loadCalibration(
  expectedHardwareTag: string,
  storage: CalibrationStorageAdapter = expoCalibrationStorage,
): Promise<CalibrationLoadResult> {
  const encoded = await storage.read();
  if (encoded === null) return dryOnly('missing');
  let decoded: unknown;
  try {
    decoded = JSON.parse(encoded);
  } catch {
    return dryOnly('corrupt');
  }
  if (typeof decoded !== 'object' || decoded === null) return dryOnly('corrupt');
  const candidate = decoded as {
    schemaVersion?: unknown;
    hardwareTag?: unknown;
    operatingLoadLb?: unknown;
  };
  if (candidate.schemaVersion !== 1) return dryOnly('schema');
  if (candidate.hardwareTag !== expectedHardwareTag) return dryOnly('hardware');
  if (typeof candidate.operatingLoadLb !== 'number' ||
      !Number.isFinite(candidate.operatingLoadLb) || candidate.operatingLoadLb < 0) {
    return dryOnly('load');
  }
  if (!isCalibrationRecord(decoded)) return dryOnly('corrupt');
  if (!hasMotionCalibrationProof(decoded)) {
    return { calibration: decoded, wetAllowed: false, reason: 'motion' };
  }
  return { calibration: decoded, wetAllowed: true, reason: 'ready' };
}

export async function saveCalibration(
  calibration: CalibrationRecord,
  expectedHardwareTag: string,
  storage: CalibrationStorageAdapter = expoCalibrationStorage,
): Promise<void> {
  if (calibration.hardwareTag !== expectedHardwareTag) {
    throw new Error('calibration hardware tag does not match this rover');
  }
  if (!isCalibrationRecord(calibration)) throw new Error('calibration record is invalid');
  await storage.write(JSON.stringify(calibration));
}
