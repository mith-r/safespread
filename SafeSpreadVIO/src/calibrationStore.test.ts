import { createCalibration, markMotionCalibrationVerified } from './calibration';
import {
  CalibrationStorageAdapter,
  loadCalibration,
  saveCalibration,
} from './calibrationStore';

class MemoryStorage implements CalibrationStorageAdapter {
  value: string | null = null;
  async read(): Promise<string | null> { return this.value; }
  async write(value: string): Promise<void> { this.value = value; }
}

const unverifiedCalibration = createCalibration({
  schemaVersion: 1,
  hardwareTag: 'rover-a',
  createdAtIso: '2026-08-26T16:00:00.000Z',
  cameraForwardFt: 0.4,
  cameraRightFt: 0.1,
  cameraYawDeg: -3,
  sprayForwardFt: -1.1,
  sprayRightFt: 0,
  operatingLoadLb: 40,
  surface: 'asphalt',
  condition: 'wet',
});
const calibration = markMotionCalibrationVerified(
  unverifiedCalibration,
  '2026-08-26T16:30:00.000Z',
);

describe('calibrationStore', () => {
  it('round trips a valid matching calibration', async () => {
    const storage = new MemoryStorage();
    await saveCalibration(calibration, 'rover-a', storage);
    await expect(loadCalibration('rover-a', storage)).resolves.toEqual({
      calibration,
      wetAllowed: true,
      reason: 'ready',
    });
  });

  it('refuses an unsupported schema and remains dry-only', async () => {
    const storage = new MemoryStorage();
    storage.value = JSON.stringify({ ...calibration, schemaVersion: 99 });
    await expect(loadCalibration('rover-a', storage)).resolves.toEqual({
      calibration: null,
      wetAllowed: false,
      reason: 'schema',
    });
  });

  it('loads phone geometry but remains dry-only until matching firmware motion proof exists', async () => {
    const storage = new MemoryStorage();
    await saveCalibration(unverifiedCalibration, 'rover-a', storage);
    await expect(loadCalibration('rover-a', storage)).resolves.toEqual({
      calibration: unverifiedCalibration,
      wetAllowed: false,
      reason: 'motion',
    });
  });

  it('fails old records closed when operating load identity is absent', async () => {
    const storage = new MemoryStorage();
    const legacy = { ...calibration } as Partial<typeof calibration>;
    delete legacy.operatingLoadLb;
    storage.value = JSON.stringify(legacy);
    await expect(loadCalibration('rover-a', storage)).resolves.toEqual({
      calibration: null,
      wetAllowed: false,
      reason: 'load',
    });
  });

  it('refuses a calibration for stale hardware', async () => {
    const storage = new MemoryStorage();
    storage.value = JSON.stringify(calibration);
    await expect(loadCalibration('rover-b', storage)).resolves.toEqual({
      calibration: null,
      wetAllowed: false,
      reason: 'hardware',
    });
  });

  it('treats corrupt JSON as unavailable and dry-only', async () => {
    const storage = new MemoryStorage();
    storage.value = '{bad json';
    await expect(loadCalibration('rover-a', storage)).resolves.toEqual({
      calibration: null,
      wetAllowed: false,
      reason: 'corrupt',
    });
  });

  it('treats a missing calibration as dry-only', async () => {
    const storage = new MemoryStorage();
    await expect(loadCalibration('rover-a', storage)).resolves.toEqual({
      calibration: null,
      wetAllowed: false,
      reason: 'missing',
    });
  });

  it('rejects writes whose hardware tag does not match', async () => {
    const storage = new MemoryStorage();
    await expect(saveCalibration(calibration, 'rover-b', storage)).rejects.toThrow(/hardware/i);
  });
});
