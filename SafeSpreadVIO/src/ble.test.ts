import { Buffer } from 'buffer';

const mockManager = {
  startDeviceScan: jest.fn(),
  stopDeviceScan: jest.fn(),
};

jest.mock('react-native-ble-plx', () => ({
  BleManager: jest.fn(() => mockManager),
}));

import {
  BLE_SCAN_TIMEOUT_MS,
  BLE_CONNECT_TIMEOUT_MS,
  isHardenedFirmwareProbe,
  isProtocolV2Notification,
  SafeSpreadBLE,
} from './ble';
import { HARDENED_FIRMWARE_CAPABILITY_ID } from './protocolV2';

describe('SafeSpreadBLE notification framing', () => {
  it('preserves firmware warning text beginning with two exclamation marks', () => {
    const warning = new Uint8Array(Buffer.from('!! Wet operation blocked\n'));
    expect(isProtocolV2Notification(warning)).toBe(false);
    const lines: string[] = [];
    const ble = new SafeSpreadBLE();
    const internal = ble as unknown as {
      logListener: ((line: string) => void) | null;
      handleNotification(bytes: Uint8Array): void;
    };
    internal.logListener = (line) => lines.push(line);
    internal.handleNotification(warning);
    expect(lines).toEqual(['!! Wet operation blocked']);
  });

  it('recognizes only complete known protocol-v2 notifications as binary', () => {
    const ack = new Uint8Array(16);
    ack.set([0x21, 0x41, 2]);
    expect(isProtocolV2Notification(ack)).toBe(true);
    expect(isProtocolV2Notification(ack.subarray(0, 8))).toBe(false);
    const textWithMagic = new Uint8Array(Buffer.from('!A plain text line\n'));
    expect(isProtocolV2Notification(textWithMagic)).toBe(false);
  });
});

describe('SafeSpreadBLE firmware compatibility', () => {
  it('requires the hardened capability instead of accepting every v2 sketch', () => {
    const base = { state: 0, epoch: 0, commandId: 1, faultCode: 0 };
    expect(isHardenedFirmwareProbe({
      ...base,
      calibrationId: HARDENED_FIRMWARE_CAPABILITY_ID,
    })).toBe(true);
    expect(isHardenedFirmwareProbe({ ...base, calibrationId: 0 })).toBe(false);
    expect(isHardenedFirmwareProbe({
      ...base,
      faultCode: 1,
      calibrationId: HARDENED_FIRMWARE_CAPABILITY_ID,
    })).toBe(false);
  });
});

describe('SafeSpreadBLE connection ownership', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('cancels a delayed connection without installing stale monitors or reporting connected', async () => {
    let scanCallback!: (error: Error | null, device: unknown) => void;
    mockManager.startDeviceScan.mockImplementation((_services, _options, callback) => {
      scanCallback = callback;
    });
    let releaseConnect!: () => void;
    const delayedConnect = new Promise<void>((resolve) => { releaseConnect = resolve; });
    const connectedDevice = {
      discoverAllServicesAndCharacteristics: jest.fn(async () => {}),
      monitorCharacteristicForService: jest.fn(),
      onDisconnected: jest.fn(),
      cancelConnection: jest.fn(async () => {}),
    };
    const scannedDevice = {
      name: 'SafeSpread',
      connect: jest.fn(async () => {
        await delayedConnect;
        return connectedDevice;
      }),
    };
    const statuses: string[] = [];
    const ble = new SafeSpreadBLE();
    const connecting = ble.connect((status) => statuses.push(status));
    scanCallback(null, scannedDevice);
    await ble.disconnect();
    releaseConnect();

    await expect(connecting).rejects.toThrow(/cancel/i);
    await Promise.resolve();
    expect(connectedDevice.cancelConnection).toHaveBeenCalled();
    expect(connectedDevice.monitorCharacteristicForService).not.toHaveBeenCalled();
    expect(statuses).toEqual(['scanning']);
  });

  it('ends a scan with actionable disconnected status when no rover appears', async () => {
    jest.useFakeTimers();
    mockManager.startDeviceScan.mockImplementation(() => {});
    const statuses: string[] = [];
    const ble = new SafeSpreadBLE();
    const connecting = ble.connect((status) => statuses.push(status));
    jest.advanceTimersByTime(BLE_SCAN_TIMEOUT_MS);
    await expect(connecting).rejects.toThrow(/not found/i);
    expect(statuses).toEqual(['scanning', 'disconnected']);
    expect(mockManager.stopDeviceScan).toHaveBeenCalled();
    jest.useRealTimers();
  });

  it('times out a rover that is discovered but never finishes connecting', async () => {
    jest.useFakeTimers();
    let scanCallback!: (error: Error | null, device: unknown) => void;
    mockManager.startDeviceScan.mockImplementation((_services, _options, callback) => {
      scanCallback = callback;
    });
    const neverConnects = new Promise<never>(() => {});
    const scannedDevice = {
      name: 'SafeSpread',
      connect: jest.fn(() => neverConnects),
      cancelConnection: jest.fn(async () => {}),
    };
    const statuses: string[] = [];
    const ble = new SafeSpreadBLE();
    const connecting = ble.connect((status) => statuses.push(status));
    scanCallback(null, scannedDevice);
    jest.advanceTimersByTime(BLE_CONNECT_TIMEOUT_MS);

    await expect(connecting).rejects.toThrow(/connection timed out/i);
    expect(statuses).toEqual(['scanning', 'disconnected']);
    expect(scannedDevice.cancelConnection).toHaveBeenCalled();
    jest.useRealTimers();
  });
});
