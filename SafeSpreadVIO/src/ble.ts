import { BleManager, Device } from 'react-native-ble-plx';
import { Buffer } from 'buffer';
import { buildAreaPacket, buildPosePacket } from './protocol';
import {
  AckV2,
  buildCommandV2,
  FaultSampleV2,
  parseAckV2,
  parseFaultSampleV2,
  parseRoutePlanV2,
  parseTelemetryV2,
  RoutePlanV2,
  TelemetryV2,
} from './protocolV2';
import { MissionTransport } from './missionControl';
import { PoseTransport } from './latestPoseSender';

const DEVICE_NAME = 'SafeSpread';
const NUS_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E';
const NUS_WRITE_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E';
const NUS_NOTIFY_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E';

export type ConnectionStatus = 'disconnected' | 'scanning' | 'connected' | 'incompatible';

type Remove = () => void;

export class SafeSpreadBLE implements PoseTransport, MissionTransport {
  private manager = new BleManager();
  private device: Device | null = null;
  private notificationSubscription: { remove(): void } | null = null;
  private disconnectSubscription: { remove(): void } | null = null;
  private pendingText = '';
  private logListener: ((line: string) => void) | null = null;
  private ackListeners = new Set<(packet: Uint8Array) => void>();
  private telemetryListeners = new Set<(telemetry: TelemetryV2) => void>();
  private routePlanListeners = new Set<(plan: RoutePlanV2) => void>();
  private faultSampleListeners = new Set<(sample: FaultSampleV2) => void>();
  private faultPacketListeners = new Set<(packet: Uint8Array) => void>();
  private disconnectListeners = new Set<() => void>();
  private connectionGeneration = 0;
  private cancelConnect: (() => void) | null = null;

  connect(
    onStatusChange: (status: ConnectionStatus) => void,
    onLog?: (line: string) => void,
  ): Promise<void> {
    this.cancelConnect?.();
    const generation = ++this.connectionGeneration;
    this.logListener = onLog ?? null;
    onStatusChange('scanning');
    return new Promise((resolve, reject) => {
      let finished = false;
      let found = false;
      const isCurrent = () => generation === this.connectionGeneration;
      const finish = (error?: unknown) => {
        if (finished) return;
        finished = true;
        if (this.cancelConnect === cancel) this.cancelConnect = null;
        if (error === undefined) resolve();
        else reject(error);
      };
      const cancel = () => finish(new Error('BLE connection cancelled'));
      this.cancelConnect = cancel;
      this.manager.startDeviceScan([NUS_SERVICE_UUID], null, async (error, scanned) => {
        if (finished || found || !isCurrent()) return;
        if (error) {
          this.manager.stopDeviceScan();
          onStatusChange('disconnected');
          finish(error);
          return;
        }
        if (scanned?.name !== DEVICE_NAME) return;

        found = true;
        this.manager.stopDeviceScan();
        let device: Device | null = null;
        try {
          device = await scanned.connect();
          if (!isCurrent() || finished) {
            await device.cancelConnection().catch(() => {});
            return;
          }
          await device.discoverAllServicesAndCharacteristics();
          if (!isCurrent() || finished) {
            await device.cancelConnection().catch(() => {});
            return;
          }
          this.device = device;
          this.notificationSubscription = device.monitorCharacteristicForService(
            NUS_SERVICE_UUID,
            NUS_NOTIFY_UUID,
            (monitorError, characteristic) => {
              if (!isCurrent() || this.device !== device) return;
              if (monitorError) {
                this.logListener?.(`[BLE] notify error: ${monitorError.message}`);
                return;
              }
              if (!characteristic?.value) return;
              this.handleNotification(new Uint8Array(Buffer.from(characteristic.value, 'base64')));
            },
          );
          this.disconnectSubscription = device.onDisconnected(() => {
            if (!isCurrent() || this.device !== device) return;
            this.device = null;
            onStatusChange('disconnected');
            for (const listener of this.disconnectListeners) listener();
          });

          try {
            await this.probeProtocolV2();
            if (!isCurrent() || finished) {
              await device.cancelConnection().catch(() => {});
              return;
            }
          } catch (probeError) {
            if (!isCurrent() || finished) {
              await device.cancelConnection().catch(() => {});
              return;
            }
            this.notificationSubscription?.remove();
            this.notificationSubscription = null;
            this.disconnectSubscription?.remove();
            this.disconnectSubscription = null;
            await device.cancelConnection().catch(() => {});
            this.device = null;
            onStatusChange('incompatible');
            finish(probeError);
            return;
          }
          onStatusChange('connected');
          finish();
        } catch (connectionError) {
          if (!isCurrent() || finished) {
            await device?.cancelConnection().catch(() => {});
            return;
          }
          this.device = null;
          onStatusChange('disconnected');
          finish(connectionError);
        }
      });
    });
  }

  async disconnect(): Promise<void> {
    this.connectionGeneration += 1;
    this.cancelConnect?.();
    this.cancelConnect = null;
    this.manager.stopDeviceScan();
    this.notificationSubscription?.remove();
    this.notificationSubscription = null;
    this.disconnectSubscription?.remove();
    this.disconnectSubscription = null;
    const connected = this.device;
    this.device = null;
    if (connected) await connected.cancelConnection();
  }

  async writePose(packet: Uint8Array): Promise<void> {
    await this.writeWithoutResponse(packet);
  }

  async writeWithResponse(packet: Uint8Array): Promise<void> {
    const device = this.requireDevice();
    await device.writeCharacteristicWithResponseForService(
      NUS_SERVICE_UUID,
      NUS_WRITE_UUID,
      Buffer.from(packet).toString('base64'),
    );
  }

  async writeCompatibilityStop(): Promise<void> {
    await this.writeWithoutResponse(new Uint8Array(['2'.charCodeAt(0)]));
  }

  subscribeAck(listener: (packet: Uint8Array) => void): Remove {
    this.ackListeners.add(listener);
    return () => this.ackListeners.delete(listener);
  }

  subscribeTelemetry(listener: (telemetry: TelemetryV2) => void): Remove {
    this.telemetryListeners.add(listener);
    return () => this.telemetryListeners.delete(listener);
  }

  subscribeRoutePlan(listener: (plan: RoutePlanV2) => void): Remove {
    this.routePlanListeners.add(listener);
    return () => this.routePlanListeners.delete(listener);
  }

  subscribeFaultSamples(listener: (sample: FaultSampleV2) => void): Remove {
    this.faultSampleListeners.add(listener);
    return () => this.faultSampleListeners.delete(listener);
  }

  subscribeFaultPackets(listener: (packet: Uint8Array) => void): Remove {
    this.faultPacketListeners.add(listener);
    return () => this.faultPacketListeners.delete(listener);
  }

  subscribeDisconnect(listener: () => void): Remove {
    this.disconnectListeners.add(listener);
    return () => this.disconnectListeners.delete(listener);
  }

  /** A v2 STOP at epoch zero is a safe compatibility probe: v2 firmware ACKs
   * it, while legacy firmware cannot interpret it as an arm/start command. */
  async probeProtocolV2(timeoutMs = 750): Promise<AckV2> {
    const commandId = (Date.now() >>> 0) || 1;
    const packet = buildCommandV2({ opcode: 3, epoch: 0, commandId });
    let timer: ReturnType<typeof setTimeout> | null = null;
    let unsubscribe: Remove = () => {};
    const acknowledgement = new Promise<AckV2>((resolve, reject) => {
      unsubscribe = this.subscribeAck((candidate) => {
        const parsed = parseAckV2(candidate);
        if (!parsed || parsed.epoch !== 0 || parsed.commandId !== commandId) return;
        if (timer) clearTimeout(timer);
        resolve(parsed);
      });
      timer = setTimeout(() => reject(new Error('SafeSpread firmware does not acknowledge protocol v2')),
        timeoutMs);
    });
    try {
      await this.writeWithResponse(packet);
      return await acknowledgement;
    } finally {
      if (timer) clearTimeout(timer);
      unsubscribe();
    }
  }

  async emergencyStop(): Promise<void> {
    const commandId = (Date.now() >>> 0) || 1;
    try {
      await this.writeWithResponse(buildCommandV2({ opcode: 3, epoch: 0, commandId }));
    } finally {
      await this.writeCompatibilityStop();
    }
  }

  // Legacy methods remain only until the v2 setup UI lands. A new connection
  // cannot reach them unless the safe v2 probe above has already succeeded.
  async sendPose(x: number, y: number, heading: number): Promise<void> {
    await this.writeWithoutResponse(buildPosePacket(x, y, heading));
  }

  async sendArea(widthFt: number, lengthFt: number): Promise<void> {
    await this.writeWithoutResponse(buildAreaPacket(widthFt, lengthFt));
  }

  async sendCommand(command: '1' | '2' | '3' | '4'): Promise<void> {
    await this.writeWithoutResponse(new Uint8Array([command.charCodeAt(0)]));
  }

  private requireDevice(): Device {
    if (!this.device) throw new Error('SafeSpread BLE is disconnected');
    return this.device;
  }

  private async writeWithoutResponse(packet: Uint8Array): Promise<void> {
    const device = this.requireDevice();
    await device.writeCharacteristicWithoutResponseForService(
      NUS_SERVICE_UUID,
      NUS_WRITE_UUID,
      Buffer.from(packet).toString('base64'),
    );
  }

  private handleNotification(bytes: Uint8Array): void {
    if (bytes[0] === 0x21) {
      const ack = parseAckV2(bytes);
      if (ack) {
        for (const listener of this.ackListeners) listener(bytes.slice());
        return;
      }
      const telemetry = parseTelemetryV2(bytes);
      if (telemetry) {
        for (const listener of this.telemetryListeners) listener(telemetry);
        return;
      }
      const plan = parseRoutePlanV2(bytes);
      if (plan) {
        for (const listener of this.routePlanListeners) listener(plan);
        return;
      }
      const sample = parseFaultSampleV2(bytes);
      if (sample) {
        for (const listener of this.faultSampleListeners) listener(sample);
        for (const listener of this.faultPacketListeners) listener(bytes.slice());
        return;
      }
      this.logListener?.(`[PROTO] rejected binary notification (${bytes.length} bytes)`);
      return;
    }

    this.pendingText += Buffer.from(bytes).toString('utf8');
    const lines = this.pendingText.split('\n');
    this.pendingText = lines.pop() ?? '';
    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed) this.logListener?.(trimmed);
    }
  }
}
