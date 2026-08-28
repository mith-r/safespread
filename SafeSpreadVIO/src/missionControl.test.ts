import { crc16Ccitt } from './protocolV2';
import {
  CalibrationWire,
  FAULT_START_POINT,
  MissionCommandRefused,
  MissionControl,
  MissionTransport,
} from './missionControl';
import { RectangleDefinition } from './rectangle';
import { RoutePlanV2 } from './protocolV2';

const rectangle: RectangleDefinition = {
  originWorld: { x: 0, y: 0, heading: 0 },
  mAxisHeadingDeg: 0,
  mFt: 20,
  nFt: 8,
  side: 'right',
  source: 'entered',
};
const calibration: CalibrationWire = {
  id: 3,
  schemaVersion: 1,
  sprayForwardFt: -0.5,
  sprayRightFt: 0,
};

function ack(epoch: number, commandId: number, state: number, calibrationId = 3, faultCode = 0) {
  const bytes = new Uint8Array(16);
  const view = new DataView(bytes.buffer);
  bytes.set([0x21, 0x41, 2, state]);
  view.setUint16(4, epoch, true);
  view.setUint32(6, commandId, true);
  view.setUint16(10, faultCode, true);
  view.setUint16(12, calibrationId, true);
  view.setUint16(14, crc16Ccitt(bytes.subarray(0, 14)), true);
  return bytes;
}

class FakeTransport implements MissionTransport {
  writes: Uint8Array[] = [];
  compatibilityStops = 0;
  listener: ((packet: Uint8Array) => void) | null = null;
  planListener: ((plan: RoutePlanV2) => void) | null = null;
  onWrite?: (packet: Uint8Array, writeNumber: number) => void | Promise<void>;

  async writeWithResponse(packet: Uint8Array) {
    this.writes.push(packet.slice());
    await this.onWrite?.(packet, this.writes.length);
  }
  async writeCompatibilityStop() { this.compatibilityStops += 1; }
  subscribeAck(listener: (packet: Uint8Array) => void) {
    this.listener = listener;
    return () => { this.listener = null; };
  }
  subscribeRoutePlan(listener: (plan: RoutePlanV2) => void) {
    this.planListener = listener;
    return () => { this.planListener = null; };
  }
  emit(packet: Uint8Array) { this.listener?.(packet); }
  emitPlan(epoch = 7) {
    this.planListener?.({
      style: 2,
      epoch,
      routeCount: 800,
      passCount: 5,
      leftRadiusFt: 7.5,
      rightRadiusFt: 6.25,
      beforeStartFt: 8.1,
      beyondEndFt: 8.4,
    });
  }
}

function packetId(packet: Uint8Array) {
  return new DataView(packet.buffer, packet.byteOffset, packet.byteLength).getUint32(6, true);
}

function autoAckConfiguration(transport: FakeTransport, epoch = 7) {
  transport.onWrite = (packet) => {
    const state = packet[1] === 0x44 ? 1 : 0;
    if (packet[1] === 0x44) transport.emitPlan(epoch);
    transport.emit(ack(epoch, packetId(packet), state));
  };
}

async function configuredControl(transport: FakeTransport, fresh = () => true) {
  autoAckConfiguration(transport);
  const control = new MissionControl(transport, 7, fresh, { timeoutMs: 5, retries: 2 });
  await control.configure(rectangle, calibration);
  return control;
}

describe('MissionControl resume', () => {
  it('sends the chosen pass in the rectangle packet, and zero by default', async () => {
    const transport = new FakeTransport();
    autoAckConfiguration(transport);
    const control = new MissionControl(transport, 7, () => true, { timeoutMs: 5, retries: 2 });
    await control.configure(rectangle, calibration, 5);
    const resumed = transport.writes.find((packet) => packet[1] === 0x44)!;
    expect(new DataView(resumed.buffer, resumed.byteOffset).getUint16(28, true)).toBe(5);

    const fresh = new FakeTransport();
    await configuredControl(fresh);
    const fromStart = fresh.writes.find((packet) => packet[1] === 0x44)!;
    expect(new DataView(fromStart.buffer, fromStart.byteOffset).getUint16(28, true)).toBe(0);
  });
});

describe('MissionControl arm refusal', () => {
  it('keeps the mission configured when the rover refuses the resume start', async () => {
    const transport = new FakeTransport();
    const control = await configuredControl(transport);
    // First Arm: the rover is not standing on the resumed pass.
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 1, 3, FAULT_START_POINT));
    await expect(control.arm()).rejects.toBeInstanceOf(MissionCommandRefused);
    expect(control.state).toBe('configured');

    // The operator moves the rover and arms again on the same mission.
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 2));
    await control.arm();
    expect(control.state).toBe('armed');
  });

  it('still treats any other Arm fault as a mission fault', async () => {
    const transport = new FakeTransport();
    const control = await configuredControl(transport);
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 1, 3, 5));
    await expect(control.arm()).rejects.toThrow(/firmware fault 5/);
    expect(control.state).toBe('fault');
  });
});

describe('MissionControl ordering and acknowledgements', () => {
  it('requires configure, a fresh pose, arm, then start', async () => {
    const transport = new FakeTransport();
    const notFresh = new MissionControl(transport, 7, () => false, { timeoutMs: 5, retries: 0 });
    await expect(notFresh.arm()).rejects.toThrow('configured');
    autoAckConfiguration(transport);
    await notFresh.configure(rectangle, calibration);
    await expect(notFresh.start()).rejects.toThrow('armed');
    await expect(notFresh.arm()).rejects.toThrow('fresh');

    const freshTransport = new FakeTransport();
    const control = await configuredControl(freshTransport);
    freshTransport.onWrite = (packet) => freshTransport.emit(ack(7, packetId(packet), 2));
    await control.arm();
    freshTransport.onWrite = (packet) => freshTransport.emit(ack(7, packetId(packet), 3));
    await control.start();
    expect(control.state).toBe('running');
  });

  it('retries twice with the same command ID and ignores wrong ACKs', async () => {
    const transport = new FakeTransport();
    const control = await configuredControl(transport);
    const beforeArm = transport.writes.length;
    transport.onWrite = (packet, writeNumber) => {
      const id = packetId(packet);
      transport.emit(ack(8, id, 2));
      transport.emit(ack(7, id + 1, 2));
      if (writeNumber === beforeArm + 2) {
        transport.emit(ack(7, id, 2));
        transport.emit(ack(7, id, 2)); // duplicate replay is idempotent
      }
    };
    await control.arm();
    const attempts = transport.writes.slice(beforeArm);
    expect(attempts).toHaveLength(2);
    expect(attempts.map(packetId)).toEqual([3, 3]);
    expect(control.state).toBe('armed');
  });

  it('times out after the initial attempt plus two retries', async () => {
    const transport = new FakeTransport();
    const control = await configuredControl(transport);
    const beforeArm = transport.writes.length;
    transport.onWrite = undefined;
    await expect(control.arm()).rejects.toThrow('ACK timeout');
    expect(transport.writes.slice(beforeArm)).toHaveLength(3);
    expect(control.state).toBe('fault');
  });

  it('rejects stale calibration acknowledgements', async () => {
    const transport = new FakeTransport();
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0, 99));
    const control = new MissionControl(transport, 7, () => true, { timeoutMs: 5, retries: 0 });
    await expect(control.configure(rectangle, calibration)).rejects.toThrow('calibration');
  });

  it('prepares and runs one dry radius measurement while idle, then reuses setup', async () => {
    const transport = new FakeTransport();
    transport.onWrite = (packet) => {
      const state = packet[1] === 0x44 ? 1 : 0;
      if (packet[1] === 0x44) transport.emitPlan();
      transport.emit(ack(7, packetId(packet), state));
    };
    const control = new MissionControl(transport, 7, () => true, {
      timeoutMs: 5,
      retries: 0,
      dryMode: true,
    });
    await control.prepareDiagnostics(calibration);
    await control.measureTurningRadii();
    await control.configure(rectangle, calibration);

    expect(transport.writes.map((packet) => [packet[1], packet[3]])).toEqual([
      [0x4b, 0],
      [0x43, 5],
      [0x44, 2],
    ]);
    expect(control.state).toBe('configured');
  });

  it('requires dry mode, prepared diagnostics, and a fresh pose for radius measurement', async () => {
    const wetTransport = new FakeTransport();
    const wet = new MissionControl(wetTransport, 7, () => true, {
      timeoutMs: 5,
      retries: 0,
      dryMode: false,
    });
    await expect(wet.measureTurningRadii()).rejects.toThrow(/dry/i);

    const transport = new FakeTransport();
    const control = new MissionControl(transport, 7, () => false, {
      timeoutMs: 5,
      retries: 0,
      dryMode: true,
    });
    await expect(control.measureTurningRadii()).rejects.toThrow(/prepared/i);
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0));
    await control.prepareDiagnostics(calibration);
    await expect(control.measureTurningRadii()).rejects.toThrow(/fresh/i);
  });

  it('requests the frozen fault dump while idle without requiring calibration identity', async () => {
    const transport = new FakeTransport();
    const control = new MissionControl(transport, 7, () => false, { timeoutMs: 5, retries: 0 });
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0, 0));
    await control.dumpFault();
    expect(transport.writes).toHaveLength(1);
    expect(transport.writes[0][1]).toBe(0x43);
    expect(transport.writes[0][3]).toBe(8);
    expect(control.state).toBe('idle');
  });

  it('runs the retained stationary self-test only after the v2 epoch is prepared', async () => {
    const transport = new FakeTransport();
    const control = new MissionControl(transport, 7, () => true, { timeoutMs: 5, retries: 0 });
    await expect(control.selfTest()).rejects.toThrow(/prepared/i);
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0));
    await control.prepareDiagnostics(calibration);
    await control.selfTest();
    expect(transport.writes.map((packet) => [packet[1], packet[3]])).toEqual([
      [0x4b, 0],
      [0x43, 4],
    ]);
  });
});

describe('MissionControl emergency stop', () => {
  it.each(['idle', 'configured', 'armed', 'running'] as const)(
    'sends acknowledged v2 Stop plus compatibility Stop from %s',
    async (target) => {
      const transport = new FakeTransport();
      const control = new MissionControl(transport, 7, () => true, { timeoutMs: 5, retries: 0 });
      if (target !== 'idle') {
        autoAckConfiguration(transport);
        await control.configure(rectangle, calibration);
      }
      if (target === 'armed' || target === 'running') {
        transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 2));
        await control.arm();
      }
      if (target === 'running') {
        transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 3));
        await control.start();
      }
      transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0));
      await control.stop();
      expect(control.state).toBe('idle');
      expect(transport.compatibilityStops).toBe(1);
      const stopPacket = transport.writes.at(-1)!;
      expect(stopPacket[1]).toBe(0x43);
      expect(stopPacket[3]).toBe(3);
    },
  );

  it('forgets prepared firmware setup after Stop so a later configure resends calibration', async () => {
    const transport = new FakeTransport();
    const control = await configuredControl(transport);
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0));
    await control.stop();
    autoAckConfiguration(transport);
    await control.configure(rectangle, calibration);
    expect(transport.writes.slice(-2).map((packet) => packet[1])).toEqual([0x4b, 0x44]);
  });

  it('priority Stop cancels a pending Configure before it can send Rectangle or Arm', async () => {
    const transport = new FakeTransport();
    let releaseCalibration!: () => void;
    const calibrationBlocked = new Promise<void>((resolve) => { releaseCalibration = resolve; });
    transport.onWrite = async (packet, writeNumber) => {
      if (writeNumber === 1) {
        await calibrationBlocked;
        transport.emit(ack(7, packetId(packet), 0));
      }
    };
    const control = new MissionControl(transport, 7, () => true, { timeoutMs: 20, retries: 0 });
    const configuring = control.configure(rectangle, calibration);
    await Promise.resolve();
    await control.stop();
    releaseCalibration();
    await expect(configuring).rejects.toThrow(/cancelled/i);
    expect(transport.writes.map((packet) => [packet[1], packet[3]])).toEqual([
      [0x4b, 0],
      [0x43, 3],
    ]);
    expect(control.state).toBe('idle');
  });
});
