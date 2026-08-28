# Reliable Pavement VIO Control Implementation Plan

> **Historical only.** This plan describes retired product behavior and must
> not be used as deployment source. The canonical app on `main` accepts entered
> rectangle dimensions only; do not restore Walk corners or Corner A/B capture.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make SafeSpread define either an entered or walked-corner pavement rectangle, follow it from fresh calibrated iPhone poses, log every mission, and fail safely when sensing, transport, power, or motion becomes untrustworthy.

**Architecture:** The Swift ARKit module emits timestamped tracking metadata; testable TypeScript converts camera pose to rover/spray-bar state, validates it, defines the rectangle, logs it, and sends bounded protocol-v2 traffic. The ESP32 accepts only current mission-scoped v2 messages, owns a deterministic safety state machine and actuator calibration, follows the preplanned route once per accepted pose, and freezes a short RAM black box on faults. Existing v1 parsing remains idle-only for rollback diagnostics and can never arm motion.

**Tech Stack:** Expo SDK 57.0.0, React Native 0.86, TypeScript 6, Swift/ARKit, `react-native-ble-plx`, `expo-file-system` ~57.0.5, `expo-sharing` ~57.0.15, Jest, Arduino ESP32 C++, host `g++` tests.

**Spec:** `docs/superpowers/specs/2026-08-26-reliable-vio-control-design.md`

## Global Constraints

- iPhone/iOS only; do not add Android behavior.
- Pavement is the target surface; keep the existing 15 percent lane overlap.
- No new rover hardware.
- Drive only from ARKit `.normal`; `.limited` and `.notAvailable` are ineligible.
- The phone is the authoritative persistent logger; ESP32 flash stores only a compact fault summary.
- Every moving fault commands neutral throttle, straight steering, and spray off; never skip a route leg.
- Protocol v2 must be versioned and mission-scoped. The new app must not arm v1 firmware.
- Preserve v1 source and parser tests through field validation; rollback is app reinstall plus firmware reflash.
- Follow Expo SDK 57 APIs documented at `https://docs.expo.dev/versions/v57.0.0/`.
- Physical calibration and wet reliability remain unclaimed until the pavement checklist is run.

## Frozen protocol-v2 layout

All multibyte values are little-endian. CRC is CRC-16/CCITT-FALSE over every
byte before the CRC (`poly=0x1021`, `init=0xFFFF`, no reflection, xor-out 0).

### Pose `!V` — 32 bytes

| Bytes | Field |
|---|---|
| 0–1 | `0x21, 0x56` (`!V`) |
| 2 | version `2` |
| 3 | flags: bit 0 tracking normal, bit 1 course valid, bit 2 calibration valid |
| 4–5 | mission epoch `uint16` |
| 6–9 | frame sequence `uint32` |
| 10–11 | capture-to-send age ms `uint16`, saturated |
| 12–15 | rover X ft `float32` |
| 16–19 | rover Y ft `float32` |
| 20–23 | chassis heading degrees `float32` |
| 24–25 | signed speed, hundredths ft/s `int16` |
| 26–27 | signed yaw rate, hundredths deg/s `int16` |
| 28–29 | calibration ID `uint16` |
| 30–31 | CRC-16 |

### Rectangle `!D` — 32 bytes

| Bytes | Field |
|---|---|
| 0–1 | `!D` |
| 2 | version `2` |
| 3 | flags: bit 0 coverage left, bit 1 dry mode, bit 2 prefer forward-only |
| 4–5 | epoch |
| 6–9 | command ID `uint32` |
| 10–13 | M/pass length ft `float32` |
| 14–17 | N/coverage width ft `float32` |
| 18–21 | clear pavement before start ft `float32` |
| 22–25 | clear pavement beyond far end ft `float32` |
| 26–27 | calibration ID |
| 28–29 | reserved zero |
| 30–31 | CRC-16 |

### Calibration `!K` — 24 bytes

| Bytes | Field |
|---|---|
| 0–1 | `!K` |
| 2 | version `2` |
| 3 | flags, reserved zero |
| 4–5 | epoch |
| 6–9 | command ID |
| 10–11 | calibration ID |
| 12–15 | spray-bar forward offset from rear axle ft `float32` |
| 16–19 | spray-bar right offset from rear axle ft `float32` |
| 20–21 | calibration schema version `uint16` |
| 22–23 | CRC-16 |

### Command `!C` — 12 bytes

Bytes 0–1 magic, byte 2 version, byte 3 opcode (`1=ARM`, `2=START`,
`3=STOP`, `4=SELF_TEST`, `5=CAL_STEER`, `6=CAL_SPEED`, `7=CAL_REVERSE`),
bytes 4–5 epoch, bytes 6–9 command ID, bytes 10–11 CRC.

### Acknowledgement `!A` — 16 bytes

Bytes 0–1 magic, byte 2 version, byte 3 state (`0=IDLE`, `1=CONFIGURED`,
`2=ARMED`, `3=RUNNING`, `4=COMPLETE`, `5=FAULT`), bytes 4–5 epoch,
bytes 6–9 command ID, bytes 10–11 fault code, bytes 12–13 calibration ID,
bytes 14–15 CRC.

### Telemetry `!T` — 32 bytes

Bytes 0–1 magic, byte 2 version, byte 3 state, bytes 4–5 epoch, bytes 6–9
consumed pose sequence, bytes 10–11 route index, bytes 12–13 route count,
bytes 14–15 signed cross-track hundredths ft, bytes 16–17 signed heading
error hundredths deg, bytes 18–19 signed speed hundredths ft/s, bytes 20–21
steering µs, bytes 22–23 throttle µs, byte 24 flags (spray/reverse/PWM OK),
byte 25 fault code, bytes 26–27 dropped packet count, bytes 28–29 pose age ms,
bytes 30–31 CRC.

### Frozen fault-buffer sample `!B` — 32 bytes

Bytes 0–1 magic, byte 2 version, byte 3 flags (bit 0 first, bit 1 last),
bytes 4–5 epoch, bytes 6–9 control sample sequence, bytes 10–11 sample index,
bytes 12–13 sample count, bytes 14–15 route index, bytes 16–17 signed
cross-track hundredths ft, bytes 18–19 signed heading error hundredths deg,
bytes 20–21 signed speed hundredths ft/s, bytes 22–23 steering µs, bytes
24–25 throttle µs, byte 26 mission state, byte 27 fault code, bytes 28–29
dropped packet count, bytes 30–31 CRC. Command opcode `8=DUMP_FAULT` requests
the frozen buffer; the firmware streams one `!B` packet per retained sample and
replays the matching command ACK after the final sample.

---

### Task 1: Protocol-v2 codec and cross-language fixtures

**Files:**
- Create: `SafeSpreadVIO/src/protocolV2.ts`
- Create: `SafeSpreadVIO/src/protocolV2.test.ts`
- Create: `auto_vio/protocol_v2.h`
- Create: `auto_vio/test/protocol_v2_test.cpp`
- Modify: `auto_vio/test/run_tests.sh`
- Preserve: `SafeSpreadVIO/src/protocol.ts`, `auto_vio/nav_math.h` v1 parsers

**Interfaces:**
- TypeScript produces `crc16Ccitt`, `buildPoseV2`, `buildRectangleV2`,
  `buildCalibrationV2`, `buildCommandV2`, `parseAckV2`, `parseTelemetryV2`,
  and `parseFaultSampleV2`.
- C++ produces matching POD structs and `parse*V2`/`build*V2` functions.
- Both suites assert the same frozen hexadecimal fixtures.

- [ ] **Step 1: Add failing TypeScript fixture tests**

Test exact sizes, headers, LE fields, CRC, signed scaling, saturation, malformed
CRC, wrong version, NaN input rejection, and ACK/telemetry/fault-sample parsing.
Use this canonical pose fixture input:

```ts
const pose: PoseV2 = {
  flags: 0b111, epoch: 0x1234, sequence: 0x01020304,
  ageMs: 125, x: 1.5, y: -2.25, heading: 359.5,
  speedFps: -1.23, yawRateDps: 45.67, calibrationId: 0xBEEF,
};
expect(buildPoseV2(pose)).toHaveLength(32);
expect(parsePoseV2(buildPoseV2(pose))).toEqual(pose);
```

- [ ] **Step 2: Run the TypeScript test and verify RED**

Run: `npm test -- --runInBand --watchman=false src/protocolV2.test.ts`

Expected: FAIL because `./protocolV2` does not exist.

- [ ] **Step 3: Implement the minimal TypeScript codec**

Use `DataView`, explicit range checks, `Number.isFinite`, and one shared
`finalizePacket(bytes)` CRC helper. Reject rather than clamp coordinates and
headings; only age and fixed-point speed/yaw fields saturate to their integer
ranges.

- [ ] **Step 4: Run TypeScript protocol tests and verify GREEN**

Run: `npm test -- --runInBand --watchman=false src/protocolV2.test.ts`

Expected: PASS.

- [ ] **Step 5: Add failing C++ tests using the emitted hex fixtures**

Copy the exact TypeScript packet hex strings into C++ byte arrays. Assert every
decoded field, CRC rejection, version rejection, non-finite float rejection,
and no output mutation on failure.

- [ ] **Step 6: Run the C++ test and verify RED**

Run from `auto_vio/test`: `g++ -std=c++17 -I. -o /tmp/protocol_v2_test protocol_v2_test.cpp`

Expected: FAIL because `../protocol_v2.h` does not exist.

- [ ] **Step 7: Implement `protocol_v2.h` and verify cross-language GREEN**

Add `protocol_v2_test` to `run_tests.sh`, then run:

`./run_tests.sh`

Expected: every existing host test and `protocol_v2_test` pass.

- [ ] **Step 8: Commit**

```bash
git add SafeSpreadVIO/src/protocolV2.ts SafeSpreadVIO/src/protocolV2.test.ts auto_vio/protocol_v2.h auto_vio/test/protocol_v2_test.cpp auto_vio/test/run_tests.sh
git commit -m "feat(protocol): add mission-scoped v2 packets"
```

### Task 2: ARKit metadata and normal-only eligibility

**Files:**
- Modify: `SafeSpreadVIO/modules/arkit-pose/ios/ArkitPoseModule.swift`
- Modify: `SafeSpreadVIO/modules/arkit-pose/src/ArkitPose.types.ts`
- Create: `SafeSpreadVIO/src/tracking.test.ts`

**Interfaces:**
- `PoseUpdatePayload` gains `frameTimestampMs`, `sequence`, `trackingReason`,
  and `mappingStatus`.
- `isPoseUsable(state)` returns true only for `normal`.

- [ ] **Step 1: Write failing tracking tests**

Assert `normal` is usable and `limited`/`notAvailable` are not. Assert every
limited reason string (`initializing`, `excessiveMotion`, `insufficientFeatures`,
`relocalizing`, `unknown`) is representable.

- [ ] **Step 2: Run and verify RED**

Run: `npm test -- --runInBand --watchman=false src/tracking.test.ts`

Expected: FAIL because `limited` currently returns true and metadata types are absent.

- [ ] **Step 3: Update TypeScript types and Swift event payload**

In Swift, map the associated limited reason explicitly, map all four
`ARFrame.WorldMappingStatus` cases, emit `frame.timestamp * 1000`, and increment
a `UInt32` sequence once per delivered frame. Add `session(_:didFailWithError:)`,
`sessionWasInterrupted`, and `sessionInterruptionEnded` events as ineligible
status updates without inventing a pose.

- [ ] **Step 4: Run tracking and full app tests**

Run: `npm test -- --runInBand --watchman=false`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add SafeSpreadVIO/modules/arkit-pose SafeSpreadVIO/src/tracking.test.ts
git commit -m "fix(app): reject degraded ARKit tracking"
```

### Task 3: Pose validation, rigid transforms, and stable arming window

**Files:**
- Create: `SafeSpreadVIO/src/posePipeline.ts`
- Create: `SafeSpreadVIO/src/posePipeline.test.ts`
- Replace ownership in: `SafeSpreadVIO/src/useVIOPose.ts`
- Modify: `SafeSpreadVIO/src/poseMath.ts`
- Modify: `SafeSpreadVIO/src/poseMath.test.ts`

**Interfaces:**

```ts
export interface MountCalibration {
  id: number; schemaVersion: number;
  cameraForwardFt: number; cameraRightFt: number; cameraYawDeg: number;
  sprayForwardFt: number; sprayRightFt: number;
}
export interface ValidatedPose {
  sequence: number; frameTimestampMs: number; receivedAtMs: number;
  camera: Pose; rover: Pose; sprayBar: Pose;
  speedFps: number; yawRateDps: number; courseDeg: number | null;
}
export type PoseDecision = { ok: true; pose: ValidatedPose } |
  { ok: false; reason: PoseRejectReason };
export class PosePipeline {
  ingest(event: PoseUpdatePayload, receivedAtMs: number): PoseDecision;
  readiness(nowMs: number): { ready: boolean; reason: string };
}
```

Default arming gate: 2.0 seconds and at least 30 accepted samples, continuous
normal tracking, newest receive age ≤150 ms, maximum position radius ≤0.10 ft,
maximum wrapped heading deviation ≤1.0 degree. Driving rejection defaults:
age >250 ms, speed >8 ft/s, acceleration >15 ft/s², yaw rate >180 deg/s, duplicate
or backward sequence/timestamp, and a position innovation >0.75 ft beyond the
distance allowed by the previous accepted speed and elapsed time.

- [ ] **Step 1: Add failing transform tests**

Cover zero offsets, a camera one foot forward at headings 0/90/180, mounting-yaw
subtraction across 0/360, and spray-bar offset. Name the production change that
breaks each assertion: camera position must no longer equal rover position when
an offset exists.

- [ ] **Step 2: Run transform tests and verify RED**

Run: `npm test -- --runInBand --watchman=false src/poseMath.test.ts`

Expected: FAIL because rigid calibration transforms do not exist.

- [ ] **Step 3: Implement rigid transforms and verify GREEN**

Keep `Pose` and angle helpers in `poseMath.ts`; add `cameraToRover` and
`roverToSprayBar`. Run the focused test until green.

- [ ] **Step 4: Add failing pipeline tests**

Use a fake monotonic clock and deterministic sequences to cover all defaults,
robust five-sample velocity/course estimation, normal-only gating, readiness
reset after one bad frame, and wrapped heading statistics near 359/0 degrees.

- [ ] **Step 5: Run pipeline tests and verify RED**

Run: `npm test -- --runInBand --watchman=false src/posePipeline.test.ts`

Expected: FAIL because `PosePipeline` does not exist.

- [ ] **Step 6: Implement the minimum pipeline and wire `useVIOPose`**

The hook exposes the latest decision, validated pose, tracking detail, readiness,
and raw event. It must not zero coordinates; rectangle-frame ownership moves to
Task 4.

- [ ] **Step 7: Run full app tests and typecheck**

Run: `npm test -- --runInBand --watchman=false`

Run: `npx tsc --noEmit`

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add SafeSpreadVIO/src/poseMath.ts SafeSpreadVIO/src/poseMath.test.ts SafeSpreadVIO/src/posePipeline.ts SafeSpreadVIO/src/posePipeline.test.ts SafeSpreadVIO/src/useVIOPose.ts
git commit -m "feat(app): validate and transform rover pose"
```

### Task 4: Entered and walked-corner rectangle definitions

**Files:**
- Create: `SafeSpreadVIO/src/rectangle.ts`
- Create: `SafeSpreadVIO/src/rectangle.test.ts`
- Modify: `SafeSpreadVIO/src/PathMap.tsx`

**Interfaces:**

```ts
export type CoverageSide = 'right' | 'left';
export interface RectangleDefinition {
  originWorld: Pose; mAxisHeadingDeg: number; mFt: number; nFt: number;
  side: CoverageSide; startClearFt: number; endClearFt: number;
  source: 'entered' | 'walked';
}
export function defineEnteredRectangle(stableRover: Pose, mFt: number, nFt: number,
  side: CoverageSide, startClearFt: number, endClearFt: number): RectangleDefinition;
export function captureCornerA(stableCamera: Pose): CornerA;
export function defineWalkedRectangle(a: CornerA, bWorld: Pose,
  startClearFt: number, endClearFt: number): RectangleDefinition;
export function worldToRectangle(world: Pose, definition: RectangleDefinition): Pose;
```

Walked capture requires diagonal length ≥3 ft, forward projection magnitude
≥1 ft, lateral projection magnitude ≥1 ft, and stable normal pose at both taps.
Negative lateral projection yields `side='left'` and positive `nFt`; it never
silently mirrors to right.

- [ ] **Step 1: Write failing rectangle tests**

Cover entered right/left, arbitrary world heading, A heading 90 degrees, walked
positive and negative lateral projection, degenerate diagonal, unstable capture,
and world-to-rectangle round trips.

- [ ] **Step 2: Run and verify RED**

Run: `npm test -- --runInBand --watchman=false src/rectangle.test.ts`

Expected: FAIL because `rectangle.ts` does not exist.

- [ ] **Step 3: Implement rectangle math and update map inputs**

Make `PathMap` receive a `RectangleDefinition` and draw the intended origin,
M arrow, N side, and headland extents. Do not add visual styling beyond labels
needed to prevent mirrored or swapped missions.

- [ ] **Step 4: Run app tests and typecheck**

Run: `npm test -- --runInBand --watchman=false`

Run: `npx tsc --noEmit`

- [ ] **Step 5: Commit**

```bash
git add SafeSpreadVIO/src/rectangle.ts SafeSpreadVIO/src/rectangle.test.ts SafeSpreadVIO/src/PathMap.tsx
git commit -m "feat(app): define rectangles two ways"
```

### Task 5: Incremental phone mission logging and export

**Files:**
- Modify: `SafeSpreadVIO/package.json`
- Modify: `SafeSpreadVIO/package-lock.json`
- Create: `SafeSpreadVIO/src/missionLog.ts`
- Create: `SafeSpreadVIO/src/missionLog.test.ts`
- Create: `SafeSpreadVIO/src/missionCsv.ts`
- Create: `SafeSpreadVIO/src/missionCsv.test.ts`

**Interfaces:**

```ts
export interface LogSink { append(line: string): Promise<void>; close(): Promise<void>; }
export class MissionLogger {
  static create(meta: MissionMetadata, sink: LogSink): Promise<MissionLogger>;
  record(record: MissionRecord): Promise<void>;
  close(): Promise<void>;
}
export function createFileLogSink(missionId: string): Promise<{ sink: LogSink; uri: string }>;
export function listMissionLogs(): MissionLogFile[];
export function exportMissionLog(uri: string): Promise<void>;
export function missionJsonlToCsv(jsonl: string): string;
```

Production uses SDK 57 `Directory`, `File`, and append-mode `FileHandle` under
`Paths.document/SafeSpread/logs`; export uses `Sharing.shareAsync`. Tests use an
in-memory sink and injected directory/share adapters, never the device filesystem.

- [ ] **Step 1: Install exact Expo dependencies**

Run from `SafeSpreadVIO`:
`npx expo install expo-file-system@~57.0.5 expo-sharing@~57.0.15`

Expected: package files list SDK-compatible versions (~57.0.5 and ~57.0.15).

- [ ] **Step 2: Write failing logger tests**

Assert metadata is first, records are serialized one JSON object per line,
concurrent `record()` calls preserve invocation order, a sink failure rejects
and permanently marks the logger failed, close flushes, and records after close
are rejected.

- [ ] **Step 3: Run and verify RED**

Run: `npm test -- --runInBand --watchman=false src/missionLog.test.ts`

Expected: FAIL because `missionLog.ts` does not exist.

- [ ] **Step 4: Implement logger and verify GREEN**

Use a single promise chain for ordered appends; do not store mission records in
React state. Create `new Directory(Paths.document, 'SafeSpread', 'logs')`, use
`Directory.create({ idempotent: true, intermediates: true })`, open each `File`
after `File.create({ intermediates: true })` with `FileMode.Append`, write UTF-8
bytes through `FileHandle.writeBytes`, close the handle, and share its `uri` with
`Sharing.shareAsync` only after checking `Sharing.isAvailableAsync()`.

- [ ] **Step 5: Write RED then GREEN CSV conversion tests**

Test stable column order, proper quoting of commas/quotes/newlines, missing
optional fields, and rejection of malformed JSONL. Implement only after the
focused test fails.

- [ ] **Step 6: Write RED then GREEN recent-log and export tests**

Using injected directory/share adapters, assert only `.jsonl` mission files are
listed, newest modification time sorts first, an unavailable share sheet is
reported without deleting the log, and the exact selected URI is shared.

- [ ] **Step 7: Run full app tests and typecheck**

Run: `npm test -- --runInBand --watchman=false`

Run: `npx tsc --noEmit`

- [ ] **Step 8: Commit**

```bash
git add SafeSpreadVIO/package.json SafeSpreadVIO/package-lock.json SafeSpreadVIO/src/missionLog.ts SafeSpreadVIO/src/missionLog.test.ts SafeSpreadVIO/src/missionCsv.ts SafeSpreadVIO/src/missionCsv.test.ts
git commit -m "feat(app): persist and export mission logs"
```

### Task 6: Bounded BLE pose sender and acknowledged commands

**Files:**
- Create: `SafeSpreadVIO/src/latestPoseSender.ts`
- Create: `SafeSpreadVIO/src/latestPoseSender.test.ts`
- Create: `SafeSpreadVIO/src/missionControl.ts`
- Create: `SafeSpreadVIO/src/missionControl.test.ts`
- Modify: `SafeSpreadVIO/src/ble.ts`

**Interfaces:**

```ts
export interface PoseTransport { writePose(packet: Uint8Array): Promise<void>; }
export class LatestPoseSender {
  offer(packet: Uint8Array): void; // one in flight, one replaceable pending
  stop(): Promise<void>;
  readonly dropped: number;
}
export class MissionControl {
  configure(def: RectangleDefinition, calibration: CalibrationWire): Promise<AckV2>;
  arm(): Promise<AckV2>;
  start(): Promise<AckV2>;
  stop(): Promise<AckV2>;
}
```

Commands use BLE writes with response and wait up to 750 ms for a matching
binary ACK; retry twice with the same command ID. Stop additionally issues the
existing one-byte `'2'` as a best-effort compatibility stop after the v2 attempt.

- [ ] **Step 1: Write failing newest-only sender tests**

Use a deferred fake transport. Offer A, B, C before A resolves; assert only A
then C are written and `dropped===1`. Cover write rejection and stop draining.

- [ ] **Step 2: Run RED, implement, and verify GREEN**

Run: `npm test -- --runInBand --watchman=false src/latestPoseSender.test.ts`

- [ ] **Step 3: Write failing mission-control tests**

Cover command ID reuse on retry, wrong epoch/ID ACK ignored, configure-before-arm,
fresh valid pose required before arm, arm-before-start, duplicate ACK idempotence,
timeout, and Stop from every state.

- [ ] **Step 4: Run RED, implement, and verify GREEN**

Run: `npm test -- --runInBand --watchman=false src/missionControl.test.ts`

- [ ] **Step 5: Refactor `ble.ts` behind injected transport methods**

Add v2 hello/ACK/telemetry parsing, use `writeCharacteristicWithResponseForService`
for commands/configuration and without-response only for poses, expose disconnect
as a mission fault, and reconnect only while idle. Do not let UI state recreate
the sender interval.

- [ ] **Step 6: Run full app verification**

Run: `npm test -- --runInBand --watchman=false`

Run: `npx tsc --noEmit`

- [ ] **Step 7: Commit**

```bash
git add SafeSpreadVIO/src/latestPoseSender.ts SafeSpreadVIO/src/latestPoseSender.test.ts SafeSpreadVIO/src/missionControl.ts SafeSpreadVIO/src/missionControl.test.ts SafeSpreadVIO/src/ble.ts
git commit -m "feat(app): bound pose writes and ack commands"
```

### Task 7: Firmware safety state and frozen RAM black box

**Files:**
- Create: `auto_vio/safety.h`
- Create: `auto_vio/fault_buffer.h`
- Create: `auto_vio/test/safety_test.cpp`
- Create: `auto_vio/test/fault_buffer_test.cpp`
- Modify: `auto_vio/test/run_tests.sh`

**Interfaces:**

```cpp
enum MissionState : uint8_t { S_IDLE, S_CONFIGURED, S_ARMED, S_RUNNING, S_COMPLETE, S_FAULT };
enum FaultCode : uint8_t { F_NONE, F_BLE, F_POSE_TIMEOUT, F_POSE_INVALID,
  F_POSE_JUMP, F_PWM, F_I2C, F_STALL, F_WRONG_DIRECTION,
  F_TRACKING_ERROR, F_ROUTE, F_CALIBRATION, F_HEADLAND };
struct SafetyInput { /* protocol, age, pose, PWM, route, motion booleans */ };
FaultCode evaluateSafety(MissionState state, const SafetyInput &in);
template <size_t N> class FaultBuffer { public: void push(ControlSample); void freeze(FaultCode); };
struct FaultSummary { /* schema, epoch, fault, route index, counters, checksum */ };
```

- [ ] **Step 1: Write failing state/fault tests**

Assert the only normal path is IDLE→CONFIGURED→ARMED→RUNNING→COMPLETE; every
listed safety condition dominates RUNNING into FAULT; Stop clears to IDLE;
Start from any non-ARMED state is rejected; and no transition ever means skip.

- [ ] **Step 2: Run safety test and verify RED**

Compile `safety_test.cpp`; expect missing `safety.h` failure.

- [ ] **Step 3: Implement safety logic and verify GREEN**

Keep this header Arduino-independent and deterministic.

- [ ] **Step 4: Write failing circular-buffer tests**

For capacity 4, push 1–6 and assert retained order 3,4,5,6; freeze, push 7, and
assert contents unchanged; assert fault code and dropped count serialize into
the frozen `!B` layout. Assert ordered chunk indices, first/last flags, and CRC.

- [ ] **Step 5: Add failing compact-summary tests**

Using an injected key/value adapter, assert one checksum-protected summary is
written on the first fault only, corrupt/schema-mismatched summaries are ignored,
and reading the summary never mutates the RAM buffer.

- [ ] **Step 6: Implement buffer/summary and run all host tests**

Run: `./run_tests.sh`

Expected: all prior and new tests pass.

- [ ] **Step 7: Commit**

```bash
git add auto_vio/safety.h auto_vio/fault_buffer.h auto_vio/test/safety_test.cpp auto_vio/test/fault_buffer_test.cpp auto_vio/test/run_tests.sh
git commit -m "feat(firmware): add fail-stop mission safety"
```

### Task 8: Integrate v2 handshake, telemetry, and safety into firmware

**Files:**
- Modify: `auto_vio/auto_vio.ino`
- Create: `auto_vio/mission_protocol.h`
- Create: `auto_vio/test/mission_protocol_test.cpp`
- Modify: `auto_vio/test/run_tests.sh`

**Interfaces:**
- `MissionProtocol` consumes decoded rectangle/calibration/command/pose messages,
  returns idempotent ACKs, and exposes the current accepted pose once per sequence.
- `auto_vio.ino` owns hardware effects only; protocol and state transitions stay
  host-testable.

- [ ] **Step 1: Write failing mission-protocol tests**

Cover v2-required arming, epoch replacement only from IDLE, duplicate command
ACK replay, stale/wrong calibration refusal, fresh normal pose before ARM,
ordered sequences, pose age 250 ms boundary, emergency Stop, and v1 idle-only
diagnostics.

- [ ] **Step 2: Run and verify RED**

Compile the focused test; expect missing `mission_protocol.h` failure.

- [ ] **Step 3: Implement protocol state and verify GREEN**

Use fixed storage only. No dynamic `String` in control-rate paths.

- [ ] **Step 4: Integrate with `auto_vio.ino`**

Replace `vioActive` and the old AUTO enum with `MissionProtocol`/`MissionState`.
Run control only when a new accepted pose sequence arrives. Emit binary ACK and
telemetry; retain low-rate text logs. Count queue overflow. On `ensurePwmReady`
failure or detected reset, command safe outputs and fault; never replay prior
moving pulses. On disconnect, fault if configured/armed/running. On first fault,
freeze the five-second RAM buffer and write its compact `FaultSummary` through
ESP32 Preferences/NVS. Accept `DUMP_FAULT` only while stopped or faulted and
stream the frozen buffer as ordered `!B` packets until reboot.

- [ ] **Step 5: Replace stall skipping with faulting**

Delete the `segmentEndIndex(...)+1` recovery branch. Route-index nonprogress plus
measured motion below threshold produces `F_STALL`; nonprogress while moving
away produces `F_ROUTE`.

- [ ] **Step 6: Run all firmware tests and compile sketch**

Run from `auto_vio/test`: `./run_tests.sh`

Run from the worktree root:
`arduino-cli compile --fqbn esp32:esp32:esp32s3 auto_vio`

Expected: syntax/type compile succeeds against the installed generic ESP32-S3
profile. The exact production board variant must still be selected at upload.

- [ ] **Step 7: Commit**

```bash
git add auto_vio/auto_vio.ino auto_vio/mission_protocol.h auto_vio/test/mission_protocol_test.cpp auto_vio/test/run_tests.sh
git commit -m "feat(firmware): require v2 arm handshake"
```

### Task 9: Headland feasibility and forward-only route preference

**Files:**
- Modify: `auto_vio/route.h`
- Modify: `auto_vio/test/route_test.cpp`
- Create: `auto_vio/headland.h`
- Create: `auto_vio/test/headland_test.cpp`
- Modify: `auto_vio/test/run_tests.sh`

**Interfaces:**

```cpp
struct RouteRequirements { float beforeStartFt; float beyondEndFt; int reversals; bool truncated; };
RouteRequirements inspectRoute(const RoutePoint*, int count, float passLengthFt);
bool headlandFits(const RouteRequirements&, float availableStartFt, float availableEndFt);
```

- [ ] **Step 1: Write failing feasibility tests**

Assert the current 21.91-foot route reports its true negative/positive M
excursions, exact-boundary clearance passes, 0.01 ft less fails, and a truncated
route can never arm.

- [ ] **Step 2: Run RED, implement, and verify GREEN**

Add the test to `run_tests.sh` and run all host tests.

- [ ] **Step 3: Add a failing forward-only preference test**

For sufficient clearance and an 18-lane field, assert the selected plan has no
reverse points and covers every lane; for insufficient continuous-turn clearance
but sufficient K-turn clearance, assert fallback uses the tested three-point
plan. Preserve 15 percent spacing.

- [ ] **Step 4: Implement the minimum route selector**

Reuse current `dubins.h` and `route.h`, which originated in commit `549ee3b`.
Port only the alternating far-lane index calculation from commit `b01e11c` into
a named helper covered by the new route tests; do not copy its stall-skip branch
and do not introduce a general-purpose planner.

- [ ] **Step 5: Run all host tests and commit**

```bash
git add auto_vio/route.h auto_vio/headland.h auto_vio/test/route_test.cpp auto_vio/test/headland_test.cpp auto_vio/test/run_tests.sh
git commit -m "feat(route): validate headland before motion"
```

### Task 10: Shared ESC direction state, steering map, and bounded speed control

**Files:**
- Create: `auto_vio/direction.h`
- Create: `auto_vio/steering_map.h`
- Create: `auto_vio/speed_control.h`
- Create: `auto_vio/test/direction_test.cpp`
- Create: `auto_vio/test/steering_map_test.cpp`
- Create: `auto_vio/test/speed_control_test.cpp`
- Modify: `auto_vio/test/run_tests.sh`
- Modify: `auto_vio/auto_vio.ino`

**Interfaces:**

```cpp
enum DrivePhase { D_NEUTRAL, D_BRAKE, D_COMMAND, D_VERIFY, D_READY, D_FAILED };
struct SteeringKnot { int pulseUs; float curvaturePerFt; };
float pulseForCurvature(const SteeringKnot*, int count, float requested);
struct SpeedPI { float feedForwardUs, integralUs; int update(float target, float measured, float dt); };
```

Direction timing is shared by self-test and navigation: 800 ms neutral, optional
300 ms brake pulse for reverse, command, then verification after ≥0.30 ft or a
2.0-second timeout. Wrong sign or no displacement fails.

Speed defaults: 1.0 ft/s straight and 0.7 ft/s turn, feed-forward learned by
calibration, proportional 30 µs/(ft/s), integral 8 µs/(ft), ±250 µs correction,
and a 350 µs absolute offset limit. A 1.0-second no-motion condition faults; it
does not keep winding throttle.

- [ ] **Step 1: Add and run failing direction tests**

Cover forward→reverse, reverse→forward, brake-required reverse, verification,
wrong direction, timeout, and identical use by self-test/navigation wrappers.

- [ ] **Step 2: Implement direction state and verify GREEN**

- [ ] **Step 3: Add and run failing steering-map tests**

Cover monotonic validation, direct straight pulse, interpolation on both sides,
clamping, asymmetric curvature, and invalid calibration refusal.

- [ ] **Step 4: Implement steering map and verify GREEN**

- [ ] **Step 5: Add and run failing speed-control tests**

Cover feed-forward at target, bounded PI, anti-windup at both limits, breakaway
ramp, bad `dt`, no-motion timeout, and reverse sign.

- [ ] **Step 6: Implement speed control and verify GREEN**

- [ ] **Step 7: Integrate all three into firmware**

Replace inferred steering center as authority, fixed moving throttle pulses, and
the separate 350 ms navigation reversal pause. Keep the old constants only as
fallback values in dry calibration mode. Feed every failure to `SafetyInput`.

- [ ] **Step 8: Run all host tests and sketch compile, then commit**

```bash
git add auto_vio/direction.h auto_vio/steering_map.h auto_vio/speed_control.h auto_vio/test/direction_test.cpp auto_vio/test/steering_map_test.cpp auto_vio/test/speed_control_test.cpp auto_vio/test/run_tests.sh auto_vio/auto_vio.ino
git commit -m "feat(firmware): calibrate motion control"
```

### Task 11: Calibration workflows and persistence

**Files:**
- Create: `SafeSpreadVIO/src/calibration.ts`
- Create: `SafeSpreadVIO/src/calibration.test.ts`
- Create: `SafeSpreadVIO/src/calibrationStore.ts`
- Create: `SafeSpreadVIO/src/calibrationStore.test.ts`
- Create: `auto_vio/calibration.h`
- Create: `auto_vio/test/calibration_test.cpp`
- Modify: `auto_vio/auto_vio.ino`
- Modify: `auto_vio/test/run_tests.sh`

**Interfaces:**
- Phone calibration schema stores mount offsets/yaw, spray offsets, timestamp,
  surface (`asphalt|concrete|pavers|other`), wet/dry, and a stable `uint16` ID
  computed from canonical fields.
- Firmware calibration fits steering knots and forward/reverse speed
  feed-forward, reports samples to the phone log, and stores only the accepted
  compact calibration plus schema/ID in ESP32 Preferences/NVS.

- [ ] **Step 1: Write failing phone yaw-calibration tests**

Use forward and return sample paths. Assert angular wrap, robust line fit,
minimum 6 ft per direction, curvature rejection, >2 degree disagreement
rejection, and repeatable calibration ID.

- [ ] **Step 2: Implement phone calibration and verify GREEN**

- [ ] **Step 3: Write failing persistence tests**

Inject a key/value adapter. Cover schema migration refusal, stale hardware tag,
round trip, corrupt JSON, and dry-only behavior when missing.

- [ ] **Step 4: Implement phone store and verify GREEN**

Production may use a single JSON file under the app document directory; do not
add another storage dependency.

- [ ] **Step 5: Write failing firmware fit tests**

Cover direct straight pulse, monotonic knot fit, noisy repeated arcs, invalid
direction, insufficient sweep, feed-forward median, and calibration ID match.

- [ ] **Step 6: Implement firmware calibration logic and integrate commands**

Calibration commands are allowed only from IDLE, dry mode, valid normal pose,
and operator confirmation. Reuse the exact direction state and safety faults
from Task 10. Never duplicate ESC timing in a calibration sketch.

- [ ] **Step 7: Run all app/firmware tests and commit**

```bash
git add SafeSpreadVIO/src/calibration.ts SafeSpreadVIO/src/calibration.test.ts SafeSpreadVIO/src/calibrationStore.ts SafeSpreadVIO/src/calibrationStore.test.ts auto_vio/calibration.h auto_vio/test/calibration_test.cpp auto_vio/auto_vio.ino auto_vio/test/run_tests.sh
git commit -m "feat: add measured pavement calibration"
```

### Task 12: Setup wizard, mission wiring, and fault UI

**Files:**
- Create: `SafeSpreadVIO/src/setupMachine.ts`
- Create: `SafeSpreadVIO/src/setupMachine.test.ts`
- Create: `SafeSpreadVIO/src/SetupWizard.tsx`
- Create: `SafeSpreadVIO/src/RunningMission.tsx`
- Modify: `SafeSpreadVIO/App.tsx`

**Interfaces:**
- Pure `setupReducer` owns connection→rectangle→calibration→readiness→arm→run
  transitions and rejects illegal UI actions.
- `SetupWizard` renders both approved rectangle modes and explicit headland,
  coverage-side, pavement, mount-offset, dry/wet, readiness, and acknowledgement
  controls.
- `RunningMission` owns Stop, current fault, telemetry, map, and log filename.
- The setup/history screen lists recent JSONL missions and offers JSONL or
  derived CSV export; the fault screen offers download of the ESP32 RAM buffer.

- [ ] **Step 1: Write failing reducer tests**

Cover both complete workflows, unstable A/B rejection, left-side confirmation,
return-to-A gate, invalid dimensions/headland, incompatible firmware, missing or
stale calibration, logging failure blocking wet only, ACK timeouts, and Stop
from every phase. Cover recent-log ordering/export and `DUMP_FAULT` assembly,
including missing/out-of-order chunks and CRC failure.

- [ ] **Step 2: Run RED, implement reducer, and verify GREEN**

- [ ] **Step 3: Build the wizard components**

Move existing controls without redesigning their visual language. Keep Stop
always visible in running/fault states. Add stable tracking reason and the
calculated rectangle preview. Do not add navigation libraries.

- [ ] **Step 4: Wire the end-to-end app flow**

Create the logger before Configure, generate epoch, configure calibration and
rectangle, stream validated rectangle-frame rover poses through
`LatestPoseSender`, wait for Arm/Start ACKs, record telemetry joined by pose
sequence, and close/export the log on Stop/Complete/Fault. On a firmware fault,
request the frozen buffer while stopped, validate/reassemble `!B` chunks, and
append them plus the persisted firmware summary to the phone mission log.

- [ ] **Step 5: Run app tests and typecheck**

Run: `npm test -- --runInBand --watchman=false`

Run: `npx tsc --noEmit`

- [ ] **Step 6: Commit**

```bash
git add SafeSpreadVIO/src/setupMachine.ts SafeSpreadVIO/src/setupMachine.test.ts SafeSpreadVIO/src/SetupWizard.tsx SafeSpreadVIO/src/RunningMission.tsx SafeSpreadVIO/App.tsx
git commit -m "feat(app): add safe rectangle setup wizard"
```

### Task 13: Replay harness and pavement field handoff

**Files:**
- Create: `auto_vio/replay/replay.cpp`
- Create: `auto_vio/replay/fixtures/straight_pass.csv`
- Create: `auto_vio/test/replay_test.sh`
- Modify: `auto_vio/test/run_tests.sh`
- Create: `docs/field-validation-pavement.md`

**Interfaces:**
- Replay reads the exported CSV columns `phone_ms,sequence,epoch,x_ft,y_ft,
  heading_deg,speed_fps,yaw_rate_dps,tracking_valid,route_index,cross_track_ft,
  heading_error_deg,steering_us,throttle_us,fault` and runs shared protocol,
  safety, route-progress, and line-control helpers without Arduino hardware.
- Output is one summary line plus nonzero exit for rejected acceptance:
  accepted/rejected samples, maximum/p95 cross-track, maximum pose age, steering
  saturation percent, fault code, and final route index.

- [ ] **Step 1: Add a failing replay shell test**

Compile and run the fixture; expect missing replay source failure. The fixture
contains one clean straight pass and one deliberately stale frame; expected
summary rejects exactly one sample and has no mission fault.

- [ ] **Step 2: Implement minimal CSV parser and shared-logic replay**

Use C++ standard library only. Reject a missing header, non-finite field, wrong
epoch, or malformed row with line number.

- [ ] **Step 3: Run replay and all host tests**

Run from `auto_vio/test`: `./run_tests.sh`

Expected: all tests, including replay fixture, pass.

- [ ] **Step 4: Write the exact pavement field checklist**

Document equipment, dry/wet distinction, operator stand-clear rules, expected
UI gates, log export after each stage, and stop criteria in the spec's nine-step
order. Include the unchanged 2.55-inch overlap budget and state that wet use is
not approved by automated tests.

- [ ] **Step 5: Run final repository verification**

Run: `git diff --check`

Run from `SafeSpreadVIO`: `npm test -- --runInBand --watchman=false`

Run from `SafeSpreadVIO`: `npx tsc --noEmit`

Run from `auto_vio/test`: `./run_tests.sh`

Run from the worktree root:
`arduino-cli compile --fqbn esp32:esp32:esp32s3 auto_vio`

- [ ] **Step 6: Commit**

```bash
git add auto_vio/replay docs/field-validation-pavement.md auto_vio/test/replay_test.sh auto_vio/test/run_tests.sh
git commit -m "test: add pavement mission replay gate"
```

## Completion boundary

Stop after automated verification and delivery of the pavement field checklist.
Report the exact Arduino/iOS build status and any hardware-only steps. Do not
claim straight-line, wet-surface, or full-rectangle physical reliability until
the user returns exported calibration and dry-run logs for replay and tuning.
