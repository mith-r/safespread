# Phone-VIO Waypoint Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `auto.ino`'s open-loop, mis-spaced dead reckoning with a system where an iPhone's ARKit visual-inertial pose is the sole position/heading source, streamed over BLE to a new ESP32 sketch that drives a bar-width-correct waypoint grid.

**Architecture:** A custom Expo Module (Swift) runs an ARKit `ARWorldTrackingConfiguration` session and emits `{x, y, heading, trackingState}` pose events to JS. The app zeroes that pose on "Start/Reset" and streams it as a 15-byte `!P` BLE packet every 100ms via `react-native-ble-plx`. A new ESP32 sketch (`auto_vio/auto_vio.ino`) parses that packet, generates a waypoint list sized to the real 17" spray bar, and steers to each waypoint in sequence. Pure logic on both sides (packet framing, pose zero-referencing, waypoint generation, navigation math) is unit-tested; hardware/sensor-dependent glue (ARKit session, BLE central, PCA9685 output) is verified manually on-device per the spec's test plan.

**Tech Stack:** Expo (TypeScript, blank template) + Expo Modules API (Swift) for ARKit access, `react-native-ble-plx` for BLE central, `jest-expo` for TS unit tests, Arduino (`arduino-esp32` core) + `Adafruit_PWMServoDriver` + built-in `BLEDevice` for firmware, `arduino-cli` for compile verification, plain `g++`-compiled asserts for firmware pure-logic tests (no framework — matches this repo's existing no-test-infra convention while still giving the risky math real coverage).

**Spec:** `docs/superpowers/specs/2026-08-25-vio-waypoint-nav-design.md`

## Global Constraints

- Field is a 480 sqft square, side length `21.91` ft.
- Spray bar is `17` in (`1.41667` ft) wide — all lane spacing must derive from this, never be hardcoded independent of it.
- `!P` packet format is frozen exactly as specified: 15 bytes — `[0]=0x21 '!'`, `[1]=0x50 'P'`, `[2..5]`=x feet float32 LE, `[6..9]`=y feet float32 LE, `[10..13]`=heading degrees (0-360) float32 LE, `[14]`=CRC=`~(sum of bytes 0-13) & 0xFF`.
- BLE device name `SafeSpread`; Nordic UART service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`, RX char `6E400002-...`, TX char `6E400003-...`.
- No new rover hardware beyond the phone. iOS-only — no Android code paths.
- Firmware VIO-timeout safety stop: no valid `!P` packet for >1000ms → hard stop, mission resets to idle.
- The phone is the *sole* pose source for this sketch. `OLD/auto/auto.ino` (Bluefruit-heading + internal dead reckoning) is left untouched as a reference/fallback, not modified.

---

## Task 1: Firmware pure logic — packet parsing

**Files:**
- Create: `auto_vio/nav_math.h`
- Test: `auto_vio/test/parse_test.cpp`

**Interfaces:**
- Produces: `bool parsePosePacket(const uint8_t* d, size_t n, float& x, float& y, float& heading)` — returns `false` (leaving x/y/heading untouched) if `n < 15`, the header bytes don't match `0x21 0x50`, or the CRC check fails.

- [ ] **Step 1: Write the failing test**

```cpp
// auto_vio/test/parse_test.cpp
#include <cassert>
#include <cstdio>
#include <cstring>
#include "../nav_math.h"

static void buildTestPacket(uint8_t* out, float x, float y, float heading) {
  out[0] = 0x21;
  out[1] = 0x50;
  memcpy(out + 2, &x, 4);
  memcpy(out + 6, &y, 4);
  memcpy(out + 10, &heading, 4);
  uint8_t sum = 0;
  for (int i = 0; i < 14; i++) sum += out[i];
  out[14] = (uint8_t)(~sum);
}

int main() {
  uint8_t packet[15];
  buildTestPacket(packet, 2.5f, -1.25f, 90.0f);

  float x = 0, y = 0, heading = 0;
  bool ok = parsePosePacket(packet, sizeof(packet), x, y, heading);
  assert(ok);
  assert(x == 2.5f);
  assert(y == -1.25f);
  assert(heading == 90.0f);

  uint8_t corrupted[15];
  memcpy(corrupted, packet, 15);
  corrupted[14] ^= 0xFF;
  float cx, cy, ch;
  assert(!parsePosePacket(corrupted, sizeof(corrupted), cx, cy, ch));

  assert(!parsePosePacket(packet, 14, x, y, heading));

  uint8_t badHeader[15];
  memcpy(badHeader, packet, 15);
  badHeader[1] = 'X';
  assert(!parsePosePacket(badHeader, sizeof(badHeader), x, y, heading));

  printf("parse_test: all assertions passed\n");
  return 0;
}
```

- [ ] **Step 2: Run test to verify it fails to compile**

Run: `g++ -std=c++17 -I. -o /tmp/parse_test auto_vio/test/parse_test.cpp`
Expected: FAIL — `nav_math.h` does not exist yet / `parsePosePacket` undeclared.

- [ ] **Step 3: Write minimal implementation**

```cpp
// auto_vio/nav_math.h
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>

inline bool parsePosePacket(const uint8_t* d, size_t n, float& x, float& y, float& heading) {
  if (n < 15 || d[0] != 0x21 || d[1] != 0x50) return false;

  uint8_t sum = 0;
  for (int i = 0; i < 14; i++) sum += d[i];
  uint8_t crc = (uint8_t)(~sum);
  if (crc != d[14]) return false;

  memcpy(&x, d + 2, 4);
  memcpy(&y, d + 6, 4);
  memcpy(&heading, d + 10, 4);
  return true;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `g++ -std=c++17 -I. -o /tmp/parse_test auto_vio/test/parse_test.cpp && /tmp/parse_test`
Expected: `parse_test: all assertions passed`

- [ ] **Step 5: Commit**

```bash
git add auto_vio/nav_math.h auto_vio/test/parse_test.cpp
git commit -m "feat(firmware): add !P packet parser with CRC check"
```

---

## Task 2: Firmware pure logic — waypoint generation

**Files:**
- Modify: `auto_vio/nav_math.h`
- Test: `auto_vio/test/waypoints_test.cpp`

**Interfaces:**
- Consumes: nothing from Task 1.
- Produces: `struct Waypoint { float x; float y; };` and `int buildWaypoints(float fieldSideFt, float barWidthFt, float overlapFraction, Waypoint* out, int maxOut)` — returns the number of waypoints written to `out`. Lane spacing is `barWidthFt * (1 - overlapFraction)`, rounded so an integer number of lanes evenly tile `fieldSideFt` (never leaving a gap wider than that spacing). Waypoints alternate pass-end / lane-shift, matching the boustrophedon pattern: `out[0]` is the first pass's far end; for each subsequent lane `i`, a shift waypoint at the new x (same y as the previous pass's end) is followed by that lane's own pass-end waypoint.

- [ ] **Step 1: Write the failing test**

```cpp
// auto_vio/test/waypoints_test.cpp
#include <cassert>
#include <cstdio>
#include <cmath>
#include "../nav_math.h"

int main() {
  // Clean round-number case: field 10ft, bar 2ft, no overlap margin.
  Waypoint wp[32];
  int count = buildWaypoints(10.0f, 2.0f, 0.0f, wp, 32);

  Waypoint expected[] = {
    {0, 10}, {2, 10}, {2, 0}, {4, 0}, {4, 10},
    {6, 10}, {6, 0}, {8, 0}, {8, 10}, {10, 10}, {10, 0}
  };
  int expectedCount = sizeof(expected) / sizeof(Waypoint);
  assert(count == expectedCount);
  for (int i = 0; i < expectedCount; i++) {
    assert(fabsf(wp[i].x - expected[i].x) < 0.001f);
    assert(fabsf(wp[i].y - expected[i].y) < 0.001f);
  }

  // Real-world case: 480 sqft field, 17in bar, 15% overlap margin.
  // Must never leave a gap wider than the bar, and must fully span the field.
  const float FIELD = 21.91f;
  const float BAR = 17.0f / 12.0f;
  Waypoint real[64];
  int realCount = buildWaypoints(FIELD, BAR, 0.15f, real, 64);
  assert(realCount > 0);
  assert(fabsf(real[0].x - 0.0f) < 0.01f);
  assert(fabsf(real[realCount - 1].x - FIELD) < 0.01f);

  float prevX = -1.0f;
  for (int i = 0; i < realCount; i++) {
    if (real[i].x != prevX) {
      if (prevX >= 0.0f) {
        float gap = real[i].x - prevX;
        assert(gap <= BAR + 0.001f);
      }
      prevX = real[i].x;
    }
  }

  printf("waypoints_test: all assertions passed (round-number count=%d, real-world count=%d)\n",
         count, realCount);
  return 0;
}
```

- [ ] **Step 2: Run test to verify it fails to compile**

Run: `g++ -std=c++17 -I. -o /tmp/waypoints_test auto_vio/test/waypoints_test.cpp`
Expected: FAIL — `Waypoint`/`buildWaypoints` undeclared.

- [ ] **Step 3: Write minimal implementation**

Append to `auto_vio/nav_math.h`:

```cpp
struct Waypoint {
  float x;
  float y;
};

inline int buildWaypoints(float fieldSideFt, float barWidthFt, float overlapFraction,
                           Waypoint* out, int maxOut) {
  float spacingTarget = barWidthFt * (1.0f - overlapFraction);
  int lanes = (int)ceilf(fieldSideFt / spacingTarget) + 1;
  if (lanes < 2) lanes = 2;
  float spacing = fieldSideFt / (float)(lanes - 1);

  auto endY = [&](int lane) -> float {
    return (lane % 2 == 0) ? fieldSideFt : 0.0f;
  };

  int idx = 0;
  if (idx < maxOut) out[idx++] = { 0.0f, endY(0) };
  for (int lane = 1; lane < lanes && idx + 1 < maxOut; lane++) {
    float x = lane * spacing;
    out[idx++] = { x, endY(lane - 1) };
    out[idx++] = { x, endY(lane) };
  }
  return idx;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `g++ -std=c++17 -I. -o /tmp/waypoints_test auto_vio/test/waypoints_test.cpp && /tmp/waypoints_test`
Expected: `waypoints_test: all assertions passed (round-number count=11, real-world count=39)`

(If the real-world count differs slightly, that's fine as long as both assertions pass — the exact count depends on the overlap constant.)

- [ ] **Step 5: Commit**

```bash
git add auto_vio/nav_math.h auto_vio/test/waypoints_test.cpp
git commit -m "feat(firmware): generate waypoint list sized to real spray bar width"
```

---

## Task 3: Firmware pure logic — navigation math

**Files:**
- Modify: `auto_vio/nav_math.h`
- Test: `auto_vio/test/nav_test.cpp`

**Interfaces:**
- Consumes: nothing from Tasks 1-2.
- Produces: `float angleDiffDeg(float target, float current)` (shortest signed difference, wrapped to [-180, 180]); `float bearingToWaypointDeg(float dx, float dy)` (0=+Y/forward, 90=+X/right, wrapped to [0, 360)); `bool waypointReached(float dx, float dy, float toleranceFt)`.

- [ ] **Step 1: Write the failing test**

```cpp
// auto_vio/test/nav_test.cpp
#include <cassert>
#include <cstdio>
#include <cmath>
#include "../nav_math.h"

int main() {
  assert(fabsf(angleDiffDeg(10.0f, 350.0f) - 20.0f) < 0.01f);
  assert(fabsf(angleDiffDeg(350.0f, 10.0f) - (-20.0f)) < 0.01f);
  assert(fabsf(angleDiffDeg(100.0f, 90.0f) - 10.0f) < 0.01f);

  assert(fabsf(bearingToWaypointDeg(0.0f, 1.0f) - 0.0f) < 0.01f);
  assert(fabsf(bearingToWaypointDeg(1.0f, 0.0f) - 90.0f) < 0.01f);
  assert(fabsf(bearingToWaypointDeg(0.0f, -1.0f) - 180.0f) < 0.01f);
  assert(fabsf(bearingToWaypointDeg(-1.0f, 0.0f) - 270.0f) < 0.01f);

  assert(waypointReached(0.3f, 0.3f, 0.5f));
  assert(!waypointReached(0.4f, 0.4f, 0.5f));

  printf("nav_test: all assertions passed\n");
  return 0;
}
```

- [ ] **Step 2: Run test to verify it fails to compile**

Run: `g++ -std=c++17 -I. -o /tmp/nav_test auto_vio/test/nav_test.cpp`
Expected: FAIL — the three functions are undeclared.

- [ ] **Step 3: Write minimal implementation**

Append to `auto_vio/nav_math.h`:

```cpp
inline float angleDiffDeg(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

inline float bearingToWaypointDeg(float dx, float dy) {
  float b = atan2f(dx, dy) * (180.0f / (float)M_PI);
  if (b < 0.0f) b += 360.0f;
  return b;
}

inline bool waypointReached(float dx, float dy, float toleranceFt) {
  return (dx * dx + dy * dy) <= (toleranceFt * toleranceFt);
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `g++ -std=c++17 -I. -o /tmp/nav_test auto_vio/test/nav_test.cpp && /tmp/nav_test`
Expected: `nav_test: all assertions passed`

- [ ] **Step 5: Commit**

```bash
git add auto_vio/nav_math.h auto_vio/test/nav_test.cpp
git commit -m "feat(firmware): add heading/bearing/waypoint-tolerance math"
```

---

## Task 4: Assemble `auto_vio.ino` and verify it compiles

**Files:**
- Create: `auto_vio/auto_vio.ino`

**Interfaces:**
- Consumes: `parsePosePacket` (Task 1), `Waypoint`/`buildWaypoints` (Task 2), `angleDiffDeg`/`bearingToWaypointDeg`/`waypointReached` (Task 3) — all from `nav_math.h`.
- Produces: the flashable sketch. Nothing downstream consumes this in-repo; it's the firmware endpoint.

- [ ] **Step 1: Install the ESP32 toolchain (one-time, if not already present)**

Run:
```bash
brew install arduino-cli
arduino-cli config init
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit PWM Servo Driver Library"
```
Expected: each command exits 0. `arduino-cli core list` should show `esp32:esp32`.

- [ ] **Step 2: Write `auto_vio/auto_vio.ino`**

```cpp
/*
 * SafeSpread - Phone-VIO Waypoint Navigator
 *
 * Position/heading come entirely from the SafeSpreadVIO iPhone app's ARKit
 * pose, sent as !P packets over BLE. No internal dead reckoning.
 *
 * Controls:
 *   '1' -> Start Autonomous Mission
 *   '2' -> Emergency Stop / Reset
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include "nav_math.h"

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define PCA9685_ADDR 0x40
const uint8_t STEER_CH = 0;
const uint8_t ESC_CH   = 4;

const int VALVE_PIN = 5;
const int PUMP_PIN  = 6;

const int NEUTRAL_US       = 1500;
const int THROTTLE_FWD_US  = 1620;
const int THROTTLE_TURN_US = 1600;

const int STEER_CENTER_US = 1500;
const int STEER_LEFT_US   = 2390;
const int STEER_RIGHT_US  = 700;

const float BAR_WIDTH_FT          = 17.0f / 12.0f;
const float LANE_OVERLAP_FRACTION = 0.15f;
const float FIELD_SIDE_FT         = 21.91f;
const float WAYPOINT_TOLERANCE_FT = 0.5f;
const unsigned long VIO_TIMEOUT_MS = 1000;

// Sized with headroom above the ~39 waypoints the current field/bar/overlap
// constants produce (see buildWaypoints in nav_math.h).
const int MAX_WAYPOINTS = 48;
Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;
int currentWaypointIndex = 0;

float robotX_ft    = 0.0f;
float robotY_ft    = 0.0f;
float robotHeading = 0.0f;
bool  vioActive    = false;
unsigned long lastVioTime = 0;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
BLECharacteristic *txCharacteristic = NULL;
volatile bool bleConnected = false;

enum AutoState { AUTO_IDLE, AUTO_NAVIGATING, AUTO_COMPLETE };
AutoState state = AUTO_IDLE;

void setChannelPulse(uint8_t channel, int microseconds) {
  uint16_t ticks = (uint16_t)(((uint32_t)microseconds * 4096UL) / 20000UL);
  pwm.setPWM(channel, 0, ticks);
}

void setSpray(bool on) {
  digitalWrite(VALVE_PIN, on ? HIGH : LOW);
  digitalWrite(PUMP_PIN, LOW);
}

void stopDrive() {
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, STEER_CENTER_US);
}

void bleLog(String msg) {
  Serial.println(msg);
  if (bleConnected && txCharacteristic != NULL) {
    msg += "\n";
    txCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    txCharacteristic->notify();
  }
}

void navigateToWaypoint(const Waypoint &wp) {
  float dx = wp.x - robotX_ft;
  float dy = wp.y - robotY_ft;

  float targetHeading = bearingToWaypointDeg(dx, dy);
  float err = angleDiffDeg(targetHeading, robotHeading);

  int steer = STEER_CENTER_US + (int)(err * 25.0f);
  steer = constrain(steer, STEER_RIGHT_US, STEER_LEFT_US);
  setChannelPulse(STEER_CH, steer);

  if (fabsf(err) > 45.0f) {
    setChannelPulse(ESC_CH, THROTTLE_TURN_US);
  } else {
    setChannelPulse(ESC_CH, THROTTLE_FWD_US);
  }

  if (waypointReached(dx, dy, WAYPOINT_TOLERANCE_FT)) {
    currentWaypointIndex++;
    stopDrive();
    setSpray(false);

    if (currentWaypointIndex >= waypointCount) {
      state = AUTO_COMPLETE;
      bleLog("=== MISSION COMPLETE ===");
    } else {
      bleLog(">>> Reached waypoint " + String(currentWaypointIndex) + "/" + String(waypointCount) +
             ". Next: (" + String(waypoints[currentWaypointIndex].x, 1) + ", " +
             String(waypoints[currentWaypointIndex].y, 1) + ")");
      if (currentWaypointIndex % 2 == 0) {
        setSpray(true);
      }
    }
  }
}

#define QSLOTS 16
#define QBYTES 32
static uint8_t qData[QSLOTS][QBYTES];
static uint8_t qLen[QSLOTS];
static volatile uint8_t qHead = 0, qTail = 0;

void queueWrite(const uint8_t *d, size_t n) {
  uint8_t next = (qHead + 1) % QSLOTS;
  if (next == qTail) return;
  if (n > QBYTES) n = QBYTES;
  memcpy(qData[qHead], d, n);
  qLen[qHead] = n;
  qHead = next;
}

static uint8_t acc[64];
static size_t  accLen = 0;

void feed(const uint8_t *d, size_t n) {
  if (n == 1) {
    if (d[0] == '1' && state == AUTO_IDLE) {
      state = AUTO_NAVIGATING;
      currentWaypointIndex = 0;
      setSpray(true);
      bleLog(">>> Mission started. " + String(waypointCount) + " waypoints.");
    } else if (d[0] == '2') {
      state = AUTO_IDLE;
      stopDrive();
      setSpray(false);
      bleLog(">>> Mission stopped.");
    }
    return;
  }

  for (size_t i = 0; i < n; i++) {
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }

  size_t i = 0;
  while (accLen - i >= 15) {
    if (acc[i] != '!') { i++; continue; }
    float x, y, heading;
    if (parsePosePacket(&acc[i], accLen - i, x, y, heading)) {
      robotX_ft = x;
      robotY_ft = y;
      robotHeading = heading;
      vioActive = true;
      lastVioTime = millis();
      i += 15;
    } else {
      i++;
    }
  }
  if (i > 0) { memmove(acc, acc + i, accLen - i); accLen -= i; }
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    uint8_t *d = c->getData();
    size_t   n = c->getLength();
    if (d && n) queueWrite(d, n);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    bleConnected = true;
    bleLog("VIO app connected.");
  }
  void onDisconnect(BLEServer *s) {
    bleConnected = false;
    vioActive = false;
    state = AUTO_IDLE;
    stopDrive();
    setSpray(false);
    s->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  setSpray(false);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(50);
  stopDrive();

  waypointCount = buildWaypoints(FIELD_SIDE_FT, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION,
                                  waypoints, MAX_WAYPOINTS);
  Serial.println("Built " + String(waypointCount) + " waypoints.");

  BLEDevice::init("SafeSpread");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *svc = server->createService(NUS_SERVICE_UUID);
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  txCharacteristic = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("VIO Waypoint Navigator ready.");
}

void loop() {
  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  if (vioActive && (millis() - lastVioTime > VIO_TIMEOUT_MS)) {
    vioActive = false;
    stopDrive();
    if (state == AUTO_NAVIGATING) {
      bleLog("!!! VIO signal lost. Stopping. !!!");
      state = AUTO_IDLE;
    }
  }

  if (state == AUTO_NAVIGATING) {
    if (vioActive) {
      navigateToWaypoint(waypoints[currentWaypointIndex]);
    } else {
      stopDrive();
    }
  }
}
```

- [ ] **Step 3: Compile-check the sketch**

Run: `arduino-cli compile --fqbn esp32:esp32:esp32s3 auto_vio`
Expected: `Sketch uses ... bytes` success output, no errors. (This FQBN is a generic ESP32-S3 dev module, used here purely to catch syntax/type errors — adjust it to match the exact board variant at actual upload time.)

- [ ] **Step 4: Commit**

```bash
git add auto_vio/auto_vio.ino
git commit -m "feat(firmware): assemble phone-VIO waypoint navigator sketch"
```

---

## Task 5: Scaffold the SafeSpreadVIO app and add the BLE packet builder

**Files:**
- Create: `SafeSpreadVIO/` (Expo app, via template)
- Create: `SafeSpreadVIO/src/protocol.ts`
- Test: `SafeSpreadVIO/src/protocol.test.ts`

**Interfaces:**
- Produces: `buildPosePacket(x: number, y: number, heading: number): Uint8Array` — 15-byte packet per the frozen `!P` format.

- [ ] **Step 1: Scaffold the app and install dependencies**

Run (from the `safespread/` project root):
```bash
npx create-expo-app@latest SafeSpreadVIO --template blank-typescript
cd SafeSpreadVIO
npx expo install expo-camera expo-dev-client
npm install react-native-ble-plx buffer
npm install --save-dev jest-expo jest @types/jest
```

Add to `SafeSpreadVIO/package.json` (merge into existing `scripts`/top-level keys):
```json
{
  "scripts": {
    "test": "jest"
  },
  "jest": {
    "preset": "jest-expo"
  }
}
```

- [ ] **Step 2: Write the failing test**

```ts
// SafeSpreadVIO/src/protocol.test.ts
import { buildPosePacket } from './protocol';

describe('buildPosePacket', () => {
  it('writes header, floats little-endian, and a matching CRC', () => {
    const packet = buildPosePacket(2.5, -1.25, 90);
    expect(packet.length).toBe(15);
    expect(packet[0]).toBe(0x21);
    expect(packet[1]).toBe(0x50);

    const view = new DataView(packet.buffer);
    expect(view.getFloat32(2, true)).toBeCloseTo(2.5);
    expect(view.getFloat32(6, true)).toBeCloseTo(-1.25);
    expect(view.getFloat32(10, true)).toBeCloseTo(90);

    let sum = 0;
    for (let i = 0; i < 14; i++) sum += packet[i];
    expect(packet[14]).toBe((~sum) & 0xFF);
  });

  it('produces a different CRC when a field changes', () => {
    const a = buildPosePacket(0, 0, 0);
    const b = buildPosePacket(1, 0, 0);
    expect(a[14]).not.toBe(b[14]);
  });
});
```

- [ ] **Step 3: Run test to verify it fails**

Run: `npx jest src/protocol.test.ts`
Expected: FAIL — `Cannot find module './protocol'`

- [ ] **Step 4: Write minimal implementation**

```ts
// SafeSpreadVIO/src/protocol.ts
export function buildPosePacket(x: number, y: number, heading: number): Uint8Array {
  const buf = new ArrayBuffer(15);
  const view = new DataView(buf);
  const bytes = new Uint8Array(buf);

  bytes[0] = 0x21;
  bytes[1] = 0x50;
  view.setFloat32(2, x, true);
  view.setFloat32(6, y, true);
  view.setFloat32(10, heading, true);

  let sum = 0;
  for (let i = 0; i < 14; i++) sum += bytes[i];
  bytes[14] = (~sum) & 0xFF;

  return bytes;
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `npx jest src/protocol.test.ts`
Expected: PASS (2 tests)

- [ ] **Step 6: Commit**

```bash
git add SafeSpreadVIO
git commit -m "feat(app): scaffold SafeSpreadVIO and add !P packet builder"
```

---

## Task 6: Pose zero-referencing math

**Files:**
- Create: `SafeSpreadVIO/src/poseMath.ts`
- Test: `SafeSpreadVIO/src/poseMath.test.ts`

**Interfaces:**
- Consumes: nothing from Task 5.
- Produces: `interface Pose { x: number; y: number; heading: number; }` and `applyOrigin(raw: Pose, origin: Pose | null): Pose` — rotates `raw` into the frame where `origin` is `(0, 0, 0)` and `origin.heading` defines the new forward (+Y) direction; returns `{x:0,y:0,heading:0}` when `origin` is `null`.

- [ ] **Step 1: Write the failing test**

```ts
// SafeSpreadVIO/src/poseMath.test.ts
import { applyOrigin, Pose } from './poseMath';

describe('applyOrigin', () => {
  it('returns zero pose when no origin is set', () => {
    expect(applyOrigin({ x: 5, y: 5, heading: 90 }, null)).toEqual({ x: 0, y: 0, heading: 0 });
  });

  it('returns zero pose when raw equals origin', () => {
    const p: Pose = { x: 3, y: 4, heading: 45 };
    const result = applyOrigin(p, p);
    expect(result.x).toBeCloseTo(0);
    expect(result.y).toBeCloseTo(0);
    expect(result.heading).toBeCloseTo(0);
  });

  it('rotates displacement into the origin heading frame', () => {
    // Origin facing 90° (world +X). Raw is 1 unit further along world +X,
    // i.e. straight ahead of where the origin was facing.
    const origin: Pose = { x: 0, y: 0, heading: 90 };
    const raw: Pose = { x: 1, y: 0, heading: 90 };
    const result = applyOrigin(raw, origin);
    expect(result.x).toBeCloseTo(0);
    expect(result.y).toBeCloseTo(1);
    expect(result.heading).toBeCloseTo(0);
  });

  it('wraps heading difference correctly across the 0/360 boundary', () => {
    const origin: Pose = { x: 0, y: 0, heading: 350 };
    const raw: Pose = { x: 0, y: 0, heading: 10 };
    const result = applyOrigin(raw, origin);
    expect(result.heading).toBeCloseTo(20);
  });
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `npx jest src/poseMath.test.ts`
Expected: FAIL — `Cannot find module './poseMath'`

- [ ] **Step 3: Write minimal implementation**

```ts
// SafeSpreadVIO/src/poseMath.ts
export interface Pose {
  x: number;
  y: number;
  heading: number; // degrees, 0-360
}

export function applyOrigin(raw: Pose, origin: Pose | null): Pose {
  if (!origin) return { x: 0, y: 0, heading: 0 };

  const dx = raw.x - origin.x;
  const dy = raw.y - origin.y;
  const theta = (origin.heading * Math.PI) / 180;
  const cos = Math.cos(theta);
  const sin = Math.sin(theta);

  const x = dx * cos - dy * sin;
  const y = dx * sin + dy * cos;
  let heading = raw.heading - origin.heading;
  heading = ((heading % 360) + 360) % 360;

  return { x, y, heading };
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `npx jest src/poseMath.test.ts`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add SafeSpreadVIO/src/poseMath.ts SafeSpreadVIO/src/poseMath.test.ts
git commit -m "feat(app): add pose zero-referencing math"
```

---

## Task 7: ARKit pose native module

**Files:**
- Create: `SafeSpreadVIO/modules/arkit-pose/` (via `create-expo-module`, then edited)
- Modify: `SafeSpreadVIO/modules/arkit-pose/index.ts`
- Modify: `SafeSpreadVIO/modules/arkit-pose/src/ArkitPose.types.ts`
- Modify: `SafeSpreadVIO/modules/arkit-pose/ios/ArkitPoseModule.swift`

**Interfaces:**
- Consumes: nothing.
- Produces: `start(): void`, `stop(): void`, `addPoseListener(listener: (event: PoseUpdatePayload) => void): { remove(): void }`, and `type PoseUpdatePayload = { x: number; y: number; heading: number; trackingState: 'normal' | 'limited' | 'notAvailable' }` — the raw (un-zeroed) ARKit pose in feet/degrees, using the same `(x, y, heading)` convention `applyOrigin` (Task 6) expects: heading 0°=+Y, 90°=+X.

- [ ] **Step 1: Scaffold the local module**

Run (from `SafeSpreadVIO/`): `npx create-expo-module@latest --local arkit-pose`

This generates `modules/arkit-pose/` with a boilerplate Swift module, TS wrapper, and `expo-module.config.json`. It's autolinked by convention — no `app.json` changes needed for it.

- [ ] **Step 2: Replace the generated types file**

```ts
// SafeSpreadVIO/modules/arkit-pose/src/ArkitPose.types.ts
export type TrackingState = 'normal' | 'limited' | 'notAvailable';

export type PoseUpdatePayload = {
  x: number;
  y: number;
  heading: number;
  trackingState: TrackingState;
};
```

- [ ] **Step 3: Replace the generated JS wrapper**

```ts
// SafeSpreadVIO/modules/arkit-pose/index.ts
import { requireNativeModule, EventEmitter } from 'expo-modules-core';
import { PoseUpdatePayload } from './src/ArkitPose.types';

const NativeModule = requireNativeModule('ArkitPose');
const emitter = new EventEmitter(NativeModule);

export function start(): void {
  NativeModule.start();
}

export function stop(): void {
  NativeModule.stop();
}

export function addPoseListener(listener: (event: PoseUpdatePayload) => void) {
  return emitter.addListener('onPoseUpdate', listener);
}

export type { PoseUpdatePayload, TrackingState } from './src/ArkitPose.types';
```

- [ ] **Step 4: Replace the generated Swift module**

```swift
// SafeSpreadVIO/modules/arkit-pose/ios/ArkitPoseModule.swift
import ExpoModulesCore
import ARKit

public class ArkitPoseModule: Module {
  private var session: ARSession?
  private var delegate: ArkitSessionDelegate?

  public func definition() -> ModuleDefinition {
    Name("ArkitPose")

    Events("onPoseUpdate")

    Function("start") {
      let session = ARSession()
      let delegate = ArkitSessionDelegate { [weak self] payload in
        self?.sendEvent("onPoseUpdate", payload)
      }
      session.delegate = delegate

      let config = ARWorldTrackingConfiguration()
      config.worldAlignment = .gravity
      session.run(config)

      self.session = session
      self.delegate = delegate
    }

    Function("stop") {
      self.session?.pause()
      self.session = nil
      self.delegate = nil
    }
  }
}

private class ArkitSessionDelegate: NSObject, ARSessionDelegate {
  private let onUpdate: ([String: Any]) -> Void

  init(onUpdate: @escaping ([String: Any]) -> Void) {
    self.onUpdate = onUpdate
  }

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    let t = frame.camera.transform
    let metersToFeet = 3.28084

    // ARKit: camera looks down its local -Z. Our convention: y = forward
    // (world -Z), x = right (world +X), heading 0=+Y, 90=+X.
    let xFt = Double(t.columns.3.x) * metersToFeet
    let yFt = Double(-t.columns.3.z) * metersToFeet

    let forwardX = Double(-t.columns.2.x)
    let forwardZ = Double(-t.columns.2.z)
    var headingDeg = atan2(forwardX, -forwardZ) * 180.0 / Double.pi
    if headingDeg < 0 { headingDeg += 360 }

    let trackingState: String
    switch frame.camera.trackingState {
    case .normal: trackingState = "normal"
    case .limited: trackingState = "limited"
    case .notAvailable: trackingState = "notAvailable"
    }

    onUpdate([
      "x": xFt,
      "y": yFt,
      "heading": headingDeg,
      "trackingState": trackingState,
    ])
  }
}
```

- [ ] **Step 5: Build and run on a physical iPhone, verify pose tracking**

Run: `npx expo prebuild -p ios && npx expo run:ios --device`

With the phone held and walked (not the rover yet — this is a bench test of the module alone), temporarily add a `console.log` in `App.tsx` (default template screen is fine) calling `ArkitPose.start()` and `ArkitPose.addPoseListener(e => console.log(e))` on mount. Walk the phone forward exactly 5 ft (tape measure), holding it level and facing forward.

Expected: logged `y` value increases to approximately `5 ± 0.3` ft; `x` stays near `0`; `trackingState` reads `normal` for the whole walk (if it reads `limited`/`notAvailable`, redo the walk with more visual texture in view — e.g. facing across the yard rather than straight down at uniform grass).

- [ ] **Step 6: Commit**

```bash
git add SafeSpreadVIO/modules
git commit -m "feat(app): add ARKit pose native module"
```

---

## Task 8: `useVIOPose` hook

**Files:**
- Create: `SafeSpreadVIO/src/useVIOPose.ts`

**Interfaces:**
- Consumes: `ArkitPose.start`, `ArkitPose.stop`, `ArkitPose.addPoseListener`, `PoseUpdatePayload` (Task 7); `applyOrigin`, `Pose` (Task 6).
- Produces: `useVIOPose(): { pose: Pose; trackingOk: boolean; zero: () => void }`.

- [ ] **Step 1: Write the hook**

```ts
// SafeSpreadVIO/src/useVIOPose.ts
import { useCallback, useEffect, useRef, useState } from 'react';
import * as ArkitPose from '../modules/arkit-pose';
import { applyOrigin, Pose } from './poseMath';

export function useVIOPose() {
  const [raw, setRaw] = useState<Pose>({ x: 0, y: 0, heading: 0 });
  const [trackingOk, setTrackingOk] = useState(false);
  const originRef = useRef<Pose | null>(null);

  useEffect(() => {
    ArkitPose.start();
    const subscription = ArkitPose.addPoseListener((event) => {
      setRaw({ x: event.x, y: event.y, heading: event.heading });
      setTrackingOk(event.trackingState === 'normal');
    });
    return () => {
      subscription.remove();
      ArkitPose.stop();
    };
  }, []);

  const zero = useCallback(() => {
    originRef.current = raw;
  }, [raw]);

  return { pose: applyOrigin(raw, originRef.current), trackingOk, zero };
}
```

- [ ] **Step 2: Verify by temporary console logging**

In `App.tsx`, temporarily render `useVIOPose()`'s `pose`/`trackingOk` via `console.log` on each render, run `npx expo run:ios --device`. Walk the phone, confirm `pose` stays `{x:0,y:0,heading:0}` until `zero()` is called (wire it to a temporary button), then confirm subsequent movement is reported relative to that button press.

Expected: pose reads `(0, 0, 0)` before any `zero()` call regardless of phone movement; after `zero()`, moving forward increases `pose.y`.

- [ ] **Step 3: Commit**

```bash
git add SafeSpreadVIO/src/useVIOPose.ts
git commit -m "feat(app): add useVIOPose hook wiring ARKit pose + zero-referencing"
```

---

## Task 9: BLE central and full UI

**Files:**
- Create: `SafeSpreadVIO/src/ble.ts`
- Modify: `SafeSpreadVIO/App.tsx`
- Modify: `SafeSpreadVIO/app.json`

**Interfaces:**
- Consumes: `buildPosePacket` (Task 5), `useVIOPose` (Task 8).
- Produces: `class SafeSpreadBLE` with `connect(onStatusChange: (status: ConnectionStatus) => void): Promise<void>`, `disconnect(): Promise<void>`, `sendPose(x: number, y: number, heading: number): Promise<void>`, and `type ConnectionStatus = 'disconnected' | 'scanning' | 'connected'`.

- [ ] **Step 1: Add Bluetooth/camera permission strings**

Merge into `SafeSpreadVIO/app.json`'s `expo` object:

```json
{
  "ios": {
    "bundleIdentifier": "com.safespread.vio",
    "infoPlist": {
      "NSCameraUsageDescription": "SafeSpreadVIO uses the camera for visual-inertial position tracking.",
      "NSBluetoothAlwaysUsageDescription": "SafeSpreadVIO connects to the SafeSpread rover over Bluetooth."
    }
  },
  "plugins": ["expo-camera", "expo-dev-client"]
}
```

- [ ] **Step 2: Write `ble.ts`**

```ts
// SafeSpreadVIO/src/ble.ts
import { BleManager, Device } from 'react-native-ble-plx';
import { Buffer } from 'buffer';
import { buildPosePacket } from './protocol';

const DEVICE_NAME = 'SafeSpread';
const NUS_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E';
const NUS_TX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E';

export type ConnectionStatus = 'disconnected' | 'scanning' | 'connected';

export class SafeSpreadBLE {
  private manager = new BleManager();
  private device: Device | null = null;

  connect(onStatusChange: (status: ConnectionStatus) => void): Promise<void> {
    onStatusChange('scanning');
    return new Promise((resolve, reject) => {
      this.manager.startDeviceScan([NUS_SERVICE_UUID], null, async (error, scanned) => {
        if (error) {
          onStatusChange('disconnected');
          reject(error);
          return;
        }
        if (scanned?.name !== DEVICE_NAME) return;

        this.manager.stopDeviceScan();
        try {
          const device = await scanned.connect();
          await device.discoverAllServicesAndCharacteristics();
          this.device = device;
          onStatusChange('connected');
          device.onDisconnected(() => {
            this.device = null;
            onStatusChange('disconnected');
          });
          resolve();
        } catch (e) {
          onStatusChange('disconnected');
          reject(e);
        }
      });
    });
  }

  async disconnect(): Promise<void> {
    this.manager.stopDeviceScan();
    if (this.device) {
      await this.device.cancelConnection();
      this.device = null;
    }
  }

  async sendPose(x: number, y: number, heading: number): Promise<void> {
    if (!this.device) return;
    const packet = buildPosePacket(x, y, heading);
    await this.device.writeCharacteristicWithoutResponseForService(
      NUS_SERVICE_UUID,
      NUS_TX_UUID,
      Buffer.from(packet).toString('base64')
    );
  }
}
```

- [ ] **Step 3: Write `App.tsx`**

```tsx
// SafeSpreadVIO/App.tsx
import React, { useEffect, useRef, useState } from 'react';
import { Pressable, StyleSheet, Text, View } from 'react-native';
import { CameraView, useCameraPermissions } from 'expo-camera';
import { useVIOPose } from './src/useVIOPose';
import { ConnectionStatus, SafeSpreadBLE } from './src/ble';

const ble = new SafeSpreadBLE();

export default function App() {
  const [permission, requestPermission] = useCameraPermissions();
  const { pose, trackingOk, zero } = useVIOPose();
  const [status, setStatus] = useState<ConnectionStatus>('disconnected');

  useEffect(() => {
    if (!permission?.granted) requestPermission();
  }, [permission]);

  useEffect(() => {
    ble.connect(setStatus).catch(() => {});
    return () => {
      ble.disconnect();
    };
  }, []);

  useEffect(() => {
    const timer = setInterval(() => {
      if (status === 'connected' && trackingOk) {
        ble.sendPose(pose.x, pose.y, pose.heading);
      }
    }, 100);
    return () => clearInterval(timer);
  }, [status, trackingOk, pose]);

  if (!permission) return <View style={styles.container} />;
  if (!permission.granted) {
    return (
      <View style={styles.container}>
        <Text style={styles.text}>Camera permission required</Text>
      </View>
    );
  }

  return (
    <View style={styles.container}>
      <CameraView style={StyleSheet.absoluteFill} facing="back" />
      <View style={styles.crosshair} />
      <View style={styles.hud}>
        <Text style={styles.text}>X: {pose.x.toFixed(1)} ft</Text>
        <Text style={styles.text}>Y: {pose.y.toFixed(1)} ft</Text>
        <Text style={styles.text}>Hdg: {pose.heading.toFixed(0)}°</Text>
        <Text style={styles.text}>BLE: {status}</Text>
        <Text style={styles.text}>Tracking: {trackingOk ? 'OK' : 'DEGRADED'}</Text>
      </View>
      <View style={styles.buttons}>
        <Pressable style={styles.button} onPress={zero}>
          <Text style={styles.buttonText}>Start / Reset</Text>
        </Pressable>
        <Pressable style={styles.button} onPress={() => ble.disconnect()}>
          <Text style={styles.buttonText}>Stop</Text>
        </Pressable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: 'black' },
  crosshair: {
    position: 'absolute',
    top: '50%',
    left: '50%',
    width: 40,
    height: 40,
    marginLeft: -20,
    marginTop: -20,
    borderWidth: 2,
    borderColor: 'white',
    borderRadius: 20,
  },
  hud: { position: 'absolute', top: 60, left: 20 },
  text: { color: 'white', fontSize: 18, fontWeight: '600' },
  buttons: {
    position: 'absolute',
    bottom: 40,
    left: 20,
    right: 20,
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  button: { backgroundColor: '#2e7d32', paddingVertical: 14, paddingHorizontal: 24, borderRadius: 8 },
  buttonText: { color: 'white', fontSize: 16, fontWeight: '700' },
});
```

- [ ] **Step 4: Verify end-to-end with the ESP32 powered on**

Flash `auto_vio/auto_vio.ino` (Task 4) to the rover's ESP32 first. Power it on, then run `npx expo run:ios --device` for the app.

Expected: HUD shows `BLE: connected` within a few seconds; `X`/`Y`/`Hdg` update live as the phone moves; pressing "Start / Reset" zeroes them to `0.0 ft / 0.0 ft / 0°`; the ESP32's serial monitor logs `VIO app connected.`

- [ ] **Step 5: Commit**

```bash
git add SafeSpreadVIO/src/ble.ts SafeSpreadVIO/App.tsx SafeSpreadVIO/app.json
git commit -m "feat(app): add BLE central and wire full VIO UI"
```

---

## Task 10: Full mission integration test

**Files:** none (verification-only task)

**Interfaces:** none — this exercises Tasks 4 and 9 together.

- [ ] **Step 1: Serial packet verification**

With the phone app connected (Task 9 running) and the ESP32's serial monitor open, walk the phone in a small square. Confirm the serial log's periodic waypoint messages (once a mission is started) reflect plausible `robotX_ft/robotY_ft` matching the phone's real displacement — cross-check by pausing the phone at a spot measured with a tape measure from the mission's start point.

Expected: no `!!! VIO signal lost !!!` messages while the phone stays connected and in view of textured surroundings; parsed X/Y track the tape-measured displacement within about a foot.

- [ ] **Step 2: On-blocks waypoint-following check**

Rover up on a stand (wheels off the ground), ESP32 running `auto_vio.ino`, phone app connected. Send `'1'` (start mission) via the BLE TX characteristic or serial. Manually move the phone (simulating rover motion) toward the first waypoint `(0, 21.91)`.

Expected: steering channel output swings toward center as the simulated heading approaches the bearing to `(0, 21.91)` (watch via `bleLog`/serial output of the computed error); throttle channel switches from `THROTTLE_TURN_US` to `THROTTLE_FWD_US` once heading error drops under 45°; mission advances to waypoint index 1 once the simulated position gets within `0.5 ft` of the target and logs `>>> Reached waypoint 1/...`.

- [ ] **Step 3: Live grass run**

Mount the phone on the rover (forward-facing or slightly downward, with some surrounding scenery in view — not pointed straight down at uniform grass), full battery, spray tank filled. Run a complete mission.

Expected: rover completes all waypoints without a VIO-timeout stop; measure actual pass spacing on the grass with a tape measure and confirm no unsprayed gap wider than the 17" bar; total sprayed area visually covers the 480 sqft square.

- [ ] **Step 4: Commit the verification note**

```bash
git commit --allow-empty -m "test: verify full phone-VIO waypoint mission end-to-end"
```
