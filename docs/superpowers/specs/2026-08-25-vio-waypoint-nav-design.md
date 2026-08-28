# Phone-VIO Waypoint Navigation — Design Spec

> **Historical only.** This specification is not a current product requirement
> and must not be used as deployment source. Follow the canonical app on
> `main`, which does not include a walked-corner setup workflow.

Date: 2026-08-25

## Problem

`auto/auto.ino` drives an 8-pass "grid" over a 480 sqft (21.91 ft x 21.91 ft)
square but does not reliably cover it. Two independent root causes:

1. **Lane spacing doesn't match the spray bar.** Passes are spaced 3.13 ft
   apart, derived only from `21.91 / (8 - 1)` — no reference to the actual
   17" (1.417 ft) spray bar. At that spacing, most of the square goes
   unsprayed between passes.
2. **Forward position is open-loop.** Heading comes from a real sensor (phone
   orientation via Bluefruit Connect), but distance traveled is
   `assumed_constant_speed * dt` (`CRUISE_SPEED_FPS = 3.37`,
   `TURN_SPEED_FPS = 1.50`). Nothing on the rover measures actual wheel
   rotation, so `posX/posY` silently diverges from the real position, and
   every later waypoint is computed from that already-wrong position — error
   compounds pass over pass.

Hardware constraint: **no new hardware may be added to the rover other than
a smartphone.** This rules out wheel encoders (the standard fix for #2).

## Goal

Cover the 480 sqft square with the 17" bar, consistently, using only:
existing ESP32 + PCA9685 + steering servo + ESC + spray valve/pump, plus one
smartphone mounted on the rover.

## Non-goals

- Multi-session mapping / persistent world anchors across runs.
- Handling arbitrary field shapes — square coverage only, matching current
  scope.

## Why phone-only position is possible here

Bluefruit Connect's Controller/sensor stream only relays raw orientation
(accel/gyro/mag/quaternion) — it has no fused position output, so it cannot
solve root cause #2 by itself.

Plain accelerometer double-integration (naive "VIO") does not solve it
either — it's still open-loop distance estimation, just replacing one bad
guess (constant speed) with another (noisy double-integrated accelerometer),
and drifts feet within a run.

**Real VIO** — Apple's `ARWorldTrackingConfiguration` (ARKit) — fuses the
camera feed with the IMU into a continuously drift-corrected 6-DOF pose.
This is a genuine closed-loop position measurement, accurate to a few cm
over a short single-session run at this scale, and requires no hardware
beyond the phone's existing camera + IMU. This is the only way to satisfy
both constraints (real position accuracy + phone-only sensing).

**Unavoidable cost:** ARKit's world-tracking pose is not exposed through
plain Expo Go. A one-time custom development build is required (EAS Build
cloud, or local Xcode). For iOS specifically, keeping that build installed
on-device needs either a Mac + Xcode (free signing, must re-install every 7
days) or a paid Apple Developer account ($99/yr) for EAS-distributed builds.
This will be confirmed/chosen at implementation time.

## Architecture

Three components, phone is the sole pose source (Bluefruit Connect is
retired):

```
 iPhone (SafeSpreadVIO app)                     ESP32 (auto_vio/auto_vio.ino)
 ┌─────────────────────────┐   BLE NUS, 10Hz    ┌───────────────────────────┐
 │ ARKit world tracking     │  !P packet (15B)  │ Parse !P -> robotX/Y/hdg  │
 │  (custom Expo Module,    │ ─────────────────>│ Waypoint list (bar-width  │
 │   Swift, ARSession)      │                    │  derived spacing)         │
 │  -> {x,y,z,yaw}          │                    │ Steer-to-waypoint control │
 │ JS: zero origin on Start │                    │ PCA9685 steer/ESC output  │
 │ BLE central              │                    │ Spray valve/pump toggle   │
 │  (react-native-ble-plx)  │                    │ VIO-timeout safety stop   │
 │ UI: camera preview,      │<─ ─ ─ ─ ─ ─ ─ ─ ─ ─│ Text log over NUS TX      │
 │  crosshair, X/Y/Hdg,     │   log/status text  │                           │
 │  BLE status, Start/Stop  │                    │                           │
 └─────────────────────────┘                    └───────────────────────────┘
```

### 1. Mobile app (`SafeSpreadVIO/`, Expo + custom native module)

- **Pose module**: a lightweight Expo Module (Swift), not a full AR
  rendering library (rejected `@reactvision/react-viro` and similar — pulls
  in a full AR scene-rendering stack we don't need, and adds a third-party
  compatibility risk we don't control). Starts an `ARSession` with
  `ARWorldTrackingConfiguration`, reads `frame.camera.transform` on each
  `session(_:didUpdate:)` callback, extracts translation (x, z -> our
  ground-plane x, y in meters, converted to feet) and yaw from the rotation
  component, emits via Expo's event emitter to JS. Also surfaces
  `ARCamera.trackingState` (normal / limited / not available) so the JS
  layer knows when the pose is untrustworthy.
- **`useVIOPose()` hook**: subscribes to native pose events, holds
  `{x, y, heading, trackingOk}` in feet/degrees. "Start/Reset" button
  captures the current raw pose as the new zero-reference; all subsequent
  readings are reported relative to it.
- **BLE (`ble.ts`)**: `react-native-ble-plx`, scans for device name
  `SafeSpread`, connects, discovers Nordic UART service
  (`6E400001-...`) / TX characteristic (`6E400002-...`). `sendPosePacket(x,
  y, heading)` builds the 15-byte packet below and sends every 100ms while
  connected and tracking is OK. Does not send while `trackingOk` is false.
- **UI**: full-screen camera preview (context/feature richness for ARKit,
  not required for the pose math itself), center crosshair, live
  `X / Y / Hdg` readout, tracking-quality indicator, BLE connection status,
  Start/Reset and Stop buttons — same shape as originally sketched.

**`!P` packet (unchanged from the original spec — this part was always
correct):**

| Bytes | Field | Type |
|---|---|---|
| 0 | `0x21` (`!`) | const |
| 1 | `0x50` (`P`) | const |
| 2-5 | x, feet | float32 LE |
| 6-9 | y, feet | float32 LE |
| 10-13 | heading, degrees (0-360) | float32 LE |
| 14 | CRC = `~(sum of bytes 0-13) & 0xFF` | uint8 |

### 2. Firmware (`auto_vio/auto_vio.ino`, new sketch — `auto/auto.ino` kept as-is for reference)

- Drops all internal dead-reckoning. `robotX_ft/robotY_ft/robotHeading` are
  set directly from parsed `!P` packets — no `posX/posY` integration, no
  `CRUISE_SPEED_FPS`/`TURN_SPEED_FPS` constants.
- Waypoint list is **derived from the real spray bar width**, not
  hardcoded: `BAR_WIDTH_FT = 17.0 / 12.0` (1.417 ft), effective lane spacing
  chosen with a safety overlap margin (~10-15% under bar width, e.g. ~1.25
  ft — exact overlap constant to be tuned during field testing since real
  spray pattern width can differ slightly from the nominal bar width), pass
  count computed as `ceil(FIELD_SIDE_FT / LANE_SPACING_FT) + 1` so the field
  is always fully tiled rather than falling short.
- State machine simplifies to: steer proportionally toward the next
  waypoint's bearing, drive forward (reduced throttle if heading error is
  large), advance to the next waypoint once within `WAYPOINT_TOLERANCE_FT`,
  toggle spray on "long pass" waypoints vs. off on "lane shift" waypoints —
  same shape as the originally-pasted ESP32 sketch's `navigateToWaypoint()`.
- Keeps the VIO-timeout safety stop (no `!P` packet for >1s => hard stop,
  mission reset to idle) — still needed since BLE can drop regardless of
  how good the pose source is.
- Keeps `'1'`/`'2'` serial/BLE button controls for start/stop, matching
  existing operational workflow.

### 3. Retired

Bluefruit Connect app and the `!O`/`!M`/`!Q` heading-only packet parsing in
`auto.ino` are not used by the new sketch — the phone app now supplies
heading itself (from the same ARKit pose), so a second heading source is
redundant.

## Error handling

- **BLE disconnect**: ESP32's existing `ServerCallbacks::onDisconnect` stops
  the drive and resets mission state (unchanged behavior).
- **VIO signal loss** (no packet >1s): hard stop, mission -> idle (unchanged
  behavior from the originally-pasted sketch).
- **ARKit tracking degraded** (`.limited`/`.notAvailable` — can happen over
  low-texture/uniform surfaces or poor lighting): app suppresses sending
  fresh `!P` packets while degraded rather than feeding the rover a garbage
  pose; UI shows a tracking-quality warning. Falling silent triggers the
  existing VIO-timeout stop on the ESP32 side after 1s, so the rover halts
  safely rather than drives on stale/bad data.

## Testing / validation plan

1. **Pose module alone**: log `{x,y,heading,trackingState}` to the phone
   console while walking a known distance/path by hand (no rover, no BLE) —
   cheapest way to confirm ARKit tracking is usable over the actual lawn
   before building anything on top of it.
2. **BLE plumbing**: confirm `!P` packets arrive and parse correctly on the
   ESP32 (serial log the parsed values) with the phone stationary, then
   walked by hand.
3. **Firmware waypoint logic on blocks**: rover up on a stand, verify
   steering/throttle react correctly to injected/walked pose changes before
   any live grass run.
4. **Live run**: full mission on grass, visually confirm pass spacing
   against the 17" bar (no gaps, acceptable overlap).

## Open items (resolved at implementation time, not blocking this design)

- Mac + Xcode vs. paid Apple Developer account for keeping the iOS dev
  build installed — pick based on what's available.
- Exact overlap-margin constant for lane spacing (start conservative, tune
  after step 4 of the test plan).
