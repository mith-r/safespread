# Reliable Pavement VIO Control — Design Spec

> **Historical only.** This specification contains retired product behavior
> and must not be used as deployment source. The canonical app on `main`
> accepts entered rectangle dimensions only; do not restore Walk corners or
> Corner A/B capture.

Date: 2026-08-26

## Problem

SafeSpread must apply brine in straight, parallel passes over rectangular
driveways, patios, and sidewalks. The current iPhone/ARKit and ESP32 system can
plan a mathematically valid route, but field behavior is not repeatable enough.
It accepts questionable ARKit poses, treats the camera as the rover control
point, has no way to reject delayed poses from an earlier origin, assumes
uncalibrated steering and speed behavior, and may skip an entire route leg after
a stall.

The existing unit and host tests prove the software's internal model is
self-consistent. They do not prove that the model matches the physical rover.
The upgraded system must therefore make sensor quality, transport freshness,
vehicle calibration, and physical tracking error observable and fail safely
when any of them is untrustworthy.

## Goal

Deliver a safety-first, diagnosable control stack that:

- defines a pavement rectangle using either entered dimensions or two walked
  opposite corners;
- transforms ARKit camera pose into calibrated rover and spray-bar poses;
- sends only fresh, ordered, mission-scoped pose data to the ESP32;
- records a complete mission log on the phone and a short fault buffer on the
  ESP32;
- calibrates mounting yaw, steering curvature, straight-ahead trim, and loaded
  pavement speed from measured motion;
- stops on sensor, transport, power, movement, or maneuver failure instead of
  silently skipping coverage;
- provides replayable data for later controller tuning.

## Scope and non-goals

This design implements items 1–6 of the reliability recommendation. Lane
overlap remains at the existing 15 percent for now; changing it is explicitly
out of scope.

The target surface is pavement: asphalt, concrete, pavers, patios, driveways,
and sidewalks. The design accounts for vibration from hard joints, sunlight and
glare, wet reflections, and traction changes caused by brine. It does not add
wheel encoders, a steering-angle sensor, RTK GNSS, or other rover hardware.

Physical calibration values and final controller gains cannot be manufactured
in software. This work supplies calibration routines, logging, replay, safety
gates, and conservative defaults. A user must run the documented dry pavement
tests before wet autonomous use.

## Operator workflows

### Mode 1: Enter M x N

1. Enter **M**, the pass length, and **N**, the coverage width.
2. Place the mounted rover at the first-pass starting corner.
3. Point the rover along the desired M direction. N extends to the rover's
   right by default; the operator can select left before arming.
4. Enter the clear pavement available before the start and beyond the far end
   for headland maneuvers.
5. Wait until the app reports connected, protocol-compatible, ARKit tracking
   normal, pose stable, and calibration valid.
6. Press Start. The app averages the stable pose window and uses the calibrated
   camera-to-chassis yaw to establish the rectangle frame.

### Mode 2: Walk opposite corners

1. Remove the phone from its repeatable quick-release mount.
2. Stand at corner A and point the top of the phone along the desired M edge.
3. Tap **Set Corner A**. The app records the world position, averaged heading,
   and a stable tracking window.
4. Walk directly to opposite corner B and tap **Set Corner B**.
5. The app projects A-to-B onto A's forward and right axes. The forward
   projection becomes M and the lateral projection becomes N.
6. The app shows the calculated M x N dimensions, orientation, coverage side,
   and a rectangle preview. A negative lateral projection is presented as a
   deliberate **Flip Coverage Side** choice, never silently mirrored.
7. Enter the clear pavement available before A and beyond the opposite M edge
   for headland maneuvers.
8. Return the phone to the repeatable mount and return the rover to corner A.
9. Start remains disabled until the calibrated axle/spray-bar pose is within
   the configured start tolerance of A, tracking is normal and stable, and the
   operator confirms the preview.

Only corner A's phone orientation supplies the missing rectangle orientation.
Two opposite positions alone define a diagonal but not the directions of a
general rectangle's sides, so the UI must explain the point-the-phone-along-M
step.

## Coordinate frames and calibration

The system owns four distinct frames:

1. **AR world:** raw ARKit camera transform and frame timestamp.
2. **Rectangle:** origin and axes established by Mode 1 or Mode 2.
3. **Rover:** rear-axle control point and chassis heading.
4. **Spray bar:** application center derived from its measured offset from the
   rover control point.

Calibration stores the camera's forward and lateral offset from the rear-axle
control point, camera mounting-yaw offset from the chassis, and spray-bar
forward/lateral offset. Each offset is applied as a rigid 2D transform using
the current calibrated chassis heading. Raw camera coordinates must never be
assigned directly to the rover state.

The mounting-yaw calibration drives a short, spray-off straight run and compares
the robust course-over-ground estimate with camera heading. Multiple samples
from both directions are averaged with angular wrapping. Calibration is rejected
if tracking degrades, the path is too short, curvature is excessive, or forward
and return estimates disagree beyond tolerance.

The app stores calibration with a schema version and timestamp. A change to the
mount, steering linkage, phone orientation, tires, vehicle load, or firmware
calibration version marks the corresponding result stale and blocks wet mode
until it is repeated. Dry diagnostic operation remains available.

## ARKit pose validation

The native Swift module emits, for every AR frame:

- raw camera x/y and heading;
- `ARFrame.timestamp`;
- tracking state and the full limited-state reason;
- world-mapping status for diagnostics;
- a monotonically increasing frame sequence.

Only `.normal` tracking is drive-eligible. `.limited` and `.notAvailable` are
logged and immediately make the pose ineligible for motion. Session interruption,
failure, relocalization, or an unexpected pose discontinuity faults the mission.

Before arming, a rolling stationary window must satisfy configured bounds for
position spread, heading spread, frame age, and continuous normal tracking. The
window, bounds, and rejection reason appear in the UI and mission log.

Native frame timestamp and phone monotonic time are preserved through the app.
Derived velocity and yaw rate use a robust multi-sample window rather than a
two-point derivative. Samples containing non-finite values, implausible speed,
implausible acceleration, implausible yaw rate, a duplicate sequence, or a
backward timestamp are rejected before BLE transmission.

## BLE protocol v2 and migration

The iOS app and main navigator move to a versioned protocol. The pose message
contains a protocol version, mission epoch, sequence number, frame age,
tracking-valid flag, calibrated rover pose, speed, yaw rate, and a CRC stronger
than the current byte-sum complement. Exact byte layout is frozen in the
implementation plan and covered by matching TypeScript and C++ fixture tests.

Pose delivery remains write-without-response for low latency, but a sender owns
at most one in-flight write and one replaceable pending pose. A newer pose
replaces the pending one; poses are never allowed to accumulate in an unbounded
React or Core Bluetooth queue.

Configuration, calibration identity, Arm, Start, Stop, and mode changes use
writes with response plus an application-level acknowledgement carrying the
same command ID and mission epoch. Start is an idempotent transaction:

1. App generates a new mission epoch.
2. App sends rectangle and calibration identity.
3. ESP32 acknowledges the accepted configuration.
4. App streams a near-zero, normal, fresh pose for that epoch.
5. ESP32 acknowledges Armed only after its own safety checks pass.
6. App sends Start and waits for Running acknowledgement.

The ESP32 always accepts an emergency Stop regardless of navigation state.
Duplicate commands with the same command ID return the previous acknowledgement
without repeating side effects.

During migration, the new app refuses to arm firmware that does not advertise
protocol v2. New firmware may parse legacy pose packets for diagnostics while
idle, but legacy packets can never arm or move the rover. Rollback is performed
by reinstalling the previous app build and reflashing the previous firmware;
the existing v1 source remains in Git until field validation finishes.

## Phone mission logging

The phone is the authoritative recorder. One JSON Lines file is created per
mission in the app's Application Support storage. It is written incrementally
outside React render state and flushed periodically and on every state/fault
transition. A small metadata record identifies app version, firmware version,
protocol version, calibration version, pavement conditions entered by the
operator, rectangle definition mode, M/N, and mission epoch.

Records include:

- native frame, app-send, and ESP-receive timing information;
- raw camera pose, calibrated rover pose, and spray-bar pose;
- tracking state/reason and mapping status;
- pose sequence, mission epoch, rejection counters, and estimated transport age;
- route point/segment, cross-track and heading errors;
- course, speed, yaw rate, steering pulse, throttle pulse, direction, and spray
  state;
- acknowledgements, state transitions, PWM/I2C status, and fault reason.

The app lists recent missions and supports Share/Export of the JSONL file and a
derived CSV. Logging failure does not keep a wet mission running: if the phone
cannot create or append the authoritative log before arming, wet Start is
blocked. Dry diagnostics may continue with a visible warning.

## ESP32 telemetry and fault buffer

The ESP32 sends compact binary telemetry at the control cadence and lower-rate
human-readable status lines. Telemetry includes the pose sequence it used so the
phone can join rover output to its source AR frame.

The ESP32 keeps a fixed circular RAM buffer containing at least the most recent
five seconds of control samples. On a fault it freezes the buffer, persists only
a compact fault summary and counters to nonvolatile storage, and offers the RAM
buffer to the phone until reboot. Continuous control-rate writes to flash are
forbidden.

Queue overflow, invalid packet, stale sequence, rejected pose, and telemetry
drop counters are observable. No queue may silently discard data without
incrementing a reported counter.

## Firmware safety state machine

Navigation states become explicit:

`IDLE -> CONFIGURED -> ARMED -> RUNNING -> COMPLETE`

Any state may transition to `FAULT`; Stop transitions safely to `IDLE`.

Motion is permitted only when all of these remain true:

- protocol and mission epoch match;
- the most recent pose is valid, ordered, and within the freshness deadline;
- ARKit tracking-valid is true;
- calibration identity matches the configured mission;
- PWM controller is awake, configured, and reachable;
- route and direction state are internally valid;
- speed, acceleration, yaw rate, and tracking error remain inside safety bounds.

A PCA9685 reset, I2C failure, BLE disconnect, pose timeout, pose discontinuity,
failed direction change, no-motion stall, unexpected motion, excessive tracking
error, or route inconsistency immediately commands neutral throttle, straight
steering, spray off, and `FAULT`. PWM recovery never replays the last moving
command automatically.

The existing behavior that skips a whole route leg after four seconds is
removed. A stalled leg faults the mission with its route and actuator context.
Restart requires an explicit Stop, a fresh epoch, and a new Arm handshake.

## Direction changes and route policy

The reverse self-test and navigation use the same ESC direction-change state
machine and timing. The state machine supports neutral dwell and any required
brake/reverse sequence, then verifies direction from timestamped displacement
before declaring the maneuver engaged. A calibration that passes under one
sequence cannot be used by navigation with a shorter sequence.

Route planning retains the existing parallel sprayed passes and 15 percent
overlap. The operator enters available clear pavement before the start and past
the far end. The planner reports required headland clearance before arming and
refuses Start if either entered clearance is insufficient, rather than allowing
the rover to discover the constraint while moving.

Where sufficient pavement clearance exists, a continuous forward-only route
that reorders lanes is preferred over repeated three-point turns. Three-point
turns remain available when needed, but require a current reverse calibration.
The planner selects one complete route before motion and never silently changes
coverage intent during a fault recovery.

## Steering and speed calibration

Steering calibration records measured curvature across several safe servo
pulses on both sides of straight, in both travel directions where practical.
It fits a monotonic piecewise-linear pulse-to-curvature map and directly
estimates the straight-ahead pulse. The existing inference from two full-lock
circles remains diagnostic input but is not authoritative.

Calibration runs are dry, slow, operator-confirmed, and abort immediately on
tracking or transport degradation. They report pavement type and whether the
surface is dry or wet. Wet brine may change traction, so wet operation requires
conservative speed until wet calibration data exists.

Ground-speed control uses timestamped robust velocity, calibrated feed-forward
for direction and load, and a bounded PI correction with anti-windup. Breakaway
is a bounded ramp, not an unconstrained integrator. No-motion and wrong-direction
conditions fault instead of increasing throttle indefinitely. Initial target
speed is conservative and turns are slower than straight passes.

## Line controller and replay

The current straight-line control law remains initially. This work first fixes
its inputs and physical actuator mapping rather than replacing it without field
evidence. The controller updates once per accepted pose/control tick, never
reprocesses the same sample at unrestricted Arduino loop speed, and receives
the calibrated rover control-point pose, measured speed, and curvature map.

A host replay harness consumes exported mission records and runs route-progress,
safety-gate, and line-control logic deterministically. It reports pose rejection,
cross-track distribution, heading error, command saturation, latency, and fault
transitions. Controller gain changes require a recorded pavement run that shows
the previous behavior and a replay result that improves predefined metrics
without weakening safety gates.

## UI

The app presents a setup wizard rather than enabling Start from the existing
single screen:

1. Connection and protocol compatibility.
2. Rectangle mode selection.
3. Enter M/N or capture/confirm opposite corners, coverage side, and available
   headland clearance.
4. Calibration status and mount offsets.
5. Live readiness checklist, pavement condition, and dry/wet selection.
6. Arm and Start acknowledgement state.

The running view keeps pose, tracking, rover state, and Stop prominent. Faults
show one primary cause, contextual measurements, and the saved log filename.
Start remains unavailable while running or faulted.

## Verification strategy

All pure logic follows red-green-refactor TDD with matching TypeScript and C++
fixtures where data crosses BLE. Automated coverage includes:

- protocol v2 encode/decode, CRC, version, epoch, sequence, and malformed data;
- pose freshness, ordering, finite/rate/outlier validation;
- stable-pose arming windows and ARKit limited-state rejection;
- camera-to-rover-to-spray-bar transforms;
- entered and walked-corner rectangle construction, left/right flipping, and
  degenerate captures;
- newest-only BLE sender behavior and idempotent command acknowledgements;
- safety-state transitions and fault dominance;
- circular fault-buffer wrap/freeze behavior;
- direction-change timing shared by test and navigation;
- steering-map fitting and bounded speed control;
- mission-log serialization and replay.

Automated tests cannot certify physical reliability. Field validation proceeds
in this order and stops at the first failed gate:

1. Stationary pavement pose and tracking-quality test under expected lighting.
2. Measured out-and-back phone pose closure test.
3. Mount-yaw and rigid-offset calibration.
4. Loaded dry steering and speed calibration on the actual pavement.
5. Repeated one-pass dry runs in both directions.
6. Parallel passes with manual repositioning, isolating line control from turns.
7. Direction-change and lane-entry tests with spray disabled.
8. Complete dry rectangle in each definition mode.
9. Low-speed wet rectangle after log review.

The 15 percent overlap provides 2.55 inches of total adjacent-pass margin. That
choice remains unchanged, but field results must report whether measured
cross-track error fits within that budget; passing software tests alone does not
authorize wet operation.

## Acceptance and stop condition

Implementation is complete when both app and firmware use protocol v2; both
rectangle modes work in pure tests and UI; normal-only fresh pose gating,
mission handshake, logging, transforms, calibration workflows, and fail-stop
behavior are automated and verified; legacy firmware cannot be armed by the new
app; and a documented field checklist is ready.

The work stops before claiming physical controller tuning or wet pavement
reliability. Those claims require exported logs from the physical calibration
and dry-run sequence.
