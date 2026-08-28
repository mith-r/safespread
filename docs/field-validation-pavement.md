# SafeSpread pavement field-validation checklist

This checklist is the hardware gate for autonomous brine spreading. Automated
tests do **not** approve straight-line, full-rectangle, or wet-pavement
reliability. Run the stages in order and stop at the first failed gate.

## Acceptance limits used throughout

- The spray bar remains 17 inches wide with 15% overlap. Adjacent pass centers
  are 14.45 inches apart, leaving **2.55 inches total adjacent-pass margin**.
- For an individual straight pass, require absolute cross-track p95 no greater
  than 1.275 inches (0.106 ft), no accepted sample above 2.55 inches (0.2125
  ft), and steering saturation no greater than 10%.
- Speed is servoed, not preset. Forward straights hold 1.5 ft/s and turns and
  reverse hold 0.7 ft/s; the loop is expected to move `throttle_us` around to
  keep them. Once a pass is under way, require measured `speed_fps` within
  0.15 ft/s of its target, and treat `throttle_us` sitting at the 1900 us
  ceiling as a failed pass -- the rover has run out of throttle and is holding
  less than the speed it was told to. A full tank needs about five seconds to
  reach cruise from a standing start; measure the band after that, not through
  it.
- The replay summary must say `acceptance=accepted`, `fault=0`, and end at the
  expected route index. Investigate every rejected sample; an isolated stale
  test frame is allowed only in the supplied software fixture, not ignored in
  a field run.
- `max_pose_age_ms` in the current CSV replay is the largest observed interval
  between control records and is a conservative transport/freshness warning.
  Any value above 250 ms during a field run is a stop condition.
- Stop immediately for degraded ARKit tracking, BLE loss, a fault, an
  unexpected direction change, visible wheel slip, a route-index decrease,
  motion outside the marked test envelope, or loss of operator control.

## Equipment and site

- Rover with the current protocol-v2 firmware, charged drive battery, charged
  control power, full actuator calibration, and spray valve proven to fail off.
- iPhone with the current SafeSpreadVIO development build, enough free storage,
  Bluetooth enabled, and notifications/audio audible.
- Completed repeatable quick-release mount. Fully seat and latch it before every
  run; do not hand-hold or move the phone after readiness begins.
- Tape measure, chalk/tape centerlines at 1-foot intervals, cones/barriers,
  wheel-path markers, and a way to measure final pose closure to 0.1 inch.
- Flat representative driveway/patio/sidewalk pavement under expected lighting.
  Record asphalt/concrete/pavers and lighting in the run notes.
- A dry test load matching the intended brine mass. Use water only for the first
  wet trial if freezing conditions and the site allow it; contain and clean up
  all liquid according to site rules.
- Two people: one test operator holding the phone app's Stop control and one
  observer with access to the rover's physical power disconnect.

## Stand-clear and pre-run rules

- Cone off the rectangle, both headlands, and a lateral buffer. Keep people,
  vehicles, pets, drop-offs, steps, public traffic, and movable obstacles out.
- Nobody stands in front of the rover, behind a reversing path, inside either
  headland, or between the rover and an obstacle. Operator and observer remain
  together outside the lateral boundary with an unobstructed view.
- Agree on the verbal command “STOP.” Either person may call it. The operator
  taps Stop; the observer disconnects physical power if neutral is not immediate.
- Before every moving run, verify the running screen keeps Stop visible, spray
  state is correct, the intended direction is displayed, and the entered
  route/headland envelope matches the cones.
- A faulted or stopped run never resumes. Save its logs, inspect the rover, then
  begin again with a fresh mission epoch and Arm handshake.

## UI gate required before Arm

Confirm all of the following on the setup wizard:

- Connected firmware advertises protocol v2 and acknowledges configuration.
- Entered M×N or walked opposite corners are confirmed, including left/right
  coverage side. For walked mode, Corner A was captured while the phone top
  pointed along M.
- Entered start and far-end clearances are inside the coned site. Configure/Arm
  must be acknowledged without `F_HEADLAND`; the exported firmware log records
  the planned route style and its calculated clearance requirement.
- The mount/steering/speed calibration identity is current for this hardware.
- Tracking is `normal`, the pose is stable for at least two seconds/30 samples,
  the newest pose is fresh, and the readiness checklist is fully green.
- Pavement surface and dry/wet condition are correct. Wet remains locked out
  until Stages 1–8 have passed and their logs have been reviewed.
- The mission log was created successfully. Wet Start is forbidden if the
  authoritative phone log is unavailable.
- Arm produces an `ARMED` acknowledgement for the current epoch; Start then
  produces `RUNNING`. Do not move on an old or missing acknowledgement.

## Log procedure after every stage

1. Tap Stop after the measurement, even if the mission reports Complete.
2. Export both JSONL and CSV from Recent missions before changing the setup.
3. Name the files `S<stage>_<mode>_<direction>_<trial>_<surface>_<dry-or-wet>`.
4. Preserve fault-buffer export and the persisted fault summary for any fault.
5. From `auto_vio/test`, build and run the replay:

   ```sh
   g++ -std=c++17 -I.. -o /tmp/safespread-replay ../replay/replay.cpp
   /tmp/safespread-replay /absolute/path/to/export.csv
   ```

6. Record the complete one-line summary beside the physical measurements. Do
   not proceed unless the stage-specific gate and the replay gate both pass.

## Ordered field stages

### 1. Stationary pose and tracking quality

- Dry pavement; drive wheels disabled or lifted; spray off.
- Fully seat the phone mount at the marked origin. Start logging and leave the
  rover untouched for 60 seconds under the dimmest and brightest expected
  lighting, including the phone orientation used during operation.
- Pass: tracking remains `normal`; readiness becomes and stays green; position
  stays within a 0.10-foot radius; wrapped heading deviation stays within 1.0
  degree; no sequence, freshness, or mapping warnings occur.
- Stop if a shadow, glare, feature-poor patch, vibration, or a person crossing
  the camera view drops readiness. Correct the environment before repeating.

### 2. Measured out-and-back phone pose closure

- Dry pavement; rover remains disabled. Walk the mounted rover exactly 20 feet
  along a taped line, turn it 180 degrees, return to the same physical marks,
  and restore the original heading. Perform three trials.
- Pass every trial: final position closes within 0.10 ft of the start, final
  heading within 1.0 degree, tracking stays `normal`, and control-record gap
  stays at or below 250 ms.
- Stop for relocalization, sequence regression, or a closure miss. Do not hide
  accumulated drift by resetting tracking mid-trial.

### 3. Mount yaw and rigid-offset calibration

- Measure rear-axle-to-camera and rear-axle-to-spray-bar forward/right offsets.
  Run the in-app mount/yaw calibration on the taped line with spray off.
- Remove and re-seat the quick-release mount five times. After every re-seat,
  repeat the origin/heading check without changing the saved calibration.
- Pass: all five rover-position estimates are within a 0.10-foot radius and
  headings within 1.0 degree; the same calibration ID remains ready.
- Stop and redesign/adjust the mount if re-seating misses this gate. The app can
  estimate pose, but it cannot infer a physically changing camera-to-rover
  transform while driving.

### 4. Loaded dry steering and speed calibration

- Load the rover to intended operating mass on the actual dry pavement; spray
  remains off. Cone the full calibration envelope.
- Run straight, left-curvature, right-curvature, forward-speed, reverse-speed,
  and direction-change calibration steps exactly as prompted. Repeat any sample
  rejected by the app; never reuse an old surface/load calibration silently.
- Pass: calibration completes with a new current ID; the fitted steering map is
  monotonic with a measured zero-curvature point; all speed/direction checks
  finish without tracking, stall, wrong-direction, PWM, or I2C faults.
- The speed step only fits the feed-forward the loop starts from; it does not
  set the speed the rover holds. Changing the straight or turn target does not
  invalidate a stored calibration and does not require repeating this stage.
- Export calibration logs before any autonomous pass.

### 5. Repeated one-pass dry runs in both directions

- Mark one straight M line at least 20 feet long, with the entered and coned clear
  pavement at each end. Enter N no wider than one spray pass. Keep spray off.
- Run three passes in +M and three in -M, starting physically centered and
  aligned each time. Speed is not selectable: the loop holds the 1.5 ft/s
  straight target.
- Pass all six: p95 absolute cross-track <=1.275 inches, maximum <=2.55 inches,
  steering saturation <=10%, pose gap <=250 ms, no fault, and no unexplained
  rejected samples. Record measured endpoint and lateral errors.

### 6. Parallel passes with manual repositioning

- Mark at least five parallel M lines 14.45 inches center-to-center. Keep turns
  disabled by tapping Stop and manually repositioning the powered-off rover
  onto each next line. Spray remains off.
- Run each line in the alternating direction a real rectangle would use.
- Pass: every line meets the Stage 5 limits, measured adjacent centerline error
  never consumes more than the total 2.55-inch overlap margin, and no visible
  uncovered gap appears in the wheel-path/marker record.
- A failure here is line-control or pose/mount error, not a turn-planning issue.

### 7. Direction changes and lane entry, spray disabled

- The current app always requests forward-only preference; there is no manual
  route-style selector. Firmware automatically falls back to a three-point
  route only when forward-only does not fit the entered clearance and the
  three-point route does. Do not claim or search for a UI selector.
- First use `Verify reverse` and the dry self-test to exercise five verified
  forward/reverse engagements. For autonomous lane entry, cone the entered
  headlands and run at least five entries at each end on the route firmware
  selects. Inspect the exported firmware log for `forward-only` or `three-point`.
- If the actual rectangle/headland combination selects three-point, watch every
  neutral dwell, requested direction, actual displacement, and next-lane entry.
  If it selects forward-only, record that result and do not force a reverse
  maneuver outside the app's supported workflow.
- Pass: every maneuver stays inside the coned headland, direction engages only
  after the neutral/brake sequence, no leg is skipped, and the rover settles
  within the Stage 5 cross-track limit before the next sprayed segment would
  begin.
- Stop for any wrong-direction motion, failed neutral dwell, wheel slip, route
  jump, or need for more clearance than Configure accepted/the firmware log
  reported.

### 8. Complete dry rectangle in both definition modes

- Keep spray off. Run one small complete dry rectangle using entered M×N and a
  second using walked opposite corners. Across the two, exercise both left and
  right coverage-side confirmation. Repeat each mode once after removing and
  re-seating the phone mount.
- Pass all four missions: route finishes without fault or skipped leg; every
  sprayed-designated pass meets the Stage 5 limits; actual path stays inside
  the confirmed rectangle plus entered headlands; replay accepts each log.
- Review all Stage 1–8 JSONL/CSV logs together before authorizing Stage 9.

### 9. Low-speed wet rectangle after log review

- This stage is **not approved by automated tests**. It may begin only after a
  human review confirms every dry gate above and no unexplained rejection,
  saturation, pose-gap, or fault pattern remains.
- Use the smallest practical rectangle, generous coned headlands, and the
  intended wet load. Speed is the same servoed 1.5 ft/s as every dry stage, so
  there is no slower setting to fall back on -- if wet traction will not hold
  that speed, stop rather than looking for one. First validate with water when
  conditions permit; then use brine. Keep the operator ready on Stop throughout.
- Run one entered and one walked-corner rectangle. Inspect traction, stopping
  distance, line error, overlap, application consistency, and all exported logs
  after each run.
- Pass only if both wet runs meet every dry numerical limit with no slip or
  increased stopping envelope. Otherwise Stop, return to dry diagnostics, and
  do not use autonomous wet spreading.
