/*
 * SafeSpread - Phone-VIO Waypoint Navigator
 *
 * Position/heading come entirely from the SafeSpreadVIO iPhone app's ARKit
 * pose, sent as !P packets over BLE. No internal dead reckoning.
 *
 * The route for the whole mission is computed once when Start is pressed and
 * thereafter only followed. Nothing during the run re-decides where to go:
 * tracking error steers the rover back onto the plan instead of producing a
 * different plan, which is what made the earlier reactive version oscillate
 * between maneuvers.
 *
 * Mission control is protocol v2. Legacy one-byte Start is deliberately
 * refused; the app owns Configure, Arm, Start, Stop, radius measurement, and
 * the retained self-test.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <math.h>
#include "direction.h"
#include "fault_buffer.h"
#include "headland.h"
#include "mission_protocol.h"
#include "nav_math.h"
#include "protocol_v2.h"
#include "pwm_health.h"
#include "radius_calibration.h"
#include "route.h"
#include "safety.h"
#include "speed_control.h"
#include "steering.h"
#include "steering_map.h"

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define PCA9685_ADDR 0x40
const uint8_t STEER_CH = 0;
const uint8_t ESC_CH   = 4;

const int VALVE_PIN = 5;
const int PUMP_PIN  = 6;

const int NEUTRAL_US = 1500;

// Safe dry-calibration starting points only. Navigation closes the loop on
// measured speed; Task 11 replaces these feed-forward magnitudes with the
// accepted pavement calibration before wet operation can arm.
const float DRY_FALLBACK_FWD_OFFSET_US = 120.0f;
const float DRY_FALLBACK_REV_OFFSET_US = 120.0f;

const int STEER_CENTER_US = 1500;
const int STEER_LEFT_US   = 2390;
const int STEER_RIGHT_US  = 700;

const float BAR_WIDTH_FT          = 21.0f / 12.0f;
const float LANE_OVERLAP_FRACTION = 0.0f;

// N x M, matching the app's two inputs and set at runtime via '!D'.
//   N (fieldPassFt)  -- length of the first pass, straight ahead from the
//                       start corner. Lanes run along this axis (+Y).
//   M (fieldWidthFt) -- total width covered by stepping lanes to the right (+X).
float fieldPassFt  = 21.91f;
float fieldWidthFt = 21.91f;

// Safe fallback until the one-button measurement has run. The measurement
// changes only these curvature values; it deliberately keeps the existing
// left/centre/right PWM signals unchanged.
float turnRadiusLeftFt  = 5.54f;
float turnRadiusRightFt = 5.05f;

// The pulse at which the wheels actually point straight ahead. It is NOT the
// midpoint of the servo's travel, and assuming it was is what made every
// outbound pass sit off to one side and every return pass off to the other --
// which shows up as pairs of overlapping lines with gaps between them, not as
// obvious drift.
//
// Explicit measured curvature map. Straight is a direct calibration knot,
// not a midpoint inferred from the two steering endpoints. Runtime control
// interpolates this table without silently relearning its center.
SteeringKnot steeringMap[3] = {
  {STEER_LEFT_US, -1.0f / 5.54f},
  {1709, 0.0f},
  {STEER_RIGHT_US, 1.0f / 5.05f},
};
int steeringMapCount = 3;
bool steeringMapValid = false;
int  lastSteerCommandUs = 1500;

float steerCentreUs() {
  const float pulse = pulseForCurvature(steeringMap, steeringMapCount, 0.0f);
  return isfinite(pulse) ? pulse : static_cast<float>(STEER_CENTER_US);
}

/** Tightest curvature the map claims is available turning the given way, as a
 *  positive magnitude. Zero when the map has not been validated. */
float maxCurvaturePerFt(bool rightward) {
  if (!steeringMapValid || steeringMapCount < 2) return 0.0f;
  const float edge = rightward ? steeringMap[steeringMapCount - 1].curvaturePerFt
                               : steeringMap[0].curvaturePerFt;
  return fabsf(edge);
}

// Steering polarity: +1 means a pulse BELOW centre steers toward increasing
// heading (clockwise / to the rover's right), matching STEER_RIGHT_US < centre.
// Measured rather than assumed -- wiring the servo backwards inverts every
// correction, which does not look like a fault, it looks like wandering.
int  steerSign = 1;
bool steerSignChecked = false;
float polarityScore = 0.0f;
float polarityLastHeading = 0.0f;
float polarityLastCommand = 0.0f;
const float POLARITY_DECIDE_AT = 60.0f;

const float MAX_STEER_OFFSET = 700.0f;

// Course over ground: the direction the rover is actually travelling, taken
// from successive positions rather than from which way the phone points. Any
// constant yaw error in how the phone is mounted cancels out of this, which it
// does not for the reported heading -- and a mounting error of a few degrees
// otherwise biases lane tracking in opposite directions on outbound and return
// passes, striping the coverage.
float prevSampleX = 0.0f, prevSampleY = 0.0f;
float courseDeg = 0.0f;
bool  courseValid = false;
const float COURSE_MIN_STEP_FT = 0.20f;
const float COURSE_SMOOTHING   = 0.35f;

float robotX_ft    = 0.0f;
float robotY_ft    = 0.0f;
float robotHeading = 0.0f;
unsigned long lastVioTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long packetCount = 0;
uint32_t invalidPacketCount = 0;
// A silent pose stream and a rejected pose stream look identical from the
// rover's telemetry, and they have opposite fixes. Count every rejection by
// cause and say so out loud, slowly enough not to flood the BLE link.
uint16_t poseRejectCounts[F_HEADLAND + 1] = {};
unsigned long lastPoseRejectLogMs = 0;
volatile uint16_t queueOverflowCount = 0;
bool dryRunMode  = false;
bool sprayActive = false;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
BLECharacteristic *txCharacteristic = NULL;
volatile bool bleConnected = false;
portMUX_TYPE safetyEventMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool emergencyStopRequested = false;
volatile bool emergencyStopNeedsAck = false;
volatile uint16_t emergencyStopEpoch = 0;
volatile uint32_t emergencyStopCommandId = 0;
volatile uint32_t emergencyStopReceivedAt = 0;
volatile bool disconnectRequested = false;
volatile bool statusTelemetryRequested = false;
volatile uint32_t safetyAbortGeneration = 0;
bool selfTestActive = false;

MissionProtocol mission;

const size_t FAULT_BUFFER_SAMPLES = 300;  // at least five seconds at 60 Hz
FaultBuffer<FAULT_BUFFER_SAMPLES> faultBuffer;

class PreferencesFaultStore {
 public:
  bool read(uint8_t *out, size_t size) {
    Preferences prefs;
    if (!prefs.begin("ss-fault", true)) return false;
    bool ok = prefs.getBytesLength("summary") == size &&
              prefs.getBytes("summary", out, size) == size;
    prefs.end();
    return ok;
  }
  bool write(const uint8_t *data, size_t size) {
    Preferences prefs;
    if (!prefs.begin("ss-fault", false)) return false;
    bool ok = prefs.putBytes("summary", data, size) == size;
    prefs.end();
    return ok;
  }
};

PreferencesFaultStore faultStore;
FaultSummaryPersistence<PreferencesFaultStore> faultSummary(faultStore, FAULT_SUMMARY_SCHEMA);
FaultSummary bootFaultSummary = {};
bool hasBootFaultSummary = false;

constexpr uint32_t HARDWARE_TAG_HASH = 0x89abcdef;

class PreferencesTurnRadiusStore {
 public:
  bool read(uint8_t *out, size_t size) {
    Preferences prefs;
    if (!prefs.begin("ss-radius", true)) return false;
    const bool ok = prefs.getBytesLength("measured") == size &&
                    prefs.getBytes("measured", out, size) == size;
    prefs.end();
    return ok;
  }
  bool write(const uint8_t *data, size_t size) {
    Preferences prefs;
    if (!prefs.begin("ss-radius", false)) return false;
    const bool ok = prefs.putBytes("measured", data, size) == size;
    prefs.end();
    return ok;
  }
};

PreferencesTurnRadiusStore turnRadiusStore;
TurnRadiusPersistence<PreferencesTurnRadiusStore> turnRadiusPersistence(turnRadiusStore);
StoredTurnRadii storedTurnRadii = {};
bool hasMeasuredTurnRadii = false;
bool radiusMeasurementActive = false;
RadiusSweep radiusSweep;

const int MAX_ROUTE_POINTS = 6000;
// The plan is 70 KB -- by far the largest thing this firmware owns. Held
// statically it does not fit alongside the BLE stack in the ESP32's DRAM
// segment, so it is taken from the heap once at boot, before Bluedroid claims
// its share. A rover that cannot get it says so and refuses to plan, rather
// than planning into a null pointer.
RoutePoint *route = nullptr;
int routeCount = 0;
int routeIndex = 0;
// Where this mission begins in the plan. Normally zero; on a resume it is the
// first point of the pass the operator chose, so a fault costs the passes that
// are left rather than the whole rectangle.
int routeStartIndex = 0;
int firstPassEnd = 0;      // route index where the all-important first pass ends
RouteStyle routeStyle = ROUTE_NONE;
RouteRequirements routeRequirements = {};
FaultCode routePlanningFault = F_ROUTE;

// How close the rover must be to a resumed pass before it may arm. Wider than
// the app's start check, because the operator is aiming at a lane in the middle
// of the rectangle by eye rather than at a marked corner.
const float RESUME_POSITION_TOLERANCE_FT = 1.5f;
const float RESUME_HEADING_TOLERANCE_DEG = 12.0f;

// Steer at a point this far ahead on the path. On a straight pass a long
// lookahead tracks smoothly; through a turn it must be shorter than the arc's
// radius or the rover simply cuts the corner and misses the maneuver.
const float LOOKAHEAD_TURN_FT = 1.0f;

// Through a turn the arc itself is fed forward from the plan, so the lookahead
// error only has to correct. Dividing that error by the raw lookahead distance
// makes the correction alone reach full lock at six degrees, which is inside
// the pose noise -- so the "correction" becomes the whole command and the
// steering behaves as a relay slamming between the two locks. Measuring the
// error against a longer distance keeps it proportional over the range that
// actually occurs, and the cap stops one bad lookahead sample from swinging the
// wheels lock to lock on its own.
const float TURN_CORRECTION_DISTANCE_FT = 3.0f;
const float TURN_CORRECTION_LIMIT_FRACTION = 0.5f;

// How sharply the rover converges onto a straight pass, in feet. Smaller is
// quicker but works the steering harder against position noise; 1.5 ft closes
// a foot of error in about 6 ft of travel without ever crossing the line.
const float LINE_DISTANCE_CONST_FT = 1.5f;
const int   ROUTE_SEARCH_WINDOW = ROUTE_PROGRESS_SEARCH_WINDOW;
const float CUSP_TOL_FT       = 0.1f;

bool escReverse = false;
bool directionRequested = false;
DirectionState driveDirection;
SpeedPI speedController;
unsigned long lastSpeedControlMs = 0;
float forwardFeedForwardUs = DRY_FALLBACK_FWD_OFFSET_US;
float reverseFeedForwardUs = DRY_FALLBACK_REV_OFFSET_US;

// If the rover cannot make measured progress toward its target, fault rather
// than silently skipping untreated pavement.
const unsigned long STALL_TIMEOUT_MS = 4000;
int lastRouteIndex = -1;
unsigned long routeIndexSince = 0;
float bestTargetDistanceFt = 0.0f;
bool targetDistanceValid = false;

// Spray while the plan says to, unless the rover is so far off the plan that
// spraying would put brine somewhere it does not belong. The old test -- being
// inside the rectangle -- cut spray off half a foot outside the first lane,
// which sits on x=0, so the most important pass of the mission was the one
// most likely to stop spraying.
const float SPRAY_OFFPLAN_FT       = 1.5f;
const float SPRAY_OFFPLAN_FIRST_FT = 3.0f;   // the first pass gets more rope
const float SPRAY_HYSTERESIS_FT    = 0.5f;
const float SPRAY_HEADING_TOLERANCE_DEG = 10.0f;
bool sprayInhibited = false;

// How far the rover currently sits from the point it is tracking, and the
// worst it has been on the pass it is driving. Position accuracy and tracking
// accuracy are different things -- the phone can know exactly where the rover
// is while the rover still fails to sit on its line -- and this is the number
// that says which of the two is the problem.
float offPlanFt = 0.0f;
float worstOffThisPass = 0.0f;
float lastCrossTrackFt = 0.0f;
float lastHeadingErrorDeg = 0.0f;
float lastPoseSpeedFps = 0.0f;
uint32_t lastControlSequence = 0;
uint16_t lastPoseAgeMs = 0;


void bleLog(String msg) {
  if (bleConnected && txCharacteristic != NULL) {
    msg += "\n";
    txCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    txCharacteristic->notify();
  }
}

void bleBinary(const uint8_t *data, size_t size) {
  if (!bleConnected || txCharacteristic == NULL) return;
  txCharacteristic->setValue((uint8_t *)data, size);
  txCharacteristic->notify();
}

void sendAck(const protocol_v2::AckV2 &ack) {
  uint8_t packet[protocol_v2::ACK_SIZE];
  if (protocol_v2::buildAckV2(ack, packet, sizeof(packet))) bleBinary(packet, sizeof(packet));
}

void sendFaultBuffer() {
  uint8_t packet[protocol_v2::FAULT_SAMPLE_SIZE];
  for (uint16_t index = 0; index < faultBuffer.size(); ++index) {
    if (faultBuffer.buildPacket(index, mission.epoch(), packet, sizeof(packet))) {
      bleBinary(packet, sizeof(packet));
      delay(4);
    }
  }
}

void resetCourse() {
  courseValid = false;
  prevSampleX = robotX_ft;
  prevSampleY = robotY_ft;
}

void updateCourse() {
  float dx = robotX_ft - prevSampleX;
  float dy = robotY_ft - prevSampleY;
  if (sqrtf(dx * dx + dy * dy) < COURSE_MIN_STEP_FT) return;

  float sample = bearingToWaypointDeg(dx, dy);
  if (!courseValid) {
    courseDeg = sample;
    courseValid = true;
  } else {
    courseDeg = fmodf(courseDeg + COURSE_SMOOTHING * angleDiffDeg(sample, courseDeg) +
                      360.0f, 360.0f);
  }
  prevSampleX = robotX_ft;
  prevSampleY = robotY_ft;
}

// Correlate what we asked the steering to do against what the heading actually
// did. Only meaningful driving forward: in reverse the same lock swings the
// nose the other way, which would cancel the evidence out.
void observeSteeringPolarity() {
  if (steerSignChecked) return;

  float turned = angleDiffDeg(robotHeading, polarityLastHeading);
  polarityLastHeading = robotHeading;

  float cmd = polarityLastCommand;
  if (fabsf(cmd) < 150.0f || fabsf(turned) < 0.5f || fabsf(turned) > 30.0f) return;

  polarityScore += (cmd > 0.0f) == (turned > 0.0f) ? fabsf(turned) : -fabsf(turned);

  if (fabsf(polarityScore) >= POLARITY_DECIDE_AT) {
    steerSignChecked = true;
    if (polarityScore < 0.0f) {
      steerSign = -steerSign;
      bleLog("!! Steering was reversed; polarity corrected automatically.");
    } else {
      bleLog("[OK] Steering polarity confirmed.");
    }
  }
}

// PCA9685 registers. An I2C ACK only proves the chip is powered and
// addressable; it says nothing about whether it is still configured.
#define PCA_MODE1     0x00
#define PCA_PRESCALE  0xFE
#define PCA_SLEEP_BIT 0x10
#define PCA_EXPECTED_PRESCALE 121   // 25MHz / (4096 * 50Hz) - 1

int lastSteerUs = STEER_CENTER_US;
int lastEscUs   = NEUTRAL_US;
unsigned long lastPwmCheck = 0;
unsigned long pwmRecoveries = 0;
bool pwmHealthy = false;
bool pwmMissionReady = false;
PwmReadinessGate pwmReadiness;

bool safetyEventPending() {
  bool pending;
  portENTER_CRITICAL(&safetyEventMux);
  pending = emergencyStopRequested || disconnectRequested;
  portEXIT_CRITICAL(&safetyEventMux);
  return pending;
}

void setChannelPulse(uint8_t channel, int microseconds) {
  if (!bleConnected || safetyEventPending()) {
    if (channel == ESC_CH) microseconds = NEUTRAL_US;
    if (channel == STEER_CH) microseconds = (int)steerCentreUs();
  }
  if (channel == STEER_CH) lastSteerUs = microseconds;
  if (channel == ESC_CH)   lastEscUs = microseconds;
  uint16_t ticks = (uint16_t)(((uint32_t)microseconds * 4096UL) / 20000UL);
  pwm.setPWM(channel, 0, ticks);
}

uint8_t readPcaRegister(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0xFF;
  if (Wire.requestFrom((int)PCA9685_ADDR, 1) != 1) return 0xFF;
  return Wire.read();
}

// A brownout -- typically the steering servo stalling and sagging the rail --
// resets the PCA9685 into its power-on state: asleep, prescaler unset, all
// outputs dead. The ESP32 rides through on its own decoupling and never
// notices, so the sketch keeps sending pulses to a chip that is ignoring them.
bool ensurePwmReady() {
  uint8_t mode1 = readPcaRegister(PCA_MODE1);
  uint8_t prescale = readPcaRegister(PCA_PRESCALE);

  // A single I2C miss can be electrical noise. Retry once before treating the
  // controller as unavailable.
  if (mode1 == 0xFF || prescale == 0xFF) {
    delay(2);
    mode1 = readPcaRegister(PCA_MODE1);
    prescale = readPcaRegister(PCA_PRESCALE);
  }

  if (mode1 == 0xFF || prescale == 0xFF) {
    pwmHealthy = false;
    return false;
  }

  bool asleep = (mode1 & PCA_SLEEP_BIT) != 0;
  bool wrongRate = abs((int)prescale - PCA_EXPECTED_PRESCALE) > 3;
  if (!asleep && !wrongRate) {
    pwmHealthy = true;
    return true;
  }

  pwmRecoveries++;
  bleLog("!! PWM chip lost its config (mode1=0x" + String(mode1, HEX) +
         " prescale=" + String(prescale) + "). Reinitialising.");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  // A reset may have occurred while moving. Re-establish only safe outputs;
  // replaying the previous throttle pulse would restart motion without a new
  // pose or command.
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, (int)steerCentreUs());

  // Recovery used to return false unconditionally, so even a successful
  // reinitialisation immediately latched fault 5. Verify the new state and
  // report ready when the chip is awake at the correct output rate.
  mode1 = readPcaRegister(PCA_MODE1);
  prescale = readPcaRegister(PCA_PRESCALE);
  bool recovered = mode1 != 0xFF && prescale != 0xFF &&
                   (mode1 & PCA_SLEEP_BIT) == 0 &&
                   abs((int)prescale - PCA_EXPECTED_PRESCALE) <= 3;
  pwmHealthy = recovered;
  if (recovered) {
    bleLog("[OK] PWM chip recovered and verified.");
  } else {
    bleLog("!! PWM recovery did not verify (mode1=0x" + String(mode1, HEX) +
           " prescale=" + String(prescale) + ").");
  }
  return recovered;
}

// In dry-run mode the spray state is still tracked and reported so the app can
// show where it *would* have sprayed, but no hardware is energised.
void setSpray(bool on) {
  if (!bleConnected || safetyEventPending()) on = false;
  if (sprayActive != on) {
    sprayActive = on;
    bleLog(on ? "[SPRAY] ON" : "[SPRAY] OFF");
  }

  if (dryRunMode) {
    digitalWrite(VALVE_PIN, LOW);
    digitalWrite(PUMP_PIN, LOW);
    return;
  }

  digitalWrite(VALVE_PIN, on ? HIGH : LOW);
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

void stopDrive() {
  driveDirection.stop();
  speedController.reset();
  directionRequested = false;
  lastSpeedControlMs = 0;
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, (int)steerCentreUs());
}

void applyTurnRadii(float leftFt, float rightFt) {
  if (!validTurnRadius(leftFt) || !validTurnRadius(rightFt)) return;
  turnRadiusLeftFt = leftFt;
  turnRadiusRightFt = rightFt;
  steeringMap[0].curvaturePerFt = -1.0f / leftFt;
  steeringMap[steeringMapCount - 1].curvaturePerFt = 1.0f / rightFt;
  steeringMapValid = validSteeringMap(steeringMap, steeringMapCount);
}

bool steeringModelReady() {
  return steeringMapValid;
}

uint16_t saturatedPacketDrops() {
  uint32_t total = mission.droppedPoses() + queueOverflowCount;
  return total > 0xffff ? 0xffff : static_cast<uint16_t>(total);
}

uint16_t saturatedCounter(uint32_t value) {
  return value > 0xffff ? 0xffff : static_cast<uint16_t>(value);
}

ControlSample currentControlSample() {
  ControlSample sample = {};
  sample.sequence = lastControlSequence;
  sample.routeIndex = routeIndex < 0 ? 0 : saturatedCounter(routeIndex);
  sample.crossTrackFt = lastCrossTrackFt;
  sample.headingErrorDeg = lastHeadingErrorDeg;
  sample.speedFps = lastPoseSpeedFps;
  sample.steeringUs = saturatedCounter(lastSteerUs);
  sample.throttleUs = saturatedCounter(lastEscUs);
  sample.state = mission.state();
  sample.fault = mission.fault();
  sample.droppedPackets = saturatedPacketDrops();
  return sample;
}

void sendTelemetry();

void enterFault(FaultCode fault) {
  if (fault == F_NONE) return;
  const bool firstFault = !faultBuffer.frozen();
  mission.setFault(fault);
  setSpray(false);
  stopDrive();

  if (!firstFault) {
    sendTelemetry();
    return;
  }
  faultBuffer.push(currentControlSample());
  faultBuffer.freeze(fault);
  sendTelemetry();

  FaultSummary summary = {};
  summary.schema = FAULT_SUMMARY_SCHEMA;
  summary.epoch = mission.epoch();
  summary.fault = fault;
  summary.routeIndex = routeIndex < 0 ? 0 : saturatedCounter(routeIndex);
  summary.droppedPackets = saturatedPacketDrops();
  summary.invalidPackets = saturatedCounter(invalidPacketCount);
  summary.controlSequence = lastControlSequence;
  if (faultSummary.persistOnce(summary)) {
    bootFaultSummary = summary;
    hasBootFaultSummary = true;
    bleLog("[FAULT SUMMARY] epoch=" + String(summary.epoch) +
           " fault=" + String(static_cast<int>(summary.fault)) +
           " pt=" + String(summary.routeIndex) +
           " drops=" + String(summary.droppedPackets) +
           " invalid=" + String(summary.invalidPackets) +
           " seq=" + String(summary.controlSequence));
  }
  bleLog("!!! FAULT " + String(static_cast<int>(fault)) +
         " -- drive neutral, steering centred, spray off. !!!");
}

void recordControlSample() {
  faultBuffer.push(currentControlSample());
}

void sendTelemetry() {
  protocol_v2::TelemetryV2 telemetry = {};
  telemetry.state = static_cast<uint8_t>(mission.state());
  telemetry.epoch = mission.epoch();
  telemetry.consumedPoseSequence = lastControlSequence;
  telemetry.routeIndex = routeIndex < 0 ? 0 : saturatedCounter(routeIndex);
  telemetry.routeCount = routeCount < 0 ? 0 : saturatedCounter(routeCount);
  telemetry.crossTrackFt = lastCrossTrackFt;
  telemetry.headingErrorDeg = lastHeadingErrorDeg;
  telemetry.speedFps = lastPoseSpeedFps;
  telemetry.steeringUs = saturatedCounter(lastSteerUs);
  telemetry.throttleUs = saturatedCounter(lastEscUs);
  telemetry.flags = (sprayActive ? 0x01 : 0) |
                    (escReverse ? 0x02 : 0) |
                    (pwmHealthy ? 0x04 : 0);
  telemetry.faultCode = static_cast<uint8_t>(mission.fault());
  telemetry.droppedPackets = saturatedPacketDrops();
  const uint32_t totalAge = static_cast<uint32_t>(lastPoseAgeMs) +
                            (millis() - lastVioTime);
  telemetry.poseAgeMs = saturatedCounter(totalAge);

  uint8_t packet[protocol_v2::TELEMETRY_SIZE];
  if (protocol_v2::buildTelemetryV2(telemetry, packet, sizeof(packet))) {
    bleBinary(packet, sizeof(packet));
  }
}

void sendRoutePlan() {
  if (routeStyle == ROUTE_NONE || routeCount <= 0 || routeRequirements.truncated) return;
  protocol_v2::RoutePlanV2 plan = {};
  plan.style = static_cast<uint8_t>(routeStyle);
  plan.epoch = mission.epoch();
  plan.routeCount = saturatedCounter(routeCount);
  plan.passCount = saturatedCounter(routePassCount(route, routeCount));
  plan.leftRadiusFt = turnRadiusLeftFt;
  plan.rightRadiusFt = turnRadiusRightFt;
  plan.beforeStartFt = routeRequirements.beforeStartFt;
  plan.beyondEndFt = routeRequirements.beyondEndFt;
  uint8_t packet[protocol_v2::ROUTE_PLAN_SIZE];
  if (protocol_v2::buildRoutePlanV2(plan, packet, sizeof(packet))) {
    bleBinary(packet, sizeof(packet));
  }
}

void announceTurnRadii() {
  bleLog("[RADIUS STATUS] left=" + String(turnRadiusLeftFt, 2) +
         " right=" + String(turnRadiusRightFt, 2) + " ft source=" +
         String(hasMeasuredTurnRadii ? "measured" : "built-in") + ".");
}

void announceArea() {
  int lanes = laneCount(fieldWidthFt, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  float covered = laneCenterX(lanes - 1, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION) +
                  BAR_WIDTH_FT * 0.5f;
  bleLog("Area " + String(fieldPassFt, 1) + " x " + String(fieldWidthFt, 1) +
         " ft -> " + String(lanes) + " lanes, covering " +
         String(covered, 1) + " ft of width.");
}

void requestEmergencyStop(uint16_t epoch, uint32_t commandId,
                          uint32_t receivedAtMs, bool needsAck) {
  portENTER_CRITICAL(&safetyEventMux);
  emergencyStopEpoch = epoch;
  emergencyStopCommandId = commandId;
  emergencyStopReceivedAt = receivedAtMs;
  emergencyStopNeedsAck = needsAck;
  emergencyStopRequested = true;
  safetyAbortGeneration++;
  portEXIT_CRITICAL(&safetyEventMux);
}

uint32_t currentSafetyAbortGeneration() {
  uint32_t generation;
  portENTER_CRITICAL(&safetyEventMux);
  generation = safetyAbortGeneration;
  portEXIT_CRITICAL(&safetyEventMux);
  return generation;
}

bool controlAuthorized() {
  return bleConnected && !safetyEventPending() && mission.state() == S_RUNNING;
}

void serviceSafetyEvents() {
  bool shouldDisconnect;
  bool shouldStop;
  bool shouldAck;
  bool shouldSendStatus;
  uint16_t stopEpoch;
  uint32_t stopCommandId;
  uint32_t stopReceivedAt;

  portENTER_CRITICAL(&safetyEventMux);
  shouldDisconnect = disconnectRequested;
  disconnectRequested = false;
  shouldStop = emergencyStopRequested;
  emergencyStopRequested = false;
  shouldAck = emergencyStopNeedsAck;
  emergencyStopNeedsAck = false;
  stopEpoch = emergencyStopEpoch;
  stopCommandId = emergencyStopCommandId;
  stopReceivedAt = emergencyStopReceivedAt;
  shouldSendStatus = statusTelemetryRequested;
  statusTelemetryRequested = false;
  portEXIT_CRITICAL(&safetyEventMux);

  if (shouldDisconnect) {
    mission.onDisconnect();
    if (mission.state() == S_FAULT) enterFault(mission.fault());
    else {
      setSpray(false);
      stopDrive();
    }
  }

  if (shouldStop) {
    protocol_v2::CommandV2 stop = {3, stopEpoch, stopCommandId};
    protocol_v2::AckV2 ack = mission.acceptCommand(stop, stopReceivedAt);
    setSpray(false);
    stopDrive();
    routeCount = 0;
    routeIndex = 0;
    resetCourse();
    if (shouldAck) sendAck(ack);
    bleLog(">>> Mission stopped.");
    announceTurnRadii();
  }

  if (shouldSendStatus) sendTelemetry();
}

#define QSLOTS 16
#define QBYTES 32
static uint8_t qData[QSLOTS][QBYTES];
static uint8_t qLen[QSLOTS];
static uint32_t qReceivedAt[QSLOTS];
static volatile uint8_t qHead = 0, qTail = 0;

void queueWrite(const uint8_t *d, size_t n) {
  const uint32_t receivedAtMs = millis();
  if (n == 1 && d[0] == '2') {
    requestEmergencyStop(0, 0, receivedAtMs, false);
    return;
  }
  if (n == protocol_v2::COMMAND_SIZE) {
    protocol_v2::CommandV2 command = {};
    if (protocol_v2::parseCommandV2(d, n, command) && command.opcode == 3) {
      requestEmergencyStop(command.epoch, command.commandId, receivedAtMs, true);
      return;
    }
  }
  if (n > QBYTES) {
    invalidPacketCount++;
    return;
  }
  uint8_t next = (qHead + 1) % QSLOTS;
  if (next == qTail) {
    if (queueOverflowCount < 0xffff) queueOverflowCount++;
    return;
  }
  memcpy(qData[qHead], d, n);
  qLen[qHead] = n;
  qReceivedAt[qHead] = receivedAtMs;
  qHead = next;
}

void feed(const uint8_t *d, size_t n, uint32_t receivedAtMs);
void consumePendingPose();

// Pose packets arrive on the BLE queue and are only parsed when the queue is
// drained, so anything that waits has to keep draining it or the rover's idea
// of where it is stops updating mid-test.
void pumpBle() {
  serviceSafetyEvents();
  while (qTail != qHead) {
    serviceSafetyEvents();
    // Pop into a local copy before feed(). A v2 self-test pumps BLE while it
    // waits, so feed can be re-entered; leaving qTail on the active slot would
    // make the nested pump consume and later replay that same command.
    uint8_t packet[QBYTES];
    const uint8_t length = qLen[qTail];
    const uint32_t receivedAt = qReceivedAt[qTail];
    memcpy(packet, qData[qTail], length);
    qTail = (qTail + 1) % QSLOTS;
    feed(packet, length, receivedAt);
  }
  serviceSafetyEvents();
  consumePendingPose();
}

/** Prove a direction through the same neutral/brake/command/verify state used
 *  by navigation. Report displacement along the rover's initial nose. */
bool diagnosticCanContinue(uint32_t generation) {
  bool pending;
  uint32_t currentGeneration;
  portENTER_CRITICAL(&safetyEventMux);
  pending = emergencyStopRequested || disconnectRequested;
  currentGeneration = safetyAbortGeneration;
  portEXIT_CRITICAL(&safetyEventMux);
  return !pending && currentGeneration == generation && bleConnected &&
         mission.state() == S_IDLE;
}

bool diagnosticWait(unsigned long durationMs, uint32_t generation) {
  const unsigned long startedAt = millis();
  while (millis() - startedAt < durationMs) {
    pumpBle();
    if (!diagnosticCanContinue(generation)) return false;
    delay(10);
  }
  return diagnosticCanContinue(generation);
}

bool measureDrive(bool reverse, uint32_t generation, float &alongFt) {
  if (!diagnosticCanContinue(generation)) return false;
  float h = robotHeading * (float)M_PI / 180.0f;
  setChannelPulse(STEER_CH, (int)steerCentreUs());
  float x0 = robotX_ft, y0 = robotY_ft;

  const int pulseUs = NEUTRAL_US + static_cast<int>(
      reverse ? -reverseFeedForwardUs : forwardFeedForwardUs);
  driveDirection.begin(reverse, reverse, millis(), robotX_ft, robotY_ft,
                       robotHeading, pulseUs, NEUTRAL_US);
  while (!driveDirection.ready() && !driveDirection.failed()) {
    pumpBle();
    if (!diagnosticCanContinue(generation)) {
      stopDrive();
      return false;
    }
    setChannelPulse(ESC_CH,
                    driveDirection.update(millis(), robotX_ft, robotY_ft));
    delay(10);
  }

  setChannelPulse(ESC_CH, NEUTRAL_US);
  if (!diagnosticWait(500, generation)) return false;

  float dx = robotX_ft - x0, dy = robotY_ft - y0;
  alongFt = dx * sinf(h) + dy * cosf(h);
  const bool verified = driveDirection.ready();
  driveDirection.stop();
  return verified;
}

// Whether the rover can actually back up is the one assumption the whole route
// rests on: every lane change is a three-point turn, and an ESC that refuses
// reverse turns each one into a circle rather than into an obvious fault. Ask
// the rover to prove it rather than assuming it.
bool runDriveTest(uint32_t generation) {
  bleLog("--- DRIVE TEST (rover will move a few ft each way) ---");
  if (!mission.poseFresh(millis())) {
    bleLog("[SKIP] No pose from the phone; cannot tell whether it moved.");
    return true;
  }

  float fwd = 0.0f, rev = 0.0f;
  bool movedFwd = measureDrive(false, generation, fwd);
  bool forwardVerified = movedFwd && fwd > 0.0f;
  if (!diagnosticCanContinue(generation)) return false;
  bleLog("Forward: moved " + String(fwd, 2) + " ft along its nose.");
  if (!movedFwd) {
    bleLog("[FAIL] Did not verify forward motion before the timeout.");
    bleLog("       Check drive battery/ESC or recalibrate feed-forward.");
  } else if (fwd < 0.0f) {
    bleLog("[FAIL] Moved BACKWARDS on a forward command -- ESC is reversed.");
  } else {
    bleLog("[PASS] Forward works.");
  }

  bool movedRev = measureDrive(true, generation, rev);
  bool reverseVerified = movedRev && rev < 0.0f;
  if (!diagnosticCanContinue(generation)) return false;
  bleLog("Reverse: moved " + String(rev, 2) + " ft along its nose.");
  if (!movedRev) {
    bleLog("[FAIL] REVERSE DID NOT ENGAGE. This is fatal for the route: every");
    bleLog("       lane change is a three-point turn, so each one becomes a");
    bleLog("       circle instead. Many ESCs need a longer neutral, or a");
    bleLog("       double-tap of reverse, before they will accept it.");
  } else if (rev > 0.0f) {
    bleLog("[FAIL] Moved FORWARDS on a reverse command -- no reverse available.");
  } else {
    bleLog("[PASS] Reverse works (" + String(-rev, 2) + " ft back).");
  }

  stopDrive();
  bleLog("--- DRIVE TEST COMPLETE ---");
  return forwardVerified && reverseVerified;
}

bool radiusMeasurementCanContinue(uint32_t generation) {
  return diagnosticCanContinue(generation) && dryRunMode &&
         mission.poseFresh(millis()) && !safetyEventPending();
}

bool measureOneTurnRadius(bool right, uint32_t generation,
                          RadiusSweepResult &result) {
  if (!radiusMeasurementCanContinue(generation)) return false;
  const int steeringUs = right ? STEER_RIGHT_US : STEER_LEFT_US;
  const int throttleUs = NEUTRAL_US + static_cast<int>(forwardFeedForwardUs);
  setSpray(false);
  setChannelPulse(STEER_CH, steeringUs);
  driveDirection.begin(false, false, millis(), robotX_ft, robotY_ft,
                       robotHeading, throttleUs, NEUTRAL_US);
  while (!driveDirection.ready() && !driveDirection.failed()) {
    pumpBle();
    if (!radiusMeasurementCanContinue(generation)) {
      stopDrive();
      return false;
    }
    setChannelPulse(STEER_CH, steeringUs);
    setChannelPulse(ESC_CH,
                    driveDirection.update(millis(), robotX_ft, robotY_ft));
    delay(10);
  }
  if (!driveDirection.ready()) {
    stopDrive();
    return false;
  }

  radiusSweep.begin(lastControlSequence, robotX_ft, robotY_ft, robotHeading);
  const unsigned long startedAt = millis();
  while (!radiusSweep.reachedTarget() && !radiusSweep.failed() &&
         radiusSweep.pathFt() < 30.0f && millis() - startedAt < 45000) {
    pumpBle();
    if (!radiusMeasurementCanContinue(generation)) {
      stopDrive();
      return false;
    }
    radiusSweep.add(lastControlSequence, robotX_ft, robotY_ft, robotHeading);
    setChannelPulse(STEER_CH, steeringUs);
    setChannelPulse(ESC_CH,
                    driveDirection.update(millis(), robotX_ft, robotY_ft));
    delay(10);
  }
  stopDrive();
  result = radiusSweep.finish(right);
  return result.valid;
}

void runTurnRadiusMeasurement() {
  if (radiusMeasurementActive) {
    bleLog("[RADIUS FAIL] A measurement is already active.");
    return;
  }
  radiusMeasurementActive = true;
  const uint32_t generation = currentSafetyAbortGeneration();
  RadiusSweepResult left = {}, right = {};

  bleLog("[RADIUS] Measuring LEFT at existing " + String(STEER_LEFT_US) +
         "us. The rover will sweep about 90 degrees.");
  if (!measureOneTurnRadius(false, generation, left)) {
    bleLog("[RADIUS FAIL] Left sweep was not long, smooth, and circular enough.");
  } else {
    bleLog("[RADIUS SAMPLE] left=" + String(left.radiusFt, 2) +
           " ft (arc check " + String(left.arcRadiusFt, 2) + " ft, " +
           String(fabsf(left.sweepDeg), 0) + " deg).");
  }

  if (left.valid && diagnosticWait(1000, generation)) {
    bleLog("[RADIUS] Measuring RIGHT at existing " + String(STEER_RIGHT_US) +
           "us. The rover will sweep back about 90 degrees.");
    if (!measureOneTurnRadius(true, generation, right)) {
      bleLog("[RADIUS FAIL] Right sweep was not long, smooth, and circular enough.");
    } else {
      bleLog("[RADIUS SAMPLE] right=" + String(right.radiusFt, 2) +
             " ft (arc check " + String(right.arcRadiusFt, 2) + " ft, " +
             String(fabsf(right.sweepDeg), 0) + " deg).");
    }
  }

  if (left.valid && right.valid) {
    const StoredTurnRadii candidate = makeStoredTurnRadii(
        HARDWARE_TAG_HASH, left.radiusFt, right.radiusFt);
    if (!turnRadiusPersistence.save(candidate, HARDWARE_TAG_HASH)) {
      bleLog("[RADIUS FAIL] The measurements could not be saved.");
    } else {
      storedTurnRadii = candidate;
      hasMeasuredTurnRadii = true;
      applyTurnRadii(candidate.leftFt, candidate.rightFt);
      bleLog("[RADIUS PASS] left=" + String(turnRadiusLeftFt, 2) +
             " right=" + String(turnRadiusRightFt, 2) + " ft saved.");
    }
  }

  setSpray(false);
  stopDrive();
  radiusMeasurementActive = false;
}

// Port of diagnostics.ino, callable at runtime so wiring can be checked
// without reflashing. Throttle stays at neutral throughout.
void runSelfTest() {
  if (selfTestActive) {
    bleLog("!! Self test already active.");
    return;
  }
  selfTestActive = true;
  const uint32_t generation = currentSafetyAbortGeneration();
  if (!diagnosticCanContinue(generation)) {
    selfTestActive = false;
    return;
  }
  bleLog("=== SELF TEST ===");

  Wire.beginTransmission(PCA9685_ADDR);
  byte error = Wire.endTransmission();

  if (error != 0) {
    if (error == 2) {
      bleLog("[FAIL] I2C NACK - PCA9685 not found at 0x40.");
      bleLog("       Check VCC/GND on the PCA9685 logic header.");
    } else if (error == 5) {
      bleLog("[FAIL] I2C timeout / bus lockup.");
      bleLog("       Check SDA/SCL wires for breaks or shorts.");
    } else {
      bleLog("[FAIL] I2C error code " + String(error));
    }
    bleLog("=== SELF TEST ABORTED ===");
    setSpray(false);
    stopDrive();
    selfTestActive = false;
    return;
  }

  bleLog("[PASS] I2C OK (PCA9685 @ 0x40)");

  uint8_t mode1 = readPcaRegister(PCA_MODE1);
  uint8_t prescale = readPcaRegister(PCA_PRESCALE);
  bool asleep = (mode1 & PCA_SLEEP_BIT) != 0;
  bool wrongRate = abs((int)prescale - PCA_EXPECTED_PRESCALE) > 3;

  if (asleep || wrongRate) {
    bleLog("[FAIL] Chip answers but is not configured:");
    if (asleep)    bleLog("       SLEEP set -- outputs are off (brownout reset?)");
    if (wrongRate) bleLog("       prescale=" + String(prescale) + ", expected ~" +
                          String(PCA_EXPECTED_PRESCALE));
    bleLog("       Reinitialising now.");
    ensurePwmReady();
  } else {
    bleLog("[PASS] Configured: awake, prescale=" + String(prescale) + " (50Hz)");
  }
  if (pwmRecoveries) {
    bleLog("[INFO] Recovered from " + String(pwmRecoveries) +
           " brownout(s) since boot -- check servo power.");
  }

  setChannelPulse(ESC_CH, NEUTRAL_US);

  bleLog("[INFO] Steering LEFT (" + String(STEER_LEFT_US) + "us)...");
  setChannelPulse(STEER_CH, STEER_LEFT_US);
  if (!diagnosticWait(1000, generation)) {
    bleLog("=== SELF TEST STOPPED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }

  bleLog("[INFO] Steering RIGHT (" + String(STEER_RIGHT_US) + "us)...");
  setChannelPulse(STEER_CH, STEER_RIGHT_US);
  if (!diagnosticWait(1000, generation)) {
    bleLog("=== SELF TEST STOPPED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }

  // Straight-ahead is the fixed steering-map knot, not the middle of the
  // servo's travel and not inferred from the radius measurement. Look along
  // the rover here: the front wheels should be dead straight.
  bleLog("[INFO] Steering CENTER (" + String((int)steerCentreUs()) +
         "us, servo mid is " + String(STEER_CENTER_US) + ")...");
  setChannelPulse(STEER_CH, (int)steerCentreUs());
  if (!diagnosticWait(500, generation)) {
    bleLog("=== SELF TEST STOPPED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }

  bleLog("[INFO] Valve ON 1s...");
  setSpray(true);
  if (!diagnosticWait(1000, generation)) {
    bleLog("=== SELF TEST STOPPED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }
  setSpray(false);
  bleLog("[INFO] Valve OFF.");

  if (!runDriveTest(generation)) {
    bleLog("=== SELF TEST FAILED: MOTION NOT VERIFIED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }

  bleLog("If I2C PASSed but the servo never moved:");
  bleLog("  -> 5V/V+ or GND screw terminal loose, or CH0 plug reversed.");
  bleLog("=== SELF TEST COMPLETE ===");
  selfTestActive = false;
}

// Curvature is positive toward increasing heading (the rover's right). Pulse
// output comes only from the validated measured map; trim shifts the complete
// map slightly without changing its shape.
void steerCurvature(float curvaturePerFt) {
  if (steerSign < 0) curvaturePerFt = -curvaturePerFt;
  float us = pulseForCurvature(steeringMap, steeringMapCount, curvaturePerFt);
  if (!isfinite(us)) {
    steeringMapValid = false;
    us = STEER_CENTER_US;
  }
  if (us < STEER_RIGHT_US) us = STEER_RIGHT_US;
  if (us > STEER_LEFT_US) us = STEER_LEFT_US;
  lastSteerCommandUs = static_cast<int>(lroundf(us));
  setChannelPulse(STEER_CH, lastSteerCommandUs);
}

float curvatureForCommand(float rightward) {
  if (rightward > MAX_STEER_OFFSET) rightward = MAX_STEER_OFFSET;
  if (rightward < -MAX_STEER_OFFSET) rightward = -MAX_STEER_OFFSET;
  if (rightward >= 0.0f) {
    return (rightward / MAX_STEER_OFFSET) *
           steeringMap[steeringMapCount - 1].curvaturePerFt;
  }
  return (-rightward / MAX_STEER_OFFSET) * steeringMap[0].curvaturePerFt;
}

void planRoute() {
  if (route == nullptr) {
    routeCount = 0;
    routeStyle = ROUTE_NONE;
    routeRequirements = {};
    routePlanningFault = F_ROUTE;
    bleLog("!! No route memory was available at boot; this rover cannot plan a mission.");
    return;
  }
  const protocol_v2::RectangleV2 &rectangle = mission.rectangle();
  routeCount = buildRoute(
      fieldPassFt, fieldWidthFt, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION,
      turnRadiusLeftFt, turnRadiusRightFt, route, MAX_ROUTE_POINTS);
  routeRequirements = inspectRoute(route, routeCount, fieldPassFt);
  routeStyle = routeCount > 0 && !routeRequirements.truncated
      ? ROUTE_THREE_POINT : ROUTE_NONE;
  routePlanningFault = routeStyle == ROUTE_NONE ? F_ROUTE : F_NONE;

  // A resume asks to skip the passes already laid down. Refuse a pass the plan
  // does not contain rather than silently starting from the beginning, which
  // would re-spray ground the operator believes is finished.
  routeStartIndex = 0;
  if (routeCount > 0 && rectangle.startPassIndex > 0) {
    const int start = routePassStartIndex(route, routeCount,
                                          static_cast<int>(rectangle.startPassIndex));
    if (start < 0) {
      bleLog("!! Resume pass " + String(rectangle.startPassIndex + 1) + " is past the end of this plan (" +
             String(routePassCount(route, routeCount)) + " passes).");
      routeCount = 0;
      routeStyle = ROUTE_NONE;
      routePlanningFault = F_ROUTE;
    } else {
      routeStartIndex = start;
    }
  }
  routeIndex = routeStartIndex;
  lastRouteIndex = -1;
  targetDistanceValid = false;
  sprayInhibited = false;
  offPlanFt = 0.0f;
  worstOffThisPass = 0.0f;
  escReverse = false;
  directionRequested = false;
  driveDirection.stop();
  speedController.reset();
  lastSpeedControlMs = 0;

  // Where the first pass ends, so it can be given more latitude before spray
  // is cut. It is the pass everything else is lined up against.
  firstPassEnd = routeCount;
  for (int i = 0; i < routeCount; i++) {
    if (!route[i].spray) { firstPassEnd = i; break; }
  }

  int lanes = laneCount(fieldWidthFt, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  int reversals = 0;
  float minY = 0.0f, maxY = 0.0f, minX = 0.0f, maxX = 0.0f;
  for (int i = 0; i < routeCount; i++) {
    if (i > 0 && route[i].reverse != route[i - 1].reverse) reversals++;
    if (route[i].x < minX) minX = route[i].x;
    if (route[i].x > maxX) maxX = route[i].x;
    if (route[i].y < minY) minY = route[i].y;
    if (route[i].y > maxY) maxY = route[i].y;
  }

  bleLog("Planned " + String(routeCount) + " pts, " + String(lanes) +
         " lanes, " + String(reversals) + " direction changes (" +
         String(routeStyle == ROUTE_FORWARD_ONLY ? "forward-only" :
                routeStyle == ROUTE_THREE_POINT ? "three-point" : "none") + ").");
  bleLog("Turn radii L=" + String(turnRadiusLeftFt, 2) + " R=" +
         String(turnRadiusRightFt, 2) + " ft, straight-ahead at " +
         String((int)steerCentreUs()) + "us.");
  bleLog("[ROUTE] headland before=" + String(routeRequirements.beforeStartFt, 2) +
         " beyond=" + String(routeRequirements.beyondEndFt, 2) +
         " ft; complete rover footprint stays outside while turning.");

  // The app cannot compute where a resumed pass begins -- forward-only plans
  // visit lanes out of order -- so the rover states it, and the app holds the
  // operator to that spot before it will arm.
  if (routeCount > 0) {
    bleLog("[RESUME] pass=" + String(routePassIndexAt(route, routeCount, routeStartIndex) + 1) +
           "/" + String(routePassCount(route, routeCount)) +
           " x=" + String(route[routeStartIndex].x, 2) +
           " y=" + String(route[routeStartIndex].y, 2) +
           " hdg=" + String(segmentHeadingDeg(route, routeCount, routeStartIndex), 1));
  }

  if (routeRequirements.truncated) {
    bleLog("!! Route hit the point limit; area is too large to plan fully.");
  }
  sendRoutePlan();
}

// Whether the rover is standing where the plan starts. Only meaningful for a
// resumed mission: for a full rectangle the plan starts under the rover by
// construction, and the app has already checked it against the rectangle.
bool atRouteStart() {
  if (routeStartIndex <= 0 || routeStartIndex >= routeCount) return true;
  const float dx = route[routeStartIndex].x - robotX_ft;
  const float dy = route[routeStartIndex].y - robotY_ft;
  const float distance = sqrtf(dx * dx + dy * dy);
  const float headingError = fabsf(angleDiffDeg(
      segmentHeadingDeg(route, routeCount, routeStartIndex), robotHeading));
  const bool reached = distance <= RESUME_POSITION_TOLERANCE_FT &&
                       headingError <= RESUME_HEADING_TOLERANCE_DEG;
  if (!reached) {
    bleLog("!! Resume start is " + String(distance, 1) + " ft and " +
           String(headingError, 0) + " deg away. Move to x=" +
           String(route[routeStartIndex].x, 2) + " y=" +
           String(route[routeStartIndex].y, 2) + " hdg=" +
           String(segmentHeadingDeg(route, routeCount, routeStartIndex), 0) +
           " and arm again.");
  }
  return reached;
}

void updateSpray() {
  const RoutePoint &p = route[routeIndex];
  if (!p.spray) {
    sprayInhibited = false;
    setSpray(false);
    return;
  }

  float dx = p.x - robotX_ft, dy = p.y - robotY_ft;
  float off = sqrtf(dx * dx + dy * dy);
  float limit = (routeIndex < firstPassEnd) ? SPRAY_OFFPLAN_FIRST_FT : SPRAY_OFFPLAN_FT;
  const float passHeading = segmentHeadingDeg(route, routeCount, routeIndex);
  const float headingError = fabsf(angleDiffDeg(passHeading, robotHeading));
  const bool aligned = headingError <= SPRAY_HEADING_TOLERANCE_DEG;

  if (sprayInhibited) {
    if (off < limit - SPRAY_HYSTERESIS_FT && aligned) sprayInhibited = false;
  } else if (off > limit || !aligned) {
    sprayInhibited = true;
    bleLog("!! Pass entry not aligned (off=" + String(off, 1) +
           " ft heading=" + String(headingError, 0) + " deg) -- spray paused.");
  }

  setSpray(!sprayInhibited);
}

// Pure pursuit: aim at a point a fixed distance ahead on the plan. Drift moves
// that point off to one side, which steers the rover back onto the line.
void runFollow() {
  if (!controlAuthorized()) {
    setSpray(false);
    stopDrive();
    return;
  }
  if (routeCount < 2) {
    bleLog("!! No route planned.");
    enterFault(F_ROUTE);
    return;
  }

  routeIndex = advanceRouteIndex(route, routeCount, routeIndex,
                                 robotX_ft, robotY_ft,
                                 ROUTE_SEARCH_WINDOW, CUSP_TOL_FT);
  const float targetDx = route[routeIndex].x - robotX_ft;
  const float targetDy = route[routeIndex].y - robotY_ft;
  const float targetDistanceFt = sqrtf(targetDx * targetDx + targetDy * targetDy);

  // Nonprogress is a fault, never permission to skip untreated pavement. If
  // the pose reports essentially no motion it is a stall; if the rover is
  // moving but still cannot advance along the route, the route/direction state
  // is inconsistent.
  if (routeIndex == lastRouteIndex) {
    if (!targetDistanceValid || targetDistanceFt + 0.05f < bestTargetDistanceFt) {
      bestTargetDistanceFt = targetDistanceFt;
      targetDistanceValid = true;
      routeIndexSince = millis();
    }
    if (millis() - routeIndexSince > STALL_TIMEOUT_MS) {
      const bool notMoving = fabsf(lastPoseSpeedFps) < 0.10f;
      const bool movingAway = targetDistanceValid &&
                              targetDistanceFt > bestTargetDistanceFt + 0.25f;
      FaultCode fault = notMoving ? F_STALL : F_ROUTE;
      bleLog(fault == F_STALL
          ? "!! Route made no forward progress for four seconds."
          : (movingAway
              ? "!! Rover is moving away from its route target."
              : "!! Rover is moving without advancing along the route."));
      enterFault(fault);
      return;
    }
  } else {
    lastRouteIndex = routeIndex;
    routeIndexSince = millis();
    bestTargetDistanceFt = targetDistanceFt;
    targetDistanceValid = true;
  }

  if (routeIndex >= routeCount - 2) {
    setSpray(false);
    stopDrive();
    mission.complete();
    bleLog("=== MISSION COMPLETE ===");
    return;
  }

  bool reversing = route[routeIndex].reverse;
  if (!directionRequested || reversing != escReverse) {
    escReverse = reversing;
    directionRequested = true;
    // Course over ground describes the leg just finished, not the one about to
    // start, and it is only sampled driving forward -- so after a reverse leg
    // it still points the way the rover was going before the three-point turn,
    // up to a hundred degrees away from where the nose is now. Steering on that
    // for the first foot of every new leg is what sent the rover out of the
    // headland on full lock in the wrong direction. Dropping it falls back to
    // the reported heading, which is correct the moment the rover is standing
    // still, and course over ground re-earns itself over the next foot.
    resetCourse();
    speedController.reset();
    speedController.feedForwardUs = reversing
        ? reverseFeedForwardUs : forwardFeedForwardUs;
    const int initialPulse = NEUTRAL_US + static_cast<int>(
        reversing ? -speedController.feedForwardUs : speedController.feedForwardUs);
    driveDirection.begin(reversing, reversing, millis(), robotX_ft, robotY_ft,
                         robotHeading, initialPulse, NEUTRAL_US);
    lastSpeedControlMs = millis();
  }

  {
    float ox = route[routeIndex].x - robotX_ft;
    float oy = route[routeIndex].y - robotY_ft;
    offPlanFt = sqrtf(ox * ox + oy * oy);
    // Report the worst of each sprayed pass, then start the next one fresh, so
    // a bad pass is visible rather than averaged away.
    if (route[routeIndex].spray) {
      if (offPlanFt > worstOffThisPass) worstOffThisPass = offPlanFt;
    } else if (worstOffThisPass > 0.0f) {
      bleLog("Pass finished, worst " + String(worstOffThisPass, 2) + " ft off line.");
      worstOffThisPass = 0.0f;
    }
  }

  // Backing up, the rover travels in the direction opposite its nose. Course
  // over ground is preferred going forward, where it cancels out any error in
  // how the phone is mounted; reverse legs are short enough that the reported
  // heading is good enough.
  float reference;
  if (reversing) {
    reference = fmodf(robotHeading + 180.0f, 360.0f);
  } else {
    reference = courseValid ? courseDeg : robotHeading;
  }

  float err, command, requestedCurvature;

  if (route[routeIndex].turning) {
    // Through a turn there is no line to hold, only a curve to follow. Drive
    // the arc the plan already contains and use the lookahead point only to
    // correct for having drifted off it. Steering the whole maneuver from that
    // error instead means nothing asks the rover to turn until it has already
    // failed to, and the answer is then always full lock.
    int la = lookaheadWithinSegment(route, routeCount, routeIndex,
                                    robotX_ft, robotY_ft, LOOKAHEAD_TURN_FT);
    float want = bearingToWaypointDeg(route[la].x - robotX_ft,
                                      route[la].y - robotY_ft);
    err = angleDiffDeg(want, reference);
    const float targetDistance = sqrtf(
        (route[la].x - robotX_ft) * (route[la].x - robotX_ft) +
        (route[la].y - robotY_ft) * (route[la].y - robotY_ft));

    const float plannedKappa = plannedCurvature(route, routeCount, routeIndex);
    float correction = purePursuitCurvature(err, targetDistance,
                                            TURN_CORRECTION_DISTANCE_FT);
    const float correctionLimit = TURN_CORRECTION_LIMIT_FRACTION *
        maxCurvaturePerFt(correction >= 0.0f);
    if (correctionLimit > 0.0f) {
      if (correction > correctionLimit) correction = correctionLimit;
      if (correction < -correctionLimit) correction = -correctionLimit;
    }
    requestedCurvature = plannedKappa + correction;
    command = curvatureToCommand(requestedCurvature,
                                 turnRadiusLeftFt, turnRadiusRightFt,
                                 MAX_STEER_OFFSET);
    lastCrossTrackFt = offPlanFt;
  } else {
    // On a straight run -- which is every sprayed pass -- steer onto the line
    // itself rather than at a point ahead of it. Chasing a point ahead reaches
    // the line still carrying heading error and crosses it, and a pass is far
    // too short to settle that out: from a foot off, chasing takes 11 ft to
    // straighten and a pass is only 14. This converges in about 6 ft and never
    // crosses over.
    float lineHeading = segmentHeadingDeg(route, routeCount, routeIndex);
    float cross = crossTrackFt(route[routeIndex].x, route[routeIndex].y,
                               lineHeading, robotX_ft, robotY_ft);
    err = angleDiffDeg(lineHeading, reference);
    lastCrossTrackFt = cross;

    float kappa = lineFollowCurvature(cross, err, LINE_DISTANCE_CONST_FT);
    command = curvatureToCommand(kappa, turnRadiusLeftFt, turnRadiusRightFt,
                                 MAX_STEER_OFFSET);
    requestedCurvature = kappa;
  }
  lastHeadingErrorDeg = err;

  if (!controlAuthorized()) {
    setSpray(false);
    stopDrive();
    return;
  }

  // Steering acts on the direction of travel the opposite way in reverse:
  // right lock swings the nose left, so the same command turns the rover's
  // path the other way.
  const float steerKappa = reversing ? -requestedCurvature : requestedCurvature;
  steerCurvature(steerKappa);

  if (!reversing) {
    observeSteeringPolarity();
    polarityLastCommand = command;

  } else {
    polarityLastHeading = robotHeading;   // don't let a reverse leg pollute it
  }

  const unsigned long speedNow = millis();
  float speedDt = lastSpeedControlMs == 0
      ? 0.02f : (speedNow - lastSpeedControlMs) / 1000.0f;
  lastSpeedControlMs = speedNow;
  if (driveDirection.phase == D_COMMAND ||
      driveDirection.phase == D_VERIFY ||
      driveDirection.phase == D_READY) {
    // Arrive at the rectangle edge already at turn speed. Entering the first
    // arc at 1.5 ft/s consumed about a foot before the speed controller could
    // slow the rover and made the physical turn wider still.
    bool turnIsNear = route[routeIndex].turning;
    for (int ahead = 1; !turnIsNear && ahead <= 6 &&
         routeIndex + ahead < routeCount; ++ahead) {
      turnIsNear = route[routeIndex + ahead].turning;
    }
    const float targetMagnitude = turnIsNear
        ? DEFAULT_TURN_SPEED_FPS : DEFAULT_STRAIGHT_SPEED_FPS;
    const float targetSpeed = reversing ? -targetMagnitude : targetMagnitude;
    const float measuredSpeed = reversing
        ? -fabsf(lastPoseSpeedFps) : fabsf(lastPoseSpeedFps);
    const int offsetUs = speedController.update(targetSpeed, measuredSpeed, speedDt);
    driveDirection.commandPulseUs = NEUTRAL_US + offsetUs;
  }
  setChannelPulse(ESC_CH,
                  driveDirection.update(speedNow, robotX_ft, robotY_ft));

  if (driveDirection.wrongDirection) {
    bleLog("!! Measured motion sign disagrees with the ESC direction command.");
    enterFault(F_WRONG_DIRECTION);
    return;
  }
  if (driveDirection.noDisplacement || speedController.stalled) {
    bleLog("!! Throttle command produced no measured pavement motion.");
    enterFault(F_STALL);
    return;
  }

  if (!controlAuthorized()) {
    setSpray(false);
    stopDrive();
    return;
  }
  if (driveDirection.ready()) {
    updateSpray();
  } else {
    setSpray(false);
  }
}

static uint8_t acc[64];
static size_t  accLen = 0;

// Which gate refused the pose, and by how much. The numbers matter: a jump
// caused by 0.2 ft/s of standstill noise and a jump caused by a four-foot
// relocalisation snap need different answers.
void notePoseReject(const PoseRejectDetail &detail) {
  const uint8_t code = static_cast<uint8_t>(detail.fault);
  if (code <= F_HEADLAND && poseRejectCounts[code] < 0xffff) poseRejectCounts[code]++;
  if (millis() - lastPoseRejectLogMs < 500) return;
  lastPoseRejectLogMs = millis();
  String line = "[POSE REJ] code=" + String(code) +
                " n=" + String(code <= F_HEADLAND ? poseRejectCounts[code] : 0);
  if (detail.fault == F_POSE_JUMP) {
    line += " dt=" + String(detail.dtSeconds * 1000.0f, 1) + "ms" +
            " got=" + String(detail.measured, 3) +
            " max=" + String(detail.allowed, 3);
  }
  bleLog(line);
}

void feed(const uint8_t *d, size_t n, uint32_t receivedAtMs) {
  if (d == nullptr || n == 0) return;

  // Protocol-v2 writes are self-contained and CRC protected. Never append a
  // malformed v2 packet to the legacy byte stream, where its leading '!'
  // could otherwise be mistaken for an old pose or rectangle.
  if (n >= 3 && d[0] == protocol_v2::MAGIC && d[2] == protocol_v2::VERSION) {
    if (d[1] == 0x56 && n == protocol_v2::POSE_SIZE) {
      protocol_v2::PoseV2 pose = {};
      if (!protocol_v2::parsePoseV2(d, n, pose)) {
        invalidPacketCount++;
      } else if (!mission.acceptPose(pose, receivedAtMs)) {
        invalidPacketCount++;
        notePoseReject(mission.lastPoseReject());
        if (mission.state() == S_ARMED || mission.state() == S_RUNNING) {
          FaultCode fault = mission.lastPoseRejectFault();
          if (rejectedPoseRequiresFault(fault)) enterFault(fault);
        }
      }
      return;
    }

    if (d[1] == 0x4b && n == protocol_v2::CALIBRATION_SIZE) {
      protocol_v2::CalibrationV2 calibration = {};
      if (!protocol_v2::parseCalibrationV2(d, n, calibration)) {
        invalidPacketCount++;
        return;
      }
      protocol_v2::AckV2 ack = mission.acceptCalibration(calibration, receivedAtMs);
      if (ack.faultCode == F_NONE && !mission.lastSetupWasDuplicate()) {
        // A new epoch owns a new black box. Never relabel a frozen buffer from
        // an older mission with the new epoch, and keep rejection counters
        // scoped to the mission they describe.
        faultBuffer.reset();
        invalidPacketCount = 0;
        for (uint8_t code = 0; code <= F_HEADLAND; ++code) poseRejectCounts[code] = 0;
        lastPoseRejectLogMs = 0;
      }
      sendAck(ack);
      return;
    }

    if (d[1] == 0x44 && n == protocol_v2::RECTANGLE_SIZE) {
      protocol_v2::RectangleV2 rectangle = {};
      if (!protocol_v2::parseRectangleV2(d, n, rectangle)) {
        invalidPacketCount++;
        return;
      }

      protocol_v2::AckV2 ack = mission.acceptRectangle(rectangle, receivedAtMs);
      if (ack.faultCode == F_NONE && !mission.lastSetupWasDuplicate()) {
        fieldPassFt = rectangle.mFt;
        fieldWidthFt = rectangle.nFt;
        dryRunMode = (rectangle.flags & 0x02) != 0;
        setSpray(false);
        announceArea();
        planRoute();
        if (routeCount < 2 || routeStyle == ROUTE_NONE) {
          enterFault(routePlanningFault);
          ack = mission.overrideLastSetupWithFault(routePlanningFault);
        }
      } else if (ack.faultCode == F_NONE && mission.lastSetupWasDuplicate()) {
        sendRoutePlan();
      }
      sendAck(ack);
      return;
    }

    if (d[1] == 0x43 && n == protocol_v2::COMMAND_SIZE) {
      protocol_v2::CommandV2 command = {};
      if (!protocol_v2::parseCommandV2(d, n, command)) {
        invalidPacketCount++;
        return;
      }

      const bool radiusOpcode = command.opcode == 5;
      if (radiusOpcode && mission.state() == S_IDLE) {
        dryRunMode = true;  // radius measurement is explicit dry motion
        setSpray(false);
      }
      if (command.opcode == 1 || command.opcode == 2 || radiusOpcode) {
        const bool ready = ensurePwmReady();
        mission.setPwmReady(ready);
      }
      if (command.opcode == 1) mission.setStartPointReached(atRouteStart());

      protocol_v2::AckV2 ack = mission.acceptCommand(
          command, receivedAtMs, radiusOpcode && dryRunMode);
      const bool duplicate = mission.lastCommandWasDuplicate();

      if (radiusOpcode) {
        sendAck(ack);  // acknowledge the operator-confirmed measurement before it moves
        if (ack.faultCode == F_NONE && !duplicate) runTurnRadiusMeasurement();
        return;
      }

      if (command.opcode == 3) {
        setSpray(false);
        stopDrive();
        routeCount = 0;
        routeIndex = 0;
        resetCourse();
        bleLog(">>> Mission stopped.");
        announceTurnRadii();
      } else if (ack.faultCode == F_NONE && command.opcode == 2 && !duplicate) {
        if (routeCount < 2) {
          enterFault(F_ROUTE);
          ack.state = static_cast<uint8_t>(mission.state());
          ack.faultCode = F_ROUTE;
        } else {
          routeIndex = routeStartIndex;
          lastRouteIndex = -1;
          routeIndexSince = millis();
          resetCourse();
          bleLog(">>> Mission started.");
        }
      }

      if (command.opcode == 8 && ack.faultCode == F_NONE) {
        sendFaultBuffer();
        sendAck(ack);  // completion ACK follows the final retained sample
      } else {
        sendAck(ack);
      }

      if (command.opcode == 4 && ack.faultCode == F_NONE && !duplicate) {
        runSelfTest();
        setSpray(false);
        stopDrive();
      }
      return;
    }

    invalidPacketCount++;
    return;
  }

  if (n == 1) {
    if (d[0] == '1') {
      bleLog("!! Legacy Start refused; configure, Arm, and Start with protocol v2.");
    } else if (d[0] == '2') {
      protocol_v2::CommandV2 stop = {3, 0, 0};
      mission.acceptCommand(stop, receivedAtMs);
      stopDrive();
      setSpray(false);
      routeCount = 0;
      routeIndex = 0;
      bleLog(">>> Mission stopped.");
    } else if (d[0] == '3') {
      if (mission.allowsLegacyDiagnostics()) {
        bleLog("[DIAG] Legacy self-test is stationary only; hardware actuation requires v2.");
        bleLog("[DIAG] BLE and packet parser are responsive.");
      } else {
        bleLog("!! Self test refused: Stop first.");
      }
    } else if (d[0] == '4') {
      if (mission.allowsLegacyDiagnostics()) {
        dryRunMode = !dryRunMode;
        setSpray(false);
        bleLog(dryRunMode ? "[MODE] DRY" : "[MODE] WET");
    if (route == nullptr) {
      bleLog("!! No route memory was available at boot; this rover cannot run a mission.");
    }
      } else {
        bleLog("!! Mode change refused: Stop first.");
      }
    }
    return;
  }

  for (size_t i = 0; i < n; i++) {
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }

  size_t i = 0;
  while (accLen - i >= 11) {
    if (acc[i] != '!') { i++; continue; }

    float np, mp;  // N = pass length, M = width, in the app's input order
    if (parseAreaPacket(&acc[i], accLen - i, np, mp)) {
      if (mission.allowsLegacyDiagnostics()) packetCount++;
      i += 11;
      continue;
    }

    if (accLen - i < 15) { i++; continue; }
    float x, y, heading;
    if (parsePosePacket(&acc[i], accLen - i, x, y, heading)) {
      if (mission.allowsLegacyDiagnostics()) packetCount++;
      i += 15;
    } else {
      i++;
    }
  }
  if (i > 0) { memmove(acc, acc + i, accLen - i); accLen -= i; }
}

void consumePendingPose() {
  protocol_v2::PoseV2 pose = {};
  if (!mission.takePose(pose)) return;

  robotX_ft = pose.x;
  robotY_ft = pose.y;
  robotHeading = pose.heading;
  lastPoseSpeedFps = pose.speedFps;
  lastControlSequence = pose.sequence;
  lastPoseAgeMs = saturatedCounter(pose.ageMs);
  lastVioTime = mission.lastPoseReceivedAtMs();
  packetCount++;

  if (!mission.poseFresh(millis())) {
    if (mission.state() == S_ARMED || mission.state() == S_RUNNING) {
      enterFault(F_POSE_TIMEOUT);
    }
  } else if (mission.state() == S_RUNNING) {
    if (!escReverse) updateCourse();
    runFollow();
  }

  recordControlSample();
  sendTelemetry();
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
    portENTER_CRITICAL(&safetyEventMux);
    statusTelemetryRequested = true;
    portEXIT_CRITICAL(&safetyEventMux);
    bleLog("VIO app connected.");
    // Re-announce state so a freshly connected app isn't showing stale UI,
    // and because these were generated at boot with nobody listening.
    announceArea();
    announceTurnRadii();
    bleLog(dryRunMode ? "[MODE] DRY" : "[MODE] WET");
    bleLog(sprayActive ? "[SPRAY] ON" : "[SPRAY] OFF");
    if (hasBootFaultSummary) {
      bleLog("[FAULT] Previous boot: code=" +
             String(static_cast<int>(bootFaultSummary.fault)) +
             " epoch=" + String(bootFaultSummary.epoch) +
             " pt=" + String(bootFaultSummary.routeIndex) +
             " seq=" + String(bootFaultSummary.controlSequence));
    }
  }
  void onDisconnect(BLEServer *s) {
    bleConnected = false;
    portENTER_CRITICAL(&safetyEventMux);
    disconnectRequested = true;
    safetyAbortGeneration++;
    portEXIT_CRITICAL(&safetyEventMux);
    s->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Before anything else allocates: this is the one block that needs a large
  // contiguous run of DRAM, and the BLE stack fragments what is left.
  route = static_cast<RoutePoint *>(calloc(MAX_ROUTE_POINTS, sizeof(RoutePoint)));
  if (route == nullptr) {
    Serial.println("FATAL: could not allocate the route buffer; missions are unavailable.");
  }

  steeringMapValid = validSteeringMap(steeringMap, steeringMapCount);
  hasMeasuredTurnRadii = turnRadiusPersistence.load(storedTurnRadii,
                                                     HARDWARE_TAG_HASH);
  if (hasMeasuredTurnRadii) {
    applyTurnRadii(storedTurnRadii.leftFt, storedTurnRadii.rightFt);
  }

  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  setSpray(false);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(50);
  stopDrive();
  pwmMissionReady = pwmReadiness.observe(ensurePwmReady());
  mission.setPwmReady(pwmMissionReady);

  hasBootFaultSummary = faultSummary.load(bootFaultSummary);

  announceArea();

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

  // The only serial output that remains. Everything else goes to the app, but
  // if the board never reaches this line -- or the app cannot connect --
  // serial is the sole remaining way to tell a hung board from a BLE problem.
  Serial.println("VIO Waypoint Navigator ready. Logging to app from here.");
}

void loop() {
  pumpBle();

  // Catch a browned-out PWM chip within half a second, whether driving or not.
  if (millis() - lastPwmCheck >= 500) {
    lastPwmCheck = millis();
    bool ready = ensurePwmReady();
    pwmMissionReady = pwmReadiness.observe(ready);
    mission.setPwmReady(pwmMissionReady);
    if (!ready && pwmReadiness.consecutiveFailures() == 1) {
      bleLog("[WARN] PWM check missed; retrying before fault.");
    }
    if (!pwmMissionReady && (mission.state() == S_CONFIGURED ||
                             mission.state() == S_ARMED ||
                             mission.state() == S_RUNNING)) {
      enterFault(F_PWM);
    }
  }

  if (mission.state() == S_ARMED || mission.state() == S_RUNNING) {
    SafetyInput safety = {
      bleConnected,
      mission.poseFresh(millis()),
      true,
      false,
      pwmMissionReady,
      pwmMissionReady,
      driveDirection.noDisplacement || speedController.stalled,
      driveDirection.wrongDirection,
      true,
      routeCount >= 2 && routeStyle != ROUTE_NONE && !routeRequirements.truncated,
      steeringModelReady(),
      routeStyle != ROUTE_NONE,
    };
    FaultCode fault = evaluateSafety(mission.state(), safety);
    if (fault != F_NONE) enterFault(fault);
  }

  // 1Hz telemetry so a silent rover is diagnosable: distinguishes "no pose
  // packets arriving" from "packets fine, navigation misbehaving".
  if (millis() - lastTelemetryTime >= 1000) {
    lastTelemetryTime = millis();
    String phase = "IDLE";
    if (mission.state() == S_CONFIGURED) phase = "CONFIG";
    else if (mission.state() == S_ARMED) phase = "ARMED";
    else if (mission.state() == S_RUNNING) phase = escReverse ? "REV" : "RUN";
    else if (mission.state() == S_COMPLETE) phase = "DONE";
    else if (mission.state() == S_FAULT) phase = "FAULT";

    bleLog("[TLM] " + phase +
           " vio=" + String(mission.poseFresh(millis()) ? "OK" : "NONE") +
           " pkts=" + String(packetCount) +
           " drop=" + String(saturatedPacketDrops()) +
           " bad=" + String(invalidPacketCount) +
           " rej=" + String(poseRejectCounts[F_POSE_JUMP]) + "j/" +
           String(poseRejectCounts[F_POSE_INVALID]) + "i" +
           " age=" + String(static_cast<uint32_t>(lastPoseAgeMs) +
                            (millis() - lastVioTime)) + "ms" +
           " pos=(" + String(robotX_ft, 1) + "," + String(robotY_ft, 1) + ")" +
           " hdg=" + String(robotHeading, 0) +
           " pt=" + String(routeIndex) + "/" + String(routeCount) +
           " off=" + String(offPlanFt, 2) +
           " ctr=" + String((int)steerCentreUs()) +
           (dryRunMode ? " DRY" : " WET") +
           (sprayActive ? " spray=ON" : " spray=OFF") +
           (mission.fault() ? " fault=" + String(static_cast<int>(mission.fault())) : "") +
           (pwmRecoveries ? " pwmfix=" + String(pwmRecoveries) : ""));
  }
}
