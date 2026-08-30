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
 * Controls (from the app):
 *   '1' -> Start    '2' -> Stop    '3' -> Self test    '4' -> Wet/Dry
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <math.h>
#include "calibration.h"
#include "direction.h"
#include "fault_buffer.h"
#include "headland.h"
#include "mission_protocol.h"
#include "nav_math.h"
#include "protocol_v2.h"
#include "route.h"
#include "route_runtime.h"
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

// Safe dry-calibration search starting points only. Navigation closes the loop
// on measured speed; accepted pavement calibration replaces these values, and
// wet operation cannot arm without the current persisted fit.
const float DRY_FALLBACK_FWD_OFFSET_US = 120.0f;
const float DRY_FALLBACK_REV_OFFSET_US = 120.0f;

const int STEER_CENTER_US = 1500;
const int STEER_LEFT_US   = 2390;
const int STEER_RIGHT_US  = 700;

const float BAR_WIDTH_FT          = 17.0f / 12.0f;
const float LANE_OVERLAP_FRACTION = 0.15f;

// N x M, matching the app's two inputs and set at runtime via '!D'.
//   N (fieldPassFt)  -- length of the first pass, straight ahead from the
//                       start corner. Lanes run along this axis (+Y).
//   M (fieldWidthFt) -- total width covered by stepping lanes to the right (+X).
float fieldPassFt  = 21.91f;
float fieldWidthFt = 21.91f;

// The rover's two turning circles, measured 2026-08-26 with the sketch in
// turn_radius/. They are not the same size -- the steering trim sits
// off-centre -- and averaging them would ask for left turns tighter than the
// rover can drive and right turns wider than it needs, so both are carried
// separately all the way through the planner.
//
// Re-measure after any steering linkage or trim change: flash
// turn_radius/turn_radius.ino, press Start, and copy the two numbers here.
float turnRadiusLeftFt  = 4.33f;
float turnRadiusRightFt = 2.92f;

// The pulse at which the wheels actually point straight ahead. It is NOT the
// midpoint of the servo's travel, and assuming it was is what made every
// outbound pass sit off to one side and every return pass off to the other --
// which shows up as pairs of overlapping lines with gaps between them, not as
// obvious drift.
//
// Explicit measured curvature map. Straight is a direct calibration knot,
// not a midpoint inferred from the two steering endpoints. Runtime control
// interpolates this table without silently relearning its center.
SteeringKnot steeringMap[MAX_CALIBRATION_KNOTS] = {
  {STEER_LEFT_US, -1.0f / 4.33f},
  {1709, 0.0f},
  {STEER_RIGHT_US, 1.0f / 2.92f},
};
int steeringMapCount = 3;
bool steeringMapValid = false;
int  lastSteerCommandUs = 1500;

float steerCentreUs() {
  const float pulse = pulseForCurvature(steeringMap, steeringMapCount, 0.0f);
  return isfinite(pulse) ? pulse : static_cast<float>(STEER_CENTER_US);
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

class PreferencesCalibrationStore {
 public:
  bool read(uint8_t *out, size_t size) {
    Preferences prefs;
    if (!prefs.begin("ss-motion", true)) return false;
    const bool ok = prefs.getBytesLength("compact") == size &&
                    prefs.getBytes("compact", out, size) == size;
    prefs.end();
    return ok;
  }
  bool write(const uint8_t *data, size_t size) {
    Preferences prefs;
    if (!prefs.begin("ss-motion", false)) return false;
    const bool ok = prefs.putBytes("compact", data, size) == size;
    prefs.end();
    return ok;
  }
};

PreferencesCalibrationStore calibrationStore;
MotionCalibrationPersistence<PreferencesCalibrationStore> calibrationPersistence(calibrationStore);
CompactMotionCalibration storedMotionCalibration = {};
bool hasStoredMotionCalibration = false;
SteeringCalibrationSample steeringCalibrationSamples[6] = {};
int steeringCalibrationSampleCount = 0;
SpeedCalibrationSample forwardSpeedSamples[MAX_SPEED_CALIBRATION_SAMPLES] = {};
int forwardSpeedSampleCount = 0;
SpeedCalibrationSample reverseSpeedSamples[MAX_SPEED_CALIBRATION_SAMPLES] = {};
int reverseSpeedSampleCount = 0;
float pendingForwardFeedForwardUs = 0.0f;
float pendingReverseFeedForwardUs = 0.0f;
bool forwardSpeedFitReady = false;
bool reverseSpeedFitReady = false;
int forwardMinimumUsefulOffsetUs = 0;
int reverseMinimumUsefulOffsetUs = 0;
int forwardMinimumMovingOffsetUs = 0;
int reverseMinimumMovingOffsetUs = 0;
SteeringCalibrationFit pendingSteeringFit = {};
bool straightCalibrationValidated = false;
bool reverseCalibrationVerified = false;
bool calibrationActive = false;
const float CALIBRATION_SIGN_DISTANCE_FT = 0.05f;

const int MAX_ROUTE_POINTS = 6000;
RoutePoint route[MAX_ROUTE_POINTS];
int routeCount = 0;
int routeIndex = 0;
int firstPassEnd = 0;      // route index where the all-important first pass ends
RouteStyle routeStyle = ROUTE_NONE;
RouteRequirements routeRequirements = {};
FaultCode routePlanningFault = F_ROUTE;

// Steer at a point this far ahead on the path. On a straight pass a long
// lookahead tracks smoothly; through a turn it must be shorter than the arc's
// radius or the rover simply cuts the corner and misses the maneuver.
const float LOOKAHEAD_TURN_FT = 1.0f;
const float PURSUIT_GAIN      = 16.0f;   // us of steering per degree of error

// How sharply the rover converges onto a straight pass, in feet. Smaller is
// quicker but works the steering harder against position noise; 1.5 ft closes
// a foot of error in about 6 ft of travel without ever crossing the line.
const float LINE_DISTANCE_CONST_FT = 1.5f;
const int   ROUTE_SEARCH_WINDOW = ROUTE_PROGRESS_SEARCH_WINDOW;
const float CUSP_TOL_FT       = 0.5f;
const float ROUTE_TERMINAL_TOL_FT = 0.2f;

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

  if (mode1 == 0xFF && prescale == 0xFF) {
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
  pwmHealthy = false;
  return false;
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

void applyMotionCalibration(const CompactMotionCalibration &calibration) {
  if (!compactCalibrationValid(calibration)) return;
  steeringMapCount = calibration.knotCount;
  for (int index = 0; index < steeringMapCount; ++index) {
    steeringMap[index] = calibration.knots[index];
  }
  steeringMapValid = validSteeringMap(steeringMap, steeringMapCount);
  if (steeringMapValid) {
    turnRadiusLeftFt = -1.0f / steeringMap[0].curvaturePerFt;
    turnRadiusRightFt = 1.0f / steeringMap[steeringMapCount - 1].curvaturePerFt;
  }
  forwardFeedForwardUs = calibration.forwardFeedForwardUs;
  reverseFeedForwardUs = calibration.reverseFeedForwardUs;
}

bool motionCalibrationMatchesMission() {
  return hasStoredMotionCalibration &&
         calibrationIdentityMatches(storedMotionCalibration,
                                    mission.calibration().schemaVersion,
                                    mission.calibrationId(), HARDWARE_TAG_HASH);
}

bool motionCalibrationReady() {
  return steeringMapValid && (dryRunMode || motionCalibrationMatchesMission());
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
    if (shouldAck) {
      // STOP commands bypass the normal BLE queue so they remain fail-safe.
      // Preserve the protocol-v2 capability marker on that fast path; the
      // phone uses an epoch-zero STOP as its harmless compatibility probe.
      if (stopEpoch == 0 && ack.faultCode == F_NONE) {
        ack.calibrationId = protocol_v2::HARDENED_FIRMWARE_CAPABILITY_ID;
      }
      sendAck(ack);
    }
    bleLog(">>> Mission stopped.");
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

bool beginCalibrationMotion(bool reverse, int steeringPulseUs, int &pulseUs,
                            uint32_t generation, int &minimumUsefulOffsetUs,
                            int &minimumMovingOffsetUs);

bool measureDrive(bool reverse, uint32_t generation, float &alongFt) {
  if (!diagnosticCanContinue(generation)) return false;
  float h = robotHeading * (float)M_PI / 180.0f;
  float x0 = robotX_ft, y0 = robotY_ft;

  int pulseUs = NEUTRAL_US + static_cast<int>(
      reverse ? -reverseFeedForwardUs : forwardFeedForwardUs);
  int &minimumUsefulOffsetUs = reverse
      ? reverseMinimumUsefulOffsetUs : forwardMinimumUsefulOffsetUs;
  int &minimumMovingOffsetUs = reverse
      ? reverseMinimumMovingOffsetUs : forwardMinimumMovingOffsetUs;
  if (!beginCalibrationMotion(reverse, static_cast<int>(steerCentreUs()),
                              pulseUs, generation, minimumUsefulOffsetUs,
                              minimumMovingOffsetUs)) return false;

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
    bleLog("[FAIL] No fresh pose from the phone; motion cannot be verified.");
    return false;
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

void resetMotionCalibrationSession() {
  steeringCalibrationSampleCount = 0;
  forwardSpeedSampleCount = 0;
  reverseSpeedSampleCount = 0;
  pendingForwardFeedForwardUs = 0.0f;
  pendingReverseFeedForwardUs = 0.0f;
  forwardSpeedFitReady = false;
  reverseSpeedFitReady = false;
  forwardMinimumUsefulOffsetUs = 0;
  reverseMinimumUsefulOffsetUs = 0;
  forwardMinimumMovingOffsetUs = 0;
  reverseMinimumMovingOffsetUs = 0;
  pendingSteeringFit = {};
  straightCalibrationValidated = false;
  reverseCalibrationVerified = false;
  memset(steeringCalibrationSamples, 0, sizeof(steeringCalibrationSamples));
  memset(forwardSpeedSamples, 0, sizeof(forwardSpeedSamples));
  memset(reverseSpeedSamples, 0, sizeof(reverseSpeedSamples));
}

void resetSpeedCalibrationDirection(bool reverse) {
  if (reverse) {
    reverseSpeedSampleCount = 0;
    pendingReverseFeedForwardUs = 0.0f;
    reverseSpeedFitReady = false;
    reverseMinimumUsefulOffsetUs = 0;
    reverseMinimumMovingOffsetUs = 0;
    memset(reverseSpeedSamples, 0, sizeof(reverseSpeedSamples));
  } else {
    forwardSpeedSampleCount = 0;
    pendingForwardFeedForwardUs = 0.0f;
    forwardSpeedFitReady = false;
    forwardMinimumUsefulOffsetUs = 0;
    forwardMinimumMovingOffsetUs = 0;
    memset(forwardSpeedSamples, 0, sizeof(forwardSpeedSamples));
  }
}

bool calibrationCanContinue(uint32_t generation) {
  if (!diagnosticCanContinue(generation) || !dryRunMode ||
      !mission.poseFresh(millis()) || safetyEventPending() || !pwmHealthy) {
    return false;
  }
  // Calibration blocks loop(), so it must perform the same periodic PCA9685
  // health check itself. A recovered brownout still aborts this movement; the
  // reinitialised chip is left at neutral and requires an explicit retry.
  if (millis() - lastPwmCheck >= 500) {
    lastPwmCheck = millis();
    const unsigned long recoveriesBefore = pwmRecoveries;
    const bool ready = ensurePwmReady();
    if (!ready || pwmRecoveries != recoveriesBefore) {
      bleLog("[CAL FAIL] PWM controller became unavailable or reset during motion.");
      return false;
    }
  }
  return true;
}

int forwardCalibrationPulseUs() {
  int offsetUs = static_cast<int>(lroundf(forwardSpeedFitReady
      ? pendingForwardFeedForwardUs : forwardFeedForwardUs));
  if (forwardMinimumMovingOffsetUs > offsetUs) {
    offsetUs = forwardMinimumMovingOffsetUs;
  }
  if (forwardMinimumUsefulOffsetUs >= offsetUs) {
    offsetUs = forwardMinimumUsefulOffsetUs + 10;
  }
  if (offsetUs < MIN_SPEED_CALIBRATION_OFFSET_US) {
    offsetUs = MIN_SPEED_CALIBRATION_OFFSET_US;
  }
  if (offsetUs > MAX_SPEED_CALIBRATION_OFFSET_US) {
    offsetUs = MAX_SPEED_CALIBRATION_OFFSET_US;
  }
  return NEUTRAL_US + offsetUs;
}

// Safely search for the loaded rover's breakaway pulse while retaining the
// exact DirectionState neutral/brake/verify sequence used by navigation. Each
// attempt is independently bounded and any wrong-direction observation aborts
// immediately. Only a no-displacement result permits the next higher pulse.
bool beginCalibrationMotion(bool reverse, int steeringPulseUs, int &pulseUs,
                            uint32_t generation, int &minimumUsefulOffsetUs,
                            int &minimumMovingOffsetUs) {
  if (!calibrationCanContinue(generation)) return false;
  const int direction = reverse ? -1 : +1;
  const int requestedOffset = pulseUs - NEUTRAL_US;
  if (requestedOffset * direction <= 0) return false;
  int offsetUs = abs(requestedOffset);
  if (offsetUs < MIN_SPEED_CALIBRATION_OFFSET_US) {
    offsetUs = MIN_SPEED_CALIBRATION_OFFSET_US;
  }
  if (offsetUs > MAX_SPEED_CALIBRATION_OFFSET_US) return false;

  while (calibrationCanContinue(generation)) {
    const int commandPulseUs = NEUTRAL_US + direction * offsetUs;
    setSpray(false);
    setChannelPulse(STEER_CH, steeringPulseUs);
    driveDirection.begin(reverse, reverse, millis(), robotX_ft, robotY_ft,
                         robotHeading, commandPulseUs, NEUTRAL_US);
    while (!driveDirection.ready() && !driveDirection.failed()) {
      pumpBle();
      if (!calibrationCanContinue(generation)) {
        stopDrive();
        return false;
      }
      setChannelPulse(STEER_CH, steeringPulseUs);
      setChannelPulse(ESC_CH,
                      driveDirection.update(millis(), robotX_ft, robotY_ft));
      delay(10);
    }

    if (driveDirection.ready()) {
      pulseUs = commandPulseUs;
      if (minimumMovingOffsetUs == 0 || offsetUs < minimumMovingOffsetUs) {
        minimumMovingOffsetUs = offsetUs;
      }
      return true;
    }

    const float headingRadians = driveDirection.headingDeg * (float)M_PI / 180.0f;
    const float observedAlong =
        (robotX_ft - driveDirection.verifyX) * sinf(headingRadians) +
        (robotY_ft - driveDirection.verifyY) * cosf(headingRadians);
    const bool observedWrongDirection =
        fabsf(observedAlong) >= CALIBRATION_SIGN_DISTANCE_FT &&
        observedAlong * direction < 0.0f;
    const bool wrongDirection = driveDirection.wrongDirection ||
                                observedWrongDirection;
    const bool noDisplacement = driveDirection.noDisplacement;
    stopDrive();
    if (wrongDirection) {
      bleLog("[CAL FAIL] Motion sign disagreed with the throttle direction.");
      return false;
    }
    if (!noDisplacement) return false;
    if (offsetUs > minimumUsefulOffsetUs) minimumUsefulOffsetUs = offsetUs;
    if (offsetUs >= MAX_SPEED_CALIBRATION_OFFSET_US) {
      bleLog("[CAL FAIL] No movement at the 350us throttle safety limit; check load, battery, ESC, and drivetrain.");
      return false;
    }

    int nextOffsetUs;
    if (minimumMovingOffsetUs > offsetUs + 10) {
      nextOffsetUs = offsetUs + (minimumMovingOffsetUs - offsetUs) / 2;
    } else {
      nextOffsetUs = offsetUs + 80;
    }
    if (nextOffsetUs <= offsetUs) nextOffsetUs = offsetUs + 10;
    if (nextOffsetUs > MAX_SPEED_CALIBRATION_OFFSET_US) {
      nextOffsetUs = MAX_SPEED_CALIBRATION_OFFSET_US;
    }
    bleLog("[CAL SEARCH] No movement at " + String(offsetUs) +
           "us offset; retrying at " + String(nextOffsetUs) +
           "us (350us hard limit).");
    offsetUs = nextOffsetUs;
  }
  stopDrive();
  return false;
}

bool measureCalibrationTravel(bool reverse, int &pulseUs, int steeringPulseUs,
                              float targetDistanceFt, uint32_t generation,
                              int &minimumUsefulOffsetUs,
                              int &minimumMovingOffsetUs,
                              float &alongFt, float &elapsedSeconds,
                              float &headingDeltaDeg) {
  if (!calibrationCanContinue(generation)) return false;
  alongFt = 0.0f;
  elapsedSeconds = 0.0f;
  headingDeltaDeg = 0.0f;
  if (!beginCalibrationMotion(reverse, steeringPulseUs, pulseUs, generation,
                              minimumUsefulOffsetUs,
                              minimumMovingOffsetUs)) return false;

  const float x0 = robotX_ft, y0 = robotY_ft, heading0 = robotHeading;
  const float radians = heading0 * (float)M_PI / 180.0f;
  const unsigned long startedAt = millis();
  alongFt = 0.0f;
  while (fabsf(alongFt) < targetDistanceFt && millis() - startedAt < 10000) {
    pumpBle();
    if (!calibrationCanContinue(generation)) {
      stopDrive();
      return false;
    }
    setChannelPulse(STEER_CH, steeringPulseUs);
    setChannelPulse(ESC_CH,
                    driveDirection.update(millis(), robotX_ft, robotY_ft));
    if (fabsf(lastPoseSpeedFps) > MAX_SPEED_CALIBRATION_FPS) {
      bleLog("[CAL FAIL] Calibration speed exceeded the 2.5 ft/s safety limit.");
      stopDrive();
      return false;
    }
    const float dx = robotX_ft - x0, dy = robotY_ft - y0;
    alongFt = dx * sinf(radians) + dy * cosf(radians);
    delay(10);
  }
  elapsedSeconds = (millis() - startedAt) / 1000.0f;
  headingDeltaDeg = angleDiffDeg(robotHeading, heading0);
  const bool signMatches = reverse ? alongFt <= -targetDistanceFt : alongFt >= targetDistanceFt;
  stopDrive();
  if (!signMatches && elapsedSeconds >= 9.9f) {
    const int offsetUs = abs(pulseUs - NEUTRAL_US);
    if (offsetUs > minimumUsefulOffsetUs) minimumUsefulOffsetUs = offsetUs;
  }
  return signMatches && elapsedSeconds > 0.0f;
}

bool measureCalibrationArc(int pulseUs, uint32_t generation,
                           SteeringCalibrationSample &sample) {
  if (!calibrationCanContinue(generation)) return false;
  int throttleUs = forwardCalibrationPulseUs();
  if (!beginCalibrationMotion(false, pulseUs, throttleUs, generation,
                              forwardMinimumUsefulOffsetUs,
                              forwardMinimumMovingOffsetUs)) return false;

  const float heading0 = robotHeading;
  float previousX = robotX_ft, previousY = robotY_ft;
  float distanceFt = 0.0f;
  float sweptDeg = 0.0f;
  const unsigned long startedAt = millis();
  while (fabsf(sweptDeg) < MIN_CALIBRATION_SWEEP_DEG &&
         millis() - startedAt < 12000) {
    pumpBle();
    if (!calibrationCanContinue(generation)) {
      stopDrive();
      return false;
    }
    setChannelPulse(STEER_CH, pulseUs);
    setChannelPulse(ESC_CH,
                    driveDirection.update(millis(), robotX_ft, robotY_ft));
    if (fabsf(lastPoseSpeedFps) > MAX_SPEED_CALIBRATION_FPS) {
      bleLog("[CAL FAIL] Calibration speed exceeded the 2.5 ft/s safety limit.");
      stopDrive();
      return false;
    }
    const float dx = robotX_ft - previousX, dy = robotY_ft - previousY;
    const float stepFt = sqrtf(dx * dx + dy * dy);
    if (stepFt < 1.0f) distanceFt += stepFt;
    previousX = robotX_ft;
    previousY = robotY_ft;
    sweptDeg = angleDiffDeg(robotHeading, heading0);
    delay(10);
  }
  stopDrive();
  if (fabsf(sweptDeg) < MIN_CALIBRATION_SWEEP_DEG || distanceFt < 3.0f) {
    if (distanceFt < 3.0f) {
      const int offsetUs = abs(throttleUs - NEUTRAL_US);
      if (offsetUs > forwardMinimumUsefulOffsetUs) {
        forwardMinimumUsefulOffsetUs = offsetUs;
      }
    }
    return false;
  }
  sample = {pulseUs,
            sweptDeg * (float)M_PI / 180.0f / distanceFt,
            fabsf(sweptDeg),
            +1};
  return true;
}

void tryCommitMotionCalibration() {
  if (!pendingSteeringFit.valid || !forwardSpeedFitReady ||
      !reverseSpeedFitReady || !reverseCalibrationVerified) return;
  const CompactMotionCalibration candidate = makeCompactCalibration(
      mission.calibration().schemaVersion, mission.calibrationId(),
      HARDWARE_TAG_HASH, pendingSteeringFit, pendingForwardFeedForwardUs,
      pendingReverseFeedForwardUs, true);
  if (!calibrationPersistence.save(candidate)) {
    bleLog("[CAL FAIL] Could not persist compact motion calibration.");
    return;
  }
  storedMotionCalibration = candidate;
  hasStoredMotionCalibration = true;
  applyMotionCalibration(storedMotionCalibration);
  bleLog("[CAL PASS] Motion calibration saved for ID " +
         String(storedMotionCalibration.calibrationId) + ".");
}

bool runSteeringCalibrationStep(uint32_t generation) {
  if (!straightCalibrationValidated) {
    float along = 0.0f, seconds = 0.0f, headingDelta = 0.0f;
    int pulse = forwardCalibrationPulseUs();
    bleLog("[CAL STEER] Straight validation: keep 8 ft of pavement clear ahead.");
    if (!measureCalibrationTravel(false, pulse, static_cast<int>(steerCentreUs()),
                                  6.0f, generation,
                                  forwardMinimumUsefulOffsetUs,
                                  forwardMinimumMovingOffsetUs,
                                  along, seconds, headingDelta)) {
      bleLog("[CAL FAIL] Straight validation did not complete 6 ft safely.");
      return false;
    }
    if (fabsf(headingDelta) > 2.0f) {
      bleLog("[CAL FAIL] Direct straight pulse bent more than 2 degrees over 6 ft.");
      return false;
    }
    straightCalibrationValidated = true;
    bleLog("[CAL PASS] Direct straight pulse verified at " +
           String(static_cast<int>(steerCentreUs())) + "us.");
    return true;
  }

  static const int PULSES[6] = {2300, 2300, 2300, 800, 800, 800};
  if (steeringCalibrationSampleCount >= 6) {
    bleLog("[CAL STEER] Steering samples already complete.");
    return true;
  }
  const int pulse = PULSES[steeringCalibrationSampleCount];
  bleLog("[CAL STEER] Arc " + String(steeringCalibrationSampleCount + 1) +
         "/6 at " + String(pulse) + "us; keep the sweep area clear.");
  SteeringCalibrationSample sample = {};
  if (!measureCalibrationArc(pulse, generation, sample)) {
    bleLog("[CAL FAIL] Arc did not produce a valid 60 degree sweep.");
    return false;
  }
  steeringCalibrationSamples[steeringCalibrationSampleCount++] = sample;
  bleLog("[CAL SAMPLE] steer=" + String(pulse) +
         " curvature=" + String(sample.curvaturePerFt, 4) + " 1/ft.");
  if (steeringCalibrationSampleCount == 6) {
    if (!fitSteeringCalibration(steeringCalibrationSamples, 6,
                                static_cast<int>(steerCentreUs()), pendingSteeringFit)) {
      bleLog("[CAL FAIL] Steering samples do not form a monotonic two-sided map.");
      return false;
    }
    bleLog("[CAL PASS] Steering map fit is complete.");
    tryCommitMotionCalibration();
  }
  return true;
}

bool runSpeedCalibrationStep(uint32_t generation) {
  const bool reverse = forwardSpeedFitReady;
  int &sampleCount = reverse ? reverseSpeedSampleCount : forwardSpeedSampleCount;
  SpeedCalibrationSample *samples = reverse ? reverseSpeedSamples : forwardSpeedSamples;
  bool &fitReady = reverse ? reverseSpeedFitReady : forwardSpeedFitReady;
  float &pendingFit = reverse
      ? pendingReverseFeedForwardUs : pendingForwardFeedForwardUs;
  int &minimumUsefulOffsetUs = reverse
      ? reverseMinimumUsefulOffsetUs : forwardMinimumUsefulOffsetUs;
  int &minimumMovingOffsetUs = reverse
      ? reverseMinimumMovingOffsetUs : forwardMinimumMovingOffsetUs;
  if (fitReady) {
    bleLog("[CAL SPEED] Forward and reverse target-speed fits are complete.");
    return true;
  }

  if (sampleCount >= MAX_SPEED_CALIBRATION_SAMPLES) {
    resetSpeedCalibrationDirection(reverse);
    bleLog("[CAL FAIL] Eight speed samples could not produce a monotonic 1.0 ft/s bracket. This direction was cleared; check traction/pose quality and retry from sample 1.");
    return false;
  }
  int nextOffsetUs = 0;
  if (!nextSpeedCalibrationOffset(samples, sampleCount,
                                  reverse ? -1 : +1, NEUTRAL_US,
                                  minimumUsefulOffsetUs, nextOffsetUs)) {
    resetSpeedCalibrationDirection(reverse);
    bleLog("[CAL FAIL] No untested throttle remained inside the 350us limit. This direction was cleared; correct the load/drive issue before retrying.");
    return false;
  }
  int pulse = NEUTRAL_US + (reverse ? -nextOffsetUs : nextOffsetUs);
  bleLog("[CAL SPEED] " + String(reverse ? "reverse " : "forward ") +
         "sample " + String(sampleCount + 1) + " at " +
         String(nextOffsetUs) + "us offset; keep 5 ft clear.");
  float along = 0.0f, seconds = 0.0f, headingDelta = 0.0f;
  if (!measureCalibrationTravel(reverse, pulse, static_cast<int>(steerCentreUs()),
                                3.0f, generation,
                                minimumUsefulOffsetUs, minimumMovingOffsetUs,
                                along, seconds, headingDelta)) {
    bleLog("[CAL FAIL] Speed run did not reach 3 ft safely; reposition, check the load/drive system, and retry.");
    return false;
  }
  if (fabsf(headingDelta) > 2.0f) {
    bleLog("[CAL FAIL] Speed run curved more than 2 degrees; fix straight trim before fitting throttle.");
    return false;
  }
  SpeedCalibrationSample sample = {
    pulse,
    along / seconds,
    fabsf(along),
    static_cast<int8_t>(reverse ? -1 : +1),
  };
  samples[sampleCount++] = sample;
  bleLog("[CAL SAMPLE] throttle=" + String(pulse) +
         " speed=" + String(sample.speedFps, 2) + " ft/s.");

  float fittedUs = 0.0f;
  if (fitSpeedFeedForward(samples, sampleCount, reverse ? -1 : +1,
                          NEUTRAL_US, fittedUs)) {
    pendingFit = fittedUs;
    fitReady = true;
    if (reverse) reverseFeedForwardUs = fittedUs;
    else forwardFeedForwardUs = fittedUs;
    bleLog("[CAL PASS] " + String(reverse ? "Reverse" : "Forward") +
           " 1.0 ft/s feed-forward=" + String(fittedUs, 1) + "us.");
  } else if (sampleCount >= MAX_SPEED_CALIBRATION_SAMPLES) {
    resetSpeedCalibrationDirection(reverse);
    bleLog("[CAL FAIL] Eight samples did not yield a valid target-speed curve. This direction was cleared; check traction/pose quality before retrying.");
    return false;
  } else if (sampleCount >= 3) {
    bleLog("[CAL SPEED] Samples do not yet bracket 1.0 ft/s monotonically; one additional bounded sample is required.");
  }
  tryCommitMotionCalibration();
  return true;
}

bool runReverseCalibrationStep(uint32_t generation) {
  if (!forwardSpeedFitReady || !reverseSpeedFitReady) {
    bleLog("[CAL FAIL] Complete both adaptive speed fits before reverse verification.");
    return false;
  }
  reverseCalibrationVerified = runDriveTest(generation);
  if (!reverseCalibrationVerified) {
    bleLog("[CAL FAIL] Reverse direction was not verified.");
    return false;
  }
  bleLog("[CAL PASS] Reverse direction sequence verified.");
  tryCommitMotionCalibration();
  return true;
}

void runCalibrationCommand(uint8_t opcode) {
  if (calibrationActive || selfTestActive) {
    bleLog("[CAL STEP FAIL] Another motion diagnostic is already active.");
    return;
  }
  calibrationActive = true;
  const uint32_t generation = currentSafetyAbortGeneration();
  bool ok = false;
  if (opcode == 5) ok = runSteeringCalibrationStep(generation);
  else if (opcode == 6) ok = runSpeedCalibrationStep(generation);
  else if (opcode == 7) ok = runReverseCalibrationStep(generation);
  setSpray(false);
  stopDrive();
  calibrationActive = false;
  bleLog(ok
      ? "[CAL STEP PASS] Motion step finished and outputs are safe."
      : "[CAL STEP FAIL] Motion step stopped; correct the logged cause and explicitly retry.");
}

// Port of diagnostics.ino, callable at runtime so wiring can be checked
// without reflashing. Drive motion is pose-verified; liquid outputs stay off.
void runSelfTest() {
  if (selfTestActive || calibrationActive) {
    bleLog("!! Self test refused while another motion diagnostic is active.");
    return;
  }
  selfTestActive = true;
  // Self-test contains forward/reverse motion. Force the same dry physical
  // output invariant as calibration before touching any actuator.
  dryRunMode = true;
  setSpray(false);
  digitalWrite(VALVE_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);
  const uint32_t generation = currentSafetyAbortGeneration();
  if (!diagnosticCanContinue(generation)) {
    stopDrive();
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
    if (!ensurePwmReady()) {
      bleLog("=== SELF TEST ABORTED: PWM CONTROLLER NOT READY ===");
      setSpray(false);
      stopDrive();
      selfTestActive = false;
      return;
    }
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

  // Centre is where the wheels really point straight, computed from the two
  // measured circles -- not the middle of the servo's travel. Look along the
  // rover here: the front wheels should be dead straight. If they are not,
  // re-measure the turning radii, because everything else is built on them.
  bleLog("[INFO] Steering CENTER (" + String((int)steerCentreUs()) +
         "us, servo mid is " + String(STEER_CENTER_US) + ")...");
  setChannelPulse(STEER_CH, (int)steerCentreUs());
  if (!diagnosticWait(500, generation)) {
    bleLog("=== SELF TEST STOPPED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }

  bleLog("[INFO] Dry spray-output check: valve and pump held OFF for 1s...");
  digitalWrite(VALVE_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);
  if (!diagnosticWait(1000, generation)) {
    bleLog("=== SELF TEST STOPPED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }
  bleLog("[PASS] Valve and pump remained inhibited.");

  if (!runDriveTest(generation)) {
    bleLog("=== SELF TEST FAILED: MOTION NOT VERIFIED ===");
    setSpray(false); stopDrive(); selfTestActive = false; return;
  }

  bleLog("If I2C PASSed but the servo never moved:");
  bleLog("  -> 5V/V+ or GND screw terminal loose, or CH0 plug reversed.");
  bleLog("=== SELF TEST COMPLETE ===");
  setSpray(false);
  stopDrive();
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
  const protocol_v2::RectangleV2 &rectangle = mission.rectangle();
  const bool preferForwardOnly = (rectangle.flags & 0x04) != 0;
  const RouteSelection selection = selectRoute(
      fieldPassFt, fieldWidthFt, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION,
      turnRadiusLeftFt, turnRadiusRightFt,
      rectangle.startClearFt, rectangle.endClearFt,
      preferForwardOnly, route, MAX_ROUTE_POINTS);
  routeCount = selection.count;
  routeStyle = selection.style;
  routeRequirements = selection.requirements;
  routePlanningFault = routeStyle == ROUTE_NONE
      ? (routeRequirements.truncated ? F_ROUTE : F_HEADLAND)
      : F_NONE;
  routeIndex = 0;
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
  bool firstSprayedSpanStarted = false;
  for (int i = 0; i < routeCount; i++) {
    if (route[i].spray) {
      firstSprayedSpanStarted = true;
    } else if (firstSprayedSpanStarted) {
      firstPassEnd = i;
      break;
    }
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
  bleLog("Needs clear pavement " + String(routeRequirements.beyondEndFt, 1) +
         " ft past the far end, " + String(routeRequirements.beforeStartFt, 1) +
         " ft behind the start.");

  if (routeStyle == ROUTE_NONE && routePlanningFault == F_HEADLAND) {
    bleLog("!! Neither forward-only nor three-point turns fit the entered clear pavement.");
  }

  if (routeRequirements.truncated) {
    bleLog("!! Route hit the point limit; area is too large to plan fully.");
  }
}

bool plannedRouteReadyForStart() {
  return routeStyle != ROUTE_NONE && !routeRequirements.truncated &&
         routeReadyForStart(route, routeCount);
}

FaultCode plannedRouteFailure() {
  return routePlanningFault == F_NONE ? F_ROUTE : routePlanningFault;
}

ApplicatorPoint calibratedApplicatorPoint() {
  const protocol_v2::CalibrationV2 &calibration = mission.calibration();
  return applicatorPoint(robotX_ft, robotY_ft, robotHeading,
                         calibration.sprayForwardFt,
                         calibration.sprayRightFt);
}

void updateSpray() {
  const RoutePoint &p = route[routeIndex];
  if (!p.spray) {
    sprayInhibited = false;
    setSpray(false);
    return;
  }

  // Route tracking remains tied to the rover pose, but physical application
  // is bounded by the calibrated location of the bar/nozzles.
  const ApplicatorPoint applicator = calibratedApplicatorPoint();
  if (!applicatorWithinPassY(robotY_ft, fieldPassFt) ||
      !applicatorWithinPassY(applicator.y, fieldPassFt)) {
    setSpray(false);
    return;
  }

  float dx = p.x - robotX_ft, dy = p.y - robotY_ft;
  float off = sqrtf(dx * dx + dy * dy);
  float limit = (routeIndex < firstPassEnd) ? SPRAY_OFFPLAN_FIRST_FT : SPRAY_OFFPLAN_FT;

  if (sprayInhibited) {
    if (off < limit - SPRAY_HYSTERESIS_FT) sprayInhibited = false;
  } else if (off > limit) {
    sprayInhibited = true;
    bleLog("!! " + String(off, 1) + " ft off plan -- spray paused.");
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
  if (!plannedRouteReadyForStart()) {
    bleLog("!! No route planned.");
    enterFault(plannedRouteFailure());
    return;
  }

  routeIndex = advanceRouteIndex(route, routeCount, routeIndex,
                                 robotX_ft, robotY_ft,
                                 ROUTE_SEARCH_WINDOW, CUSP_TOL_FT);
  const float targetDx = route[routeIndex].x - robotX_ft;
  const float targetDy = route[routeIndex].y - robotY_ft;
  const float targetDistanceFt = sqrtf(targetDx * targetDx + targetDy * targetDy);

  const ApplicatorPoint applicator = calibratedApplicatorPoint();
  if (routeTerminalReachedOrPassed(route, routeCount, routeIndex, applicator,
                                   ROUTE_TERMINAL_TOL_FT)) {
    setSpray(false);
    stopDrive();
    mission.complete();
    bleLog("=== MISSION COMPLETE ===");
    return;
  }

  const bool reversing = route[routeIndex].reverse;
  const bool directionChanged =
      driveDirectionChanged(directionRequested, escReverse, reversing);
  if (!directionRequested || directionChanged) {
    if (directionChanged) resetCourse();
    escReverse = reversing;
    directionRequested = true;
    speedController.reset();
    speedController.feedForwardUs = reversing
        ? reverseFeedForwardUs : forwardFeedForwardUs;
    const int initialPulse = NEUTRAL_US + static_cast<int>(
        reversing ? -speedController.feedForwardUs : speedController.feedForwardUs);
    driveDirection.begin(reversing, reversing, millis(), robotX_ft, robotY_ft,
                         robotHeading, initialPulse, NEUTRAL_US);
    lastSpeedControlMs = millis();
  }

  // Nonprogress is a fault, never permission to skip untreated pavement. If
  // the pose reports essentially no motion it is a stall; if the rover is
  // moving but still cannot advance along the route, the route/direction state
  // is inconsistent. Neutral/brake/verification phases deliberately make no
  // route progress, so their time cannot count against the dwell watchdog.
  if (!driveDirection.ready()) {
    lastRouteIndex = routeIndex;
    routeIndexSince = millis();
    bestTargetDistanceFt = targetDistanceFt;
    targetDistanceValid = false;
  } else if (routeIndex == lastRouteIndex) {
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
    // Through a turn there is no line to hold, only a curve to follow, so aim
    // at a point a short way along it.
    int la = lookaheadWithinSegment(route, routeCount, routeIndex,
                                    robotX_ft, robotY_ft, LOOKAHEAD_TURN_FT);
    float want = bearingToWaypointDeg(route[la].x - robotX_ft,
                                      route[la].y - robotY_ft);
    err = angleDiffDeg(want, reference);
    command = err * PURSUIT_GAIN;
    requestedCurvature = curvatureForCommand(command);
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
  steerCurvature(reversing ? -requestedCurvature : requestedCurvature);

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
    const float targetMagnitude = route[routeIndex].turning
        ? DEFAULT_TURN_SPEED_FPS : DEFAULT_STRAIGHT_SPEED_FPS;
    const float targetSpeed = reversing ? -targetMagnitude : targetMagnitude;
    const float measuredSpeed = lastPoseSpeedFps;
    const int offsetUs = speedController.update(targetSpeed, measuredSpeed, speedDt);
    if (speedController.configurationInvalid) {
      bleLog("!! Stored speed feed-forward is outside the safe throttle envelope.");
      enterFault(F_CALIBRATION);
      return;
    }
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
        if (mission.state() == S_ARMED || mission.state() == S_RUNNING) {
          enterFault(F_POSE_INVALID);
        }
      } else if (!mission.acceptPose(pose, receivedAtMs)) {
        invalidPacketCount++;
        if (mission.state() == S_ARMED || mission.state() == S_RUNNING) {
          FaultCode fault = mission.lastPoseRejectFault();
          enterFault(fault == F_NONE ? F_POSE_INVALID : fault);
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
        resetMotionCalibrationSession();
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
        if (!plannedRouteReadyForStart()) {
          const FaultCode fault = plannedRouteFailure();
          enterFault(fault);
          ack = mission.overrideLastSetupWithFault(fault);
        } else if (!motionCalibrationReady()) {
          bleLog("!! Wet operation requires a stored motion calibration matching this mission ID.");
          enterFault(F_CALIBRATION);
          ack = mission.overrideLastSetupWithFault(F_CALIBRATION);
        }
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

      const bool calibrationOpcode = command.opcode >= 5 && command.opcode <= 7;
      const bool dryMotionOpcode = command.opcode >= 4 && command.opcode <= 7;
      if (dryMotionOpcode && mission.state() == S_IDLE) {
        dryRunMode = true;  // self-test/calibration are explicit dry-motion confirmations
        setSpray(false);
        digitalWrite(VALVE_PIN, LOW);
        digitalWrite(PUMP_PIN, LOW);
      }
      if (command.opcode == 1 || command.opcode == 2 || dryMotionOpcode) {
        const bool ready = ensurePwmReady();
        mission.setPwmReady(ready);
      }

      // A successful Start ACK is cached for idempotent retries. Reject an
      // incomplete/truncated route before acceptCommand can cache success.
      if (command.opcode == 2 && command.epoch == mission.epoch() &&
          mission.state() == S_ARMED && !plannedRouteReadyForStart()) {
        const FaultCode fault = plannedRouteFailure();
        enterFault(fault);
        protocol_v2::AckV2 routeAck = {
            static_cast<uint8_t>(mission.state()), command.epoch,
            command.commandId, static_cast<uint16_t>(fault),
            mission.calibrationId()};
        sendAck(routeAck);
        return;
      }

      protocol_v2::AckV2 ack = mission.acceptCommand(
          command, receivedAtMs, dryMotionOpcode && dryRunMode);
      const bool duplicate = mission.lastCommandWasDuplicate();

      if (calibrationOpcode) {
        sendAck(ack);  // acknowledge the operator-confirmed step before it moves
        if (ack.faultCode == F_NONE && !duplicate) runCalibrationCommand(command.opcode);
        return;
      }

      if (command.opcode == 3) {
        setSpray(false);
        stopDrive();
        routeCount = 0;
        routeIndex = 0;
        resetCourse();
        bleLog(">>> Mission stopped.");
      } else if (ack.faultCode == F_NONE && command.opcode == 2 && !duplicate) {
        routeIndex = 0;
        lastRouteIndex = -1;
        routeIndexSince = millis();
        resetCourse();
        bleLog(">>> Mission started.");
      }

      if (command.opcode == 3 && command.epoch == 0 &&
          ack.faultCode == F_NONE) {
        ack.calibrationId = protocol_v2::HARDENED_FIRMWARE_CAPABILITY_ID;
      }

      if (command.opcode == 8 && ack.faultCode == F_NONE) {
        sendFaultBuffer();
        sendAck(ack);  // completion ACK follows the final retained sample
      } else {
        sendAck(ack);
      }

      if (command.opcode == 4 && ack.faultCode == F_NONE && !duplicate) {
        runSelfTest();
      }
      return;
    }

    invalidPacketCount++;
    if (mission.state() == S_ARMED || mission.state() == S_RUNNING) {
      enterFault(F_POSE_INVALID);
    }
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
    bleLog("Turn radii L=" + String(turnRadiusLeftFt, 2) + " R=" +
           String(turnRadiusRightFt, 2) + " ft.");
    bleLog(hasStoredMotionCalibration
        ? "[CAL] Stored motion calibration loaded."
        : "[CAL] No valid stored motion calibration; wet operation is blocked.");
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
  // Make direct spray outputs inactive before any peripheral, storage, or BLE
  // work. Program neutral as soon as the PCA9685 can accept a command.
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, STEER_CENTER_US);
  delay(50);

  Serial.begin(115200);

  steeringMapValid = validSteeringMap(steeringMap, steeringMapCount);
  hasStoredMotionCalibration = calibrationPersistence.load(storedMotionCalibration);
  if (hasStoredMotionCalibration) applyMotionCalibration(storedMotionCalibration);

  setSpray(false);
  stopDrive();
  mission.setPwmReady(ensurePwmReady());

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
    mission.setPwmReady(ready);
    if (!ready && (mission.state() == S_CONFIGURED ||
                   mission.state() == S_ARMED || mission.state() == S_RUNNING)) {
      enterFault(F_PWM);
    }
  }

  if (mission.state() == S_ARMED || mission.state() == S_RUNNING) {
    SafetyInput safety = {
      bleConnected,
      mission.poseFresh(millis()),
      true,
      false,
      pwmHealthy,
      pwmHealthy,
      driveDirection.noDisplacement || speedController.stalled,
      driveDirection.wrongDirection,
      true,
      plannedRouteReadyForStart(),
      motionCalibrationReady(),
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
