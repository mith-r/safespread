/*
 * SafeSpread - Position-Aware Closed-Loop Lawnmower Coverage Rover
 * 
 * Internal 2D Cartesian Coordinate Frame (X = Lateral feet, Y = Longitudinal feet)
 * Area: 480 sqft (21.9 ft x 21.9 ft), 8 Passes spaced 3.13 ft apart.
 * 
 * Controls (Bluefruit Connect Control Pad or Serial Monitor):
 *   '1' -> Start Autonomous Area Coverage
 *   '2' -> Stop / Reset to Neutral
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define PCA9685_ADDR   0x40
const uint8_t STEER_CH = 0;
const uint8_t ESC_CH   = 4;
const float   SERVO_HZ = 50.0;

const int VALVE_PIN = 5;
const int PUMP_PIN  = 6;

// Microsecond Pulse Widths
const int NEUTRAL_US      = 1500;
const int THROTTLE_FWD_US = 1620; // Calibrated 3.37 ft/s cruise
const int THROTTLE_TURN_US= 1600; // Turning throttle

const int STEER_CENTER_US = 1500;
const int STEER_LEFT_US   = 2390;
const int STEER_RIGHT_US  = 700;

// Geometry for 480 sqft Square: 21.91 ft x 21.91 ft
const int   TOTAL_PASSES       = 8;
const float SQUARE_SIDE_FT     = 21.91; // Total length of each pass
const float LANE_SPACING_FT    = 3.13;  // Distance between parallel passes (37.6 inches)
const float CRUISE_SPEED_FPS   = 3.37;  // Forward speed (ft/sec)
const float TURN_SPEED_FPS     = 1.50;  // Lateral speed during turns (ft/sec)

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
BLECharacteristic *txCharacteristic = NULL;

// ---- Internal 2D Position Estimator ----
float posX = 0.0; // Lateral position across square (0.0 to 21.9 ft)
float posY = 0.0; // Longitudinal position down the lane (0.0 to 21.9 ft)
float currentYaw       = 0.0;
float baseHeading      = 0.0; // 0° reference
float targetHeading    = 0.0;
bool  firstPacketFound = false;

unsigned long lastOdometryTime = 0;
unsigned long lastSerialTime   = 0;
unsigned long stateTimer       = 0;

enum AutoState {
  AUTO_IDLE,
  AUTO_SPRAY_PASS,
  AUTO_PAUSE_1,
  AUTO_TURN_90_OUT,
  AUTO_PAUSE_2,
  AUTO_DRIVE_LATERAL,
  AUTO_PAUSE_3,
  AUTO_TURN_90_INTO_LANE,
  AUTO_PAUSE_4,
  AUTO_COMPLETE
};

AutoState state = AUTO_IDLE;
int currentPass = 1;
volatile bool bleConnected = false;

void setChannelPulse(uint8_t channel, int microseconds) {
  uint16_t ticks = (uint16_t)(((uint32_t)microseconds * 4096UL) / 20000UL);
  pwm.setPWM(channel, 0, ticks);
}

void bleLog(String msg) {
  Serial.println(msg);
  if (bleConnected && txCharacteristic != NULL) {
    msg += "\n";
    txCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    txCharacteristic->notify();
  }
}

void setSpray(bool on) {
  digitalWrite(VALVE_PIN, on ? HIGH : LOW);
  digitalWrite(PUMP_PIN,  LOW);
}

void stopDrive() {
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, STEER_CENTER_US);
}

void resetMission() {
  stopDrive();
  setSpray(false);
  posX = 0.0;
  posY = 0.0;
  currentPass = 1;
  state = AUTO_IDLE;
  bleLog(">>> MISSION STOPPED / RESET to (0.0, 0.0)");
}

void startMission() {
  baseHeading = currentYaw;
  targetHeading = baseHeading;
  posX = 0.0;
  posY = 0.0;
  currentPass = 1;
  setSpray(true);
  stateTimer = millis();
  state = AUTO_SPRAY_PASS;
  bleLog("==========================================");
  bleLog(">>> MISSION STARTED: 480 SQFT GRID");
  bleLog(">>> Origin (0.0, 0.0) ft | Heading: " + String(baseHeading, 1) + "°");
  bleLog("==========================================");
}

float angleDiff(float target, float current) {
  float d = target - current;
  while (d > 180.0)  d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// Circular Queue & Packet Parser
#define QSLOTS 16
#define QBYTES 32
static uint8_t  qData[QSLOTS][QBYTES];
static uint8_t  qLen[QSLOTS];
static volatile uint8_t qHead = 0, qTail = 0;

void queueWrite(const uint8_t *d, size_t n) {
  uint8_t next = (qHead + 1) % QSLOTS;
  if (next == qTail) return;
  if (n > QBYTES) n = QBYTES;
  memcpy(qData[qHead], d, n);
  qLen[qHead] = n;
  qHead = next;
}

static uint8_t acc[128];
static size_t  accLen = 0;

float parseIEEE754Float(const uint8_t *b) {
  float f;
  memcpy(&f, b, 4);
  return f;
}

void handleButton(char button, bool pressed) {
  if (!pressed) return;
  if (button == '1') startMission();
  else if (button == '2' || button == '4') resetMission();
}

void processIncomingYaw(float newYaw) {
  currentYaw = newYaw;
  if (!firstPacketFound) {
    baseHeading = newYaw;
    firstPacketFound = true;
  }
}

void feed(const uint8_t *d, size_t n) {
  if (n == 1) {
    if (d[0] == '1' || d[0] == 's' || d[0] == 'S') { startMission(); return; }
    if (d[0] == '2' || d[0] == 'x' || d[0] == 'X') { resetMission(); return; }
  }

  for (size_t i = 0; i < n; i++) {
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }

  size_t i = 0;
  while (accLen - i >= 5) {
    if (acc[i] != '!') { i++; continue; }

    char kind = (char)acc[i + 1];

    if (kind == 'B') {
      uint8_t sum = acc[i] + acc[i + 1] + acc[i + 2] + acc[i + 3];
      if ((uint8_t)~sum == acc[i + 4]) {
        char b = (char)acc[i + 2];
        bool pressed = (acc[i + 3] == '1');
        handleButton(b, pressed);
        i += 5;
        continue;
      }
    }

    if (kind == 'O' && (accLen - i >= 15)) {
      processIncomingYaw(parseIEEE754Float(&acc[i + 2]));
      i += 15;
      continue;
    }

    if (kind == 'M' && (accLen - i >= 15)) {
      float mx = parseIEEE754Float(&acc[i + 2]);
      float my = parseIEEE754Float(&acc[i + 6]);
      float mag = atan2(my, mx) * (180.0f / 3.14159265f);
      if (mag < 0) mag += 360.0f;
      processIncomingYaw(mag);
      i += 15;
      continue;
    }

    if (kind == 'Q' && (accLen - i >= 19)) {
      float qx = parseIEEE754Float(&acc[i + 2]);
      float qy = parseIEEE754Float(&acc[i + 6]);
      float qz = parseIEEE754Float(&acc[i + 10]);
      float qw = parseIEEE754Float(&acc[i + 14]);

      float qYaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / 3.14159265f);
      if (qYaw < 0) qYaw += 360.0f;
      processIncomingYaw(qYaw);
      i += 19;
      continue;
    }

    i++;
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
    firstPacketFound = false;
    bleLog("Connected to SafeSpread!");
  }
  void onDisconnect(BLEServer *s) {
    bleConnected = false;
    resetMission();
    s->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN,  OUTPUT);
  setSpray(false);

  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeOut(30);

  pwm.begin();
  pwm.setPWMFreq(SERVO_HZ);
  delay(50);
  stopDrive();

  BLEDevice::init("SafeSpread");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *svc = server->createService(NUS_SERVICE_UUID);
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());

  txCharacteristic = svc->createCharacteristic(
      NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  lastOdometryTime = millis();
  Serial.println("SafeSpread 2D Internal Path Tracker Ready.");
}

void loop() {
  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  unsigned long now = millis();
  float dt = (now - lastOdometryTime) / 1000.0f;
  lastOdometryTime = now;

  // --- 2D Dead Reckoning Kinematic Integrator (50Hz) ---
  if (state != AUTO_IDLE && dt > 0.001f && dt < 0.5f) {
    float speed = 0.0f;
    if (state == AUTO_SPRAY_PASS || state == AUTO_DRIVE_LATERAL) {
      speed = CRUISE_SPEED_FPS;
    } else if (state == AUTO_TURN_90_OUT || state == AUTO_TURN_90_INTO_LANE) {
      speed = TURN_SPEED_FPS;
    }

    // Relative angle against initial origin vector (0° is +Y, 90° is +X)
    float relAngleRad = angleDiff(currentYaw, baseHeading) * (3.14159265f / 180.0f);
    posX += speed * sin(relAngleRad) * dt;
    posY += speed * cos(relAngleRad) * dt;
  }

  // Periodic Telemetry to Serial Monitor & Phone HUD (2Hz)
  if (now - lastSerialTime >= 500) {
    lastSerialTime = now;
    if (state != AUTO_IDLE) {
      String posReport = "[MAP] Pass " + String(currentPass) + "/8 | Pos: (" + String(posX, 1) + ", " + String(posY, 1) + ") ft | Angle: " + String(currentYaw, 1) + "°";
      Serial.println(posReport);
      if (bleConnected && txCharacteristic != NULL) {
        posReport += "\n";
        txCharacteristic->setValue((uint8_t*)posReport.c_str(), posReport.length());
        txCharacteristic->notify();
      }
    }
  }

  // Active Heading PID Steering Calculation
  float err = angleDiff(targetHeading, currentYaw);
  int steerCorrection = constrain((int)(STEER_CENTER_US + err * 25.0f), STEER_RIGHT_US, STEER_LEFT_US);
  unsigned long elapsed = now - stateTimer;

  switch (state) {
    case AUTO_IDLE:
      stopDrive();
      break;

    // --- 1. SPRAY PASS: Driven by longitudinal Y position (Stops at 21.9 ft) ---
    case AUTO_SPRAY_PASS:
      setChannelPulse(STEER_CH, steerCorrection);
      setChannelPulse(ESC_CH, THROTTLE_FWD_US);

      // Check if rover has reached the lane end (Y >= 21.9 ft on odd passes, Y <= 0.0 ft on even passes)
      if ((currentPass % 2 == 1 && posY >= SQUARE_SIDE_FT) || 
          (currentPass % 2 == 0 && posY <= 0.0f) || 
          (elapsed >= 7000)) { // Safety timeout fallback

        setSpray(false);
        stopDrive();

        if (currentPass >= TOTAL_PASSES) {
          state = AUTO_COMPLETE;
        } else {
          float turnOffset = (currentPass % 2 == 1) ? 90.0f : -90.0f;
          targetHeading = fmod(targetHeading + turnOffset + 360.0f, 360.0f);
          stateTimer = millis();
          state = AUTO_PAUSE_1;
          bleLog(">>> End of Lane Reached! Pos Y: " + String(posY, 1) + " ft. Turning 90°...");
        }
      }
      break;

    case AUTO_PAUSE_1:
      stopDrive();
      if (elapsed >= 250) {
        stateTimer = millis();
        state = AUTO_TURN_90_OUT;
      }
      break;

    // --- 2. HEADLAND 90° TURN ---
    case AUTO_TURN_90_OUT: {
      int turnSteer = (currentPass % 2 == 1) ? STEER_RIGHT_US : STEER_LEFT_US;
      setChannelPulse(STEER_CH, turnSteer);
      setChannelPulse(ESC_CH, THROTTLE_TURN_US);

      if (abs(err) < 10.0f || elapsed >= 3000) {
        stopDrive();
        stateTimer = millis();
        state = AUTO_PAUSE_2;
      }
      break;
    }

    case AUTO_PAUSE_2:
      stopDrive();
      if (elapsed >= 250) {
        stateTimer = millis();
        state = AUTO_DRIVE_LATERAL;
      }
      break;

    // --- 3. LATERAL SHIFT: Controlled by target X lane coordinate ---
    case AUTO_DRIVE_LATERAL: {
      setChannelPulse(STEER_CH, steerCorrection);
      setChannelPulse(ESC_CH, THROTTLE_FWD_US);

      float targetX = (float)currentPass * LANE_SPACING_FT; // Exact target coordinate (Pass 1->3.1ft, Pass 2->6.3ft, etc.)

      if (posX >= targetX || elapsed >= 1200) {
        stopDrive();
        float turnOffset = (currentPass % 2 == 1) ? 90.0f : -90.0f;
        targetHeading = fmod(targetHeading + turnOffset + 360.0f, 360.0f);
        stateTimer = millis();
        state = AUTO_PAUSE_3;
        bleLog(">>> Reached Lane Coordinate X: " + String(posX, 1) + " ft (Target: " + String(targetX, 1) + " ft)");
      }
      break;
    }

    case AUTO_PAUSE_3:
      stopDrive();
      if (elapsed >= 250) {
        stateTimer = millis();
        state = AUTO_TURN_90_INTO_LANE;
      }
      break;

    // --- 4. RETURN 90° TURN INTO LANE ---
    case AUTO_TURN_90_INTO_LANE: {
      int turnSteer = (currentPass % 2 == 1) ? STEER_RIGHT_US : STEER_LEFT_US;
      setChannelPulse(STEER_CH, turnSteer);
      setChannelPulse(ESC_CH, THROTTLE_TURN_US);

      if (abs(err) < 10.0f || elapsed >= 3000) {
        stopDrive();
        stateTimer = millis();
        state = AUTO_PAUSE_4;
      }
      break;
    }

    case AUTO_PAUSE_4:
      stopDrive();
      if (elapsed >= 250) {
        currentPass++;
        setSpray(true);
        stateTimer = millis();
        state = AUTO_SPRAY_PASS;
        bleLog("==========================================");
        bleLog(">>> Starting Pass " + String(currentPass) + "/8 at X=" + String(posX, 1) + " ft");
        bleLog("==========================================");
      }
      break;

    case AUTO_COMPLETE:
      resetMission();
      bleLog("==========================================");
      bleLog("=== ALL 480 SQFT FULLY COVERED! ===");
      bleLog("==========================================");
      break;
  }
}
