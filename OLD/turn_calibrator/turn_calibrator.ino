/*
 * SafeSpread - Smartphone-Guided Autonomous Brining Rover
 * 
 * Uses the onboard smartphone's Gyro/Compass sensor fusion streamed over
 * Adafruit Bluefruit LE Connect (!Q Quaternion packets) for active heading hold.
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

const int NEUTRAL      = 1500;
const int THROTTLE_FWD = 1620;
const int THROTTLE_REV = 1380;
const int STEER_CENTER = 1500;
const int STEER_LEFT   = 2390;
const int STEER_RIGHT  = 700;

// Mission Geometry
const int TOTAL_PASSES = 8;
const unsigned long PASS_TIME_MS = 6500;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

// ---- Live Orientation Variables from Phone ----
float currentYaw = 0.0;      // 0.0 to 360.0 degrees
float targetHeading = 0.0;   // Setpoint for straight driving
bool hasHeading = false;

enum AutoState {
  AUTO_IDLE,
  AUTO_SPRAY_PASS,
  AUTO_TURN_STEP1_FWD,
  AUTO_TURN_STEP2_REV,
  AUTO_TURN_STEP3_ALIGN,
  AUTO_COMPLETE
};

AutoState state = AUTO_IDLE;
unsigned long stateTimer = 0;
int currentPass = 1;
volatile bool connected = false;

void setSpray(bool on) {
  digitalWrite(VALVE_PIN, on ? HIGH : LOW);
  digitalWrite(PUMP_PIN,  LOW); // Keep pump off for dry testing
}

void stopDrive() {
  pwm.writeMicroseconds(ESC_CH, NEUTRAL);
  pwm.writeMicroseconds(STEER_CH, STEER_CENTER);
}

// Normalizes angular difference between -180 and +180 deg
float angleDiff(float target, float current) {
  float d = target - current;
  while (d > 180.0)  d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// =====================================================================
// BLE Packet Queue & Parser
// =====================================================================
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

// Helper to unpack 32-bit float from byte array
float parseIEEE754Float(const uint8_t *b) {
  float f;
  memcpy(&f, b, 4);
  return f;
}

void feed(const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }

  size_t i = 0;
  while (accLen - i >= 5) {
    if (acc[i] != '!') { i++; continue; }

    char kind = (char)acc[i + 1];

    // Button packet: '!B' + button + state + CRC (5 bytes)
    if (kind == 'B' && (accLen - i >= 5)) {
      char b = (char)acc[i + 2];
      bool pressed = (acc[i + 3] == '1');
      if (pressed) {
        if (b == '1') { // Start run
          targetHeading = currentYaw;
          currentPass = 1;
          setSpray(true);
          stateTimer = millis();
          state = AUTO_SPRAY_PASS;
        } else if (b == '2' || b == '4') { // Stop / Reset
          setSpray(false);
          stopDrive();
          state = AUTO_IDLE;
        }
      }
      i += 5;
      continue;
    }

    // Quaternion Orientation Packet: '!Q' + 4 floats (16 bytes) + CRC = 19 bytes
    if (kind == 'Q' && (accLen - i >= 19)) {
      float qx = parseIEEE754Float(&acc[i + 2]);
      float qy = parseIEEE754Float(&acc[i + 6]);
      float qz = parseIEEE754Float(&acc[i + 10]);
      float qw = parseIEEE754Float(&acc[i + 14]);

      // Calculate yaw angle in degrees
      float rawYaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / 3.14159265f);
      if (rawYaw < 0) rawYaw += 360.0f;
      currentYaw = rawYaw;
      hasHeading = true;

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
  void onConnect(BLEServer *s) { connected = true; }
  void onDisconnect(BLEServer *s) {
    connected = false;
    setSpray(false);
    stopDrive();
    state = AUTO_IDLE;
    s->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN,  OUTPUT);
  setSpray(false);

  Wire.begin();
  delay(100);

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
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Smartphone Guidance Ready.");
}

void loop() {
  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - stateTimer;

  switch (state) {
    case AUTO_IDLE:
      stopDrive();
      break;

    case AUTO_SPRAY_PASS: {
      // Proportional Heading Correction for Laser-Straight Driving
      float err = angleDiff(targetHeading, currentYaw);
      int steerCorrection = constrain((int)(STEER_CENTER + err * 18.0f), STEER_RIGHT, STEER_LEFT);
      
      pwm.writeMicroseconds(STEER_CH, steerCorrection);
      pwm.writeMicroseconds(ESC_CH, THROTTLE_FWD);

      if (elapsed >= PASS_TIME_MS) {
        setSpray(false);
        stopDrive();

        if (currentPass >= TOTAL_PASSES) {
          state = AUTO_COMPLETE;
        } else {
          // Set new target heading 180 degrees opposite
          targetHeading = fmod(targetHeading + 180.0f, 360.0f);
          stateTimer = now;
          state = AUTO_TURN_STEP1_FWD;
        }
      }
      break;
    }

    case AUTO_TURN_STEP1_FWD:
      pwm.writeMicroseconds(STEER_CH, (currentPass % 2 == 1) ? STEER_RIGHT : STEER_LEFT);
      pwm.writeMicroseconds(ESC_CH, THROTTLE_FWD);

      if (elapsed >= 1800) {
        stopDrive();
        delay(200);
        stateTimer = millis();
        state = AUTO_TURN_STEP2_REV;
      }
      break;

    case AUTO_TURN_STEP2_REV: {
      pwm.writeMicroseconds(STEER_CH, (currentPass % 2 == 1) ? STEER_LEFT : STEER_RIGHT);
      pwm.writeMicroseconds(ESC_CH, THROTTLE_REV);

      // Closed-Loop: Reverses until the phone's heading is within 15 degrees of target 180!
      float turnRemaining = abs(angleDiff(targetHeading, currentYaw));
      if (turnRemaining < 15.0f || elapsed >= 3500) {
        stopDrive();
        delay(200);
        stateTimer = millis();
        state = AUTO_TURN_STEP3_ALIGN;
      }
      break;
    }

    case AUTO_TURN_STEP3_ALIGN:
      pwm.writeMicroseconds(STEER_CH, STEER_CENTER);
      pwm.writeMicroseconds(ESC_CH, THROTTLE_FWD);

      if (elapsed >= 800) {
        currentPass++;
        setSpray(true);
        stateTimer = millis();
        state = AUTO_SPRAY_PASS;
      }
      break;

    case AUTO_COMPLETE:
      setSpray(false);
      stopDrive();
      state = AUTO_IDLE;
      break;
  }

  delay(20);
}
