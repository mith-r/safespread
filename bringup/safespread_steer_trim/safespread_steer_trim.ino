/*
 * SafeSpread - STEERING TRIM tool  (calibration only)
 * Adafruit Metro ESP32-S3 + PCA9685 + Bluefruit Connect "Control Pad"
 *
 * PURPOSE
 *   This sketch does ONE thing: help you find the exact microsecond value
 *   where your front wheels point dead straight. It does NOT drive the ESC
 *   or touch the relays. Once you find the number, copy it into
 *   safespread_ble_pca9685.ino  ->  const int STEER_CENTER = <number>;
 *
 * HOW TO USE
 *   1. Upload this sketch, open Serial Monitor at 115200.
 *   2. In Bluefruit Connect, connect to "SafeSpreadTrim" and open the
 *      Controller -> Control Pad.
 *   3. Tap  LEFT (7)  / RIGHT (8) to nudge the wheels a little each tap.
 *      HOLD the button to keep nudging (auto-repeat).
 *   4. Tap  UP (5)   = bigger step        DOWN (6) = smaller step
 *   5. Tap  1        = re-center to your starting guess (in case you
 *                      lose track and want to start over)
 *   6. When the wheels look straight, read the "CENTER = #### us" line
 *      in Serial Monitor. That's your STEER_CENTER value.
 *
 * The servo is held at the current trim value continuously, so the wheels
 * move the instant you tap a button.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---- Nordic UART Service (what Bluefruit Connect talks to) ----
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> board
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // board -> phone

// ---- PCA9685 ----
#define PCA9685_ADDR   0x40
const uint8_t STEER_CH = 1;            // same channel as the main sketch
const float   SERVO_HZ = 50.0;
const uint32_t OSC_FREQ = 27000000;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

// ---- Trim state ----
const int START_GUESS = 1458;         // your current STEER_CENTER, as a starting point
const int TRIM_MIN    = 1200;         // safety rails so we never drive the servo past its travel
const int TRIM_MAX    = 1800;
int  center = START_GUESS;            // <-- this is the value we are hunting for

// ---- Nudge step ----
int  stepUs = 5;                      // microseconds per tap; UP/DOWN change this
const int STEP_MIN = 1;
const int STEP_MAX = 25;

// ---- Auto-repeat while a button is held ----
const unsigned long REPEAT_INTERVAL_MS = 120;

volatile bool btnLeft = false, btnRight = false;
volatile bool connected = false;
unsigned long lastRepeat = 0;

// =====================================================================
// Raw-write queue (BLE callback only copies bytes; parsing is in loop)
// =====================================================================
#define QSLOTS 12
#define QBYTES 32
static uint8_t  qData[QSLOTS][QBYTES];
static uint8_t  qLen[QSLOTS];
static volatile uint8_t qHead = 0, qTail = 0;

void queueWrite(const uint8_t *d, size_t n) {
  uint8_t next = (qHead + 1) % QSLOTS;
  if (next == qTail) return;               // queue full, drop
  if (n > QBYTES) n = QBYTES;
  memcpy(qData[qHead], d, n);
  qLen[qHead] = n;
  qHead = next;
}

// ---- Apply the current center to the servo and report it ----
void applyCenter() {
  center = constrain(center, TRIM_MIN, TRIM_MAX);
  pwm.writeMicroseconds(STEER_CH, center);
  Serial.print("   CENTER = ");
  Serial.print(center);
  Serial.print(" us   (step ");
  Serial.print(stepUs);
  Serial.println(" us)");
}

// ---------- Button handling ----------
void handleButton(char button, bool pressed) {
  switch (button) {
    case '7': btnLeft  = pressed; if (pressed) { center -= stepUs; applyCenter(); } break;  // LEFT
    case '8': btnRight = pressed; if (pressed) { center += stepUs; applyCenter(); } break;  // RIGHT
    case '5':                                                                               // UP = bigger step
      if (pressed) { stepUs = min(stepUs + 1, STEP_MAX); Serial.print("   step -> "); Serial.println(stepUs); }
      break;
    case '6':                                                                               // DOWN = smaller step
      if (pressed) { stepUs = max(stepUs - 1, STEP_MIN); Serial.print("   step -> "); Serial.println(stepUs); }
      break;
    case '1':                                                                               // reset to starting guess
      if (pressed) { center = START_GUESS; Serial.println("   reset to start guess"); applyCenter(); }
      break;
    default: break;
  }
}

// ---------- Packet parser ----------
// Control Pad packet: '!' 'B' <button '1'..'8'> <state '0'|'1'> <checksum>
static uint8_t acc[64];
static size_t  accLen = 0;

void feed(const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }
  size_t i = 0;
  while (accLen - i >= 5) {
    if (acc[i] != '!') { i++; continue; }
    uint8_t sum = acc[i] + acc[i + 1] + acc[i + 2] + acc[i + 3];
    if ((uint8_t)~sum != acc[i + 4]) { i++; continue; }
    if ((char)acc[i + 1] == 'B') handleButton((char)acc[i + 2], acc[i + 3] == '1');
    i += 5;
  }
  if (i > 0) { memmove(acc, acc + i, accLen - i); accLen -= i; }
}

// ---------- BLE callbacks ----------
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
    btnLeft = btnRight = false;
    s->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== SafeSpread STEERING TRIM ===");
  Serial.println("Left/Right = nudge, Up/Down = step size, 1 = reset.");

  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(OSC_FREQ);
  pwm.setPWMFreq(SERVO_HZ);
  delay(10);
  applyCenter();                      // park the servo at the starting guess

  BLEDevice::init("SafeSpreadTrim");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *svc = server->createService(NUS_SERVICE_UUID);
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE |
                   BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  BLECharacteristic *tx = svc->createCharacteristic(
      NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  tx->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("Advertising as \"SafeSpreadTrim\" - connect and open Control Pad.");
}

void loop() {
  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  // Auto-repeat: if a direction button is held, keep nudging.
  unsigned long now = millis();
  if ((btnLeft || btnRight) && (now - lastRepeat >= REPEAT_INTERVAL_MS)) {
    lastRepeat = now;
    if (btnLeft)  center -= stepUs;
    if (btnRight) center += stepUs;
    applyCenter();
  }
}
