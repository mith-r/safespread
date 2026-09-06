/*
 * SafeSpread - BLE RC drive test (PCA9685 + Grove relays)
 * Adafruit Metro ESP32-S3 + Bluefruit Connect "Control Pad"
 *
 * Outputs:
 *   PCA9685 ch 2 = steering servo
 *   PCA9685 ch 4 = ESC
 *   Grove D4      = valve relay
 *   Grove D6      = pump relay
 *
 * Control Pad mapping:
 *   5 = up/forward   6 = down/reverse
 *   7 = left         8 = right
 *   1 = toggle PUMP  2 = toggle VALVE
 *   4 = emergency stop (drive neutral + both relays off)
 *
 * Open Serial Monitor at 115200 to see every byte received.
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
const uint8_t STEER_CH = 1;       // servo on column 1
const uint8_t ESC_CH   = 4;       // ESC on column 4
const float   SERVO_HZ = 50.0;
const uint32_t OSC_FREQ = 27000000;   // tweak if your pulse widths measure long/short

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

// ---- Grove relay pins ----
const int VALVE_PIN = 4;          // Grove D4
const int PUMP_PIN  = 6;          // Grove D6
const bool RELAY_ACTIVE_HIGH = true;

// ---- Pulse widths (us) ----
const int NEUTRAL       = 1500;
const int THROTTLE_FWD  = 1590;   // start gentle, raise once you trust it
const int THROTTLE_REV  = 1350;
const int STEER_CENTER  = 1400;
const int STEER_LEFT    = 1100;
const int STEER_RIGHT   = 1700;

// ---- Throttle slew rate ----
const int  RAMP_STEP_US = 5;      // us per tick
const unsigned long RAMP_INTERVAL_MS = 20;

// ---- Debug ----
const bool DEBUG_RAW   = true;    // print raw bytes of every BLE write
const bool DEBUG_STATE = true;    // print outputs whenever they change

volatile bool btnUp = false, btnDown = false, btnLeft = false, btnRight = false;
volatile bool pumpOn = false, valveOn = false;
volatile bool connected = false;

int throttleNow = NEUTRAL;
unsigned long lastRamp = 0;

// ---------- Relay helper ----------
void writeRelay(int pin, bool on) {
  digitalWrite(pin, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

// =====================================================================
// Raw-write queue. The BLE callback runs on the Bluetooth task, so it
// only copies bytes; all printing and parsing happens in loop().
// =====================================================================
#define QSLOTS 12
#define QBYTES 32
static uint8_t  qData[QSLOTS][QBYTES];
static uint8_t  qLen[QSLOTS];
static volatile uint8_t qHead = 0, qTail = 0;

void queueWrite(const uint8_t *d, size_t n) {
  uint8_t next = (qHead + 1) % QSLOTS;
  if (next == qTail) return;                 // queue full, drop
  if (n > QBYTES) n = QBYTES;
  memcpy(qData[qHead], d, n);
  qLen[qHead] = n;
  qHead = next;
}

// ---------- Button handling ----------
const char *buttonName(char b) {
  switch (b) {
    case '1': return "1 (PUMP toggle)";
    case '2': return "2 (VALVE toggle)";
    case '3': return "3";
    case '4': return "4 (E-STOP)";
    case '5': return "5 UP";
    case '6': return "6 DOWN";
    case '8': return "8 RIGHT";
    case '7': return "7 LEFT";
    default:  return "?";
  }
}

void handleButton(char button, bool pressed) {
  switch (button) {
    case '5': btnUp    = pressed; break;
    case '6': btnDown  = pressed; break;
    case '8': btnLeft  = pressed; break;
    case '7': btnRight = pressed; break;
    case '1':                                  // toggle on press only
      if (pressed) {
        pumpOn = !pumpOn;
        Serial.print("   -> PUMP  "); Serial.println(pumpOn ? "ON" : "OFF");
      }
      break;
    case '2':
      if (pressed) {
        valveOn = !valveOn;
        Serial.print("   -> VALVE "); Serial.println(valveOn ? "ON" : "OFF");
      }
      break;
    case '4':                                  // e-stop
      if (pressed) {
        btnUp = btnDown = btnLeft = btnRight = false;
        pumpOn = valveOn = false;
        Serial.println("   -> E-STOP: drive neutral, relays off");
      }
      break;
    default: break;
  }
}

// ---------- Packet parser ----------
// Control Pad packet: '!' 'B' <button '1'..'8'> <state '0'|'1'> <checksum>
// checksum = ~(sum of the first 4 bytes)
static uint8_t acc[64];
static size_t  accLen = 0;

void printRaw(const uint8_t *d, size_t n) {
  Serial.print("[RX ");
  Serial.print(n);
  Serial.print(" B] hex:");
  for (size_t i = 0; i < n; i++) {
    Serial.print(d[i] < 0x10 ? " 0" : " ");
    Serial.print(d[i], HEX);
  }
  Serial.print("  ascii: ");
  for (size_t i = 0; i < n; i++) {
    char c = (char)d[i];
    Serial.print((c >= 32 && c <= 126) ? c : '.');
  }
  Serial.println();
}

void feed(const uint8_t *d, size_t n) {
  if (DEBUG_RAW) printRaw(d, n);

  for (size_t i = 0; i < n; i++) {                       // append, drop oldest if full
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }

  size_t i = 0;
  while (accLen - i >= 5) {
    if (acc[i] != '!') { i++; continue; }
    uint8_t sum = acc[i] + acc[i + 1] + acc[i + 2] + acc[i + 3];
    if ((uint8_t)~sum != acc[i + 4]) {
      Serial.print("   -> '!' at offset ");
      Serial.print(i);
      Serial.println(" but checksum failed (not a 5-byte Control Pad packet?)");
      i++;
      continue;
    }
    char kind = (char)acc[i + 1];
    if (kind == 'B') {
      char b = (char)acc[i + 2];
      bool pressed = (acc[i + 3] == '1');
      Serial.print("   -> BUTTON ");
      Serial.print(buttonName(b));
      Serial.println(pressed ? "  PRESSED" : "  released");
      handleButton(b, pressed);
    } else {
      Serial.print("   -> non-button packet, type '");
      Serial.print(kind);
      Serial.println("' (ignored)");
    }
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
    btnUp = btnDown = btnLeft = btnRight = false;   // failsafe
    pumpOn = valveOn = false;
    s->startAdvertising();
  }
};

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1500);                       // give USB-CDC time to enumerate
  Serial.println("\n=== SafeSpread BLE drive test (PCA9685) ===");

  // Relays off before anything else
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN,  OUTPUT);
  writeRelay(VALVE_PIN, false);
  writeRelay(PUMP_PIN,  false);

  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(OSC_FREQ);
  pwm.setPWMFreq(SERVO_HZ);
  delay(10);

  pwm.writeMicroseconds(ESC_CH, NEUTRAL);      // arm the ESC
  pwm.writeMicroseconds(STEER_CH, STEER_CENTER);
  delay(3000);

  BLEDevice::init("SafeSpread");
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

  Serial.println("Advertising as \"SafeSpread\" - waiting for connection");
}

// ---------- Loop ----------
void loop() {
  // 1. Drain and parse anything the phone sent
  static bool wasConnected = false;
  if (connected != wasConnected) {
    Serial.println(connected ? "** BLE connected **"
                             : "** BLE disconnected - failsafe to neutral **");
    wasConnected = connected;
  }

  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  // 2. Steering: immediate
  int steerTarget = STEER_CENTER;
  if (connected) {
    if (btnLeft && !btnRight)      steerTarget = STEER_LEFT;
    else if (btnRight && !btnLeft) steerTarget = STEER_RIGHT;
  }
  pwm.writeMicroseconds(STEER_CH, steerTarget);

  // 3. Throttle: ramped
  int throttleTarget = NEUTRAL;
  if (connected) {
    if (btnUp && !btnDown)      throttleTarget = THROTTLE_FWD;
    else if (btnDown && !btnUp) throttleTarget = THROTTLE_REV;
  }

  unsigned long now = millis();
  if (now - lastRamp >= RAMP_INTERVAL_MS) {
    lastRamp = now;
    if (throttleNow < throttleTarget)
      throttleNow = min(throttleNow + RAMP_STEP_US, throttleTarget);
    else if (throttleNow > throttleTarget)
      throttleNow = max(throttleNow - RAMP_STEP_US, throttleTarget);
    pwm.writeMicroseconds(ESC_CH, throttleNow);
  }

  // 4. Relays: only energized while connected
  bool pumpOut  = connected && pumpOn;
  bool valveOut = connected && valveOn;
  static int lastPump = -1, lastValve = -1;
  if ((int)pumpOut != lastPump)   { writeRelay(PUMP_PIN,  pumpOut);  lastPump  = pumpOut; }
  if ((int)valveOut != lastValve) { writeRelay(VALVE_PIN, valveOut); lastValve = valveOut; }

  // 5. Report output changes
  if (DEBUG_STATE) {
    static int lastSteer = -1, lastThrottleTarget = -1;
    static int rptPump = -1, rptValve = -1;
    if (steerTarget != lastSteer || throttleTarget != lastThrottleTarget ||
        (int)pumpOut != rptPump || (int)valveOut != rptValve) {
      Serial.print("   OUT  steer=");
      Serial.print(steerTarget);
      Serial.print("us  throttle target=");
      Serial.print(throttleTarget);
      Serial.print("us  pump=");
      Serial.print(pumpOut ? "ON" : "OFF");
      Serial.print("  valve=");
      Serial.println(valveOut ? "ON" : "OFF");
      lastSteer = steerTarget;
      lastThrottleTarget = throttleTarget;
      rptPump = pumpOut;
      rptValve = valveOut;
    }
  }
}
