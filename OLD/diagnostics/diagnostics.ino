/*
 * SafeSpread - Live Hardware & Wiring Diagnostic Tool
 * 
 * Upload this sketch, open the Serial Monitor at 115200 baud,
 * and gently wiggle each wire/connector one by one.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA_ADDR 0x40
const uint8_t STEER_CH = 0;
const uint8_t ESC_CH   = 4;

Adafruit_PWMServoDriver pwm(PCA_ADDR);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("\n==========================================");
  Serial.println("   SAFESPREAD WIRING DIAGNOSTIC MONITOR   ");
  Serial.println("==========================================");
  Serial.println("Instructions:");
  Serial.println("1. Watch the live status below.");
  Serial.println("2. Wiggle each jumper wire, Grove cable, and screw terminal.");
  Serial.println("3. Look for errors appearing when a specific wire is moved.\n");

  Wire.begin();
  Wire.setTimeOut(30); // Prevent code from hanging if I2C disconnects
}

void loop() {
  Serial.println("------------------------------------------");
  
  // 1. Test I2C Ping to PCA9685
  Wire.beginTransmission(PCA_ADDR);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("[PASS] I2C Communication: OK (PCA9685 reachable @ 0x40)");
    
    // Attempt initialization / configuration
    pwm.begin();
    pwm.setPWMFreq(50);

    // 2. Test Output Command
    Serial.println("[INFO] Sending test pulses (Steer -> Left, ESC -> Neutral)...");
    pwm.writeMicroseconds(STEER_CH, 2100); // Sweep steering left
    pwm.writeMicroseconds(ESC_CH, 1500);
    delay(1000);

    Serial.println("[INFO] Sending test pulses (Steer -> Right)...");
    pwm.writeMicroseconds(STEER_CH, 900);  // Sweep steering right
    delay(1000);

    Serial.println("[INFO] Returning to Center (1500 us)...");
    pwm.writeMicroseconds(STEER_CH, 1500);

  } else if (error == 2) {
    Serial.println("[FAIL] I2C ERROR: NACK received (Address 0x40 not found!)");
    Serial.println("       -> Check 4-pin logic header on PCA9685:");
    Serial.println("          1. VCC (3.3V) wire loose or disconnected");
    Serial.println("          2. GND wire loose or disconnected");
    Serial.println("          3. Grove shield partially unseated from Metro headers");
  } else if (error == 5) {
    Serial.println("[FAIL] I2C ERROR: Bus Timeout / Hardware Lockup");
    Serial.println("       -> Check SDA / SCL lines:");
    Serial.println("          1. SDA or SCL jumper wire broken or intermittent");
    Serial.println("          2. SDA/SCL lines shorted together or to GND");
  } else {
    Serial.print("[FAIL] I2C ERROR Code: ");
    Serial.println(error);
  }

  // 3. Physical Servo Power Reminder
  Serial.println("\n[CHECKLIST]:");
  Serial.println(" - If I2C says [PASS] but steering servo does NOT move:");
  Serial.println("   => Green screw terminal 5V/V+ or GND wire is loose/open.");
  Serial.println("   => Servo 3-pin connector on CH0 is loose or reversed.");
  Serial.println(" - If I2C alternates between [PASS] and [FAIL] while moving:");
  Serial.println("   => You found the loose jumper wire / connection!");
  
  delay(1500);
}
