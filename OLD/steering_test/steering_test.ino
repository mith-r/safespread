#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);

// Set this to your steer servo's PCA9685 channel (0-15)
const uint8_t STEER_CH = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pwm.begin();
  pwm.setPWMFreq(50); // 50 Hz standard for RC servos

  Serial.println("=== Servo Steering Calibration ===");
  Serial.println("Enter a pulse width in microseconds (e.g., 1500).");
  Serial.println("Recommended range: 900 to 2100 us.");
  Serial.println("Starting at neutral center: 1500 us.");

  // Move to default center on startup
  pwm.writeMicroseconds(STEER_CH, 1500);
}

void loop() {
  if (Serial.available() > 0) {
    int val = Serial.parseInt();

    // Ignore trailing newline/zero values
    if (val > 0) {
      if (val >= 600 && val <= 2400) {
        pwm.writeMicroseconds(STEER_CH, val);
        Serial.print("Applied pulse: ");
        Serial.print(val);
        Serial.println(" us");
      } else {
        Serial.print("Value out of safety range (600 - 2400 us): ");
        Serial.println(val);
      }
    }

    // Clear any extra characters in the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
