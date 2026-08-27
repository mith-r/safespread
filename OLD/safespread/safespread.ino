#include <ESP32Servo.h>

Servo esc;    // throttle, GPIO 42
Servo steer;  // steering, GPIO 41

void setup() {
  esc.attach(42, 1000, 2000);    // pin, min pulse (µs), max pulse (µs)
  steer.attach(21, 1000, 2000);

  // Arm the ESC: hold neutral for a few seconds at power-up
  esc.writeMicroseconds(1500);
  steer.writeMicroseconds(1500);
  delay(3000);
}

void loop() {
  // Steering sweep test
  steer.writeMicroseconds(1700);  // right
  delay(1000);
  steer.writeMicroseconds(1300);  // left
  delay(1000);
  steer.writeMicroseconds(1500);  // center
  delay(1000);

  // Gentle throttle blip
  esc.writeMicroseconds(1600);    // slow forward
  delay(1500);
  esc.writeMicroseconds(1500);    // back to neutral
  delay(2000);
}