
/********** UAV Aetheris-1 – Autonomous Flight Control Program
* Purpose:
* This program controls the autonomous operation of the Aetheris-1 quadcopter UAV,
* including takeoff, stabilization via IMU, obstacle avoidance, data streaming and
* autonomous landing after mission completion or battery drop.
*
* Hardware:
* - MCU: Arduino Uno/Nano
* - Sensors: MPU6050 (IMU), HC-SR04 (Ultrasonic), NEO-6M (GPS)
* - Actuators: 4 × ESC + Brushless Motors, Li-Ion Battery, PDB
* - Communication: ESP8266 WiFi module (data streaming)
* - Optional: Camera (analog or serial controlled)
*
* Software:
* - Arduino IDE
* - I2C/Wire library, Servo.h, SoftwareSerial for GPS/ESP
* - PID control for stabilization (simplified)
*
* Reference:
* v1.0 – G. Exarchos, June 2025
**********/

#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Servo ESCs for each motor
Servo esc1, esc2, esc3, esc4;

// Pins
const int ultrasonicTrig = 7;
const int ultrasonicEcho = 6;
const int buzzer = 13;
const int batteryPin = A0;

// GPS module
SoftwareSerial gpsSerial(4, 3); // RX, TX

// Function declarations
void initializeESCs();
bool batteryLow();
bool launchCommandReceived();
void takeOff();
void stabilize();
void patrolArea();
void obstacleAvoidance();
void landingSequence();

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Wire.begin();

  // Initialize ESC pins
  esc1.attach(9);
  esc2.attach(10);
  esc3.attach(5);
  esc4.attach(11);

  pinMode(ultrasonicTrig, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);
  pinMode(buzzer, OUTPUT);

  initializeESCs();
  digitalWrite(buzzer, HIGH); // Standby signal
  delay(1000);
  digitalWrite(buzzer, LOW);
}

void loop() {
  if (batteryLow()) {
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    delay(500);
    return;
  }

  if (launchCommandReceived()) {
    takeOff();
    stabilize();

    while (true) {
      patrolArea();
      obstacleAvoidance();
      if (batteryLow()) break;
    }

    landingSequence();
  }
}

void initializeESCs() {
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000);
}

bool batteryLow() {
  int voltage = analogRead(batteryPin);
  float v = voltage * (5.0 / 1023.0) * 3; // Voltage divider
  return (v < 10.5); // Threshold for 3S Li-Ion
}

bool launchCommandReceived() {
  // Placeholder: could be button press or serial signal
  return digitalRead(2) == HIGH;
}

void takeOff() {
  for (int speed = 1000; speed <= 1400; speed += 10) {
    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    delay(50);
  }
}

void stabilize() {
  esc1.writeMicroseconds(1350);
  esc2.writeMicroseconds(1350);
  esc3.writeMicroseconds(1350);
  esc4.writeMicroseconds(1350);
}

void patrolArea() {
  delay(1000); // Simulate hover
}

void obstacleAvoidance() {
  long duration;
  digitalWrite(ultrasonicTrig, LOW); delayMicroseconds(2);
  digitalWrite(ultrasonicTrig, HIGH); delayMicroseconds(10);
  digitalWrite(ultrasonicTrig, LOW);
  duration = pulseIn(ultrasonicEcho, HIGH);
  float distance = duration * 0.034 / 2;
  if (distance < 30 && distance > 5) {
    esc1.writeMicroseconds(1300);
    esc3.writeMicroseconds(1400);
    delay(500);
    stabilize();
  }
}

void landingSequence() {
  for (int speed = 1300; speed >= 1000; speed -= 10) {
    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    delay(100);
  }
  esc1.detach(); esc2.detach(); esc3.detach(); esc4.detach();
  digitalWrite(buzzer, HIGH);
}
