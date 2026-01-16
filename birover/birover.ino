#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <utility>

#define L_INT_PIN 2
#define R_INT_PIN 3

const float ACCEL_MULT = 1.025;
const uint64_t ACCEL_STEP = 5;  // millis

// Motor state
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor motor_left = *AFMS.getMotor(1);
Adafruit_DCMotor motor_right = *AFMS.getMotor(2);

uint16_t motorSpeed = 0;
uint64_t last_time = 0;
char prev_dir = 'r';

// Photointerruptor state
int64_t l_cnt = 0;
int64_t r_cnt = 0;

void handleLInterrupt() {
  l_cnt++;
}

void handleRInterrupt() {
  r_cnt++;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  // Motor setup
  AFMS.begin();

  motor_left.setSpeed(motorSpeed);
  motor_left.run(RELEASE);
  motor_right.setSpeed(motorSpeed);
  motor_right.run(RELEASE);

  last_time = millis();

  // Photointerruptor setup
  pinMode(L_INT_PIN, INPUT_PULLUP);
  pinMode(R_INT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_INT_PIN), handleLInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_INT_PIN), handleRInterrupt, CHANGE);
}

std::pair<uint8_t, uint8_t> get_motor_dirs(char dir) {
  switch (dir) {
    case 'w':
      return { BACKWARD, BACKWARD };
    case 's':
      return { FORWARD, FORWARD };
    case 'a':
      return { FORWARD, BACKWARD };
    case 'd':
      return { BACKWARD, FORWARD };
    default:
      return { RELEASE, RELEASE };
  }
}

// Arbitrary values I chose
uint16_t get_max_speed(char dir) {
  switch (dir) {
    case 'w':
      return 255;
    case 's':
      return 64;
    case 'a':
      return 128;
    case 'd':
      return 128;
    default:
      return 0;
  }
}

void loop() {
  char dir = prev_dir;

  // Receive direction state over Bluetooth
  if (Serial1.available()) {
    dir = Serial1.read();
  }

  if (dir == prev_dir) {  // Update speed (acceleration)
    uint16_t max_speed = get_max_speed(dir);
    if (motorSpeed != max_speed && dir != 'r') {
      uint64_t curr_time = millis();
      uint64_t diff_time = curr_time - last_time;
      if (diff_time > ACCEL_STEP) {
        motorSpeed *= ACCEL_MULT;
        last_time = curr_time;

          Serial.print(" ");
          Serial.print(l_cnt);
          Serial.print(" ");
          Serial.println(r_cnt);
          Serial.print("Diff: ");
          Serial.println(r_cnt - l_cnt);
      }
      motorSpeed = std::min(motorSpeed, max_speed);
    }
  } else {  // Reset speed
    motorSpeed = 50;
  }

  auto [left_dir, right_dir] = get_motor_dirs(dir);

  motor_left.setSpeed(motorSpeed);
  motor_right.setSpeed(motorSpeed);

  motor_left.run(left_dir);
  motor_right.run(right_dir);

  prev_dir = dir;
}
