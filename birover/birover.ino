#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <utility>

const float ACCEL_MULT = 1.025;
const uint64_t ACCEL_STEP = 5;  // millis

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor motor_left = *AFMS.getMotor(1);
Adafruit_DCMotor motor_right = *AFMS.getMotor(2);

uint16_t motorSpeed = 0;
uint64_t last_time = 0;

char prev_dir = 'r';

void setup() {
  AFMS.begin();

  Serial.begin(9600);
  Serial1.begin(9600);

  motor_left.setSpeed(motorSpeed);
  motor_left.run(RELEASE);
  motor_right.setSpeed(motorSpeed);
  motor_right.run(RELEASE);

  last_time = millis();
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
  if (Serial1.available()) {
    dir = Serial1.read();
    Serial.println(dir);
  }

  if (dir == prev_dir) {
    uint16_t max_speed = get_max_speed(dir);
    if (motorSpeed != max_speed && dir != 'r') {
      uint64_t curr_time = millis();
      uint64_t diff_time = curr_time - last_time;
      if (diff_time > ACCEL_STEP) {
        motorSpeed *= ACCEL_MULT;
        last_time = curr_time;
      }
      motorSpeed = std::min(motorSpeed, max_speed);
    }
  } else {
    motorSpeed = 50;
  }

  auto [left_dir, right_dir] = get_motor_dirs(dir);

  motor_left.setSpeed(motorSpeed);
  motor_right.setSpeed(motorSpeed);

  motor_left.run(left_dir);
  motor_right.run(right_dir);

  prev_dir = dir;
}
