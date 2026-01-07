#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <utility>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motor_left = AFMS.getMotor(1);
Adafruit_DCMotor *motor_right = AFMS.getMotor(2);

uint16_t motorSpeed = 0;

char prev_dir = 'r';

void setup() {
  AFMS.begin();

  Serial.begin(9600);
  Serial1.begin(9600);

  motor_left->setSpeed(motorSpeed);
  motor_left->run(RELEASE);
  motor_right->setSpeed(motorSpeed);
  motor_right->run(RELEASE);
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
  if (Serial1.available()) {
    char dir = Serial1.read();

    if (dir == prev_dir) {
      if (dir != 'r') {
        motorSpeed *= 1.325;
        motorSpeed = std::min(motorSpeed, get_max_speed(dir));
      }
    } else {
      motorSpeed = 50;
    }

    Serial.println(dir);

    auto [left_dir, right_dir] = get_motor_dirs(dir);

    motor_left->setSpeed(motorSpeed);
    motor_right->setSpeed(motorSpeed);

    motor_left->run(left_dir);
    motor_right->run(right_dir);

    prev_dir = dir;
  }
}
