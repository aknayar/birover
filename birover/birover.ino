#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motor_left = AFMS.getMotor(1);
Adafruit_DCMotor *motor_right = AFMS.getMotor(2);

int motorSpeed = 75;
uint8_t left_dir = BACKWARD;
uint8_t right_dir = BACKWARD;

void setup() {
  AFMS.begin();

  motor_left->setSpeed(motorSpeed);
  motor_left->run(RELEASE);
  motor_right->setSpeed(motorSpeed);
  motor_right->run(RELEASE);
}

void loop() {
  motor_left->run(left_dir);
  motor_right->run(right_dir);
}