#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <utility>

#define L_INT_PIN 2
#define R_INT_PIN 3

const float ACCEL_MULT = 1.05;
const uint64_t ACCEL_STEP = 5;  // millis

// Bluetooth State
#define TIMEOUT_MILLIS 250
#define NUM_VALUES 3
#define SPIN_THRESHOLD 0.9f

#define REST \
  { RELEASE, 0 }

uint8_t rem_values = 0;
uint8_t buf[NUM_VALUES];

uint64_t prev_millis = 0;

uint64_t prev_full = 0;
uint64_t total_full = 0;
uint64_t num_full = 0;

// Motor state
#define MAX_SPEED 255

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor motor_left = *AFMS.getMotor(1);
Adafruit_DCMotor motor_right = *AFMS.getMotor(2);

// Photointerruptor state
int64_t l_cnt = 0;
int64_t r_cnt = 0;

void handleLInterrupt() {
  l_cnt++;
}

void handleRInterrupt() {
  r_cnt++;
}

void update_motor(Adafruit_DCMotor* motor, std::pair<uint8_t, uint16_t> params) {
  motor->setSpeed(params.second);
  motor->run(params.first);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  // Motor setup
  AFMS.begin();

  update_motor(&motor_left, REST);
  update_motor(&motor_right, REST);

  // Photointerruptor setup
  pinMode(L_INT_PIN, INPUT_PULLUP);
  pinMode(R_INT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_INT_PIN), handleLInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_INT_PIN), handleRInterrupt, CHANGE);
}

float deserialize_val(byte b) {
  return (((int)b) - 128) / 100.0f;
}

float normalize(float value) {
  return (value + 1.0f) / 2.0f;  // [-1, 1] -> [0, 1]
}

uint8_t flip_direction(uint8_t direction) {
  return ((direction - 1) ^ 1) + 1;  // flip 1 and 2
}

std::pair<std::pair<uint8_t, uint16_t>,
          std::pair<uint8_t, uint16_t>>
solve_motors(float steering,
             float throttle,
             float brake) {
  float velocity = normalize(throttle) - normalize(brake);
  float speed = abs(velocity);

  uint16_t target_left_speed = speed * MAX_SPEED;
  uint16_t target_right_speed = speed * MAX_SPEED;

  uint8_t left_direction = BACKWARD;
  uint8_t right_direction = BACKWARD;

  float steering_multiplier = 1.0f;
  bool spin = false;
  if (abs(steering) <= SPIN_THRESHOLD) {
    steering_multiplier = (SPIN_THRESHOLD - abs(steering)) / SPIN_THRESHOLD;
  } else {
    spin = true;
    steering_multiplier = (abs(steering) - SPIN_THRESHOLD) * (1 / (1 - SPIN_THRESHOLD));
  }
  if (steering < 0) {
    target_left_speed *= steering_multiplier;
    if (spin) left_direction = FORWARD;
  } else {
    target_right_speed *= steering_multiplier;
    if (spin) right_direction = FORWARD;
  }

  if (velocity < 0) {
    left_direction = flip_direction(left_direction);
    right_direction = flip_direction(right_direction);
  }

  if (speed == 0) {
    left_direction = RELEASE;
    right_direction = RELEASE;
  }

  return { { left_direction, target_left_speed }, { right_direction, target_right_speed } };
}

void loop() {
  uint64_t curr_millis = millis();
  if (curr_millis - prev_millis > TIMEOUT_MILLIS) {
    rem_values = 0;
    update_motor(&motor_left, REST);
    update_motor(&motor_right, REST);
  }

  if (Serial1.available() > 0) {
    prev_millis = curr_millis;

    if (rem_values == 0) {
      rem_values = Serial1.read();

      // Protect against Arduino waking up in the middle of a message
      if (rem_values != NUM_VALUES) {
        rem_values = 0;
      }
    }

    while (Serial1.available() > 0 && rem_values > 0) {
      buf[NUM_VALUES - rem_values--] = Serial1.read();
    }

    // Full message received
    if (rem_values == 0) {
      if (prev_full != 0) {
        total_full += curr_millis - prev_full;
        num_full++;
      }

      // Debug
      if (num_full % 10 == 0) {
        Serial.print("Average (ms): ");
        Serial.println(total_full / num_full);
      }

      float steering = deserialize_val(buf[0]);
      float brake = deserialize_val(buf[1]);
      float throttle = deserialize_val(buf[2]);
      // Serial.print(steering);
      // Serial.print(",");
      // Serial.print(brake);
      // Serial.print(",");
      // Serial.println(throttle);

      // Convert throttle, brake, steering into motor values
      auto [l, r] = solve_motors(steering, throttle, brake);
      update_motor(&motor_left, l);
      update_motor(&motor_right, r);



      prev_full = curr_millis;
    }
  }
}
