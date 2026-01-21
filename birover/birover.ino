#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <utility>

#define L_INT_PIN 2
#define R_INT_PIN 3
#define LED_PIN 7

struct MotorState {
  uint8_t direction;
  uint16_t speed;
};

// Bluetooth State
constexpr uint64_t TIMEOUT_MILLIS = 250;
constexpr uint8_t NUM_VALUES = 3;
constexpr float SPIN_THRESHOLD = 1.0f;
constexpr MotorState REST = { RELEASE, 0 };

uint8_t buf[NUM_VALUES];

uint64_t prev_millis = 0;

// Motor state
constexpr uint16_t MAX_SPEED = 255;

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

void update_motor(Adafruit_DCMotor& motor, const MotorState& state) {
  motor.run(state.direction);
  motor.setSpeed(state.speed);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Motor setup
  AFMS.begin();

  update_motor(motor_left, REST);
  update_motor(motor_right, REST);

  // Photointerruptor setup
  pinMode(L_INT_PIN, INPUT_PULLUP);
  pinMode(R_INT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_INT_PIN), handleLInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_INT_PIN), handleRInterrupt, CHANGE);
}

float deserialize_val(const byte b) {
  return (((int)b) - 16) / 10.0f;
}

float normalize(const float value) {
  return (value + 1.0f) / 2.0f;  // [-1, 1] -> [0, 1]
}

uint8_t flip_direction(const uint8_t direction) {
  return ((direction - 1) ^ 1) + 1;  // flip 1 and 2
}

std::pair<MotorState, MotorState>
solve_motors(const float steering,
             const float throttle,
             const float brake) {
  const float velocity = (normalize(throttle) - normalize(brake)) * MAX_SPEED;
  const float speed = abs(velocity);

  if (speed == 0.0f) {
    return { REST, REST };
  }

  // "BACKWARD" is really forward...
  MotorState left_state = { BACKWARD, speed };
  MotorState right_state = { BACKWARD, speed };

  float steering_multiplier = 1.0f;
  bool spin = false;
  if (abs(steering) <= SPIN_THRESHOLD) {
    steering_multiplier = (SPIN_THRESHOLD - abs(steering)) / SPIN_THRESHOLD;
  } else {
    spin = true;
    steering_multiplier = (abs(steering) - SPIN_THRESHOLD) * (1 / (1 - SPIN_THRESHOLD));
  }
  if (steering < 0) {
    left_state.speed *= steering_multiplier;
    if (spin) left_state.direction = FORWARD;
  } else {
    right_state.speed *= steering_multiplier;
    if (spin) right_state.direction = FORWARD;
  }

  if (velocity < 0) {
    left_state.direction = flip_direction(left_state.direction);
    right_state.direction = flip_direction(right_state.direction);
  }

  return { left_state, right_state };
}

void loop() {
  const uint64_t curr_millis = millis();
  if (curr_millis - prev_millis > TIMEOUT_MILLIS) {
    update_motor(motor_left, REST);
    update_motor(motor_right, REST);
    digitalWrite(LED_PIN, HIGH);
  }

  if (Serial1.available() >= 1 + NUM_VALUES) {
    digitalWrite(LED_PIN, LOW);
    prev_millis = curr_millis;

    if (Serial1.read() != NUM_VALUES) return;

    Serial1.readBytes(buf, NUM_VALUES);

    // Full message received
    const float steering = deserialize_val(buf[0]);
    const float brake = deserialize_val(buf[1]);
    const float throttle = deserialize_val(buf[2]);

    // Convert throttle, brake, steering into motor values
    const auto [l, r] = solve_motors(steering, throttle, brake);
    update_motor(motor_left, l);
    update_motor(motor_right, r);
  }
}
