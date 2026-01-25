#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <utility>

#define MICROS_S 1000000

#define NUM_MOTORS 2
#define LEFT 0
#define RIGHT 1

#define L_INT_PIN 2
#define R_INT_PIN 3
#define LED_PIN 7

#define FOR_MOTORS(i) for (int i = 0; i < NUM_MOTORS; i++)

uint8_t interrupt_pins[NUM_MOTORS] = { L_INT_PIN, R_INT_PIN };

struct MotorState {
  uint8_t direction;
  uint16_t speed;
};

struct MotorTarget {
  uint8_t direction;
  float rpm;
};

// Bluetooth State
constexpr uint64_t TIMEOUT_MILLIS = 500;
constexpr uint8_t NUM_VALUES = 2;
constexpr MotorState REST = { RELEASE, 0 };

#define NUM_BITS_DEFAULT 5
#define NUM_BITS_SPIN 3
const float VALUES_DEFAULT = pow(2, NUM_BITS_DEFAULT - 1) - 1;
const float VALUES_SPIN = pow(2, NUM_BITS_SPIN - 1) - 1;
constexpr uint8_t SHIFT_DEFAULT = 0;
constexpr uint8_t SHIFT_SPIN = NUM_BITS_DEFAULT;
constexpr uint8_t MASK_DEFAULT = (1 << NUM_BITS_DEFAULT) - 1;
constexpr uint8_t MASK_SPIN = (1 << NUM_BITS_SPIN) - 1;

uint8_t buf[NUM_VALUES];

uint64_t prev_millis = 0;

volatile float steering, spinning, velocity;

// Motor state
constexpr uint16_t MAX_SPEED = 255;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor motor_left = *AFMS.getMotor(1);
Adafruit_DCMotor motor_right = *AFMS.getMotor(2);
Adafruit_DCMotor motors[NUM_MOTORS] = { motor_left, motor_right };

// Control state
#define KP 15.0f
#define KD 0.5f
#define KI 25.0f

constexpr float TICKS_PER_ROT = 40.0f;
constexpr float MAX_RPM = 5.0f;
constexpr uint64_t INTERVAL_MICROS = 30000;
uint64_t last_interval = 0;

struct SpeedController {
  volatile int64_t ticks;
  int64_t ticks_tmp;

  uint64_t prev_time;

  float dt;
  float ticks_per_sec;
  float ticks_per_sec_f;
  float rpm;

  float prev_error;
  float error_int;

  uint8_t prev_direction;
};

SpeedController controllers[NUM_MOTORS] = { SpeedController{}, SpeedController{} };

template<int i>
void updatePosition() {
  controllers[i].ticks++;
}

void update_motor(Adafruit_DCMotor& motor, const MotorState& state) {
  motor.run(state.direction);
  motor.setSpeed(state.speed);
}

void setup() {
  Serial.begin(115200);
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

  attachInterrupt(digitalPinToInterrupt(L_INT_PIN), updatePosition<LEFT>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_INT_PIN), updatePosition<RIGHT>, CHANGE);

  // Control setup
  last_interval = millis();
}

float deserialize_value(const uint8_t b,
                        const float values,
                        const uint8_t shift,
                        const uint8_t mask) {

  return ((((int)b >> shift) & mask) / values) - 1.0f;
}

uint8_t flip_direction(const uint8_t direction) {
  return ((direction - 1) ^ 1) + 1;  // flip 1 and 2
}

MotorTarget translate_input(const int idx,
                            const float steering,
                            const float spinning,
                            const float velocity_raw) {
  const bool spin = spinning != 0;
  const float velocity = (spin ? spinning : velocity_raw) * MAX_RPM;
  const float speed = abs(velocity);

  // "BACKWARD" is really forward...
  MotorTarget target = { BACKWARD, speed };

  float steering_multiplier = 1.0f;
  if (!spin) steering_multiplier = 1.0f - abs(steering);

  if (spin) {
    if (spinning < 0 && idx == LEFT) {
      target.direction = FORWARD;
    }
    if (spinning > 0 && idx == RIGHT) {
      target.direction = FORWARD;
    }
  } else {
    if (steering < 0 && idx == LEFT) {
      target.rpm *= steering_multiplier;
    }
    if (steering > 0 && idx == RIGHT) {
      target.rpm *= steering_multiplier;
    }
  }

  if (!spin && velocity < 0) {
    target.direction = flip_direction(target.direction);
  }

  if (target.rpm == 0) {
    target.direction = controllers[idx].prev_direction;
  }

  controllers[idx].prev_direction = target.direction;

  return target;
}

MotorState solve_motor(int idx, MotorTarget target) {
  SpeedController& controller = controllers[idx];

  float error = target.rpm - controller.rpm;

  float error_der = (error - controller.prev_error) / controller.dt;

  controller.prev_error = error;

  controller.error_int = controller.error_int + error * (controller.dt);

  float output = KP * error + KD * error_der + KI * controller.error_int;

  uint16_t speed = min(max(0.0f, output), MAX_SPEED);


  // Serial.print(speed);
  // Serial.print(" ");
  // Serial.print(speed < 30);
  // Serial.print(" ");
  // Serial.print(controller.rpm < 0.5);
  // Serial.print(" ");
  // Serial.println(controller.rpm);
  if (speed < 30) {
    speed = 0;
    target.direction = RELEASE;
  }

  return { target.direction, (uint8_t)speed };
}

void loop() {
  const uint64_t curr_micros = micros();
  const uint64_t curr_millis = curr_micros / 1000;

  // Handle possible Bluetooth disconnect
  if (curr_millis - prev_millis > TIMEOUT_MILLIS) {
    update_motor(motor_left, REST);
    update_motor(motor_right, REST);
    digitalWrite(LED_PIN, HIGH);
  }

  // Update motor positions
  uint64_t diff_micros = curr_micros - last_interval;
  if (diff_micros > INTERVAL_MICROS) {

    // Locally update ticks
    noInterrupts();
    for (int i = 0; i < NUM_MOTORS; i++) {
      controllers[i].ticks_tmp = controllers[i].ticks;
      controllers[i].ticks = 0;
    }
    interrupts();

    // Update velocities
    FOR_MOTORS(i) {
      SpeedController& controller = controllers[i];

      float new_ticks_per_sec = controller.ticks_tmp / ((curr_micros - last_interval) / (float)MICROS_S);
      controller.ticks_per_sec_f = 0.6 * controller.ticks_per_sec_f + 0.2 * new_ticks_per_sec + 0.2 * controller.ticks_per_sec;
      controller.ticks_per_sec = new_ticks_per_sec;
      controller.rpm = controller.ticks_per_sec_f / TICKS_PER_ROT;
      controller.dt = diff_micros / (float)MICROS_S;
    }

    last_interval = curr_micros;
  }

  // Process incoming Bluetooth message
  if (Serial1.available() >= 1 + NUM_VALUES) {
    digitalWrite(LED_PIN, LOW);
    prev_millis = curr_millis;

    if (Serial1.read() == NUM_VALUES) {
      Serial1.readBytes(buf, NUM_VALUES);

      // Full message received
      steering = deserialize_value(buf[0], VALUES_DEFAULT, SHIFT_DEFAULT, MASK_DEFAULT);
      spinning = deserialize_value(buf[0], VALUES_SPIN, SHIFT_SPIN, MASK_SPIN);
      velocity = deserialize_value(buf[1], VALUES_DEFAULT, SHIFT_DEFAULT, MASK_DEFAULT);

      // Serial.print(0);
      // Serial.print(" ");
      // Serial.print(5);
      // Serial.print(" ");
      // Serial.print(controllers[0].rpm);
      // Serial.print(" ");
      // Serial.println(controllers[1].rpm);
    };
  }

  // Convert throttle, brake, steering into motor values
  FOR_MOTORS(i) {
    const MotorTarget target = translate_input(i, steering, spinning, velocity);
    const MotorState state = solve_motor(i, target);
    update_motor(motors[i], state);
  }
}
