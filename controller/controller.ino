#define TIMEOUT_MILLIS 1000
#define NUM_VALUES 3

uint8_t rem_values = 0;
uint8_t buf[NUM_VALUES];

uint64_t prev_millis = 0;

uint64_t prev_full = 0;
uint64_t total_full = 0;
uint64_t num_full = 0;


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}


float deserialize_val(byte b) {
  return (((int)b) - 128) / 100.0f;
}


void loop() {
  uint64_t curr_millis = millis();
  if (curr_millis - prev_millis > TIMEOUT_MILLIS) {
    rem_values = 0;
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

      prev_full = curr_millis;
    }
  }
}
