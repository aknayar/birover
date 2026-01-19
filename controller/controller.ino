#define LED_PIN 7

#define TIMEOUT_MILLIS 1000
#define NUM_VALUES 3

byte read_chars = 0;
byte rem_chars = 0;
char buf[NUM_VALUES];

uint64_t prev_millis = 0;
uint64_t prev_full = 0;

uint64_t total_full = 0;
uint64_t num_full = 0;


bool initialized = false;


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(LED_PIN, OUTPUT);
}


float deserialize_val(byte b) {
  return (((int)b) - 128) / 100.0f;
}


void loop() {
  uint64_t curr_millis = millis();
  if (curr_millis - prev_millis > TIMEOUT_MILLIS) {
    // initialized = false;
    rem_chars = 0;
    read_chars = 0;
  }

  // if (Serial1.available() < rem_chars) {
  //   return;
  // }

  if (Serial1.available() > 0) {
    // Serial.print("in buf: ");
    // Serial.println(Serial1.available());
    prev_millis = curr_millis;

    // if (!initialized) {
    //   if (Serial1.read() != 0) {
    //     return;
    //   }

    //   Serial.println("got zero");

    //   // Clear the buffer
    //   while (Serial1.available()) {
    //     Serial1.read();
    //   }

    //   initialized = true;
    //   Serial1.write(1);
    //   Serial.println("sent 1");
    //   return;
    // }

    // Serial.print("rem_chars=");
    // Serial.println(rem_chars);

    if (rem_chars == 0) {
      rem_chars = Serial1.read();

      // Protect against Arduino waking up in the middle of a message
      if (rem_chars != NUM_VALUES) {
        rem_chars = 0;
      }
      // Serial.print("just read ");
      // Serial.println(rem_chars);
      read_chars = 0;
      // Serial.print("I need ");
      // Serial.println(rem_chars);
    }

    while (Serial1.available() > 0 && rem_chars > 0) {
      buf[read_chars++] = Serial1.read();
      // Serial.print("happened, read ");
      // Serial.println(static_cast<byte>(buf[read_chars - 1]));
      rem_chars--;
    }

    // Full messages received!
    if (rem_chars == 0) {
      // Serial.print("took: ");
      // Serial.print(curr_millis - prev_full);
      // Serial.print("got: ");
      // Serial.print(buf);
      // Serial.println("");
      if (prev_full != 0) {
        total_full += curr_millis - prev_full;
        num_full++;
      }

      if (num_full % 10 == 0) {
        Serial.print("avging (ms): ");
        Serial.println(total_full / num_full);


      }
      float steering = deserialize_val(buf[0]);
      float brake = deserialize_val(buf[1]);
      float throttle = deserialize_val(buf[2]);
      Serial.print(steering);
      Serial.print(",");
      Serial.print(brake);
      Serial.print(",");
      Serial.println(throttle);

      prev_full = curr_millis;
    }
  }
}
