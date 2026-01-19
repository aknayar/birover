#define LED_PIN 7

void setup() {
  Serial.begin(9600);
  Serial1.begin(38400);

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (Serial1.available() > 0) {
    Serial.print(Serial1.available());
    Serial.print(" bytes are available\n");

    byte c = Serial1.read();
    Serial.print("Got {");
    Serial.print((int)c);
    Serial.print("}\n");

    if (c == '1') {
      digitalWrite(LED_PIN, HIGH);
      Serial.print("Turned LED on\n");
    } else if (c == '0') {
      digitalWrite(LED_PIN, LOW);
      Serial.print("Turned LED off\n");
    }
  }
}
