void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
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
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("Turned LED on\n");
    } else if (c == '0') {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("Turned LED off\n");
    }
  }
}
