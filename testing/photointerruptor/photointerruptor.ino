#define PH_INT_PIN A0

void setup() {
  Serial.begin(4800);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PH_INT_PIN, INPUT);
}

void loop() {
  int ph_int_state = analogRead(PH_INT_PIN);
  if (ph_int_state > 10) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.println(ph_int_state);
}