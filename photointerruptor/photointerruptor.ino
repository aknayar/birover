#define PH_INT_PIN A0
#define LED_PIN 7

void setup() {
  Serial.begin(4800);

  pinMode(LED_PIN, OUTPUT);
  pinMode(PH_INT_PIN, INPUT);
}

void loop() {
  int ph_int_state = analogRead(PH_INT_PIN);
  if (ph_int_state > 10) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  Serial.println(ph_int_state);
}