#define PIN 2
#define DTIME 500

void setup() {
  pinMode(PIN, OUTPUT);
}

void loop() {
  digitalWrite(PIN, HIGH);
  delay(DTIME);
  digitalWrite(PIN, LOW);
  delay(DTIME);
}
