int threshold = 20;
int touched_value = 0;
int LED_PIN = 4;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  touched_value = touchRead(T3);
    Serial.println("T3 : " + String(touchRead(T3)));
  if (touched_value < threshold) {
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(LED_PIN, LOW);
  }
  delay(50);
}
