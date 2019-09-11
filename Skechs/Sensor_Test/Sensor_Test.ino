const int PIN_ANALOG_INPUT = 0;
#define R1 10000
void setup() {
  Serial.begin( 9600 );
}

void loop() {
  int i = analogRead( PIN_ANALOG_INPUT );
  float v = i * 5.0 / 1023.0;

  float o = (v * R1) /( 5 - v); 
  Serial.println( v );
  Serial.print( o );
  Serial.println(" ohm");
  delay( 1000 ); 
}
