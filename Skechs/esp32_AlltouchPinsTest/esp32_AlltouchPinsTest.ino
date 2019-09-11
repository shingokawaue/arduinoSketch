int touched_value[10];

void setup() {
    Serial.begin(9600);
}

int i;

void loop(){

touched_value[0] = touchRead(T0);
touched_value[1] = touchRead(T1);
touched_value[2] = touchRead(T2);
touched_value[3] = touchRead(T3);
touched_value[4] = touchRead(T4);
touched_value[5] = touchRead(T5);
touched_value[6] = touchRead(T6);
touched_value[7] = touchRead(T7);
touched_value[8] = touchRead(T8);
touched_value[9] = touchRead(T9);


for(i = 0; i < 9; i++ ){
String s;

s = String(touched_value[i]);
  if (s.length() == 1){
    s = "0" + s;
  }
  
  Serial.print(" T" + String(i) + ":" + s);
}

Serial.println(" T9:" + String(touched_value[9]));



    delay(50);
}
