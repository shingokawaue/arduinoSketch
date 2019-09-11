#include <SoftwareSerial.h>
SoftwareSerial gateSerial(2, 3); // RX, TX

const float GATE_THRESHOLD = 350.0f;//門の距離センサーの閾値

float previousGateSensor1 = 4.50;//門の距離センサー１の前回の値
float previousGateSensor2 = 4.50;//門の距離センサー2の前回の値
void setup() {
  Serial.begin(9600); // ハードウェアシリアルを準備
  gateSerial.begin(9600); // ソフトウェアシリアルの初期化
}

void loop() {
  //if (gateSerial.available()) Serial.write(gateSerial.read());
  //if (Serial.available()) gateSerial.write(Serial.read());
  
if(gateSerial.available() > 2){
while(gateSerial.available() > 0){
gateSerial.read();
}
}



 if(gateSerial.available() == 2){
Serial.print(gateSerial.read());
Serial.print(" : ");
  Serial.print(gateSerial.read());
Serial.println();
 }
 


}
