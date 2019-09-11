#include <SoftwareSerial.h>
SoftwareSerial gateSerial(8, 9); // RX, TXconst int TRIG_1ST = 2; // HY-SRF05(超音波距離センサ)出力ピン
const int TRIG_1ST = 2; // HY-SRF05(超音波距離センサ)出力ピン
const int ECHO_1ST = 3; // HY-SRF05(超音波距離センサ)入力ピン
const int TRIG_2ND = 4; // HY-SRF05(超音波距離センサ)出力ピン
const int ECHO_2ND = 5; // HY-SRF05(超音波距離センサ)入力ピン
const int KYORISENSORSAMPLINGNUM = 1;//距離センサー、サンプリング数
const int KYORISENSORSAMPLINGMARGIN = 50;//距離センサーのサンプリングの間隔


float distance1st[KYORISENSORSAMPLINGNUM];
float distance2nd[KYORISENSORSAMPLINGNUM];
int sampleCounter = 0;
float longestdistance1st;
float longestdistance2nd;
void setup() {
  Serial.begin(9600);
  gateSerial.begin(9600);
  pinMode(TRIG_1ST,OUTPUT);
  pinMode(ECHO_1ST,INPUT);
  pinMode(TRIG_2ND,OUTPUT);
  pinMode(ECHO_2ND,INPUT);
  }

void loop() {
sampleCounter = 0;
longestdistance1st = 0;
longestdistance2nd = 0;


  while(sampleCounter < KYORISENSORSAMPLINGNUM){   //サンプリング
  delay(KYORISENSORSAMPLINGMARGIN);
  // 超音波の出力終了
  digitalWrite(TRIG_1ST,LOW);
  digitalWrite(TRIG_2ND,LOW);
  delayMicroseconds(1);
  // 超音波を出力
  digitalWrite(TRIG_1ST,HIGH);
  digitalWrite(TRIG_2ND,HIGH);
  delayMicroseconds(11);
  // 超音波を出力終了して、出力した超音波が返って来る時間を計測
  digitalWrite(TRIG_1ST,LOW);
  int t1 = pulseIn(ECHO_1ST,HIGH);
  digitalWrite(TRIG_2ND,LOW);
  int t2 = pulseIn(ECHO_2ND,HIGH);
  // 計測した時間と音速から反射物までの距離を計算
  distance1st[sampleCounter] = t1*0.017;
  distance2nd[sampleCounter] = t2*0.017;
  if(longestdistance1st < distance1st[sampleCounter]) longestdistance1st = distance1st[sampleCounter];
  if(longestdistance2nd < distance2nd[sampleCounter]) longestdistance2nd = distance2nd[sampleCounter];
  sampleCounter++;
  }

  
  // 計算結果をシリアル通信で出力
 //float data[1];
// float tf;
// data[0] = longestdistance1st;

// byte b[4];
// memcpy(b , &longestdistance1st,4);
  //Serial.write((byte*)data, sizeof(float) );
 // Serial.write(b, sizeof(float) );
// memcpy(&tf,b,4);
  Serial.print((byte)(longestdistance1st / 2));
  gateSerial.write((byte)(longestdistance1st / 2));
 //Serial.print(tf);
//  data[0] = longestdistance2nd;
 Serial.print(" : ");
  Serial.println((byte)(longestdistance2nd / 2));
  gateSerial.write((byte)(longestdistance2nd / 2));
 // Serial.println();
}
