#include <Arduino.h>

#include <MyNetSerial.h>



union {//float,と１バイトデータ変換用
    float f;
    uint8_t u[4];
}Float;





#include <Wire.h>//lcd
#include <LiquidCrystal_I2C.h>//lcd
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#define   CONTRAST_PIN   9
#define   BACKLIGHT_PIN  7
#define   CONTRAST       125

#define TRIG_1ST 14 // HY-SRF05(超音波距離センサ)出力ピン
#define ECHO_1ST 15 // HY-SRF05(超音波距離センサ)入力ピン

#define GATELIGHT_SWITCH 7//ゲートの街灯スイッチ　アクティブロー
#define GATELIGHT_RELAY_IN 8//ゲートの街灯用リレー　OUTPUT

#define KYORISENSORSAMPLINGNUM 1//距離センサー、サンプリング数
#define KYORISENSORSAMPLINGMARGIN 50//距離センサーのサンプリングの間隔

float distance1st[KYORISENSORSAMPLINGNUM];
int sampleCounter = 0;
float longestdistance1st;

MyNetSerial mynet(MCID_GATE_UNO);
MySerialData mydata[4];


//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          setup()

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void setup() {
  
  
Serial.begin(MYNET_BPS);
  pinMode(TRIG_1ST,OUTPUT);
  pinMode(ECHO_1ST,INPUT);
  pinMode(13,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(GATELIGHT_SWITCH,INPUT_PULLUP);
  pinMode(GATELIGHT_RELAY_IN,OUTPUT);
 //lcdモニターーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
 
 lcd.begin(16,2);
 lcd.backlight();
  for(int i = 0; i< 3; i++){
   lcd.backlight();
   delay(100);
   lcd.noBacklight();
   delay(100);
 }
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
    //lcdモニタ終ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
    
  }//setup
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          loop()

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int loopcount = 0;
void loop() {

 // if (digitalRead(GATELIGHT_SWITCH)){
//digitalWrite(GATELIGHT_RELAY_IN , HIGH);
//  }else{
 //     digitalWrite(GATELIGHT_RELAY_IN , LOW);
//    }



  //距離センサー sample複数採取用
sampleCounter = 0;
longestdistance1st = 0;
//cpu温度、電圧
float temp ,Vcc;
temp = cpuTemp();
Vcc = cpuVcc();

if (Serial.available() > 0)
  {
    mynet.receiveData(mydata);
    displayMyData();
  }


  
  while(sampleCounter < KYORISENSORSAMPLINGNUM){   //サンプリング
  delay(KYORISENSORSAMPLINGMARGIN);
  // 超音波の出力終了
  digitalWrite(TRIG_1ST,LOW);
  delayMicroseconds(1);
  // 超音波を出力
  digitalWrite(TRIG_1ST,HIGH);
  delayMicroseconds(11);
  // 超音波を出力終了して、出力した超音波が返って来る時間を計測
  digitalWrite(TRIG_1ST,LOW);
  int t1 = pulseIn(ECHO_1ST,HIGH);
  // 計測した時間と音速から反射物までの距離を計算
  distance1st[sampleCounter] = t1*0.017;
  if(longestdistance1st < distance1st[sampleCounter]) longestdistance1st = distance1st[sampleCounter];
  sampleCounter++;
  }//while

  
  // 計算結果をシリアル通信で出力
  
 
if (loopcount == 10){
  mynet.sendData(MCID_MASTER_MEGA,SDPP_SENSOR_VALUE_REPORT,PID_GATE_DTC1,DTCT_DISTANCE_CM,longestdistance1st);
  
}
 lcd.setCursor(0,0);
 lcd.print(longestdistance1st,2);
  
  
lcd.print("cpu");
lcd.print(temp,1);
// lcd.setCursor(7,1);
// lcd.print(": vcc");
// lcd.print(Vcc,2);
 

  delay(100);
  loopcount++;
  if(loopcount == 20) loopcount = 0;
}//loop
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          MyMethods

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

float cpuTemp(){
  long sum = 0;
  adcSetup(0xC8);
  for(int n=0; n < 100; n++){
sum = sum + adc();
  }
  return (sum * 1.1 /102.4)- 342.5;//AT328pの内部基準電圧は1.1V
}

float cpuVcc(){
long sum=0;
adcSetup(0x4E);//01001110
//ADMUXのビットパターン
//REFS1:REFS0:ADLAR:-:MUX3:MUX2:MUX1:MUX0
//REFS1とREFS０は０は参照電圧の設定
//ADLARは、読み取った値を格納するレジスタADCLとADCHが左詰(1)か右詰(0)かを設定。aruduinoでは0固定
//MUX3~0はアナログピン選択のためのビット
for(int n=0; n < 10; n++){
  sum = sum + adc();
}
return (1.1 * 10240.0)/sum;
}

void adcSetup(byte data){
  ADMUX = data;
  ADCSRA |= (1 << ADSC);
  ADCSRA |= 0x07;
  delay(10);
}

unsigned int adc(){
  unsigned int dL,dH;
  //ADCSRAのビットパターン ※２５６０と同じ！（ADC Control and Status Register A
//ADEN:ADSC:ADATE:ADIF:ADIE:ADPS2:ADPS1:ADPS0
  ADCSRA |= ( 1 << ADSC);
  while(ADCSRA & (1 << ADSC) ){
  }
  dL = ADCL;
  dH = ADCH;
  return dL | (dH << 8);
}










void displayMyData()
{
  for (int i = 0; i < 4; i++)
  {
    if (mydata[i].isContained == IS_CONTAINED_NO)
      continue; //データが入ってない
    else if (mydata[i].isContained == IS_CONTAINED_ERROR)
    { //
      mydata[i].isContained = 0;
      continue;
    }
    else if (mydata[i].isContained == IS_CONTAINED_YES)
    {
 

      switch (mydata[i].receiverMCID)
      { //受信者MCIDで分岐
      case MCID_GATE_UNO:
        dataToMe(i);
        break;
      }
    }
  }
}

//argument :ID of the data array(mydata) to be displayed
void dataToMe(const int id)
{
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(mydata[id].senderMCID);
  lcd.print(mydata[id].receiverMCID);
  lcd.print(mydata[id].sdpp);
  lcd.print(mydata[id].pid);
  lcd.print(mydata[id].dtfm);
  lcd.print(mydata[id].dtct);
  lcd.print(mydata[id].value[0]);
  lcd.print(mydata[id].value[1]);
  lcd.print(mydata[id].value[2]);
  lcd.print(mydata[id].value[3]);
  if (mydata[id].value[0] == 0x01){
digitalWrite(GATELIGHT_RELAY_IN , HIGH);
  }
  else{
    digitalWrite(GATELIGHT_RELAY_IN , LOW);
  }
  mydata[id].isContained = IS_CONTAINED_NO;
}
