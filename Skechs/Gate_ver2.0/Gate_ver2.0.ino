//ESP32
#include "MyToString.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <MyNetSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "HardwareSerial.h"
//DS3231 リアルタイムクロックモジュール
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>


/*****************************************************************/
/*                        PINS                                    */
/*****************************************************************/
const uint8_t GATELED_SW_PIN = 13; //ゲートの街灯用リレー　OUTPUT
const uint8_t FAN_PWM_PIN = 15;
const uint8_t RX2 = 16;
const uint8_t TX2 = 17;
const uint8_t GATELIGHT_RELAY_PIN = 27; //ゲートの街灯用リレー　OUTPUT
const uint8_t RX1 = 32;
const uint8_t TX1 = 33;
const uint8_t DS18B20_SINGLE_BUS = 14           ;
const uint8_t PHOTOREGISTER_VOLTAGE_PIN = 36;//フォトレジスタ電圧測定ピン(VP)
/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/
const uint16_t GATELED_ON_THRESHOLD = 5000;//ohm
const uint16_t GATELED_OFF_THRESHOLD = 2000;//ohm

const uint8_t EEP_LIGHTSOFFTIME_ADRS = 0;//0~1
const uint8_t EEP_GATELEDSW_ADRS = 2;//2
const uint8_t TEMPERATURE_PRECISION = 32;//32bit
//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼------------------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼-----------------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼-------------------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼----------------------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼----------------------------------
MyNetSerial mynet(MCID_GATE_1);
HardwareSerial MySerial1(1);//UART1 ( RX=GPIO32, TX=GPIO33 )
HardwareSerial MySerial2(2);//UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
RTC_DS3231 rtc;
OneWire oneWire(DS18B20_SINGLE_BUS);// Setup a oneWire instance(not just Maxim/Dallas temperature ICs)
DallasTemperature dallastemp(&oneWire);// Pass our oneWire reference to Dallas Temperature.
// arrays to hold device addresses
DeviceAddress outsideThermometer, insideThermometer;
int16_t lightsOffTime = -1;
boolean TouchedT3 = false;

float insidetemp , outsidetemp , Vcc;
float photoR = 0;
bool photoswitch = false;//フォトレジスタによるスイッチ
bool bGateLedSw = false;//d
bool ledAlarmOff = false;
bool ledState = false;

bool timeOffFlag = false;
DateTime dtime, ntime; //dtime:countsecが０の時の時間　ntime 今の時間
uint32_t countsec = 0;//時計用
uint32_t lightsOffChangedStamp = 0;
volatile SemaphoreHandle_t timerSemaphore;//タイマー計測用
//----------------------------BLE------------------------------------------------------
//------▼▼▼-------------------------------------------------------------------------------------------------
//--------▼------▼--▼▼---------▼------------------------------------------------------------------------------
//--------▼------▼▼----▼---▼▼▼▼▼▼------▼▼▼-----▼--▼▼-----▼--▼▼-----▼-----▼---▼▼▼▼-------▼---------
//--------▼------▼------▼-------▼--------▼-----▼----▼▼----▼---▼▼----▼---▼-----▼---▼-----▼--▼▼▼▼▼------
//--------▼------▼------▼-------▼--------▼▼▼▼▼----▼----------▼-----------▼-----▼---▼▼▼▼-------▼---------
//--------▼------▼------▼-------▼--------▼-----------▼----------▼-----------▼-----▼---▼------------▼---------
//------▼▼▼----.▼-----▼-------▼----------▼▼▼------▼----------▼-------------▼▼▼▼---▼------------▼---------

void IRAM_ATTR onTimer() {
  countsec++;
  if (dtime.unixtime()) {
    ntime = DateTime(dtime.unixtime() + (0, 0, countsec));
  }

  if ( ntime.unixtime() != 0 && ntime.second() == 0 && ntime.minute() == (lightsOffTime % 100) && ntime.hour() == (lightsOffTime / 100) )
    timeOffFlag = true;

  //loopを確認できるセマフォを与える(安全にdigitalRead/writeするため)
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//                ▼▼▼       ▼▼        ▼             ▼    ▼    ▼▼▼▼
//               ▼          ▼   ▼    ▼▼▼▼          ▼    ▼    ▼     ▼
//                 ▼▼      ▼▼▼▼      ▼             ▼    ▼    ▼▼▼▼
//                    ▼     ▼           ▼             ▼   ▼     ▼
//-----------------▼▼--------▼▼--------▼--------------▼▼-------▼-----------------------------------------------
//-------------------------------------------------------------------------------------------------------
void setup() {

  // 使用するサイズを宣言する
  EEPROM.begin(10);
  Serial.begin(115200);
   MySerial1.begin(MyNetSerial::BPS , SERIAL_8N1, RX1, TX1); //UART1 ( RX=GPIO32, TX=GPIO33 )
  mynet.beginPort(MySerial1, MCID_GATE_SENSORS);
  MySerial2.begin(MyNetSerial::BPS , SERIAL_8N1, RX2, TX2); //UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
  mynet.beginPort(MySerial2, MCID_MEGA_SOC);

  pinMode(10, OUTPUT);
  pinMode(GATELIGHT_RELAY_PIN, OUTPUT);
  pinMode(GATELED_SW_PIN, OUTPUT);
  ledcSetup(0, 100000, 8); //channel,frequency,解像度(8bit)
  ledcAttachPin(FAN_PWM_PIN, 0); //connect pin to channel

  //----------------------------------------------------------------------------------
  dallastemp.begin();
 // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(dallastemp.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (dallastemp.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

    if (!dallastemp.getAddress(insideThermometer, 1)) Serial.println("Unable to find address for Device 1");
  if (!dallastemp.getAddress(outsideThermometer, 0)) Serial.println("Unable to find address for Device 0");
  // set the resolution to 9 bit per device
  dallastemp.setResolution(insideThermometer, TEMPERATURE_PRECISION);
  dallastemp.setResolution(outsideThermometer, TEMPERATURE_PRECISION);
  //----------------------------------------------------------------------------------
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  delay(100);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.clearDisplay();
  display.println("Starting work!");
  display.display();
  //----------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------
  //セマフォ管理
  timerSemaphore = xSemaphoreCreateBinary();
  //プリスケーラを80分周器にする(ESP Reference参照)
  hw_timer_t * timer = timerBegin(0, 80, true);
  //タイムアウトした時に行う処理(関数)を定義
  timerAttachInterrupt(timer, &onTimer, true);
  //何秒毎に行うか定義
  timerAlarmWrite(timer, 1000000, true);
  //タイマースタート
  timerAlarmEnable(timer);
  //----------------------------------------------------------------------------------
  //DS3231 RTC リアルタイムクロックモジュール----------------------------------------------------------------
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //rtc.adjust(  DateTime(2019, 6, 5, 16, 25, 20)  );
  dtime = rtc.now();
  //-------------------------------------------------------------------------------------------------------
  EEPROM.get<int16_t>(EEP_LIGHTSOFFTIME_ADRS, lightsOffTime);
}//setup
//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼-------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼-------------------------------------
int loopcount = 0;

void loop() {

  //  Serial.println(digitalRead(GATEINFRA1_IN_PIN));
  // Serial.println(digitalRead(GATEINFRA2_IN_PIN));



  //Serial Process----------------------------------
  if (MySerial1.available())//シリアルデータが届いていれば、mydata[4]に読み込んで、内容を表示、もしくは命令実行
  {
    mynet.receive(MySerial1);
    processMyData();
  }
  if (MySerial2.available())//シリアルデータが届いていれば、mydata[4]に読み込んで、内容を表示、もしくは命令実行
  {
    mynet.receive(MySerial2);
    processMyData();
  }
  //Serial Process---end----------------------------

  //loopcount process-------------------------------
  if (loopcount % 90 == 5) {
    photo (); //フォトトランジスタチェック

    mynet.send(MCID_MEGA_SOC, SDPP_TIME_REQUEST, PID_LIGHTSOFFTIME); //LIGHTSOFF時間リクエスト
  }
  if (loopcount % 50 == 7 && dtime.unixtime() < 1000000000) {
    mynet.send(MCID_MEGA_SOC, SDPP_TIME_REQUEST, PID_TIME); //時間リクエスト
  }
  if (loopcount % 50 == 14 && lightsOffTime == -1) {
    mynet.send(MCID_MEGA_SOC, SDPP_TIME_REQUEST, PID_LIGHTSOFFTIME); //LIGHTSOFF時間リクエスト
  }

  if (loopcount % 200 == 17 && lightsOffTime == -1) {
    if (ledState) mynet.send(MCID_MEGA_SOC, SDPP_ON_REPORT, PID_GATE_LED);
    else mynet.send(MCID_MEGA_SOC, SDPP_OFF_REPORT, PID_GATE_LED);
  }

  if (loopcount % 100 == 34) {
    dallastemp.requestTemperatures(); // Send the command to get temperatures
    insidetemp = dallastemp.getTempC(insideThermometer);
    outsidetemp = dallastemp.getTempC(outsideThermometer);
    int8_t t = (int8_t)insidetemp - 25;
    if (t < 0) ledcWrite(0, 80);
    else if (t >= 10) ledcWrite(0, 255);
    else if (t >= 7) ledcWrite(0, 200);
    else if (t >= 5) ledcWrite(0, 160);
    else if (t >= 3) ledcWrite(0, 120);
    else ledcWrite(0, 100);

    mynet.sendFloat(insidetemp , MCID_MEGA_SOC, SDPP_SENSOR_VALUE_REPORT, PID_GATE_1_INSIDE_TEMP);
  }

  if (loopcount % 100 ==84) {
    mynet.sendFloat(outsidetemp , MCID_MEGA_SOC, SDPP_SENSOR_VALUE_REPORT, PID_GATE_1_OUTSIDE_TEMP);
  }
  //loopcount process----end------------------------

  //Flag process------------------------------------
  if (timeOffFlag) {
    mynet.send(MCID_MEGA_SOC, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
    bGateLedSw = false;
    ledAlarmOff = true;
    timeOffFlag = false;
  }
  //Flag process-----end----------------------------

  //Stamp process-------------------------------------
  if (lightsOffChangedStamp != 0 && (lightsOffChangedStamp + 30) < countsec) {//消灯タイマーの時間を変更して30秒以上経ったらEEPROMに保存
    EEPROM.write(EEP_LIGHTSOFFTIME_ADRS, lowByte(lightsOffTime));
    EEPROM.write(EEP_LIGHTSOFFTIME_ADRS + 1, highByte(lightsOffTime));
    EEPROM.commit();
    lightsOffChangedStamp = 0;
  }
  //Stamp process---end-------------------------------

  dispStatus();
  delay(20);
  loopcount++;
  if (loopcount == 1000) loopcount = 0;
}//loop
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          MyMethods

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

void dispStatus() {
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0, 0);
  if (ntime.unixtime() != 0) {
    display.print(ntime.year());
    display.print('/');
    display.print(ntime.month());
    display.print('/');
    display.print(ntime.day());
    display.print("  ");
    display.print(ntime.hour() / 10); display.print(ntime.hour() % 10);
    display.print(':');
    display.print(ntime.minute() / 10); display.print(ntime.minute() % 10);
    display.print(':');
    display.print(ntime.second() / 10); display.print(ntime.second() % 10);
  } else {
    display.print("time unknown");
  }
  display.setCursor(0, 8);
  display.print("gateSW:");
  display.print(bGateLedSw);
  display.setCursor(0, 16);
  display.print("LightsOffTime  ");
  display.print(MyToString::int16ToTimeString(lightsOffTime) );
  display.setCursor(0, 24);
  display.setTextSize(2);
  display.print(photoR);
  display.print("R ");
  display.setTextSize(1);
  if (photoswitch) {
    display.print("on");
  } else {
    display.print("off");
  }
  display.setCursor(0, 40);
  display.print("ledAlarmOff:");
  display.print(ledAlarmOff);
  display.setCursor(0, 48);
  display.print("in:");
  display.print(insidetemp);
  display.print("'C out:");
  display.print(outsidetemp);
  display.setCursor(106, 56);
  display.print(loopcount);
  display.display();

}

void photo() {
  //bool photoswitch = false;//フォトレジスタによるスイッチ
  //bool bGateLedSw = false;//暗い時にGateLedをつけるかどうかのスイッチ
  //bool ledAlarmOff = false;//時間が来てアラームがオフされた。（朝にリセット）
  //bool ledState = false;//GateLedが付いているかどうか
  int i = analogRead( PHOTOREGISTER_VOLTAGE_PIN );
  float v = i * 3.3 / 4096.0;
  photoR = (v * 10000) / ( 3.3 - v);

  if ((long)photoR > GATELED_ON_THRESHOLD && !ledAlarmOff) {//暗くて、ledAlarmOffがoffの時
    photoswitch = true;
    if (!bGateLedSw) {
      bGateLedSw = true;
      mynet.send(MCID_MEGA_SOC, SDPP_ON_COMMAND, PID_GATE_LED_SW);//暗くなった時に勝手にスイッチつける
    }
  } else if ((long)photoR < GATELED_OFF_THRESHOLD) { //明るい時
    photoswitch = false;
    ledAlarmOff = false;
  }

  mynet.sendFloat(photoR , MCID_MEGA_SOC, SDPP_SENSOR_VALUE_REPORT, PID_GATE_PHOTO);

  if (ledAlarmOff) mynet.send(MCID_MEGA_SOC, SDPP_ON_REPORT , PID_GATE_LED_ALARM_OFF);
  else mynet.send(MCID_MEGA_SOC, SDPP_OFF_REPORT , PID_GATE_LED_ALARM_OFF);

  gateLED();
}

void gateLED() {
  if (photoswitch && bGateLedSw)
    gateLedOn();
  else
    gateledOff();
}


void gateLedOn() {
  digitalWrite(GATELIGHT_RELAY_PIN , HIGH);
  ledState = true;
  mynet.send(MCID_MEGA_SOC, SDPP_ON_REPORT, PID_GATE_LED);
}
void gateledOff() {
  digitalWrite(GATELIGHT_RELAY_PIN , LOW);
  ledState = false;
  mynet.send(MCID_MEGA_SOC, SDPP_OFF_REPORT, PID_GATE_LED);
}


//-------------Methods related to MyNet------------------------------------------------------------------------------



void processMyData()
{
  for (int i = 0; i < 4; i++)
  {
    SerialData* sd = mynet.containedPick();
    while ( sd != NULL) {

      switch (sd->receiver())
      { //受信者MCIDで分岐
        case MCID_GATE_1:
          dataToMe(sd);
          break;
      }
      sd = mynet.containedPick();
    }
  }
}//End Of processMyData()


//argument :ID of the data array(mydata) to be displayed
void dataToMe(SerialData * sd)
{
  Serial.println("check0");
  Serial.println(sd->pidchar());
  delay(10);

  switch (sd->pid()) {

    case PID_GATE_LED_SW: {
        switch (sd->sdpp()) {
          case SDPP_ON_COMMAND : {}
          case SDPP_CONFIRM_ON : {
              bGateLedSw = true;
              ledAlarmOff = false;
            }
            break;
          case SDPP_OFF_COMMAND : {
              ledAlarmOff = true;
            }//breakせずにスルーさせる
          case SDPP_CONFIRM_OFF : {
              bGateLedSw = false;
            }
            break;
        }//switch (sd->sdpp)
      }//case PID_GATE_LED_SW:
      break;

    case PID_TIME: {
        switch (sd->sdpp()) {
          case SDPP_TIME_REPORT : {
              dtime = DateTime(sd->uint32() - countsec);
            }//case SDPP_TIME_REPORT :
            break;
        }//switch
      }//case PID_TIME:
      break;

    case PID_LIGHTSOFFTIME: {
        lightsOffTime = sd->int16();
        lightsOffChangedStamp = countsec;
      }//case PID_LIGHTSOFFTIME:
      break;
  }//switch (sd->pid)


}//End Of dataToMe()







//float cpuTemp() {
//  long sum = 0;
//  adcSetup(0xC8);
//  for (int n = 0; n < 100; n++) {
//    sum = sum + adc();
//  }
//  return (sum * 1.1 / 102.4) - 342.5; //AT328pの内部基準電圧は1.1V
//}
//
//float cpuVcc() {
//  long sum = 0;
//  adcSetup(0x4E);//01001110
//  //ADMUXのビットパターン
//  //REFS1:REFS0:ADLAR:-:MUX3:MUX2:MUX1:MUX0
//  //REFS1とREFS０は０は参照電圧の設定
//  //ADLARは、読み取った値を格納するレジスタADCLとADCHが左詰(1)か右詰(0)かを設定。aruduinoでは0固定
//  //MUX3~0はアナログピン選択のためのビット
//  for (int n = 0; n < 10; n++) {
//    sum = sum + adc();
//  }
//  return (1.1 * 10240.0) / sum;
//}
//
//void adcSetup(byte data) {
//  ADMUX = data;
//  ADCSRA |= (1 << ADSC);
//  ADCSRA |= 0x07;
//  delay(10);
//}
//
//unsigned int adc() {
//  unsigned int dL, dH;
//  //ADCSRAのビットパターン ※２５６０と同じ！（ADC Control and Status Register A
//  //ADEN:ADSC:ADATE:ADIF:ADIE:ADPS2:ADPS1:ADPS0
//  ADCSRA |= ( 1 << ADSC);
//  while (ADCSRA & (1 << ADSC) ) {
//  }
//  dL = ADCL;
//  dH = ADCH;
//  return dL | (dH << 8);
//}
