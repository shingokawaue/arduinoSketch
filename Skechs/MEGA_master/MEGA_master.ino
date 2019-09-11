#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoSTL.h>
#include <cassert>
#include <MyNetSerial.h>
#include "MyToString.h"
#include "Arduino.h"
#include <BME280I2C.h>
//-------------------------------------------------------------------------------------------------------
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//------EEPROM-------------------------------------------------------------------------------------------------
#include "MyEEPROM.h"
#include "Int16TimeCalc.h"
#include <avr/io.h>

#include <util/delay.h>
#include <avr/interrupt.h>
#include <TM1637Display.h>//4degit display 
//DS3231 リアルタイムクロックモジュール
#include <RTClib.h>

#include <OneWire.h>
#include <DallasTemperature.h>
//-------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include "Ucglib.h"
//-------------------------------------------------------------------------------------------------------
#include "TM1637.h"


//-------------------------------------------------------------------------------------------------------
//---▼--------▼--------------------▼---▼------------▼---------------------▼▼---▼---------------------------------
//---▼-▼---▼-▼----▼----▼-----▼▼▼▼▼▼-----------▼------▼▼▼----------▼----------▼--▼▼--------▼▼-------------
//---▼---------▼------▼------------▼--▼-------▼---▼----▼▼▼▼▼---------▼-----▼---▼------▼---▼▼▼▼▼-----------
//---▼---------▼-----▼--------▼▼▼▼▼▼▼-----▼---▼-----▼------▼--------▼-----▼---▼-----▼----▼-----------------
//---▼---------▼----▼--------------▼-▼---------▼▼▼-------▼▼▼----------▼-----▼---▼------▼-----▼▼▼------------
#define CHATTERING_AVOIDANCE_TIME 30 //割り込みハンドラ内でdelay()を使用したチャタリング回避時間　ミリ秒
#define CHATTERING_AVOIDANCE_TIME_TC1 500 //timerCounter1を使ったチャタリング回避時間　ミリ秒　（チャタリングがひどい時用）
#define RENDA_TIME 300 //押しっぱなし連打
#define F_CPU 16000000 //クロック周波数　（ここで設定できるわけではない、delay()で使用
#define GATE_THRESHOLD 50.0f //門の距離センサーの閾値
#define TRACE_X 13
#define TRACE_Y 1
#define GATE_LED_SW_PORT PB6 //pin12
#define GATE_LED_SW_PCINT PCINT6
/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/

const uint8_t TEMPERATURE_PRECISION = 9;//精度

const int8_t ON_STATE = 1;
const int8_t OFF_STATE = 0;
const int8_t UNKNOWN_STATE = -1;
const uint8_t LEDCON1_NUMOF_MAX72XX = 2;//7segLEDモジュールmax72xxデバイスの接続数
const uint8_t EEP_LIGHTSOFFTIME_ADRS = 0;//0~1
const uint8_t EEP_LIGHTSOFFTIME_UT_ADRS = 2;//2~5
const uint8_t EEP_GATELEDSW_ADRS = 6;//6
const uint8_t EEP_GATELEDSW_UT_ADRS = 7;//7~10
/*****************************************************************/
/*          ▼▼▼▼     ▼
            ▼     ▼         ▼  ▼▼
            ▼▼▼▼     ▼    ▼▼   ▼
            ▼          ▼    ▼     ▼
            ▼          ▼    ▼     ▼
*/
/*****************************************************************/
//Arduino Mega: 50(MISO)、51(MOSI)、52(SCK)、53(SS)
const uint8_t SPI_RESET = PC1;//
//const uint8_t SPI_MISO = PB3;//　データ入力 master in slave out
//const uint8_t SPI_MOSI = PB2;//　データ出力 master out slave in
//const uint8_t SPI_SCK = PB1;//　シリアルクロック
//const uint8_t SPI_SS = 53;//slave select
const uint8_t GATE_LED_STATE_PIN = 10;
const uint8_t GATE_LED_SW_STATE_PIN = 11;
const uint8_t GATE_LED_SW_PIN = 12;
const uint8_t TM1637_CLK = 24;//pins definitions for TM1637 and can be changed to other ports
const uint8_t TM1637_DIO = 25;
const uint8_t PIR_IN1 = 26;//人感センサー
const uint8_t PIR_IN2 = 27;//人感センサー
const uint8_t SSD1306_BUTTON = 28;//SSD1306 表示切替用ボタン
const uint8_t FOOT_IN = 29;//フットセンサー
const uint8_t DS18B20_SINGLE_BUS = 34;
const uint8_t SN74HC595_SER = 39; ////8bit shift register (SerialData)
const uint8_t SN74HC595_RCLK = 40; //8bit shift register (LatchClock)
const uint8_t SN74HC595_SRCLK = 41; //8bit shift register (ShiftClock)
const uint8_t FOOT_LED = 48;
const uint8_t PIR_LED = 49;
//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼--------------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼--------------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼----------------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼-------------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼---------------------------

OneWire oneWire(DS18B20_SINGLE_BUS);// Setup a oneWire instance(not just Maxim/Dallas temperature ICs)
DallasTemperature dallastemp(&oneWire);// Pass our oneWire reference to Dallas Temperature.
float temperature;
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
float temp(NAN), hum(NAN), pres(NAN);
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);
float previousGateSensor1 = 4.50; //門の距離センサー１の前回の値
float previousGateSensor2 = 4.50; //門の距離センサー2の前回の値
MyNetSerial mynet(MCID_MASTER_MEGA);
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
TM1637Display tm1637(TM1637_CLK, TM1637_DIO); //set up the 4-Digit Display.

std::pair<bool, uint32_t> btGateLedSw = std::make_pair(false, 0);
std::pair<int16_t, uint32_t> itLightsOffTime = std::make_pair(-1, 0);
float fGatePhoto = MyNetSerial::FLOAT_UNKNOWN;//ESP32SOCのみpairにして時間管理
float fGateInsideTemp = MyNetSerial::FLOAT_UNKNOWN;//ESP32SOCのみpairにして時間管理
float fGateOutsideTemp = MyNetSerial::FLOAT_UNKNOWN;//ESP32SOCのみpairにして時間管理
bool bGateLedSwToggle = false;//トグル予約的な
bool bGateLedSwOff = false;//オフ予約的な
int8_t iGateLed = UNKNOWN_STATE;
//イベントハンドラカウンター

int isrPCINT0 = 0;//counter
int isrPCINT6 = 0;//counter
int isrTIMER1_COMPA = 0;//counter
int isrTIMER4_COMPA = 0;
int pushpin = -1;
uint32_t lightsOffChangedStamp = 0;//lightsOffTimeを変更して１分以上経ったらEEPROMに保存する　時間記録用
uint32_t gateLedSwChangedStamp = 0;//gateLED_SWを変更して１分以上経ったらEEPROMに保存する　時間記録用
bool lightsOffSend = false;
DateTime dtime , last_receiveTime_esp32A; //起動時間
uint32_t countsec = 0;//時計用
bool timeSyncFlag = false;//一日一回時間合わせ
bool secondFlag = false;//１秒ごとに立つフラグ
bool minuteFlag = false;
StaticJsonDocument<200> jsonDocument;
bool previousPIR1 = false;//人感センサー
bool previousPIR2 = false;//人感センサー
bool previousFOOT = false;//foot sensor
bool preSSD1306Button = false;
uint8_t ssd1306DisplayNum = 1;
bool buzzerFlag = false;
uint32_t doorPirStamp = 0;
uint32_t doorFootStamp = 0;
uint32_t SocPirStamp = 0;//事務所天井の人感センサー。SOCからの報告で更新する。
bool isSomeoneInOffice = false;

uint32_t megaSocStamp = 0;//通信確認用
uint32_t esp32cStamp = 0;//通信確認用

uint32_t gate1Stamp = 0;//通信確認用
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//------▼▼▼-------------------------------------------------------------------------------------------------
//--------▼------▼--▼▼---------▼------------------------------------------------------------------------------
//--------▼------▼▼----▼---▼▼▼▼▼▼------▼▼▼-----▼--▼▼-----▼--▼▼-----▼-----▼---▼▼▼▼-------▼--------
//--------▼------▼------▼-------▼--------▼-----▼----▼▼----▼---▼▼----▼---▼-----▼---▼-----▼--▼▼▼▼▼--------
//--------▼------▼------▼-------▼--------▼▼▼▼▼----▼----------▼-----------▼-----▼---▼▼▼▼-------▼-----------
//--------▼------▼------▼-------▼--------▼-----------▼----------▼-----------▼-----▼---▼------------▼-------
//------▼▼▼----.▼-----▼-------▼----------▼▼▼------▼----------▼-------------▼▼▼▼---▼------------▼-------
ISR(PCINT0_vect) { //PBxの論理変化によって呼び出される割り込みハンドラ
  _delay_ms(CHATTERING_AVOIDANCE_TIME);//チャタリング待ち（チャタリングにより再びPCIFRのPCIF0に１がセットされるかもしれないので待つ）
  PCIFR |= _BV(PCIF0); //フラグに１を書き込むことで、フラグをリセット（チャタリング回避）
  isrPCINT0++;
  if (bit_is_clear(PINB, GATE_LED_SW_PORT)) {
    bGateLedSwToggle = true;
    return;
  }
}

ISR(TIMER5_COMPA_vect) {//TC5 1秒ごとのイベント
  countsec++;
  if ( (countsec % 86400) == 0) timeSyncFlag = true; //一日一回時間合わせ

  secondFlag = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//                ▼▼▼       ▼▼        ▼             ▼    ▼    ▼▼▼▼
//               ▼          ▼   ▼    ▼▼▼▼          ▼    ▼    ▼     ▼
//                 ▼▼      ▼▼▼▼      ▼             ▼    ▼    ▼▼▼▼
//                    ▼     ▼           ▼             ▼   ▼     ▼
//-----------------▼▼--------▼▼--------▼--------------▼▼-------▼-----------------------------------------------
//-------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);       // ハードウェアシリアルを準備
  Serial1.begin(MyNetSerial::BPS);//MCID_MEGA_SOC
  Serial2.begin(MyNetSerial::BPS);//MCID_ESP32_C
  Serial3.begin(MyNetSerial::BPS);//MCID_SCREEN_UNO
  mynet.debugSerial();

  mynet.beginPort(Serial1, MCID_MEGA_SOC);
  mynet.beginPort(Serial2, MCID_ESP32_C);
  mynet.beginPort(Serial3, MCID_SCREEN_UNO);
  mynet.setDigitalWrite(digitalWrite);
  mynet.setShiftout(shiftOut);
  mynet.setShiftRegister(SN74HC595_SER, SN74HC595_RCLK, SN74HC595_SRCLK);
  mynet.setShiftRegiWriteBit(Serial1, 0b00001000);
  mynet.setShiftRegiReadBit(Serial1, 0b00000100);
  mynet.setShiftRegiWriteBit(Serial2, 0b00100000);
  mynet.setShiftRegiReadBit(Serial2, 0b00010000);
  mynet.setShiftRegiWriteBit(Serial3, 0b10000000);
  mynet.setShiftRegiReadBit(Serial3, 0b01000000);


  //割り込みの設定
  //PinChange
  DDRB = 0xff; //全出力
  DDRB &= ~_BV(GATE_LED_SW_PORT);//GATE_LED_SW_PORTを入力に
  PORTB = _BV(GATE_LED_SW_PORT);//GATE_LED_SW_PORT PULL_UP
  PCICR = 1 << PCIE0; //Pin Change Interrupt Control Register  (PCI0を有効に　PCINT0~7
  PCMSK0 = 1 << GATE_LED_SW_PCINT; //Pin Change Mask Register (GATE_LED_SW_PCINTを有効に
  //Timer/Counter5 16bit(時計用
  TCCR5A = 0b00000000;//コンペアマッチA,B,C共にオフ　WGM(WaveGenerationMode) 1100 ICR5がTOPのCTCモード
  TCCR5B = 0b00011101;//1024分周
  OCR5A = 15624; //16mHz / 1024 = 15625 (1second)
  ICR5 = 15624; //16mHz / 1024 = 15625 (1second)
  TIMSK5 |= _BV(OCIE5A);//コンペアマッチAの割り込みを有効に。
  sei();//割り込み許可
  //割り込みの設定終わり

  Wire.begin();

  //while
  if (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  dallastemp.begin();

  Serial.println("begin");
  pinMode(13, OUTPUT);
  pinMode(GATE_LED_STATE_PIN, OUTPUT);
  pinMode(GATE_LED_SW_STATE_PIN, OUTPUT);
  pinMode(GATE_LED_STATE_PIN, OUTPUT);
  pinMode(SN74HC595_SRCLK, OUTPUT);//8bit shift register
  pinMode(SN74HC595_RCLK,  OUTPUT);//8bit shift register
  pinMode(SN74HC595_SER,   OUTPUT);//8bit shift register
  pinMode(FOOT_LED,   OUTPUT);
  pinMode(PIR_LED,   OUTPUT);
  pinMode(PIR_IN1,   INPUT);//人感センサー
  pinMode(PIR_IN2,   INPUT);//人感センサー
  pinMode(FOOT_IN,   INPUT_PULLUP);
  pinMode(SSD1306_BUTTON,   INPUT_PULLUP);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  Serial.println("display.begin");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // init done

  delay(100);
  Serial.println("display.began");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 0);
  display.clearDisplay();
  display.println("Starting work!");
  display.display();

  //DS3231 RTC リアルタイムクロックモジュール----------------------------------------------------------------
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  display.println("find RTC!");
  display.display();
  //rtc.adjust( DateTime(2019, 3,2, 20, 22, 50)  );
  dtime = rtc.now();

  //-------------------------------------------------------------------------------------------------------
  lcd.init();
  lcd.backlight();
  //lcd.setCursor(0, 0);
  lcd.print("Starting...");

  itLightsOffTime.first = (int16_t)makeWord(EEPROM[EEP_LIGHTSOFFTIME_ADRS], EEPROM[EEP_LIGHTSOFFTIME_ADRS + 1]);
  //itLightsOffTime.first = (int16_t)300;//初期化用
  itLightsOffTime.second = MyEEPROM::readUint32(EEP_LIGHTSOFFTIME_UT_ADRS);
  //itLightsOffTime.second = 0;//初期化用
  btGateLedSw.first = EEPROM[EEP_GATELEDSW_ADRS];
  btGateLedSw.second = MyEEPROM::readUint32(EEP_GATELEDSW_UT_ADRS);
  display.println("find RTC!");
  display.display();
  //調整用
  //    EEPROM[EEP_LIGHTSOFFTIME_ADRS] = highByte((int16_t)300);
  //    EEPROM[EEP_LIGHTSOFFTIME_ADRS + 1] = lowByte((int16_t)300);
  //    MyEEPROM::writeUint32(EEP_LIGHTSOFFTIME_UT_ADRS, 0);

  lcd.clear();
  uint8_t buf = 0b10101010;
  digitalWrite(SN74HC595_RCLK, LOW); //シフトレジスタにはラッチクロックをLOWにしてから書き込む
  shiftOut(SN74HC595_SER, SN74HC595_SRCLK, LSBFIRST, buf); //シフトレジスタにデータを書き込む便利関数shiftOut
  digitalWrite(SN74HC595_RCLK, HIGH); //書き込み終わったらラッチクロックをHIGHに（シフトレジスタからパラレル出力される
  delay(500);
  buf = 0b01010101;
  digitalWrite(SN74HC595_RCLK, LOW); //シフトレジスタにはラッチクロックをLOWにしてから書き込む
  shiftOut(SN74HC595_SER, SN74HC595_SRCLK, LSBFIRST, buf); //シフトレジスタにデータを書き込む便利関数shiftOut
  digitalWrite(SN74HC595_RCLK, HIGH); //書き込み終わったらラッチクロックをHIGHに（シフトレジスタからパラレル出力される
  delay(500);
  for (int i = 0; i < 9 ; i++) {
    buf = 1 << i;
    digitalWrite(SN74HC595_RCLK, LOW); //シフトレジスタにはラッチクロックをLOWにしてから書き込む
    shiftOut(SN74HC595_SER, SN74HC595_SRCLK, LSBFIRST, buf); //シフトレジスタにデータを書き込む便利関数shiftOut
    digitalWrite(SN74HC595_RCLK, HIGH); //書き込み終わったらラッチクロックをHIGHに（シフトレジスタからパラレル出力される
    delay(80);
  }

  tm1637.setBrightness(7); //set the diplay to maximum brightness
  tm1637.showNumberDecEx(0 , 0b00100000, true); //Display the numCounter cvalue;

}//setup

void gateLedSwSet(bool state, bool sendA = true , bool sendB = true) {
  if (btGateLedSw.first && !state) {
    btGateLedSw.first = false;
    digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
    if (sendB) mynet.sendUT(btGateLedSw.second, MCID_MEGA_SOC, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
  } else if (!btGateLedSw.first && state) {
    btGateLedSw.first = true;
    digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
    if (sendB) mynet.sendUT(btGateLedSw.second, MCID_MEGA_SOC, SDPP_ON_COMMAND, PID_GATE_LED_SW);
  }
  gateLedSwChangedStamp = countsec;
}

//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼-------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼-------------------------------------
int loopcount = 0;

void loop()
{

  if (!digitalRead(FOOT_IN)) {
    digitalWrite(FOOT_LED, HIGH);
    if (!previousFOOT &&  (doorFootStamp + 10000) < millis()   ) {
      buzzerFlag = true;
      doorFootStamp = millis();
    }
    previousFOOT = true;
  } else {
    digitalWrite(FOOT_LED, LOW);
    previousFOOT = false;
  }

  if (!digitalRead(SSD1306_BUTTON)) {
    if (preSSD1306Button == false) {
      ssd1306DisplayNum = (ssd1306DisplayNum + 1) % 2;
    }
    preSSD1306Button = true;
  } else {
    preSSD1306Button = false;
  }
  
//  if (digitalRead(PIR_IN1)) {
//    digitalWrite(PIR_LED, HIGH);
//    if (!previousPIR1 && ( (doorPirStamp + 10000) < millis() ) && digitalRead(PIR_IN2) ) {
//      doorPirStamp = millis();
//    }
//    previousPIR1 = true;
//  } else {
//    digitalWrite(PIR_LED, LOW);
//    previousPIR1 = false;
//  }
//  if (digitalRead(PIR_IN2)) {
//    digitalWrite(PIR_LED, HIGH);
//    if (!previousPIR2 && ( (doorPirStamp + 10000) < millis() ) && digitalRead(PIR_IN1) ) {
//      doorPirStamp = millis();
//    }
//    previousPIR2 = true;
//  } else {
//    if (!previousPIR1)digitalWrite(PIR_LED, LOW);
//    previousPIR2 = false;
//  }

  //Serial Process----------------------------------
  if (Serial1.available())
  {
    Serial.println("------r1------");
    delay(10);
    mynet.receive(Serial1);
  }
  if (Serial2.available())
  {
    Serial.println("------r2------");
    delay(10);
    mynet.receive(Serial2);
  }
  processMyData();
  //Serial Process---end----------------------------

  //loopcount process-------------------------------
  if (loopcount == 0 ) {
    loopZero();
  }

  if ( (loopcount % 10 - 3) == 0) loop10();

  if ( (loopcount % 500 - 17) == 0) loop500();



  if (loopcount == 600) {
    mynet.send(MCID_ESP32_C, SDPP_PERIODIC_CONFIRMATION);

  }
  //  if (loopcount == 300) {
  //    mynet.send(MCID_MEGA_SOC, SDPP_PERIODIC_CONFIRMATION);
  //  }
  if (loopcount % 1000 == 73) gateLedSwConfirm();
  //loopcount process---end-------------------------

  //Flag process------------------------------------
  if (bGateLedSwToggle)  {
    btGateLedSw.second = rtc.now().unixtime();
    gateLedSwToggle();
  }

  if (timeSyncFlag) {
    mynet.send(MCID_ESP32_C, SDPP_TIME_REQUEST , PID_TIME);
    timeSyncFlag = false;
  }

  if (secondFlag) {//secondFlag is set every seconds
    if (rtc.now().second() == 0 && (rtc.now().minute() == 0 || rtc.now().minute() == 30) ) {
      sendWeatherJson();
    }
    if (rtc.now().second() == 0 && ( (rtc.now().minute() % 5) == 0 ) ) {//send the unixtime to screen_uno every 5 minutes
      mynet.sendUT(rtc.now().unixtime() , MCID_SCREEN_UNO, SDPP_TIME_REPORT, PID_TIME);
    }
    if (rtc.now().second() == 0) minuteFlag = true;
    secondFlag = false;
  }

  if (minuteFlag) {
    //communication check
    if ( (esp32cStamp + 120) < rtc.now().unixtime()) mynet.send(MCID_ESP32_C, SDPP_PERIODIC_CONFIRMATION);
    //communication check end
    minuteFlag = false;
  }

  if (buzzerFlag) {
    //if (!isSomeoneInOffice) {
    mynet.send(MCID_ESP32_C, SDPP_ON_REPORT, PID_DOOR_INSIDE_PIR);
    //}
    buzzerFlag = false;
  }
  //Flag process-----end----------------------------

  //Stamp process-------------------------------------
  if (lightsOffChangedStamp != 0 && (lightsOffChangedStamp + 15) < countsec) {//消灯タイマーの時間を変更して15s以上経ったらEEPROMに保存
    EEPROM[EEP_LIGHTSOFFTIME_ADRS] = highByte(itLightsOffTime.first);
    EEPROM[EEP_LIGHTSOFFTIME_ADRS + 1] = lowByte(itLightsOffTime.first);
    MyEEPROM::writeUint32(EEP_LIGHTSOFFTIME_UT_ADRS, itLightsOffTime.second);
    lightsOffChangedStamp = 0;
  }

  if (gateLedSwChangedStamp != 0 && (gateLedSwChangedStamp + 15) < countsec) { //変更して15s以上経ったらEEPROMに保存
    EEPROM[EEP_GATELEDSW_ADRS] = btGateLedSw.first;
    MyEEPROM::writeUint32(EEP_GATELEDSW_UT_ADRS, btGateLedSw.second);
    gateLedSwChangedStamp = 0;
  }

  if ( (SocPirStamp + 300) < rtc.now().unixtime() ) isSomeoneInOffice = false;
  else isSomeoneInOffice = true;
  //Stamp process---end-------------------------------

  loopcount++;
  if (loopcount == 1000) {
    loopcount = 0;
  }
  dispStatus();
  delay(100);
} //End Of loop

//--▼---------▼--------------------------------------------------------------------------------------------
//--▼▼------▼▼-------▼▼▼▼-----------▼--------▼-----------------------------▼-----▼▼▼-----------------
//--▼--▼--▼--▼-----▼-------▼-----▼▼▼▼▼▼----▼--▼▼--------▼▼▼-------▼▼▼----▼----------------------
//--▼----▼----▼----▼▼▼▼▼▼▼---------▼--------▼▼----▼----▼-----▼----▼---▼-----▼▼▼-------------------
//--▼----------▼------▼------▼---------▼--------▼------▼----▼-----▼----▼---▼----▼----▼------------------
//--▼----------▼--------▼▼▼-----------▼--------▼------▼------▼▼▼-------▼▼▼-----▼▼▼-------------------



void gateLedSwToggle() { //スイッチを押した時に呼び出される
  gateLedSwSet(!btGateLedSw.first);
  bGateLedSwToggle = false;
}

void gateLedSwConfirm() {
  //  lcd2.setCursor(3, 1);
  if (btGateLedSw.first) {
    digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
    mynet.sendUT(btGateLedSw.second, MCID_MEGA_SOC, SDPP_ON_REPORT, PID_GATE_LED_SW);
  } else {
    digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
    mynet.sendUT(btGateLedSw.second, MCID_MEGA_SOC, SDPP_OFF_REPORT, PID_GATE_LED_SW);
  }
}

//--------lightsOffSet↑-----------------------------------------------------------------------------------------------


void trace(char* c) {
  //lcd.setCursor(TRACE_X, TRACE_Y);
  // lcd.print(c);
}
//-------------------------------------------------------------------------------------------------------
//--▼---------▼--------------------------------------------------------------------------------------------
//--▼▼------▼▼-------▼▼▼▼-----------▼--------▼-----------------------------▼-----▼▼▼--------------
//--▼--▼--▼--▼-----▼-------▼-----▼▼▼▼▼▼----▼--▼▼--------▼▼▼-------▼▼▼----▼--------------------------
//--▼----▼----▼----▼▼▼▼▼▼▼---------▼--------▼▼----▼----▼-----▼----▼---▼-----▼▼▼----------------------
//--▼----------▼----▼-------------------▼--------▼------▼----▼-----▼----▼---▼----------▼-----------------------
//--▼----------▼------▼------▼---------▼--------▼------▼----▼-----▼----▼---▼----▼----▼-----------------------
//--▼----------▼--------▼▼▼-----------▼--------▼------▼------▼▼▼-------▼▼▼-----▼▼▼------------------------

//void serialLedShiftOut() {

//}


void dispSSD() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0, 0);
  display.print("MASTER MEGA");
  display.setCursor(0, 8);
  display.setTextColor(WHITE);

  if (ssd1306DisplayNum == 1) {
     display.print("communication check");
    display.setCursor(0, 16);
    display.print("ESP32C:");
    printTON(16, esp32cStamp);

    display.setCursor(0, 24);
    display.setTextColor(WHITE);
    display.print("MEGA SOC:");
    printTON(24, megaSocStamp);

    display.setCursor(0, 32);
    display.setTextColor(WHITE);
    display.print("Gate:");
    printTON(32, gate1Stamp);

    display.display();
    return;
  }



  //if (ssd1306DisplayNum == 0){
    display.print("Gate_Photo:");
    if (fGatePhoto != MyNetSerial::FLOAT_UNKNOWN) {
      display.print(fGatePhoto);
      display.print("R");
    }
    display.setCursor(0, 16);
    display.print("UnixTime:");
    display.setCursor(63, 16);
    display.print(rtc.now().unixtime());
    display.setCursor(0, 24);
    display.print("GateSw:");
    display.print(btGateLedSw.first);
    display.setCursor(63, 24);
    display.print(btGateLedSw.second);
    display.setCursor(0, 32);
    display.print("lot:");
    display.print(MyToString::int16ToTimeString(itLightsOffTime.first));
    display.setCursor(63, 32);
    display.print(itLightsOffTime.second);
    display.setCursor(63, 40);
    display.print(isSomeoneInOffice);
    //now time
    display.setCursor(0, 56);
    display.print(rtc.now().year() % 100); display.print('/'); display.print(rtc.now().month()); display.print('/');
    display.print(rtc.now().day()); display.print(' ');
    String loc;
    if (rtc.now().unixtime() != 0) {
      loc = ( String(rtc.now().hour() / 10) + String(rtc.now().hour() % 10) + ':'
              + String(rtc.now().minute() / 10) + String(rtc.now().minute() % 10) );
    } else {
      loc = "??:??";
    }
    display.print(loc);
    display.print(' '); display.print(rtc.now().second());
    display.display();
  //}


}

void printSSD1(){
 
}

void printTON(uint8_t row, uint32_t st) { //use for dispSSD method
  display.setCursor(80, row);
  if (st == 0) {
    display.setTextColor(WHITE);
    display.print("NO");
    display.setTextColor(BLACK, WHITE);
    display.setCursor(60, row);
    display.print("NG");
    return;
  }
  //uint32_t et = rtc.now().unixtime() - st;
  if ( rtc.now().unixtime() - st < 300) {
    display.setTextColor(WHITE);
    display.print("OK");

  } else {
    display.setTextColor(BLACK, WHITE);
    display.print("NG");
    display.setTextColor(WHITE);
  }
  display.setCursor(60, row);
  display.print(rtc.now().unixtime() - st);
}


void checkChangedTime() {//xxxxxxxx
  if ( (itLightsOffTime.second - 100) > rtc.now().unixtime()) {
    itLightsOffTime.second = 0;
  }
  if ( (btGateLedSw.second - 100) > rtc.now().unixtime()) {
    btGateLedSw.second = 0;
  }
}

void loopZero() {


}//End Of loopZero

void loop500() {
  bme280read();
  //cpu電圧
  float vcc;
  float presHpa = pres / 100;
  vcc = cpuVcc();
  dallastemp.requestTemperatures(); // Send the command to get temperatures
  temperature = dallastemp.getTempCByIndex(0);
  lcd.setCursor(8, 1);
  lcd.print(vcc);
  lcd.print('v');
  lcd.setCursor(7, 0);
  lcd.print(presHpa);
  lcd.print("hPa");
  lcd.setCursor(0, 1);
  lcd.print(temperature);
  lcd.print("'C");
  lcd.setCursor(0, 0);
  lcd.print(hum);
  lcd.print( '%' );

  int buf = presHpa;
  if (buf >= 1000) {
    tm1637.showNumberDec(buf); //Display the numCounter cvalue;
  } else {
    int bb = presHpa * 10;//小数点第一位をbbに入れる
    bb = bb % 10;
    buf = (buf * 10) + bb;
    tm1637.showNumberDecEx(buf, 0b00100000);
  }

}
void bme280read()
{
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
}

void dispStatus() {
  lcd.setCursor(13, 1);
  lcd.print(loopcount);
  lcd.print("  ");
  lcd.display();
}
void loop10() {
dispSSD();
}

float cpuVcc()
{
  long sum = 0;
  adcSetup(0x5E); //01011110
  //ADMUXのビットパターン ※at328pとは違う！（ADC Multiplexer Selection Register
  //REFS1:0  REFS0:1  ADLAR:0  MUX4:1  MUX3:1  MUX2:1  MUX1:1  MUX0:0
  //REFS1とREFS０は０は参照電圧の設定 01で　AVCC with external capacitor at AREF pin を参照電圧にする。
  //ADLARは、読み取った値を格納するレジスタADCLとADCHが左詰(1)か右詰(0)かを設定。aruduinoでは0固定
  //MUX4~0はアナログピン選択のためのビット

  //MUX5~0を011110として、Input channelにインターナル1.1vを選択
  //ADCSRBのビットパターン ※at328pとは違う！（ADC Control and Status Register B
  //-:ACME:-:-:MUX5:ADTS2:ADTS1:ADTS0
  //ACME (Analog Comparator Multiplexed Input 比較測定器
  //ADTSよーわからん0でよし
  //MUX5はアナログピン選択する

  for (int n = 0; n < 10; n++)//１０回測定して平均をとる
  {
    sum = sum + adc();
  }
  return (1.1 * 10240.0) / sum;
}

void adcSetup(byte data)
{
  ADMUX = data;
  ADCSRA |= (1 << ADSC);
  ADCSRA |= 0x07; //00000111
  //ADCSRAのビットパターン ※at328pと同じ！（ADC Control and Status Register A
  //ADEN:ADSC:ADATE:ADIF:ADIE:ADPS2:ADPS1:ADPS0
  delay(10);
}

unsigned int adc()
{
  unsigned int dL, dH;
  ADCSRA |= (1 << ADSC);//ADSC:ADC Start Conversion
  while (ADCSRA & (1 << ADSC))//When the conversion is complete,ADSC return to zero.
  {
  }
  dL = ADCL;
  dH = ADCH;
  return dL | (dH << 8);
}

/*
  CPU温度センサーと電源電圧の読み出し、表示デモ
  2014/7/19 ラジオペンチ
  http://radiopench.blog96.fc2.com/
*/
//-------------Methods related to MyNet------------------------------------------------------------------------------

void processMyData()
{
  SerialData* sd = mynet.containedPick();
  while ( sd != NULL) {

    switch (sd->receiver())
    { //受信者MCIDで分岐
      case MCID_MASTER_MEGA:
        dataToMe(sd);
        break;
    }
    sd = mynet.containedPick();
  }
}

//argument :ID of the data array(mydata) to be displayed
void dataToMe(SerialData * sd)
{
  switch (sd->sender())
  {
    case MCID_MEGA_SOC:
      dataMegaSocToMe(sd);
      break;
    case MCID_ESP32_C:
      dataEsp32CToMe(sd);
      break;
  }
}

void dataEsp32CToMe(SerialData * sd) {
  esp32cStamp = rtc.now().unixtime();//生存確認スタンプ
  switch (sd->pid()) {
    case PID_LIGHTSOFFTIME:
      {
        switch (sd->sdpp())
        {
          case SDPP_TIME_REQUEST://時間リクエスト
            { //switchでネストするときはcaseにカッコが必要
              mynet.sendInt16(itLightsOffTime.first, MCID_ESP32_C, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
            }//case SDPP_TIME_REQUEST:
            break;
          case SDPP_TIME_REPORT: //時間報告
            { //switchでネストするときはcaseにカッコが必要
              itLightsOffTime.second = rtc.now().unixtime();
              itLightsOffTime.first = sd->int16();
              lightsOffChangedStamp = countsec;
              mynet.sendInt16UT(itLightsOffTime.second , itLightsOffTime.first, MCID_MEGA_SOC, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
            }//case SDPP_TIME_REPORT:
            break;
        }//switch sdpp
      }//case PID_LIGHTSOFFTIME:
      break;
    case PID_TIME: { //SDPP_TIME_REPORTしか来ない前提
        rtc.adjust( DateTime(sd->updatetime() + 1)  );
      }
      break;
    case PID_NONE: {
        switch (sd->sdpp()) {
          case SDPP_PERIODIC_CONFIRMATION: //定期確認
            mynet.send(MCID_ESP32_C, SDPP_PERIODIC_CONFIRMATION_BACK);
            break;
          case SDPP_PERIODIC_CONFIRMATION_BACK: //定期確認返し
            //なんもせん
            break;
        }
      }//case PID_NONE:
      break;
  }// switch pid

}

void dataMegaSocToMe(SerialData * sd) {
  megaSocStamp = rtc.now().unixtime();//生存確認スタンプ
  switch (sd->pid()) {
    case PID_GATE_PHOTO: { //報告しか来ない前提
        fGatePhoto = sd->float_();
      }//case PID_GATE_PHOTO:
      break;
    case PID_GATE_1_INSIDE_TEMP: { //報告しか来ない前提
        fGateInsideTemp = sd->float_();
      }//case PID_GATE_1_INSIDE_TEMP:
      break;
    case PID_GATE_1_OUTSIDE_TEMP: { //報告しか来ない前提
        fGateOutsideTemp = sd->float_();
      }//case PID_GATE_1_OUTSIDE_TEMP:
      break;
    case PID_GATE_LED:
      { //
        switch (sd->sdpp()) {
          case SDPP_ON_REPORT: {
              iGateLed = ON_STATE;
              digitalWrite(GATE_LED_STATE_PIN, HIGH);
            }//case SDPP_ON_REPORT:
            break;
          case SDPP_OFF_REPORT: {
              iGateLed = OFF_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
            }//case SDPP_OFF_REPORT:
            break;
          case SDPP_UNKNOWN_REPORT: {
              iGateLed = UNKNOWN_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
            }//case SDPP_OFF_REPORT:
            break;
        }//switch (sd->sdpp())

      }//case PID_GATE_LED_SW:
      break;
    case PID_GATE_LED_SW:
      {
        switch (sd->sdpp()) {
          case SDPP_ON_REPORT: {
              if (btGateLedSw.second > (sd->updatetime() + 1000)) { //こちらの方が情報が新しければ送り返す
                mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED_SW);
                return;
              }
            }//through
          case SDPP_ON_COMMAND: { //through
              btGateLedSw.second = sd->updatetime();//update timeの更新
              btGateLedSw.first = ON_STATE;
              digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
            }//}case SDPP_ON_COMMAND:
            break;
          case SDPP_OFF_REPORT: {
              if (btGateLedSw.second > (sd->updatetime() + 1000)) { //こちらの方が情報が新しければ送り返す
                mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED_SW);
                return;
              }
            }//through
          case SDPP_OFF_COMMAND: {
              btGateLedSw.second = sd->updatetime();//update timeの更新
              btGateLedSw.first = OFF_STATE;
              digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
            }//}case SDPP_OFF_COMMAND:
            break;
          case SDPP_ONOFF_REQUEST: {
              if (btGateLedSw.first) mynet.sendUT(btGateLedSw.second, MCID_MEGA_SOC, SDPP_ON_REPORT, PID_GATE_LED_SW);
              else mynet.sendUT(btGateLedSw.second, MCID_MEGA_SOC, SDPP_OFF_REPORT, PID_GATE_LED_SW);
            }
            break;
        }//switch(sd->sdpp()

      }//case PID_GATE_LED_SW:
      break;
    case PID_LIGHTSOFFTIME:
      {
        switch (sd->sdpp())
        {
          case SDPP_TIME_REQUEST://時間リクエスト
            { //switchでネストするときはcaseにカッコが必要
              mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first,
                                MCID_MEGA_SOC, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
            }//case SDPP_TIME_REQUEST:
            break;
          case SDPP_TIME_REPORT: //時間報告
            { //switchでネストするときはcaseにカッコが必要

              if (itLightsOffTime.second > (sd->updatetime() + 1000)) { //こちらの方が情報が新しければ送り返す
                mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first,
                                  MCID_MEGA_SOC, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
                return;
              } else { //update timeの更新
                itLightsOffTime.second = sd->updatetime();
              }
              itLightsOffTime.first = sd->int16();
              lightsOffChangedStamp = countsec;
            }//case SDPP_TIME_REPORT:
            break;
        }//switch sdpp
      }//case PID_LIGHTSOFFTIME:
      break;
    case PID_TIME:
      { //TIME_REQUESTしか来ない前提
        mynet.sendUT(rtc.now().unixtime() , MCID_MEGA_SOC, SDPP_TIME_REPORT, PID_TIME); //uint32_t
      }
      break;
    case PID_MEGA_SOC_PIR: {
        SocPirStamp = rtc.now().unixtime();
      }//case PID_MEGA_SOC_PIR
      break;
    case PID_NONE: {
        switch (sd->sdpp()) {
          case SDPP_PERIODIC_CONFIRMATION: //定期確認
            mynet.send(MCID_MEGA_SOC, SDPP_PERIODIC_CONFIRMATION_BACK);
            break;
          case SDPP_PERIODIC_CONFIRMATION_BACK: //定期確認返し
            //なんもせん
            break;
        }
      }//case PID_NONE:
      break;
  }// switch pid

}


void sendWeatherJson() {
  jsonDocument["year"] = rtc.now().year();
  jsonDocument["month"] = rtc.now().month();
  jsonDocument["day"] = rtc.now().day();
  jsonDocument["hour"] = rtc.now().hour();
  jsonDocument["minute"] = rtc.now().minute();
  jsonDocument["pressure"] = pres;
  jsonDocument["officeTemp"] = temperature;
  jsonDocument["officeHumi"] = hum;
  jsonDocument["gate1BoxTemp"] = fGateInsideTemp;
  jsonDocument["gate1OutTemp"] = fGateOutsideTemp;
  char buf[200];
  serializeJson(jsonDocument, buf);
  mynet.sendCharArray(buf, MCID_ESP32_C, SDPP_POST_JSON, PID_DB_WEATHER);
}

/*


  DDRK = 0xff;//全出力
  DDRK &= ~_BV(LIGHTS_OFF_UP_PORT);//LIGHTS_OFF_UP_PORTを入力に
  DDRK &= ~_BV(LIGHTS_OFF_DOWN_PORT);//LIGHTS_OFF_DOWN_PORTを入力に

  PORTK = _BV(LIGHTS_OFF_UP_PORT);//LIGHTS_OFF_UP_PORT PULL_UP
  PORTK |= _BV(LIGHTS_OFF_DOWN_PORT);//LIGHTS_OFF_DOWN_PORT PULL_UP

  PCICR |= 1 << PCIE2;//Pin Change Interrupt Control Register  (PCI2を有効に　PCINT16~23

  PCMSK2 = _BV(LIGHTS_OFF_UP_PCINT);//Pin Change Mask Register (LIGHTS_OFF_UP_PCINTを有効に
  PCMSK2 |= _BV(LIGHTS_OFF_DOWN_PCINT);//Pin Change Mask Register (LIGHTS_OFF_DOWN_PCINTを有効に
  //PCINTは論理変化でのみgenerates an interrupt 他の設定は出来ない　INTの場合は４つの設定がある
  //Timer/Counter1 16bit(チャタリング回避用)
  //  TCCR1A = 0b00000000;//コンペアマッチA,B,C共にオフ　WGM(WaveGenerationMode) 1100 ICR1がTOPのCTCモード
  //  TCCR1B = 0b00011101;//InputCaptureNoizeCancelerオフ　クロック分周数1024（1024/１6Mhz(クロック) = 0.000064s
  // 0.064msごとにカウント
  //  ICR1 = (int32_t)(CHATTERING_AVOIDANCE_TIME_TC1 / 0.064); //TOP値(Timer/Counter1の周期)の指定　16bitなので最大65535。
  //Timer/Counter4 16bit(押しっぱなし連打用
  TCCR4A = 0b00000000;//コンペアマッチA,B,C共にオフ　WGM(WaveGenerationMode) 0000 NORMALmode
  TCCR4B = 0b00000101;//InputCaptureNoizeCancelerオフ　クロック分周数1024（1024/１6Mhz(クロック) = 0.000064s  0.064msごとにカウント


  //ISR(PCINT6_vect) { //PK(analog8~15)の論理変化によって呼び出される割り込みハンドラ
  //
  //  if ( bit_is_clear(PINK, LIGHTS_OFF_UP_PORT) || bit_is_clear(PINK, LIGHTS_OFF_DOWN_PORT) ) {
  //    if (bit_is_clear(TIMSK4, OCIE4A)) { //コンペアマッチAが無効なら
  //      uint8_t sreg;
  //      sreg = SREG;//ステータスレジスタを保存
  //      cli();//割込み禁止  (16bitのレジスタの読み書きには、16bitレジスタ共通の１時レジスタ(TEMP)を使うので、割り込み禁止する必要あり。
  //      uint16_t buf = 0;
  //      TCNT4 = buf;
  //      OCR4A = (int32_t)(RENDA_TIME / 0.064); //16mHz / 1024 = 15625 (1second)
  //      TIMSK4 |= _BV(OCIE4A);//コンペアマッチAの割り込みを有効に。　（TC1が1周したらTIMER1_COMPA_vectが実行される
  //      TIFR4 |= _BV(OCF4A);//コンペアマッチA フラグリセット
  //      SREG = sreg;//SREGを戻す。禁止状態が戻る。（SREGのIビットを戻すから）
  //
  //      if ( bit_is_clear(PINK, LIGHTS_OFF_DOWN_PORT) ) {
  //        lightsOffMinus();
  //      } else if ( bit_is_clear(PINK, LIGHTS_OFF_UP_PORT) ) {
  //        lightsOffPlus();
  //      }
  //
  //    }
  //    return;
  //  }
  //
  //  TIMSK4 &= ~_BV(OCIE4A); //コンペアマッチAの割り込みを無効に
  //}//End Of ISR(PCINT6_vect)
  //
  //ISR(TIMER4_COMPA_vect) {//押しっぱなし連打
  //  isrTIMER4_COMPA++;
  //  lc1.setInt(0, 6, isrTIMER4_COMPA);
  //
  //  uint8_t sreg;
  //  sreg = SREG;//ステータスレジスタを保存
  //  cli();//割込み禁止  (16bitのレジスタの読み書きには、16bitレジスタ共通の１時レジスタ(TEMP)を使うので、割り込み禁止する必要あり。
  //  uint16_t buf = 0;
  //  TCNT4 = buf;
  //  SREG = sreg;//SREGを戻す。禁止状態が戻る。（SREGのIビットを戻すから）
  //
  //  if ( bit_is_clear(PINK, LIGHTS_OFF_DOWN_PORT) ) {
  //    lightsOffMinus();
  //  } else if ( bit_is_clear(PINK, LIGHTS_OFF_UP_PORT) ) {
  //    lightsOffPlus();
  //  }
  //
  //}

  //チャタリング回避　遺産
  // if (bit_is_clear(PINK, PK0)) { //PK0(analog8)がLOWの時
  //    //以下、TC1を使ったチャタリング回避
  //    isrPCINT6++;
  //    lc1.setInt(1, 0, isrPCINT6);
  //    PCMSK2 &= ~_BV(PCINT16); //Pin Change Mask Register (PCINT16を無効に　analog8番ピン
  //    if (bit_is_clear(TIMSK1, OCIE1A)) { //コンペアマッチAが無効なら
  //      uint8_t sreg;
  //      sreg = SREG;//ステータスレジスタを保存
  //      cli();//割込み禁止  (16bitのレジスタの読み書きには、16bitレジスタ共通の１時レジスタ(TEMP)を使うので、割り込み禁止する必要あり。
  //      uint16_t buf = TCNT1 - 1;//現在のカウント-1をコンペアマッチAの値に設定
  //      OCR1A = buf;
  //      PCIFR |= _BV(PCIF2); //フラグに１を書き込むことで、フラグをリセット（チャタリング回避）
  //      TIMSK1 |= _BV(OCIE1A);//コンペアマッチAの割り込みを有効に。　（TC1が1周したらTIMER1_COMPA_vectが実行される
  //      TIFR1 |= _BV(OCF1A);//コンペアマッチA フラグリセット
  //      SREG = sreg;//SREGを戻す。禁止状態が戻る。（SREGのIビットを戻すから）
  //
  //
  //    }
  //  }
  //ISR(TIMER1_COMPA_vect) {//TC1のチャタリング回避後に実行する処理
  //  isrTIMER1_COMPA++;
  //  lc1.setInt(1, 4, isrTIMER1_COMPA);
  //
  //
  //  PCMSK2 |= (1 << PCINT16);//割り込み有効に
  //  TIMSK1 &= ~_BV(OCIE1A);//コンペアマッチAの割り込みを無効に
  //
  //}//遺産終わり

*/
