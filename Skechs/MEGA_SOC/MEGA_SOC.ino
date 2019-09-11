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

//DS3231 リアルタイムクロックモジュール
#include <RTClib.h>

#include <OneWire.h>
#include <DallasTemperature.h>
//-------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include "Ucglib.h"
//-------------------------------------------------------------------------------------------------------
#include <TM1637Display.h> //light of clock

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
const uint8_t TM1637_HM_CLK = 25;//pins definitions for TM1637 and can be changed to other ports
const uint8_t TM1637_HM_DIO = 24;
const uint8_t TM1637_LO_CLK = 23;//pins definitions for TM1637 and can be changed to other ports
const uint8_t TM1637_LO_DIO = 22;
const uint8_t DS18B20_SINGLE_BUS = 34;
const uint8_t SN74HC595_SER = 39; ////8bit shift register (SerialData)
const uint8_t SN74HC595_RCLK = 40; //8bit shift register (LatchClock)
const uint8_t SN74HC595_SRCLK = 41; //8bit shift register (ShiftClock)

const uint8_t PIR_IN = 46;
const uint8_t PIR_LED = 47;
//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼--------------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼--------------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼----------------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼-------------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼---------------------------


RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x3F, 20, 4);
float previousGateSensor1 = 4.50; //門の距離センサー１の前回の値
float previousGateSensor2 = 4.50; //門の距離センサー2の前回の値
TM1637Display lightOffClock(TM1637_LO_CLK, TM1637_LO_DIO);
MyNetSerial mynet(MCID_MEGA_SOC);
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//Ucglib_SSD1331_18x96x64_UNIVISION_HWSPI ucg(MOSI, SS, SPI_RESET);
Ucglib_SSD1331_18x96x64_UNIVISION_SWSPI ucg(/*sclk=*/ SCK, /*data=*/MOSI,
    /*cd=*/ MISO, /*cs=*/ SS, /*reset=*/ SPI_RESET);
//Arduino Mega: 50(MISO)、51(MOSI)、52(SCK)、53(SS)

std::pair<bool, uint32_t> btGateLedSw = std::make_pair(false, 0);
std::pair<int16_t, uint32_t> itLightsOffTime = std::make_pair(-1, 0);
std::pair<int8_t, uint32_t> itGateLed = std::make_pair(UNKNOWN_STATE, 0);
std::pair<float, uint32_t> ftGatePhoto = std::make_pair(MyNetSerial::FLOAT_UNKNOWN, 0);//MEGA_SOCのみpairにして時間管理
std::pair<float, uint32_t> ftGateInsideTemp = std::make_pair(MyNetSerial::FLOAT_UNKNOWN, 0);//MEGA_SOCのみpairにして時間管理
std::pair<float, uint32_t> ftGateOutsideTemp = std::make_pair(MyNetSerial::FLOAT_UNKNOWN, 0);//MEGA_SOCのみpairにして時間管理
bool bGateLedSwToggle = false;//トグル予約的な
bool bGateLedSwOff = false;//オフ予約的な
bool bLedAlarmOff = false;
int8_t iGateLed = UNKNOWN_STATE;
//イベントハンドラカウンター

int isrPCINT0 = 0;//counter
int isrPCINT6 = 0;//counter
int isrTIMER1_COMPA = 0;//counter
int isrTIMER4_COMPA = 0;
int pushpin = -1;
uint32_t lightsOffChanged = 0;//lightsOffTimeを変更して１分以上経ったらEEPROMに保存する　時間記録用
uint32_t gateLedSwChanged = 0;//gateLED_SWを変更して１分以上経ったらEEPROMに保存する　時間記録用
bool lightsOffSend = false;
DateTime dtime , ntime;//ntime 現在の時間
uint32_t countsec = 0;//時計用
bool timeSyncFlag = false;
bool previousPIR = false;//人感センサー
bool pirFlag = false;
uint32_t pirStamp = 0;
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
  ntime = DateTime(dtime + (0, 0, countsec));
  checkChangedTime();
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
  Serial1.begin(MyNetSerial::BPS);//MCID_GATE_1
  //Serial2.begin(MyNetSerial::BPS);//OMOYA
  Serial3.begin(MyNetSerial::BPS);//MCID_MASTER_MEGA
  mynet.debugSerial();

  mynet.beginPort(Serial1, MCID_GATE_1);
  //mynet.beginPort(Serial2, MCID_MASTER_MEGA);
  mynet.beginPort(Serial3, MCID_MASTER_MEGA);
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
  TCCR5B = 0b00011101;
  OCR5A = 15624; //16mHz / 1024 = 15625 (1second)
  ICR5 = 15624; //16mHz / 1024 = 15625 (1second)
  TIMSK5 |= _BV(OCIE5A);//コンペアマッチAの割り込みを有効に。
  sei();//割り込み許可
  //割り込みの設定終わり

  Serial.println("begin");
  pinMode(13, OUTPUT);
  pinMode(GATE_LED_STATE_PIN, OUTPUT);
  pinMode(GATE_LED_SW_STATE_PIN, OUTPUT);
  pinMode(GATE_LED_STATE_PIN, OUTPUT);
  pinMode(SN74HC595_SRCLK, OUTPUT);//8bit shift register
  pinMode(SN74HC595_RCLK,  OUTPUT);//8bit shift register
  pinMode(SN74HC595_SER,   OUTPUT);//8bit shift register
  pinMode(PIR_IN,   INPUT);//jinkansensor
  pinMode(PIR_LED,   OUTPUT);//jinkansensor
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  Serial.println("display.begin");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // init done

  delay(100);
  Serial.println("display.began");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 40);
  display.clearDisplay();
  display.println("Starting work!");
  display.display();

  //DS3231 RTC リアルタイムクロックモジュール----------------------------------------------------------------
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //rtc.adjust( DateTime(2019, 4,12, 16, 44, \\\\\\           0)  );
  dtime = rtc.now();
  //-------------------------------------------------------------------------------------------------------

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  itLightsOffTime.first = (int16_t)makeWord(EEPROM[EEP_LIGHTSOFFTIME_ADRS], EEPROM[EEP_LIGHTSOFFTIME_ADRS + 1]);
  //itLightsOffTime.first = (int16_t)300;//初期化用
  itLightsOffTime.second = MyEEPROM::readUint32(EEP_LIGHTSOFFTIME_UT_ADRS);
  //itLightsOffTime.second = 0;//初期化用
  btGateLedSw.first = EEPROM[EEP_GATELEDSW_ADRS];
  btGateLedSw.second = MyEEPROM::readUint32(EEP_GATELEDSW_UT_ADRS);

  lightOffClock.setBrightness(0x0f);

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
}//setup

void gateLedSwSet(bool state, bool sendG = true , bool sendM = true) {
  //lcd2.setCursor(3, 1);
  if (btGateLedSw.first && !state) {
    btGateLedSw.first = false;
    digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
    if (sendG) mynet.send(MCID_GATE_1, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
    if (sendM) mynet.sendUT(btGateLedSw.second, MCID_GATE_1, SDPP_ON_COMMAND, PID_GATE_LED_SW);
  } else if (!btGateLedSw.first && state) {
    btGateLedSw.first = true;
    digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
    if (sendG) mynet.send(MCID_GATE_1, SDPP_ON_COMMAND, PID_GATE_LED_SW);
    if (sendM) mynet.sendUT(btGateLedSw.second, MCID_GATE_1, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
  }
  gateLedSwChanged = countsec;
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

  //  if (lightsOffChanged != 0 && (lightsOffChanged + 15) < countsec) {//消灯タイマーの時間を変更して15s以上経ったらEEPROMに保存
  //    EEPROM[EEP_LIGHTSOFFTIME_ADRS] = highByte(itLightsOffTime.first);
  //    EEPROM[EEP_LIGHTSOFFTIME_ADRS + 1] = lowByte(itLightsOffTime.first);
  //    MyEEPROM::writeUint32(EEP_LIGHTSOFFTIME_UT_ADRS, itLightsOffTime.second);
  //    lightsOffChanged = 0;
  //  }
  //
  //  if (gateLedSwChanged != 0 && (gateLedSwChanged + 15) < countsec) { //変更して15s以上経ったらEEPROMに保存
  //    EEPROM[EEP_GATELEDSW_ADRS] = btGateLedSw.first;
  //    MyEEPROM::writeUint32(EEP_GATELEDSW_UT_ADRS, btGateLedSw.second);
  //    gateLedSwChanged = 0;
  //  }

  //  `  if (loopcount % 1000 == 73) gateLedSwConfirm();

  //Serial.print("/");
  
  if (digitalRead(PIR_IN)) {//事務所天井の人感センサー
    digitalWrite(PIR_LED, HIGH);
    if (!previousPIR) {//前回LOWだった時。（HIGHになった瞬間）
      pirFlag = true;
      mynet.send(MCID_MASTER_MEGA, SDPP_ON_REPORT ,PID_MEGA_SOC_PIR);
    }
    previousPIR = true;
  } else {
    digitalWrite(PIR_LED, LOW);
    previousPIR = false;
  }
  
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
  if (Serial3.available())
  {
    Serial.println("------r3------");
    delay(10);
    mynet.receive(Serial3);
  }
  processMyData();
  //Serial Process---end----------------------------

  //loopcount process-------------------------------
  if (loopcount == 0 ) {
    loopZero();
  }

  if ( (loopcount % 10 - 3) == 0) loop10();

  if ( (loopcount % 100 - 17) == 0) loop100();

  if (loopcount == 600) {
    mynet.send(MCID_GATE_1, SDPP_PERIODIC_CONFIRMATION);
  }
  if (loopcount == 300) {
    mynet.send(MCID_MASTER_MEGA, SDPP_PERIODIC_CONFIRMATION);
  }
  //loopcount process---end-------------------------

  //Flag process------------------------------------
  if (bGateLedSwToggle)  {
    btGateLedSw.second = rtc.now().unixtime();
    gateLedSwToggle();
  }
  if (timeSyncFlag) {
    mynet.send(MCID_MASTER_MEGA, SDPP_TIME_REQUEST , PID_TIME);
    timeSyncFlag = false;
  }
  //Flag process-----end----------------------------

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


//--------lightsOffSet↑-----------------------------------------------------------------------------------------------

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

void checkChangedTime() {
  if (ntime.unixtime() == 0) return;
  if (itLightsOffTime.second - 100 > ntime.unixtime()) {
    itLightsOffTime.second = 0;
  }
  if (btGateLedSw.second - 100 > ntime.unixtime()) {
    btGateLedSw.second = 0;
  }
}

void loopZero() {
  if ( (itGateLed.second + 30) < rtc.now().unixtime())  {
    itGateLed.first = UNKNOWN_STATE;
    digitalWrite(GATE_LED_STATE_PIN, LOW);
    mynet.send(MCID_MASTER_MEGA, SDPP_UNKNOWN_REPORT, PID_GATE_LED);
  }
  if ( (ftGatePhoto.second + 30) < rtc.now().unixtime())  {
    ftGatePhoto.first = MyNetSerial::FLOAT_UNKNOWN;
    mynet.sendFloat(MyNetSerial::FLOAT_UNKNOWN, MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_PHOTO);
  }
  if ( (ftGateInsideTemp.second + 30) < rtc.now().unixtime())  {
    ftGateInsideTemp.first = MyNetSerial::FLOAT_UNKNOWN;
    mynet.sendFloat(MyNetSerial::FLOAT_UNKNOWN, MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_1_INSIDE_TEMP);
  }
  if ( (ftGateOutsideTemp.second + 30) < rtc.now().unixtime())  {
    ftGateOutsideTemp.first = MyNetSerial::FLOAT_UNKNOWN;
    mynet.sendFloat(MyNetSerial::FLOAT_UNKNOWN, MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_1_OUTSIDE_TEMP);
  }
}//End Of loopZero

void loop100() {
  if (itLightsOffTime.first == -1)
    mynet.send(MCID_MASTER_MEGA, SDPP_TIME_REQUEST, PID_LIGHTSOFFTIME);
  if (btGateLedSw.second == 0)
    mynet.send(MCID_MASTER_MEGA, SDPP_ONOFF_REQUEST, PID_GATE_LED_SW);
}


void dispStatus() {

  lcd.setCursor(17, 3);
  lcd.print(loopcount / 100);
  lcd.print((loopcount % 100) / 10);
  lcd.print(loopcount % 10);
  if (loopcount % 10 == 0) {
    lcd.setCursor(0, 0);
    if (ftGatePhoto.first == MyNetSerial::FLOAT_UNKNOWN) {
      lcd.print("unknown   ");
    } else {
      lcd.print(ftGatePhoto.first);
      lcd.print("   ");
      lcd.setCursor(7, 0);
      lcd.print('R');
    }
    lcd.setCursor(9, 0);
    if (ftGateInsideTemp.first == MyNetSerial::FLOAT_UNKNOWN) {
      lcd.print("unknown   ");
    } else {
      lcd.print(ftGateInsideTemp.first);
      lcd.print("   ");
      lcd.setCursor(15, 0);
      lcd.print("'C");
    }
    lcd.setCursor(0, 1);
    if (ftGateOutsideTemp.first == MyNetSerial::FLOAT_UNKNOWN) {
      lcd.print("unknown   ");
    } else {
      lcd.print(ftGateOutsideTemp.first);
      lcd.print("   ");
      lcd.setCursor(6, 1);
      lcd.print("'C");
    }

  }

  lcd.display();
}
void loop10() {
  lightOffClock.showNumberDecEx(itLightsOffTime.first, 0b01000000, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0, 0);
  display.print(" MEGA_SOC ");
  display.setCursor(0, 8);
  display.setTextColor(WHITE);
  display.print("Gate_Photo:");
  if (ftGatePhoto.first != MyNetSerial::FLOAT_UNKNOWN) {
    display.print(ftGatePhoto.first);
    display.print("R");
  } else {
    display.print("unknown");
  }
  display.setCursor(0, 16);
  display.print("UnixTime:");
  display.setCursor(63, 16);
  display.print(ntime.unixtime());
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
  display.setCursor(0, 40);
  display.print("GateLed:");
  display.print(itGateLed.first);
  display.setCursor(63, 40);
  display.print(itGateLed.second);
  display.setCursor(63, 48);
  display.print("AlarmOff:");
  display.print(bLedAlarmOff);
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
}

void checkLightsOff() {
  delay(1);
  if ( (itLightsOffTime.first / 100) == ntime.hour() && (itLightsOffTime.first % 100) == ntime.minute())
    delay(1);
}

float cpuVcc()
{
  long sum = 0;
  adcSetup(0x5E); //01011110
  //ADMUXのビットパターン ※at328pとは違う！（ADC Multiplexer Selection Register
  //REFS1:0  REFS0:1  ADLAR:0  MUX4:1  MUX3:1  MUX2:1  MUX1:1  MUX0:0
  //REFS1とREFS０は０は参照電圧の設定
  //ADLARは、読み取った値を格納するレジスタADCLとADCHが左詰(1)か右詰(0)かを設定。aruduinoでは0固定
  //MUX4~0はアナログピン選択のためのビット

  //ADCSRBのビットパターン ※at328pとは違う！（ADC Control and Status Register B
  //-:ACME:-:-:MUX5:ADTS2:ADTS1:ADTS0
  //ACME (Analog Comparator Multiplexed Input 比較測定器
  //ADTSよーわからん0でよし
  //MUX5はアナログピン選択する

  for (int n = 0; n < 10; n++)
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
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
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
      case MCID_MEGA_SOC:
        dataToMe(sd);
        break;
    }
    sd = mynet.containedPick();
  }
}

//argument :ID of the data array(mydata) to be displayed
void dataToMe(SerialData* sd)
{
  switch (sd->sender())
  {
    case MCID_GATE_1:
      dataGate1ToMe(sd);
      break;
    case MCID_MASTER_MEGA:
      dataMasterToMe(sd);
      break;
  }
}

void dataGate1ToMe(SerialData* sd)
{
  switch (sd->pid()) {
    case PID_GATE_LED_SW: { //COMMANDしか来ない前提
        btGateLedSw.second = rtc.now().unixtime();//update timeの更新
        if (sd->sdpp() == SDPP_ON_COMMAND) {
          btGateLedSw.first = ON_STATE;
          digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
          mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_ON_COMMAND, PID_GATE_LED_SW);
        }
        else if (sd->sdpp() == SDPP_OFF_COMMAND) {
          btGateLedSw.first = OFF_STATE;
          digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
          mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
        }

      }//case PID_GATE_LED_SW:
      break;
    case PID_NONE: {
        switch (sd->sdpp()) {
          case SDPP_PERIODIC_CONFIRMATION: //定期確認
            { //switchでネストするときはcaseにカッコが必要
              mynet.send(MCID_GATE_1, SDPP_PERIODIC_CONFIRMATION_BACK);
              break;
            }
            //        case SDPP_PERIODIC_CONFIRMATION_BACK: //定期確認返し
            //          //なんもせん
            //          break;
        }//switch
      }//case PID_NONE:
      break;
    case PID_GATE_PHOTO: { //フォトレジスタ
        ftGatePhoto.first = sd->float_();
        ftGatePhoto.second = rtc.now().unixtime();
        mynet.sendFloat(ftGatePhoto.first, MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_PHOTO);
      }//case PID_GATE_PHOTO:
      break;
    case PID_GATE_1_INSIDE_TEMP: { //中の温度センサ
        ftGateInsideTemp.first = sd->float_();
        ftGateInsideTemp.second = rtc.now().unixtime();
        mynet.sendFloat(ftGateInsideTemp.first, MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_1_INSIDE_TEMP);
      }//case PID_GATE_1_INSIDE_TEMP:
      break;
    case PID_GATE_1_OUTSIDE_TEMP: { //外の温度センサ
        ftGateOutsideTemp.first = sd->float_();
        ftGateOutsideTemp.second = rtc.now().unixtime();
        mynet.sendFloat(ftGateOutsideTemp.first, MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_1_OUTSIDE_TEMP);
      }//case PID_GATE_1_OUTSIDE_TEMP:
      break;

    case PID_GATE_LED: {
        itGateLed.second = rtc.now().unixtime();
        switch (sd->sdpp()) {
          case SDPP_ON_REPORT: {
              itGateLed.first = ON_STATE;
              digitalWrite(GATE_LED_STATE_PIN, HIGH);
              mynet.send(MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED);
            }
            break;
          case SDPP_OFF_REPORT: {
              itGateLed.first = OFF_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
              mynet.send(MCID_MASTER_MEGA, SDPP_OFF_REPORT, PID_GATE_LED);
            }
            break;
        }//switch (sd->sdpp())
      }//case PID_GATE_LED:
      break;
    case PID_GATE_LED_ALARM_OFF: {
        switch (sd->sdpp()) {
          case SDPP_ON_REPORT:
            bLedAlarmOff = true;
            break;
          case SDPP_OFF_REPORT:
            bLedAlarmOff = false;
            break;
        }//switch (sd->sdpp())
      }//case PID_GATE_LED_ALARM_OFF:
      break;

  }//switch (mydata[id].pid)

}

void dataMasterToMe(SerialData* sd)
{
  switch (sd->pid()) {
    case PID_GATE_LED_SW: {
        switch (sd->sdpp()) {
          case SDPP_ON_REPORT: {
              if (btGateLedSw.second > (sd->updatetime() + 1000)) { //こちらの方が情報が新しければ送り返す
                mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED_SW);
                return;
              }
            }//through
          case SDPP_ON_COMMAND: {
              //update timeの更新
              btGateLedSw.second = sd->updatetime();
              btGateLedSw.first = ON_STATE;
              digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
              if (sd->sdpp() == SDPP_ON_COMMAND)
                mynet.send( MCID_GATE_1, SDPP_ON_COMMAND, PID_GATE_LED_SW);
              else
                mynet.send( MCID_GATE_1, SDPP_OFF_REPORT, PID_GATE_LED_SW);
            }
            break;


          case SDPP_OFF_REPORT: {
              if (btGateLedSw.second > (sd->updatetime() + 1000)) { //こちらの方が情報が新しければ送り返す
                mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_OFF_REPORT, PID_GATE_LED_SW);
                return;
              }
            }//through
          case SDPP_OFF_COMMAND: {
              //update timeの更新
              btGateLedSw.second = sd->updatetime();
              btGateLedSw.first = OFF_STATE;
              digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
              if (sd->sdpp() == SDPP_OFF_COMMAND)
                mynet.send( MCID_GATE_1, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
              else
                mynet.send( MCID_GATE_1, SDPP_OFF_REPORT, PID_GATE_LED_SW);
            }
            break;
        }//switch (sd->sdpp())
      }//case PID_GATE_LED_SW:
      break;

    case PID_LIGHTSOFFTIME: {
        switch (sd->sdpp())
        {
          case SDPP_TIME_REQUEST:
            if (itLightsOffTime.first != -1) {//時間を保持していれば送る　保持してなければ何もしない
              mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
            }
            break;
          case SDPP_TIME_REPORT: {
              if (itLightsOffTime.second > (sd->updatetime() + 1000)) { //こちらの方が情報が新しければ送り返す
                mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
                return;
              }
              else { //update timeの更新
                itLightsOffTime.second = sd->updatetime();
              }
              itLightsOffTime.first = sd->int16();
              mynet.sendInt16(itLightsOffTime.first, MCID_GATE_1, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
            }
            break;
        }//switch (mynets.sdpp())

      }//case PID_LIGHTSOFFTIME:
      break;
    case PID_TIME: {//REPORTしか来ない前提
        rtc.adjust( DateTime(sd->updatetime() + 1)  );
      }//case PID_TIME:
      break;
    case PID_NONE: {
        switch (sd->sdpp()) {
          case SDPP_PERIODIC_CONFIRMATION: //定期確認
            { //switchでネストするときはcaseにカッコが必要
              mynet.send(MCID_MASTER_MEGA, SDPP_PERIODIC_CONFIRMATION_BACK);

            }
            break;
            //        case SDPP_PERIODIC_CONFIRMATION_BACK: //定期確認返し
            //          //なんもせん
            //          break;
        }//switch
      }//case PID_NONE:
      break;

  }//switch (mydata[id].pid)

}

void sendLedSwToGate1() {
  if (btGateLedSw.first)
    mynet.send( MCID_GATE_1, SDPP_ON_COMMAND, PID_GATE_LED_SW);
  else
    mynet.send( MCID_GATE_1, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
}
void sendLedSwToMaster() {
  if (btGateLedSw.first)
    mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED_SW);
  else
    mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_OFF_REPORT, PID_GATE_LED_SW);
}
/*
  {
  switch (sd->pid()) {
    case PID_GATE_PHOTO: { //報告しか来ない前提
        fGatePhoto = sd->float_();
      }//case PID_GATE_PHOTO:
      break;
    case PID_GATE_LED:
      { //
        switch (sd->sdpp()) {
          case SDPP_ON_REPORT: {
              iGateLed = ON_STATE;
              digitalWrite(GATE_LED_STATE_PIN, HIGH);
              mynet.send(MCID_ESP32_A, SDPP_ON_REPORT, PID_GATE_LED);
            }//case SDPP_ON_REPORT:
            break;
          case SDPP_OFF_REPORT: {
              iGateLed = OFF_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
              mynet.send(MCID_ESP32_A, SDPP_OFF_REPORT, PID_GATE_LED);
            }//case SDPP_OFF_REPORT:
            break;
          case SDPP_UNKNOWN_REPORT: {
              iGateLed = UNKNOWN_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
              mynet.send( MCID_ESP32_A, SDPP_UNKNOWN_REPORT, PID_GATE_LED);
            }//case SDPP_OFF_REPORT:
            break;
        }//switch (sd->sdpp())

      }//case PID_GATE_LED_SW:
      break;
    case PID_GATE_LED_SW:
      { //ONOFF_REQUESTしか来ない前提
        if (btGateLedSw.first) mynet.sendUT(btGateLedSw.second, MCID_ESP32_B, SDPP_ON_REPORT, PID_GATE_LED_SW);
        else mynet.sendUT(btGateLedSw.second, MCID_ESP32_B, SDPP_OFF_REPORT, PID_GATE_LED_SW);
      }//case PID_GATE_LED_SW:
      break;
    case PID_LIGHTSOFFTIME:
      {
        switch (sd->sdpp())
        {
          case SDPP_TIME_REQUEST://時間リクエスト
            { //switchでネストするときはcaseにカッコが必要
              mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first,
                                MCID_ESP32_B, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
            }//case SDPP_TIME_REQUEST:
            break;
          case SDPP_TIME_REPORT: //時間報告
            { //switchでネストするときはcaseにカッコが必要

              if (itLightsOffTime.second > sd->updatetime() ) { //こちらの方が情報が新しければ送り返す
                mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first,
                                  MCID_ESP32_B, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
                return;
              } else if (itLightsOffTime.second < sd->updatetime() ) { //update timeの更新
                itLightsOffTime.second = sd->updatetime();
              }
              itLightsOffTime.first = sd->int16();
              lightsOffChanged = countsec;
            }//case SDPP_TIME_REPORT:
            break;
        }//switch sdpp
      }//case PID_LIGHTSOFFTIME:
      break;
    case PID_TIME:
      { //TIME_REQUESTしか来ない前提
        mynet.sendUint32(ntime.unixtime() , MCID_ESP32_B, SDPP_TIME_REPORT, PID_TIME); //uint32_t
      }
      break;
    case PID_NONE: {
        switch (sd->sdpp()) {
          case SDPP_PERIODIC_CONFIRMATION: //定期確認
            mynet.send(MCID_ESP32_B, SDPP_PERIODIC_CONFIRMATION_BACK);
            break;
          case SDPP_PERIODIC_CONFIRMATION_BACK: //定期確認返し
            //なんもせん
            break;
        }
      }//case PID_NONE:
      break;
  }// switch pid

  }
*/
