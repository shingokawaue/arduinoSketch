#include <MyNetSerial.h>
#include "MyNetStringData.h"
#include "MyToString.h"
#include <EEPROM.h>
#include "Int16TimeCalc.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>//DateTime用　とりあえず
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
#include <TM1637Display.h>//4degit display lightsOffClock
//DS3231 リアルタイムクロックモジュール
#include <RTClib.h>
//-------------------------------------------------------------------------------------------------------
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


//---▼--------▼--------------------▼---▼------------▼---------------------▼▼---▼-------------------------------
//---▼-▼---▼-▼----▼----▼-----▼▼▼▼▼▼-----------▼------▼▼▼----------▼----------▼--▼▼--------▼▼-----------
//---▼---▼----▼-----▼-▼----------▼---▼--------▼▼▼----▼------▼-----▼▼▼▼---▼---▼▼----▼----▼----▼---------
//---▼---------▼------▼------------▼--▼-------▼---▼----▼▼▼▼▼---------▼-----▼---▼------▼---▼▼▼▼▼----------
//---▼---------▼-----▼--------▼▼▼▼▼▼▼-----▼---▼-----▼------▼--------▼-----▼---▼-----▼----▼----------------
//---▼---------▼----▼--------------▼-▼---------▼▼▼-------▼▼▼----------▼-----▼---▼------▼-----▼▼▼---------
/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/
#define SERVICE_UUID        "7e3d2ed7-d652-475a-8d11-c6fbdc0ac387"
#define CHARACTERISTIC_UUID "19eee066-2be8-49ab-b12c-1be06b2f98ed"
const int8_t OFF_STATE = 0;
const int8_t ON_STATE = 1;
const int8_t UNKNOWN_STATE = -1;
const float GATE_PHOTO_VALUE_UNKNOWN = 99999;
const uint8_t EEP_LIGHTSOFFTIME_ADRS = 0;//0~1 int16_t
const uint8_t EEP_LIGHTSOFFTIME_UT_ADRS = 2;//2~5 uint32_t
const uint8_t EEP_GATELEDSW_ADRS = 6;//6 bool
const uint8_t EEP_GATELEDSW_UT_ADRS = 7;//7-10 uint32_t

const uint8_t TOUCH_THRESHOLD_B = 20;
/*****************************************************************/
/* PINS                                                  */
/*****************************************************************/
//T0 = 4;
const uint8_t  GATE_LED_SW_STATE_PIN = 23;
const uint8_t  GATE_LED_STATE_PIN = 5;
const uint8_t RX2 = 16;
const uint8_t TX2 = 17;
//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼--------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼-------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼--------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼---------------------

BLECharacteristic *pCharacteristic;
MyNetSerial mynet(MCID_ESP32_SOC);
StringData mynets(MCID_ESP32_SOC);

#include "HardwareSerial.h"
HardwareSerial MySerial(2);//UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )

std::string previous;
uint16_t loopcount = 0;
int dataOut = -1;//データ重複送信防止カウント
std::pair<int8_t, uint32_t> itGateLed = std::make_pair(UNKNOWN_STATE, 0);//SOCのみpairにして時間管理
std::pair<bool, uint32_t> btGateLedSw = std::make_pair(false, 0);
std::pair<int16_t, uint32_t> itLightsOffTime = std::make_pair(-1, 0);
std::pair<float, uint32_t> ftGatePhoto = std::make_pair(GATE_PHOTO_VALUE_UNKNOWN, 0);//SOCのみpairにして時間管理
DateTime dtime, ntime; //dtime:countsecが０の時の時間　ntime 今の時間
uint32_t countsec = 0;//時計用
uint32_t lightsOffChanged = 0;//lightsOffTimeを変更して　時間記録用
uint32_t gateLedSwChanged = 0;//gateLedSwChangedを変更して　時間記録用
volatile SemaphoreHandle_t timerSemaphore;//タイマー計測用
RTC_DS3231 rtc;//RTC (リアルタイムクロックモジュール　黄色い電池のやつ
long charaStamp = 0;
//------▼▼▼-------------------------------------------------------------------------------------------------
//--------▼------▼--▼▼---------▼------------------------------------------------------------------------------
//--------▼------▼▼----▼---▼▼▼▼▼▼------▼▼▼-----▼--▼▼-----▼--▼▼-----▼-----▼---▼▼▼▼-------▼---------
//--------▼------▼------▼-------▼--------▼-----▼----▼▼----▼---▼▼----▼---▼-----▼---▼-----▼--▼▼▼▼▼------
//--------▼------▼------▼-------▼--------▼▼▼▼▼----▼----------▼-----------▼-----▼---▼▼▼▼-------▼---------
//--------▼------▼------▼-------▼--------▼-----------▼----------▼-----------▼-----▼---▼------------▼---------
//------▼▼▼----.▼-----▼-------▼----------▼▼▼------▼----------▼-------------▼▼▼▼---▼------------▼---------
boolean TouchedT0 = false;
void gotTouch() {
  TouchedT0 = true;
}
//void IRAM_ATTR onTimer() {
//  countsec++;
//  if (dtime.unixtime()) {
//    ntime = DateTime(dtime.unixtime() + (0, 0, countsec));
//  }
//  //loopを確認できるセマフォを与える(安全にdigitalRead/writeするため)
//  xSemaphoreGiveFromISR(timerSemaphore, NULL);
//
//  checkChangedTime();
//}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//                ▼▼▼       ▼▼        ▼             ▼    ▼    ▼▼▼▼
//               ▼          ▼   ▼    ▼▼▼▼          ▼    ▼    ▼     ▼
//                 ▼▼      ▼▼▼▼      ▼             ▼    ▼    ▼▼▼▼
//                    ▼     ▼           ▼             ▼   ▼     ▼
//-----------------▼▼--------▼▼--------▼--------------▼▼-------▼-----------------------------------------------
//-------------------------------------------------------------------------------------------------------

void setup() {

  EEPROM.begin(11);// 仮想EEEPROMを使用するサイズを宣言する
  pinMode(GATE_LED_STATE_PIN, OUTPUT);
  pinMode(GATE_LED_SW_STATE_PIN, OUTPUT);
  touchAttachInterrupt(T0, gotTouch, TOUCH_THRESHOLD_B);
  Serial.begin(115200);
  MySerial.begin(MyNetSerial::BPS , SERIAL_8N1, RX2, TX2); //UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
  mynet.beginPort(MySerial, MCID_GATE_1);
  //mynet.debugSerial();
  mynets.debugSerial();
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  delay(100);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.clearDisplay();
  display.println("Starting BLE work!");
  display.display();

  BLEDevice::init("esp32 SOC");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setValue("I began!!");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  display.println("Characteristic defined! Now you can read it in your phone!");
  display.display();

  //なぜかエラー出るので今は使わない
  //  //セマフォ管理
  //  timerSemaphore = xSemaphoreCreateBinary();
  //  //プリスケーラを80分周器にする(ESP Reference参照)
  //  hw_timer_t * timer = timerBegin(0, 80, true);
  //  //タイムアウトした時に行う処理(関数)を定義
  //  timerAttachInterrupt(timer, &onTimer, true);
  //  //何秒毎に行うか定義
  //  timerAlarmWrite(timer, 1000000, true);
  //  //タイマースタート
  //  timerAlarmEnable(timer);
  //----------------------------------------------------------------------------------

  //DS3231 RTC リアルタイムクロックモジュール----------------------------------------------------------------
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //rtc.adjust(  DateTime(2019, 2, 27, 20, 8, 29)  );//時間調整用　アップロード&再起動　約２9秒
  dtime = rtc.now();
  //-------------------------------------------------------------------------------------------------------
  EEPROM.get<int16_t>(EEP_LIGHTSOFFTIME_ADRS, itLightsOffTime.first);
  EEPROM.get<uint32_t>(EEP_LIGHTSOFFTIME_UT_ADRS, itLightsOffTime.second);
  EEPROM.get<bool>(EEP_GATELEDSW_ADRS, btGateLedSw.first);
  EEPROM.get<uint32_t>(EEP_GATELEDSW_UT_ADRS, btGateLedSw.second);

}//End Of setup
//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼---------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼------------------------------
void loop() {

  if (MySerial.available())
  {
    Serial.print("-r-");
    delay(10);
    mynet.receive(MySerial);
    processMyData();
  }

  if (lightsOffChanged != 0 && (lightsOffChanged + 30) < countsec) {//消灯タイマーの時間を変更して30秒以上経ったらEEPROMに保存
    EEPROM.put(EEP_LIGHTSOFFTIME_ADRS, itLightsOffTime.first);
    EEPROM.put(EEP_LIGHTSOFFTIME_UT_ADRS, itLightsOffTime.second);
    EEPROM.commit();
    lightsOffChanged = 0;
  }

  if (gateLedSwChanged != 0 && (gateLedSwChanged + 30) < countsec) {//街灯ボタンの時間を変更して30秒以上経ったらEEPROMに保存
    EEPROM.put(EEP_GATELEDSW_ADRS, btGateLedSw.first);
    EEPROM.put(EEP_GATELEDSW_UT_ADRS, btGateLedSw.second);
    EEPROM.commit();
    gateLedSwChanged = 0;
  }

  std::string value = pCharacteristic->getValue();
  if  (previous != value) {
    previous = value;
    mynets.read(value);
    processMyStringData();
  }

  if (lightsOffChanged != 0 && (lightsOffChanged + 1) < countsec) {
    //    mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
    setChara(
      mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_DOORSIDE, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
    );
    lightsOffChanged = 0;
  }

  if ( (loopcount % 50) - 10 == 0 && itLightsOffTime.first == -1) {
    setChara(
      mynets.stringFixed(MCID_ESP32_B, SDPP_TIME_REQUEST , PID_LIGHTSOFFTIME)
    );
  }
  
  if (TouchedT0 && touchRead(T0) > TOUCH_THRESHOLD_B ) {
    TouchedT0 = false;
    if(btGateLedSw.first)btGateLedSw.first = false;
    else btGateLedSw.first = true;
    digitalWrite(GATE_LED_SW_STATE_PIN, btGateLedSw.first);

    btGateLedSw.second = rtc.now().unixtime();
    sendESP32_BbtGateLedSw ();
    if (btGateLedSw.first) mynet.send(MCID_GATE_1, SDPP_ON_COMMAND, PID_GATE_LED_SW);
    else mynet.send(MCID_GATE_1, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
  }
  
Serial.println(touchRead(T0));
  if (dataOut > -1) dataOut--;

  if (loopcount == 0) loopZero();

  loopcount++;
  if (loopcount == 1000) loopcount = 0;
  dispStatus();
  delay(10);
}//End Of loop

//-------------------------------------------------------------------------------------------------------
//--▼---------▼--------------------------------------------------------------------------------------------
//--▼▼------▼▼-------▼▼▼▼-----------▼--------▼-----------------------------▼-----▼▼▼--------------------
//--▼--▼--▼--▼-----▼-------▼-----▼▼▼▼▼▼----▼--▼▼--------▼▼▼-------▼▼▼----▼------------------------
//--▼----▼----▼----▼▼▼▼▼▼▼---------▼--------▼▼----▼----▼-----▼----▼---▼-----▼▼▼-------------------
//--▼----------▼----▼-------------------▼--------▼------▼----▼-----▼----▼---▼----------▼-----------------
//--▼----------▼------▼------▼---------▼--------▼------▼----▼-----▼----▼---▼----▼----▼-------------------
//--▼----------▼--------▼▼▼-----------▼--------▼------▼------▼▼▼-------▼▼▼-----▼▼▼--------------------
void loopZero() {
  checkChangedTime();
}

void checkChangedTime() {
  if (itLightsOffTime.second > rtc.now().unixtime()) {
    itLightsOffTime.second = 0;
  }
  if (btGateLedSw.second > rtc.now().unixtime()) {
    btGateLedSw.second = 0;
  }
  if (itGateLed.second > rtc.now().unixtime()) {
    btGateLedSw.second = 0;
  }
  if ( (itGateLed.second + 30) < rtc.now().unixtime())  {
    itGateLed.first = UNKNOWN_STATE;
    digitalWrite(GATE_LED_STATE_PIN, LOW);
    pCharacteristic->setValue(
      mynets.stringFixed(MCID_ESP32_B, SDPP_UNKNOWN_REPORT, PID_GATE_LED)
    );
  }
  if (ftGatePhoto.second > rtc.now().unixtime()) {
    ftGatePhoto.second = 0;
  }
  if ( (ftGatePhoto.second + 30) < rtc.now().unixtime())  {
    ftGatePhoto.first = GATE_PHOTO_VALUE_UNKNOWN;
    pCharacteristic->setValue(
      mynets.stringFloatFixedUT(ftGatePhoto.second, ftGatePhoto.first, MCID_ESP32_B, SDPP_SENSOR_VALUE_REPORT, PID_GATE_PHOTO)
    );
  }
}

void dispStatus() {
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.clearDisplay();
  display.print("C.V.:");
  display.print(previous.c_str());
  display.setCursor(0, 16);
  display.print("lot:");
  display.print(MyToString::int16ToTimeString(itLightsOffTime.first) );
  display.setCursor(112, 16);
  display.print(dataOut);
  //now time
  display.setCursor(0, 56);
  DateTime now = rtc.now();
  display.print(now.year() % 100); display.print('/'); display.print(now.month()); display.print('/');
  display.print(now.day()); display.print(' ');
  String loc;
  if (rtc.now().unixtime() != 0) {
    loc = ( String(now.hour() / 10) + String(now.hour() % 10) + ':'
            + String(now.minute() / 10) + String(now.minute() % 10) );
  } else {
    loc = "??:??";
  }
  display.print(loc);
  display.print(' '); display.print(now.second());
  display.setCursor(0, 48);
  display.print("GateSw:");
  switch (btGateLedSw.first) {
    case 1:
      display.print("ON  ");
      break;
    case 0:
      display.print("OFF ");
      break;
  }
  display.setCursor(63, 48);
  display.print(btGateLedSw.second);
  display.setCursor(0, 24);
  display.print("GateLed:");
  switch (itGateLed.first) {
    case ON_STATE:
      display.print("ON ");
      break;
    case OFF_STATE:
      display.print("OFF");
      break;
    case UNKNOWN_STATE:
      display.print("???");
      break;
  }
  display.setCursor(0, 32);
  display.print("unixtime:");
  display.print( rtc.now().unixtime() );
  display.setCursor(0, 40);
  display.print("Gate_Photo:");
  if (ftGatePhoto.first != GATE_PHOTO_VALUE_UNKNOWN && (ftGatePhoto.second + 30) > rtc.now().unixtime() ) {
    display.print(ftGatePhoto.first);
    display.print("R");
  } else {
    display.print("unknown");
  }
  display.setCursor(106, 56);
  display.print(loopcount);
  display.display();
}

/*****************************************************************/
/* FUNCTIONS related to MyNetString                                           */
/*****************************************************************/

void setChara(std::string val) {
  uint16_t t = millis() - charaStamp;
  if (t < 100) delay(100);
  pCharacteristic->setValue(val);
  charaStamp = millis();
}

void processMyStringData() {
  mynets.setContained(false);
  switch (mynets.sender()) {
    case MCID_ESP32_SOC: {
        return;//自分が書いたものには反応しない
      }
      break;
    case MCID_ESP32_B: {
        switch (mynets.pid()) {
          case PID_GATE_LED_SW:
            processGateLedSwitch();
            break;
          case PID_TIME:
            setChara(
              mynets.stringUint32Fixed(rtc.now().unixtime(), MCID_ESP32_B, SDPP_TIME_REPORT, PID_TIME )
            );
            break;
          case PID_LIGHTSOFFTIME:
            processLightsOffTime();
            break;
        }//switch
      }//case MCID_ESP32_B:
      break;

  }//switch (mynets.sender())
}//End Of processMyData()

void processGateLedSwitch() {
  //Serial.print("pgls,");
  if (mynets.sdpp() == SDPP_ONOFF_REQUEST) {
    sendESP32_BbtGateLedSw();
    return;
  }
  if (btGateLedSw.second > mynets.updatetime() ) { //こちらの方が情報が新しければ送り返す
    return;
  } else if (btGateLedSw.second < mynets.updatetime() ) { //update timeの更新
    btGateLedSw.second = mynets.updatetime();
  }
  switch (mynets.sdpp())
  {
    case SDPP_ON_REPORT:
      btGateLedSw.first = true;
      digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
      mynet.send(MCID_GATE_1, SDPP_ON_COMMAND, PID_GATE_LED_SW);
      break;
    case SDPP_OFF_REPORT:
      btGateLedSw.first = false;
      digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
      mynet.send(MCID_GATE_1, SDPP_OFF_COMMAND, PID_GATE_LED_SW);
      break;
  }//switch (mynets.sdpp())
}

void processLightsOffTime() {
  switch (mynets.sdpp())
  {
    case SDPP_TIME_REQUEST:
      if (itLightsOffTime.first != -1) {//時間を保持していれば送る　保持してなければ何もしない
        setChara(
          mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_B, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
        );
      }
      break;
    case SDPP_TIME_REPORT: {
        if (itLightsOffTime.second > mynets.updatetime() ) { //こちらの方が情報が新しければ送り返す
          setChara(
            mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_B, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
          );
          return;
        }
        else if (itLightsOffTime.second < mynets.updatetime() ) { //update timeの更新
          itLightsOffTime.second = mynets.updatetime();
        }
        itLightsOffTime.first = mynets.int16();
        mynet.sendInt16(itLightsOffTime.first, MCID_GATE_1, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
      }
      break;
  }//switch (mynets.sdpp())
}

void sendESP32_BbtGateLedSw() {
  if (btGateLedSw.second == 0) return;
  if (btGateLedSw.first) {
    setChara(
      mynets.stringFixedUT(btGateLedSw.second, MCID_ESP32_B, SDPP_ON_REPORT, PID_GATE_LED_SW)
    );
  } else {
    setChara(
      mynets.stringFixedUT(btGateLedSw.second, MCID_ESP32_B, SDPP_OFF_REPORT, PID_GATE_LED_SW)
    );
  }
}
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------

void processMyData() {
  SerialData* sd = mynet.containedPick();

  while ( sd != NULL) {
    dataToMe(sd);
    sd = mynet.containedPick();
  }//while

}



void dataToMe(SerialData* sd)//GATE_1からしか来ない前提
{
  switch (sd->pid()) {
    case PID_GATE_LED_SW: { //COMMANDしか来ない前提
        btGateLedSw.second = rtc.now().unixtime();//update timeの更新
        if (sd->sdpp() == SDPP_ON_COMMAND) {
          btGateLedSw.first = ON_STATE;
          digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
        }
        else if (sd->sdpp() == SDPP_OFF_COMMAND) {
          btGateLedSw.first = OFF_STATE;
          digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
        }
        sendESP32_BbtGateLedSw();

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
        setChara(
          mynets.stringFloatFixed(ftGatePhoto.first, MCID_ESP32_B, SDPP_SENSOR_VALUE_REPORT, PID_GATE_PHOTO)
        );
      }//case PID_GATE_PHOTO:
      break;
    case PID_GATE_LED: {
        itGateLed.second = rtc.now().unixtime();
        switch (sd->sdpp()) {
          case SDPP_ON_REPORT: {
              itGateLed.first = ON_STATE;
              digitalWrite(GATE_LED_STATE_PIN, HIGH);
              setChara(
                mynets.stringFixed(MCID_ESP32_B, SDPP_ON_REPORT, PID_GATE_LED)
              );
            }
            break;
          case SDPP_OFF_REPORT: {
              itGateLed.first = OFF_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
              setChara(
                mynets.stringFixed(MCID_ESP32_B, SDPP_OFF_REPORT, PID_GATE_LED)
              );
            }
            break;
        }//switch (sd->sdpp())
      }//case PID_GATE_LED:
      break;

  }//switch (mydata[id].pid)

}
