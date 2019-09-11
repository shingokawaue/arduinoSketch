
#include <MyNetSerial.h>
#include "MyNetStringData.h"
#include "MyToString.h"
#include "driver/pcnt.h"//pulse counter
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
//using namespace vitcon;
//-------------------------------------------------------------------------------------------------------
#include "BLEDevice.h"
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
const int8_t OFF_STATE = 0;
const int8_t ON_STATE = 1;
const int8_t UNKNOWN_STATE = -1;
const uint8_t TOUCH_THRESHOLD_B = 2;
const uint8_t LOT_PL_VAL = 15;//パルスカウンタで+-する値
const int8_t PCNT_H_LIM_VAL = 20; //パルスカウンタの上限
const int8_t PCNT_L_LIM_VAL = -20; //パルスカウンタの下限
/*****************************************************************/
/* PINS                                                  */
/*****************************************************************/
//T1 0
const uint8_t BUILT_IN_LED = 2;
const uint8_t  GATE_LED_SW_STATE_PIN = 23;
const uint8_t  GATE_LED_STATE_PIN = 5;
const uint8_t RX2 = 16;
const uint8_t TX2 = 17;
const uint8_t TM1637_LIGHTS_OFF_CLK = 14; //Set the CLK pin connection to the display
const uint8_t TM1637_LIGHTS_OFF_DIO = 15; //Set the DIO pin connection to the display
const uint8_t PULSE_INPUT_PIN = 34;//パルスの入力ピン 今回はエンコーダのA相を接続(DT)
const uint8_t PULSE_CTRL_PIN = 35; //制御ピン 今回はエンコーダのB相を接続(CLK)
//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼--------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼-------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼--------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼---------------------

MyNetSerial mynet(MCID_ESP32_A);
StringData mynets(MCID_ESP32_A);

#include "HardwareSerial.h"
HardwareSerial MySerial(2);//UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
TM1637Display lightsOffClock(TM1637_LIGHTS_OFF_CLK, TM1637_LIGHTS_OFF_DIO); //set up the 4-Digit Display.


boolean TouchedT0 = false;
boolean minute1 = false;//時間を１ずつ調整する

std::string preChara;//BLEのやつ
uint16_t loopcount = 0;
int dataOut = -1;//データ重複送信防止カウント
std::pair<bool, uint32_t> btGateLedSw = std::make_pair(false, 0);
std::pair<int16_t, uint32_t> itLightsOffTime = std::make_pair(-1, 0);
int8_t iGateLed = UNKNOWN_STATE;
DateTime dtime, ntime; //dtime:countsecが０の時の時間　ntime 今の時間
uint32_t countsec = 0;//時計用
uint32_t lightsOffChanged = 0;//lightsOffTimeを変更して　時間記録用
uint16_t lastReceiveMEGA = 0;

volatile SemaphoreHandle_t timerSemaphore;//タイマー計測用

int16_t pulsecount = 0; //パルスカウント数

//----------------------------BLE------------------------------------------------------
// The remote service we wish to connect to.
static BLEUUID SERVICE_UUID("aa3fb0cd-1380-4939-a496-d99dc3147984");
// The characteristic of the remote service we are interested in.
static BLEUUID CHARACTERISTIC_UUID("d58496a1-bf67-42f4-b727-3123ea7e1e31");
BLECharacteristic *pCharacteristic;
std::string cvalue;
long charaStamp = 0;
//----------------------------BLE------------------------------------------------------
//------▼▼▼-------------------------------------------------------------------------------------------------
//--------▼------▼--▼▼---------▼------------------------------------------------------------------------------
//--------▼------▼▼----▼---▼▼▼▼▼▼------▼▼▼-----▼--▼▼-----▼--▼▼-----▼-----▼---▼▼▼▼-------▼---------
//--------▼------▼------▼-------▼--------▼-----▼----▼▼----▼---▼▼----▼---▼-----▼---▼-----▼--▼▼▼▼▼------
//--------▼------▼------▼-------▼--------▼▼▼▼▼----▼----------▼-----------▼-----▼---▼▼▼▼-------▼---------
//--------▼------▼------▼-------▼--------▼-----------▼----------▼-----------▼-----▼---▼------------▼---------
//------▼▼▼----.▼-----▼-------▼----------▼▼▼------▼----------▼-------------▼▼▼▼---▼------------▼---------
void gotTouch() {
  TouchedT0 = true;
}

void IRAM_ATTR onTimer() {
  countsec++;
  if (dtime.unixtime()) {
    ntime = DateTime(dtime.unixtime() + (0, 0, countsec));
  }
  lastReceiveMEGA = (countsec + 1) - (mynet.lastReceivePort(MySerial) / 1000);
  //loopを確認できるセマフォを与える(安全にdigitalRead/writeするため)
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
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

void setup() {

  touchAttachInterrupt(T0, gotTouch, TOUCH_THRESHOLD_B);
  pinMode(GATE_LED_SW_STATE_PIN, OUTPUT);
  pinMode(GATE_LED_STATE_PIN, OUTPUT);
  Serial.begin(115200);
  MySerial.begin(MyNetSerial::BPS , SERIAL_8N1, RX2, TX2); //UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
  mynet.beginPort(MySerial, MCID_MASTER_MEGA);
  //mynet.debugSerial();
  mynets.debugSerial();
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // init done
  delay(100);

  //----------------------------BLE------------------------------------------------------
  display.setCursor(0, 0);
  display.println("Starting Arduino BLE Client application...");
  display.display();
  BLEDevice::init("Long name works now");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  setChara("I'm a server!!");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  //----------------------------BLE end------------------------------------------------------

  //---------pulse count-------------------------------------------------------------------------
  pcnt_config_t pcnt_config;//パルスカウント設定用の構造体の宣言
  pcnt_config.pulse_gpio_num = PULSE_INPUT_PIN;
  pcnt_config.ctrl_gpio_num = PULSE_CTRL_PIN;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_DEC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
  pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;
  pcnt_unit_config(&pcnt_config);//ユニット初期化
  pcnt_counter_pause(PCNT_UNIT_0);//カウンタ一時停止
  pcnt_counter_clear(PCNT_UNIT_0);//カウンタ初期化
  pcnt_counter_resume(PCNT_UNIT_0);//カウント開始

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
  lightsOffClock.setBrightness(0x0a); //set the diplay to maximum brightness
  lightsOffClock.showNumberDecEx(0 , 0b01000000, true); //Display the numCounter cvalue;

}//End Of setup
//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼---------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼------------------------------
void loop() {


  if (MySerial.available()) {
    mynet.receive(MySerial);
  }
  processMyData();

  cvalue = pCharacteristic->getValue();
  if  (preChara != cvalue) {
    preChara = cvalue;
    mynets.read(cvalue);
    processMyStringData();
  }

  if (lightsOffChanged != 0 && (lightsOffChanged + 1) < countsec) {
    mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
    setChara(
      mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_DOORSIDE, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
    );
    lightsOffChanged = 0;
  }

  if ( (loopcount % 50) - 5 == 0 && dtime.unixtime() < 1000000) {
    mynet.send(MCID_MASTER_MEGA, SDPP_TIME_REQUEST , PID_TIME);
  }
  if ( (loopcount % 50) - 10 == 0 && itLightsOffTime.first == -1) {
    mynet.send(MCID_MASTER_MEGA, SDPP_TIME_REQUEST , PID_LIGHTSOFFTIME);
  }

  pcnt_get_counter_value(PCNT_UNIT_0, &pulsecount);
  if (pulsecount != 0) pulseUpdateLightsOff();

  if (dataOut > -1) dataOut--;

  if (TouchedT0 && touchRead(T0) >= TOUCH_THRESHOLD_B ) {
    TouchedT0 = false;
    minute1 = !minute1;
  }

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


void checkChangedTime() {
  if (ntime.unixtime() == 0) return;
  if (itLightsOffTime.second - 100 > ntime.unixtime()) {
    itLightsOffTime.second = 0;
  }
  if (btGateLedSw.second - 100 > ntime.unixtime()) {
    btGateLedSw.second = 0;
  }
}

void pulseUpdateLightsOff() {//パルスカウンタが動いた時に呼び出される。　MEGAへは更新後数秒待ち、loopから送信
  pcnt_counter_clear(PCNT_UNIT_0);
  if (itLightsOffTime.first == -1) return;
  uint8_t changevalue;
  if (minute1)changevalue = 1;
  else changevalue = LOT_PL_VAL;
  Int16TimeCalc::addMinutes(itLightsOffTime.first , pulsecount * changevalue);
  if (ntime.unixtime() != 0) itLightsOffTime.second = ntime.unixtime();
  lightsOffClock.showNumberDecEx(itLightsOffTime.first , 0b01000000, true);
  lightsOffChanged = countsec;
}

void dispStatus() {
  display.clearDisplay();
  display.setTextColor(BLACK,WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(" A ");
  display.setTextColor(WHITE);
  display.print("C.V.:");
  display.print(preChara.c_str());
  display.setCursor(0, 24);
  display.print("lot:");
  display.print(MyToString::int16ToTimeString(itLightsOffTime.first));
  display.setCursor(63, 24);
  display.print(itLightsOffTime.second);
  display.setCursor(112, 16);
  display.print(dataOut);
  display.setCursor(0 , 32);
  display.print( lastReceiveMEGA );
  display.setCursor(0, 48);
  display.print("unixtime:");
  display.print(ntime.unixtime());
  //now time
  display.setCursor(0, 56);
  display.print(ntime.year() % 100); display.print('/'); display.print(ntime.month()); display.print('/');
  display.print(ntime.day()); display.print(' ');
  String loc;
  if (ntime.unixtime() != 0) {
    loc = ( String(ntime.hour() / 10) + String(ntime.hour() % 10) + ':'
            + String(ntime.minute() / 10) + String(ntime.minute() % 10) );
  } else {
    loc = "??:??";
  }
  display.print(loc);
  display.print(' '); display.print(ntime.second());
  display.setCursor(106, 56);
  display.print(loopcount);
  display.display();
}

/*****************************************************************/
/* FUNCTIONS related to MyNetString                                           */
/*****************************************************************/
void sendESP32_DbtGateLedSw() {
  if (btGateLedSw.second == 0) return;
  if (btGateLedSw.first) {
    setChara(
      mynets.stringFixedUT(btGateLedSw.second, MCID_ESP32_DOORSIDE, SDPP_ON_REPORT, PID_GATE_LED_SW )
    );
  } else {
    setChara(
      mynets.stringFixedUT(btGateLedSw.second, MCID_ESP32_DOORSIDE, SDPP_OFF_REPORT, PID_GATE_LED_SW )
    );
  }
}


void processMyStringData() {
  mynets.setContained(false);
  switch (mynets.sender()) {
    case MCID_ESP32_A: {
        return;//自分が書いたものには反応しない
      }
    case MCID_ESP32_DOORSIDE: {
        switch (mynets.pid()) {
          case PID_GATE_LED_SW:
            processGateLedSwitch();
            break;
          case PID_TIME:
            if (ntime.unixtime() != 0) {//時間を保持していれば送る　保持してなければ何もしない
              setChara(
                mynets.stringUint32Fixed(ntime.unixtime(), MCID_ESP32_DOORSIDE, SDPP_TIME_REPORT, PID_TIME )
              );
            }
            break;
          case PID_LIGHTSOFFTIME:

            processLightsOffTime();
            break;
        }//switch (mynets.pid())

      }//case MCID_ESP32_A:
      break;
  }//switch (mynets.sender())
}//End Of processMyData()

void processGateLedSwitch() {
  if (mynets.sdpp() == SDPP_ONOFF_REQUEST) {
    sendESP32_DbtGateLedSw();
    return;
  }

  if (btGateLedSw.second > mynets.updatetime() ) { //こちらの方が情報が新しければ送り返す
    sendESP32_DbtGateLedSw();
    return;
  } else if (btGateLedSw.second < mynets.updatetime() ) { //update timeの更新
    btGateLedSw.second = mynets.updatetime();
  }
  switch (mynets.sdpp())
  {
    case SDPP_ON_REPORT:
      btGateLedSw.first = true;
      digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
      sendMEGAbtGateLedSw();
      break;
    case SDPP_OFF_REPORT:
      btGateLedSw.first = false;
      digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
      sendMEGAbtGateLedSw();
      break;
  }//switch (mynets.sdpp())
}

void processLightsOffTime() {

  switch (mynets.sdpp())
  {
    case SDPP_TIME_REQUEST:
      if (itLightsOffTime.first != -1) {//時間を保持していれば送る　保持してなければ何もしない
        setChara(
          mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_DOORSIDE, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
        );
      }
      break;
    case SDPP_TIME_REPORT: {
        if (itLightsOffTime.second > mynets.updatetime() ) { //こちらの方が情報が新しければ送り返す
          setChara(
            mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_DOORSIDE, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
          );
          Serial.println(mynets.updatetime());
          Serial.println(itLightsOffTime.second);
          return;
        }
        else if (itLightsOffTime.second < mynets.updatetime() ) { //update timeの更新
          itLightsOffTime.second = mynets.updatetime();
        }
        itLightsOffTime.first = mynets.int16();
        lightsOffClock.showNumberDecEx(itLightsOffTime.first , 0b01000000, true);
        delay(1);
        mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
      }
      break;
  }//switch (mynets.sdpp())
}
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
void setChara(std::string val) {
  uint16_t t = millis() - charaStamp;
  if (t < 100) delay(100);
  pCharacteristic->setValue(val);
  charaStamp = millis();
}

void sendMEGAbtGateLedSw() {
  if (btGateLedSw.first) {
    mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED_SW);
  } else {
    mynet.sendUT(btGateLedSw.second, MCID_MASTER_MEGA, SDPP_OFF_REPORT, PID_GATE_LED_SW);
  }
}


void processMyData() {
  SerialData* sd = mynet.containedPick();

  while ( sd != NULL) {
    digitalWrite(BUILT_IN_LED, HIGH);

    switch (sd->receiver())
    { //受信者MCIDで分岐
      case MCID_ESP32_A:
        dataToMe(sd);
        break;
    }
    sd = mynet.containedPick();
  }//while

}



void dataToMe(SerialData* sd)//基本MASTER_MEGAからしか来ない前提
{
  switch (sd->pid()) {
    case PID_GATE_LED:
      {

        switch (sd->sdpp()) {
          case SDPP_ON_REPORT: {
              iGateLed = ON_STATE;
              digitalWrite(GATE_LED_STATE_PIN, HIGH);
              setChara(
                mynets.stringFixed( MCID_ESP32_DOORSIDE, SDPP_ON_REPORT, PID_GATE_LED )
              );
            }//case SDPP_ON_REPORT:
            break;
          case SDPP_OFF_REPORT: {
              iGateLed = OFF_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
              setChara(
                mynets.stringFixed( MCID_ESP32_DOORSIDE, SDPP_OFF_REPORT, PID_GATE_LED )
              );
            }//case SDPP_OFF_REPORT:
            break;
          case SDPP_UNKNOWN_REPORT: {
              iGateLed = UNKNOWN_STATE;
              digitalWrite(GATE_LED_STATE_PIN, LOW);
              setChara(
                mynets.stringFixed( MCID_ESP32_DOORSIDE, SDPP_UNKNOWN_REPORT, PID_GATE_LED )
              );
            }//case SDPP_UNKNOWN_REPORT:
            break;
        }//switch (sd->sdpp())

      }//case PID_GATE_LED_SW:
      break;
    case PID_GATE_LED_SW: { //switchでネストするときはcaseにカッコが必要
        if (btGateLedSw.second > sd->updatetime() ) { //こちらの方が情報が新しければ送り返す
          sendMEGAbtGateLedSw();
          return;
        } else if (btGateLedSw.second < sd->updatetime() ) { //update timeの更新
          btGateLedSw.second = sd->updatetime();
        }

        switch (sd->sdpp())
        {
          case SDPP_ON_COMMAND:
          case SDPP_ON_REPORT:
            btGateLedSw.first = true;
            digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
            sendESP32_DbtGateLedSw();
            break;
          case SDPP_OFF_COMMAND:
          case SDPP_OFF_REPORT:
            btGateLedSw.first = false;
            digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
            sendESP32_DbtGateLedSw();
            break;
        }//switch (sd->sdpp())
      }//case PID_GATE_LED_SW:
      break;
    case PID_LIGHTSOFFTIME:
      {
        switch (sd->sdpp()) {
          case SDPP_TIME_REPORT:
            if (itLightsOffTime.second > sd->updatetime() ) { //こちらの方が情報が新しければ送り返す
              mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
              return;
            } else if (itLightsOffTime.second < sd->updatetime() ) { //update timeの更新
              itLightsOffTime.second = sd->updatetime();
            }
            itLightsOffTime.first = sd->int16();
            lightsOffClock.showNumberDecEx(itLightsOffTime.first , 0b01000000, true);
            setChara(
              mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_DOORSIDE, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
            );
            break;
        }//switch
      }//case PID_MASTER_LIGHTSOFFTIME:
      break;
    case PID_TIME://sd->uint32()はunixtime(second)
      {
        dtime = DateTime(sd->uint32() - countsec);
      }//case PID_TIME:
      break;
    case PID_NONE:
      {
        switch (sd->sdpp()) {
          case SDPP_PERIODIC_CONFIRMATION: //定期確認
            { //switchでネストするときはcaseにカッコが必要
              mynet.send(MCID_MASTER_MEGA, SDPP_PERIODIC_CONFIRMATION_BACK);
              break;
            }
            //        case SDPP_PERIODIC_CONFIRMATION_BACK: //定期確認返し
            //          //なんもせん
            //          break;
        }//switch
      }//case PID_NONE:
      break;
  }//switch (mydata[id].pid)

}
