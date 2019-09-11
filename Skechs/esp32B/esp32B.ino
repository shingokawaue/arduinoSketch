
#include <MyNetSerial.h>
#include "MyNetStringData.h"
#include "MyToString.h"
#include "driver/pcnt.h"//pulse counter
#include "Int16TimeCalc.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
#include <TM1637Display.h>//4degit display lightsOffClock

//-------------------------------------------------------------------------------------------------------
#include "BLEDevice.h"

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
const float GATE_PHOTO_VALUE_UNKNOWN = 99999;
const uint8_t TOUCH_THRESHOLD_B = 2;
const uint8_t LOT_PL_VAL = 15;//パルスカウンタで+-する値
const int8_t PCNT_H_LIM_VAL = 20; //パルスカウンタの上限
const int8_t PCNT_L_LIM_VAL = -20; //パルスカウンタの下限
/*****************************************************************/
/* PINS                                                  */
/*****************************************************************/
//T0 4
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

MyNetSerial mynet(MCID_ESP32_B);
StringData mynets(MCID_ESP32_B);
TM1637Display lightsOffClock(TM1637_LIGHTS_OFF_CLK, TM1637_LIGHTS_OFF_DIO); //set up the 4-Digit Display.
boolean TouchedT0 = false;
boolean minute1 = false;//時間を１ずつ調整する
RTC_DS3231 rtc;
#include "HardwareSerial.h"
HardwareSerial MySerial(2);//UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
std::string preChara;//BLEのやつ
uint16_t loopcount = 0;
int dataOut = -1;//データ重複送信防止カウント
int8_t iGateLed = UNKNOWN_STATE;//SOCのみpairにして時間管理
std::pair<bool, uint32_t> btGateLedSw = std::make_pair(false, 0);
std::pair<int16_t, uint32_t> itLightsOffTime = std::make_pair(-1, 0);
float fGatePhoto = GATE_PHOTO_VALUE_UNKNOWN;//SOCのみpairにして時間管理

//----------------------------BLE------------------------------------------------------
// The remote service we wish to connect to.
static BLEUUID serviceUUID("7e3d2ed7-d652-475a-8d11-c6fbdc0ac387");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("19eee066-2be8-49ab-b12c-1be06b2f98ed");
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
//----------------------------BLE------------------------------------------------------

DateTime dtime, ntime; //dtime:countsecが０の時の時間　ntime 今の時間
uint32_t countsec = 0;//時計用
volatile SemaphoreHandle_t timerSemaphore;//タイマー計測用
uint32_t lightsOffChanged = 0;//lightsOffTimeを変更して　時間記録用
int16_t pulsecount = 0; //パルスカウント数
long charaStamp = 0;
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
  //  lastReceiveMEGA = (countsec + 1) - (mynet.lastReceivePort(MySerial) / 1000);//この行を入れると止まる
  //loopを確認できるセマフォを与える(安全にdigitalRead/writeするため)
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

/*****************************************************************/
/* CLASS FOR BLE                                                  */
/*****************************************************************/
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
    }

    void onDisconnect(BLEClient* pclient) {
      connected = false;
      Serial.println("onDisconnect");
    }
};
/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = true;

      } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks


/*****************************************************************/
/* BLE FUNCTIONS                                                  */
/*****************************************************************/
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  display.print("Notify callback for characteristic ");
  display.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  display.print(" of data length ");
  display.println(length);
  display.print("data: ");
  display.println((char*)pData);
  display.display();
}


bool connectToServer() {
  display.setCursor(0, 10);
  display.print("Forming a connection to ");
  display.println(myDevice->getAddress().toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();
  display.println("- Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address,
  //it will be recognized type of peer device address (public or private)
  display.println("- Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    display.print("Failed to find our service UUID: ");
    display.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    display.display();
    return false;
  }
  display.println("- Found our service");
  display.display();

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    display.print("Failed to find our characteristic UUID: ");
    display.println(charUUID.toString().c_str());
    pClient->disconnect();
    display.display();
    return false;
  }
  display.println("- Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    display.print("The characteristic value was: ");
    display.println(value.c_str());
    display.display();
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
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
  pinMode(GATE_LED_STATE_PIN, OUTPUT);
  pinMode(GATE_LED_SW_STATE_PIN, OUTPUT);
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

  delay(1000);
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  //----------------------------BLE end------------------------------------------------------
  //DS3231 RTC リアルタイムクロックモジュール----------------------------------------------------------------
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //rtc.adjust( DateTime(2019, 3,8, 20, 31, 0)  );//Upload Speed 2000000で、約29秒
  dtime = rtc.now();
  //-------------------------------------------------------------------------------------------------------
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

  //----------------------------------------------------------------------------------//----------------------------------------------------------------------------------
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
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      display.println("We are now connected to the BLE Server.");
    } else {
      display.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    display.display();
    doConnect = false;
  }

  if (pRemoteCharacteristic->canRead()) {//read the characteristic value
    std::string value = pRemoteCharacteristic->readValue();
    if (preChara != value) {
      preChara = value;
      mynets.read(value);
      processMyStringData();
    }
  }

  if (MySerial.available()) {
    mynet.receive(MySerial);
  }
  processMyData();

  if ( (loopcount % 50) - 10 == 0 && itLightsOffTime.first == -1) {
    mynet.send(MCID_MASTER_MEGA, SDPP_TIME_REQUEST , PID_LIGHTSOFFTIME);
  }

  if (lightsOffChanged != 0 && (lightsOffChanged + 1) < countsec) {
    mynet.sendInt16UT(itLightsOffTime.second, itLightsOffTime.first, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
    setChara(
      mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_SOC, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
    );
    lightsOffChanged = 0;
  }

  pcnt_get_counter_value(PCNT_UNIT_0, &pulsecount);
  if (pulsecount != 0) pulseUpdateLightsOff();

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
  display.setTextColor(BLACK, WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(" B ");
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.print("C.V.:");
  display.print(preChara.c_str());
  display.setCursor(0, 16);
  display.print("GateLED:");
  display.print(iGateLed);
  display.setCursor(64, 16);
  display.print("ph:");
  display.print(fGatePhoto / 1000);
  display.print("kR");
  display.setCursor(0, 24);
  display.print("Gate_SW:");
  display.print(btGateLedSw.first);
  display.setCursor(63, 24);
  display.print(btGateLedSw.second);
  display.setCursor(0, 32);
  display.print("lot:");
  display.print(MyToString::int16ToTimeString(itLightsOffTime.first));
  display.setCursor(63, 32);
  display.print(itLightsOffTime.second);
  display.setCursor(112, 40);
  display.print(dataOut);

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
void setChara(std::string val) {
  uint16_t t = millis() - charaStamp;
  if (t < 100) delay(100);
  pRemoteCharacteristic->writeValue(val);
  charaStamp = millis();
}
void sendESP32_SOCbtGateLedSw() {
  if (btGateLedSw.second == 0) return;
  if (btGateLedSw.first) {
    setChara(
      mynets.stringFixedUT(btGateLedSw.second, MCID_ESP32_SOC, SDPP_ON_REPORT, PID_GATE_LED_SW )
    );
  } else {
    setChara(
      mynets.stringFixedUT(btGateLedSw.second, MCID_ESP32_SOC, SDPP_OFF_REPORT, PID_GATE_LED_SW )
    );
  }
}


void processMyStringData() {
  mynets.setContained(false);
  switch (mynets.sender()) {
    case MCID_ESP32_B: {
        return;//自分が書いたものには反応しない
      }
      break;
    case MCID_ESP32_SOC: {
        switch (mynets.pid()) {
          case PID_GATE_LED: {
              switch (mynets.sdpp()) {
                case SDPP_ON_REPORT: {
                    iGateLed = ON_STATE;
                    digitalWrite(GATE_LED_STATE_PIN, HIGH);
                    mynet.send(MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED);
                  }//case SDPP_ON_REPORT:
                  break;
                case SDPP_OFF_REPORT: {
                    iGateLed = OFF_STATE;
                    digitalWrite(GATE_LED_STATE_PIN, LOW);
                    mynet.send( MCID_MASTER_MEGA, SDPP_OFF_REPORT, PID_GATE_LED);
                  }//case SDPP_OFF_REPORT:
                  break;
                case SDPP_UNKNOWN_REPORT: {
                    iGateLed = UNKNOWN_STATE;
                    digitalWrite(GATE_LED_STATE_PIN, LOW);
                    mynet.send( MCID_MASTER_MEGA, SDPP_UNKNOWN_REPORT, PID_GATE_LED);
                  }//case SDPP_OFF_REPORT:
                  break;
              }//switch (sd->sdpp())

            }//case PID_GATE_LED:
            break;
          case PID_GATE_LED_SW: {
              if (mynets.sdpp() == SDPP_ONOFF_REQUEST) {
                sendESP32_SOCbtGateLedSw();
                return;
              }
              switch (mynets.sdpp()) {
                  if (btGateLedSw.second > mynets.updatetime() ) { //こちらの方が情報が新しければ送り返す
                    sendESP32_SOCbtGateLedSw();
                    return;
                  } else if (btGateLedSw.second < mynets.updatetime() ) { //update timeの更新
                    btGateLedSw.second = mynets.updatetime();
                  }
                case SDPP_ON_COMMAND:
                case SDPP_ON_REPORT:
                  btGateLedSw.first = true;
                  digitalWrite(GATE_LED_SW_STATE_PIN, HIGH);
                  sendMEGAbtGateLedSw();
                  break;
                case SDPP_OFF_COMMAND:
                case SDPP_OFF_REPORT:
                  btGateLedSw.first = false;
                  digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
                  sendMEGAbtGateLedSw();
                  break;
              }//switch (mynets.sdpp())
            }//case PID_GATE_LED_SW:
            break;
          case PID_TIME://リクエストしか来ない前提
            if (ntime.unixtime() != 0) {//時間を保持していれば送る　保持してなければ何もしない
              setChara(
                mynets.stringUint32Fixed(ntime.unixtime(), MCID_ESP32_SOC, SDPP_TIME_REPORT, PID_TIME )
              );
            }//case PID_GATE_LED_SW:
            break;
          case PID_GATE_PHOTO: { //報告しか来ない前提
              Serial.print("checkfloat");
              fGatePhoto = mynets.float_();
              mynet.sendFloat(fGatePhoto, MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_PHOTO );
            }//case PID_GATE_PHOTO:
            break;
          case PID_LIGHTSOFFTIME: { //リクエストしか来ない前提
              if (itLightsOffTime.first != -1) {//時間を保持していれば送る　保持してなければ何もしない
                setChara(
                  mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_SOC, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
                );
              }
            }
            break;
        }//switch (mynets.pid())

      }//case MCID_ESP32_SOC:
      break;
  }//switch (mynets.sender())
}//End Of processMyData()

//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
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
      case MCID_ESP32_B:
        dataToMe(sd);
        break;
    }
    sd = mynet.containedPick();
  }//while

}



void dataToMe(SerialData* sd)//基本MASTER_MEGAからしか来ない前提
{
  switch (sd->pid()) {
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
            sendESP32_SOCbtGateLedSw();
            break;
          case SDPP_OFF_COMMAND:
          case SDPP_OFF_REPORT:
            btGateLedSw.first = false;
            digitalWrite(GATE_LED_SW_STATE_PIN, LOW);
            sendESP32_SOCbtGateLedSw();
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
            setChara(
              mynets.stringInt16FixedUT(itLightsOffTime.second, itLightsOffTime.first, MCID_ESP32_SOC, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME )
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
