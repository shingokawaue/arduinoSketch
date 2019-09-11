#include <ArduinoJson.h>
#include <MyNetSerial.h>
#include "MyToString.h"
#include "driver/pcnt.h"//pulse counter
#include "Int16TimeCalc.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
#include <TM1637Display.h>//4degit display lightsOffClock
//using namespace vitcon;
#include <WiFi.h>
#include <HTTPClient.h>
//-------------------------------------------------------------------------------------------------------

typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  float pressure;
  float officeTemp;
  float officeHumi;
  float gate1BoxTemp;
  float gate1OutTemp;
} WEATHER;

//---▼--------▼--------------------▼---▼------------▼---------------------▼▼---▼-------------------------------
//---▼-▼---▼-▼----▼----▼-----▼▼▼▼▼▼-----------▼------▼▼▼----------▼----------▼--▼▼--------▼▼-----------
//---▼---▼----▼-----▼-▼----------▼---▼--------▼▼▼----▼------▼-----▼▼▼▼---▼---▼▼----▼----▼----▼---------
//---▼---------▼------▼------------▼--▼-------▼---▼----▼▼▼▼▼---------▼-----▼---▼------▼---▼▼▼▼▼----------
//---▼---------▼-----▼--------▼▼▼▼▼▼▼-----▼---▼-----▼------▼--------▼-----▼---▼-----▼----▼----------------
//---▼---------▼----▼--------------▼-▼---------▼▼▼-------▼▼▼----------▼-----▼---▼------▼-----▼▼▼---------
/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/
const uint8_t WEATHER_JSON_LENGTH = 200;
const uint8_t TOUCH_THRESHOLD_B = 2;
const uint8_t LOT_PL_VAL = 15;//パルスカウンタで+-する値
const int8_t PCNT_H_LIM_VAL = 20; //パルスカウンタの上限
const int8_t PCNT_L_LIM_VAL = -20; //パルスカウンタの下限
const char* HOST_UNIXTIME = "http://192.168.0.199:80/unixtime.php";
const char* HOST_INSERT_WEATHER = "http://192.168.0.199/dbmynet/weather/insert.php?";
//wifi
const char* ssid     = "TP-LINK_3E8A";
const char* password = "93574297";

const char* host = "192.168.0.199";
const char* streamId   = "....................";
const char* privateKey = "....................";
const int httpPort = 80;
//wifi end
/*****************************************************************/
/* PINS                                                  */
/*****************************************************************/
//T1 0
const uint8_t BUILT_IN_LED = 2;
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

MyNetSerial mynet(MCID_ESP32_C);
HTTPClient http;
#include "HardwareSerial.h"
HardwareSerial MySerial(2);//UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
TM1637Display lightsOffClock(TM1637_LIGHTS_OFF_CLK, TM1637_LIGHTS_OFF_DIO); //set up the 4-Digit Display.


boolean TouchedT0 = false;
boolean minute1 = false;//時間を１ずつ調整する
char* weatherJsonBuf = new char[WEATHER_JSON_LENGTH];
String httpGetString;
uint16_t loopcount = 0;
int dataOut = -1;//データ重複送信防止カウント
int16_t lightsOffTime = -1;
uint32_t countsec = 0;//時計用
uint32_t lightsOffChanged = 0;//lightsOffTimeを変更して　時間記録用
uint16_t lastReceiveMEGA = 0;

volatile SemaphoreHandle_t timerSemaphore;//タイマー計測用
int16_t pulsecount = 0; //パルスカウント数
WEATHER weather = {0, 0, 0, 0, 0, 0, 0, 0, 0};
IPAddress ip(192, 168, 0, 148);           // for fixed IP Address
IPAddress gateway(192,168, 0, 1);        //
IPAddress subnet(255, 255, 255, 0);      //
IPAddress DNS(192, 168, 0, 1);          //

WiFiServer server(80);
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

  touchAttachInterrupt(T0, gotTouch, TOUCH_THRESHOLD_B);
  Serial.begin(115200);
  MySerial.begin(MyNetSerial::BPS , SERIAL_8N1, RX2, TX2); //UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
  mynet.beginPort(MySerial, MCID_MASTER_MEGA);
  mynet.debugSerial();
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // init done
  delay(100);


  //---------wifi-------------------------------------------------------------------------
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Connecting to ");
  display.print(ssid);
  display.display();
  WiFi.config(ip, gateway, subnet, DNS);   // Set fixed IP address
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    display.print(".");
    display.display();
  }

  display.print("");
  display.print("WiFi connected");
  display.print("IP address: ");
  display.print(WiFi.localIP());
  display.display();

  server.begin();
  delay(300);
  //---------wifi------end-------------------------------------------------------------------

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


  if (lightsOffChanged != 0 && (lightsOffChanged + 1) < countsec) {
    mynet.sendInt16(lightsOffTime, MCID_MASTER_MEGA, SDPP_TIME_REPORT, PID_LIGHTSOFFTIME);
    lightsOffChanged = 0;
  }

  if ( (loopcount % 50) - 10 == 0 && lightsOffTime == -1) {
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

uint32_t getUnixTimeFromRaspPi() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  http.begin(HOST_UNIXTIME);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) return (uint32_t)(http.getString().toInt());
  else return 0;
}

void insertToMynetWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, weatherJsonBuf);
  if (error)
    return;
  weather.year = doc["year"];
  weather.month = doc["month"];
  weather.day = doc["day"];
  weather.hour = doc["hour"];
  weather.minute = doc["minute"];
  weather.pressure = doc["pressure"];
  weather.officeTemp = doc["officeTemp"];
  weather.officeHumi = doc["officeHumi"];
  weather.gate1BoxTemp = doc["gate1BoxTemp"];
  weather.gate1OutTemp = doc["gate1OutTemp"];
  String buf = HOST_INSERT_WEATHER;
  buf = buf + "year=" + weather.year + "&month=" + weather.month + "&day=" + weather.day + "&hour=" + weather.hour
        + "&minute=" + weather.minute + "&pressure=" + weather.pressure + "&officeTemp=" + weather.officeTemp
        + "&officeHumi=" + weather.officeHumi + "&gate1BoxTemp=" + weather.gate1BoxTemp
        + "&gate1OutTemp=" + weather.gate1OutTemp;
  http.begin(buf);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    httpGetString = http.getString();
  } else {
    httpGetString = "";
  }
}

void pulseUpdateLightsOff() {//パルスカウンタが動いた時に呼び出される。　MEGAへは更新後数秒待ち、loopから送信
  pcnt_counter_clear(PCNT_UNIT_0);
  if (lightsOffTime == -1) return;
  uint8_t changevalue;
  if (minute1)changevalue = 1;
  else changevalue = LOT_PL_VAL;
  Int16TimeCalc::addMinutes(lightsOffTime , pulsecount * changevalue);
  lightsOffClock.showNumberDecEx(lightsOffTime , 0b01000000, true);
  lightsOffChanged = countsec;
}

void dispStatus() {
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(" ESP32_C ");
  display.setTextColor(WHITE);
  display.setCursor(60, 0);
  display.print("lot:");
  display.print(MyToString::int16ToTimeString(lightsOffTime));
  display.setCursor(106, 8);
  display.print(loopcount);
  display.setCursor(88, 8);
  display.print(dataOut);
  display.setCursor(0 , 8);
  display.print( lastReceiveMEGA );
  display.setCursor(0, 16);
  display.print(weather.year); display.print('/');
  display.print(weather.month); display.print('/');
  display.print(weather.day); display.print('/');
  display.print(weather.hour); display.print(':');
  display.println(weather.minute);
  display.println(weather.pressure);
  display.print(weather.officeTemp); display.print(':'); display.println(weather.officeHumi);
  display.print(weather.gate1BoxTemp); display.print(':'); display.println(weather.gate1OutTemp);
  display.print(httpGetString);
  display.display();
}

//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------
//-------------Methods related to MyNet--------------------------


void processMyData() {
  SerialData* sd = mynet.containedPick();

  while ( sd != NULL) {
    digitalWrite(BUILT_IN_LED, HIGH);

    switch (sd->receiver())
    { //受信者MCIDで分岐
      case MCID_ESP32_C:
        dataToMe(sd);
        break;
    }
    sd = mynet.containedPick();
  }//while

}



void dataToMe(SerialData* sd)//基本MASTER_MEGAからしか来ない前提
{
  switch (sd->pid()) {
    case PID_LIGHTSOFFTIME:
      {
        switch (sd->sdpp()) {
          case SDPP_TIME_REPORT:
            lightsOffTime = sd->int16();
            lightsOffClock.showNumberDecEx(lightsOffTime , 0b01000000, true);
            break;
        }//switch
      }//case PID_MASTER_LIGHTSOFFTIME:
      break;
    case PID_TIME: { //sdpp_time_requestしか来ない前提
        uint32_t ut = getUnixTimeFromRaspPi();
        if (ut != 0)
          mynet.sendUT(ut , MCID_MASTER_MEGA , SDPP_TIME_REPORT , PID_TIME);
      }
      break;
    case PID_DB_WEATHER:
      {
        weatherJsonBuf = sd->charHeap();
        insertToMynetWeather();
      }
      break;
    case PID_DOOR_INSIDE_PIR:
      {
        if (WiFi.status() != WL_CONNECTED) return;
        http.begin("http://192.168.0.150/BUZZER");
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
          httpGetString = http.getString();
        } else {
          httpGetString = "";
        }
                http.begin("http://192.168.0.149/BUZZER");
        httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
          httpGetString = http.getString();
        } else {
          httpGetString = "";
        }
      }//PID_DOOR_INSIDE_PIR:
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
