#include "hhhhwhw.h"
#include <driver/dac.h>
#include "Int16TimeCalc.h"
#include "driver/pcnt.h"//pulse counter
//#include "HardwareSerial.h"
#include <RTClib.h>//DateTime用　とりあえず
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TM1637Display.h>//4degit display lightsOffClock
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif





//---▼--------▼--------------------▼---▼------------▼---------------------▼▼---▼------------------------------------
//---▼-▼---▼-▼----▼----▼-----▼▼▼▼▼▼-----------▼------▼▼▼----------▼----------▼--▼▼--------▼▼--------------
//---▼---▼----▼-----▼-▼----------▼---▼--------▼▼▼----▼------▼-----▼▼▼▼---▼---▼▼----▼----▼----▼------------
//---▼---------▼------▼------------▼--▼-------▼---▼----▼▼▼▼▼---------▼-----▼---▼------▼---▼▼▼▼▼------------
//---▼---------▼-----▼--------▼▼▼▼▼▼▼-----▼---▼-----▼------▼--------▼-----▼---▼-----▼----▼------------------
//---▼---------▼----▼--------------▼-▼---------▼▼▼-------▼▼▼----------▼-----▼---▼------▼-----▼▼▼-------------

const uint8_t TOUCH_THRESHOLD_B = 40;
const uint8_t LOT_PL_VAL = 15;//パルスカウンタで+-する値
const int8_t PCNT_H_LIM_VAL = 20; //パルスカウンタの上限
const int8_t PCNT_L_LIM_VAL = -20; //パルスカウンタの下限
const uint32_t SAMPLE_DELAY_US = (uint32_t)(1000000 / 44100);
/**********************************                                     */
/*****************************************************************/
//T0 4
const uint8_t  GATE_LED_STATE_PIN = 5;
const uint8_t TM1637_LIGHTS_OFF_DIO = 12; //Set the DIO pin connection to the display
const uint8_t TM1637_LIGHTS_OFF_CLK = 13; //Set the CLK pin connection to theC display

const uint8_t RX2 = 16;
const uint8_t TX2 = 17;
const uint8_t PULSE_CTRL_PIN = 18; //制御ピン 今回はエンコーダのB相を接続(CLK)
const uint8_t PULSE_INPUT_PIN = 19;//パルスの入力ピン 今回はエンコーダのA相を接続(DT)

//const uint8_t I2C_SDA = 21;
//const uint8_t I2C_SCL = 22;
const uint8_t GATE_LED_SW_STATE_PIN = 23;

const uint8_t PLAY_PIN = 25;






/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/
const int8_t OFF_STATE = 0;
const int8_t ON_STATE = 1;
const int8_t UNKNOWN_STATE = -1;

//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼------------------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼----------------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼------------------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼----------------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼-----------------------------
HardwareSerial MySerial(2);//UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
Adafruit_SSD1306 display(-1);//i2c 128_64 OLED display
TM1637Display lightsOffClock(TM1637_LIGHTS_OFF_CLK, TM1637_LIGHTS_OFF_DIO); //set up the 4-Digit Display.

std::pair<bool, uint32_t> btGateLED_SW = std::make_pair(false, 0);
std::pair<int16_t, uint32_t> itLightsOffTime = std::make_pair(-1, 0);
std::string preChara;
int8_t iGateLed = UNKNOWN_STATE;
int16_t pulsecount = 0; //パルスカウント数
uint32_t lightsOffChanged = 0;
uint16_t loopcount = 0;
DateTime dtime, ntime; //dtime:countsecが０の時の時間　ntime 今の時間
int32_t countsec = 0;//時計用
volatile SemaphoreHandle_t timerSemaphore;//タイマー計測用
bool clockPoint = true;

//------▼▼▼-------------------------------------------------------------------------------------------------
//--------▼------▼--▼▼---------▼------------------------------------------------------------------------------
//--------▼------▼▼----▼---▼▼▼▼▼▼------▼▼▼-----▼--▼▼-----▼--▼▼-----▼-----▼---▼▼▼▼-------▼--------------
//--------▼------▼------▼-------▼--------▼-----▼----▼▼----▼---▼▼----▼---▼-----▼---▼-----▼--▼▼▼▼▼-----------
//--------▼------▼------▼-------▼--------▼▼▼▼▼----▼----------▼-----------▼-----▼---▼▼▼▼-------▼--------------
//--------▼------▼------▼-------▼--------▼-----------▼----------▼-----------▼-----▼---▼------------▼---------
//------▼▼▼----.▼-----▼-------▼----------▼▼▼------▼----------▼-------------▼▼▼▼---▼------------▼--------------
boolean TouchedT0 = false;
void gotTouch() {
  TouchedT0 = true;
}
void IRAM_ATTR onTimer() {
  countsec++;
  if (dtime.unixtime()) {
    ntime = DateTime(dtime.unixtime() + (0, 0, countsec));
    if (clockPoint) clockPoint = false;
    else clockPoint = true;
  }
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
  pinMode(GATE_LED_SW_STATE_PIN , OUTPUT);
  pinMode(GATE_LED_STATE_PIN , OUTPUT);
  touchAttachInterrupt(T0, gotTouch, TOUCH_THRESHOLD_B);
digitalWrite(GATE_LED_SW_STATE_PIN , HIGH);
  // initialize both serial ports:
  Serial.begin(115200);
  while (!Serial);
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D  (for the 128x64)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  delay(100);

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
  lightsOffClock.showNumberDecEx(0 , 0b01000000, true); //Display the numCounter value;

  Serial.println("set up finish");
}//End Of setup

//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼---------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼---------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼----------------------------------------

void loop() {


  if (TouchedT0 && touchRead(T0) > TOUCH_THRESHOLD_B ) {
    TouchedT0 = false;
    play(raw_audio,sizeof(raw_audio));

  }

  pcnt_get_counter_value(PCNT_UNIT_0, &pulsecount);
  if (pulsecount != 0) pulseUpdateLightsOff();

  loopcount++;
  if (loopcount == 1000) loopcount = 0;
  dispStatus();
  delay(10);
}// End Of loop

//-------------------------------------------------------------------------------------------------------
//--▼---------▼--------------------------------------------------------------------------------------------
//--▼▼------▼▼-------▼▼▼▼-----------▼--------▼-----------------------------▼-----▼▼▼------------------------
//--▼--▼--▼--▼-----▼-------▼-----▼▼▼▼▼▼----▼--▼▼--------▼▼▼-------▼▼▼----▼-----------------------------
//--▼----▼----▼----▼▼▼▼▼▼▼---------▼--------▼▼----▼----▼-----▼----▼---▼-----▼▼▼-------------------------
//--▼----------▼----▼-------------------▼--------▼------▼----▼-----▼----▼---▼----------▼-----------------------
//--▼----------▼------▼------▼---------▼--------▼------▼----▼-----▼----▼---▼----▼----▼------------------------
//--▼----------▼--------▼▼▼-----------▼--------▼------▼------▼▼▼-------▼▼▼-----▼▼▼-------------------------

/*****************************************************************/
/* UTILITY FUNCTIONS                                                  */
/*****************************************************************/

void play(const unsigned char* ptr, uint32_t size) {
  digitalWrite(GATE_LED_SW_STATE_PIN,HIGH);
  uint64_t count = 0;
  while (count < size) {
    dacWrite(PLAY_PIN, ptr[count] / 2);
    ets_delay_us(SAMPLE_DELAY_US);//44.1kHz
    ++count;
  }
  digitalWrite(GATE_LED_SW_STATE_PIN,LOW);
}

void pulseUpdateLightsOff() {
  pcnt_counter_clear(PCNT_UNIT_0);
  if (itLightsOffTime.first == -1) return;
  Int16TimeCalc::addMinutes(itLightsOffTime.first , pulsecount * LOT_PL_VAL);
  if (ntime.unixtime() != 0) itLightsOffTime.second = ntime.unixtime();
  lightsOffClock.showNumberDecEx(itLightsOffTime.first , 0b01000000, true);
  lightsOffChanged = countsec;
}



void dispStatus() {
  display.clearDisplay();
  //connection
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("led:");
  display.print(iGateLed);
  display.setCursor(46, 0);
  display.print("ut:");
  display.print(ntime.unixtime() );
  //characteristic value
  display.setCursor(0, 38);
  display.print("charaVal: ");
  display.println(preChara.c_str());
  //loopcount
  display.setCursor(96, 56);
  display.print(loopcount);
  //countsec
  display.setCursor(10, 56);
  display.print(countsec);
  //gateLED_SW
  display.setCursor(0, 9);
  display.print("SW:");
  display.print(btGateLED_SW.first);
  display.print(":");
  display.print(btGateLED_SW.second % 100000);
  display.setCursor(40, 56);
  display.print(itLightsOffTime.first);
  display.setCursor(70, 9);
  display.print("lot:");
  display.print(itLightsOffTime.second % 100000);

  //now time
  display.setCursor(5, 16);
  display.setTextSize(3);
  String loc;
  if (ntime.unixtime() != 0) {
    loc = ( String(ntime.hour() / 10) + String(ntime.hour() % 10) + ':'
            + String(ntime.minute() / 10) + String(ntime.minute() % 10) );
  } else {
    loc = "??:??";
  }
  display.print(loc);
  display.setTextSize(2);
  display.setCursor(100, 21);
  display.print(ntime.second());
  display.setTextSize(1);
  display.display();
}
