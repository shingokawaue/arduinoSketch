#include "hhhhwhw.h"
const int HWWCOUNT = 179688;
#include "clap.h"
#include <driver/dac.h>
#include "driver/pcnt.h"//pulse counter

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
const int8_t PCNT_H_LIM_VAL = 20; //パルスカウンタの上限
const int8_t PCNT_L_LIM_VAL = -20; //パルスカウンタの下限

const int TOUCH_THRESHOLD = 20;
const uint32_t SAMPLE_DELAY_US = (uint32_t)(1000000 / 44100);
/**********************************                                     */
/*****************************************************************/
const uint8_t RECORD_LED = 2;
const uint8_t PLAY_LED = 5;
const uint8_t HWW_LED = 0;
const uint8_t PULSE_CTRL_PIN = 18; //制御ピン 今回はエンコーダのB相を接続(CLK)
const uint8_t PULSE_INPUT_PIN = 19;//パルスの入力ピン 今回はエンコーダのA相を接続(DT)

const uint8_t PLAY_PIN_L = 25;
const uint8_t PLAY_PIN_R = 26;
const uint8_t MIKE_IN = 35;
//T0 4
//T4 13
//T3 15



/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/


//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼------------------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼----------------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼------------------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼----------------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼-----------------------------
Adafruit_SSD1306 display(-1);//i2c 128_64 OLED display

int16_t pulsecount = 0; //パルスカウント数
int8_t volumeDiv = 0;//再生元をこの数で割る
uint8_t* recordArray = new uint8_t[88200];
boolean TouchedT0 = false;
boolean TouchedT4 = false;
boolean TouchedT3 = false;
//------▼▼▼-------------------------------------------------------------------------------------------------
//--------▼------▼--▼▼---------▼------------------------------------------------------------------------------
//--------▼------▼▼----▼---▼▼▼▼▼▼------▼▼▼-----▼--▼▼-----▼--▼▼-----▼-----▼---▼▼▼▼-------▼--------------
//--------▼------▼------▼-------▼--------▼-----▼----▼▼----▼---▼▼----▼---▼-----▼---▼-----▼--▼▼▼▼▼-----------
//--------▼------▼------▼-------▼--------▼▼▼▼▼----▼----------▼-----------▼-----▼---▼▼▼▼-------▼--------------
//--------▼------▼------▼-------▼--------▼-----------▼----------▼-----------▼-----▼---▼------------▼---------
//------▼▼▼----.▼-----▼-------▼----------▼▼▼------▼----------▼-------------▼▼▼▼---▼------------▼--------------

void gotTouch() {
  TouchedT0 = true;
}
void gotTouch4() {
  TouchedT4 = true;
}
void gotTouch3() {
  TouchedT3 = true;
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
  Serial.begin(115200);
  pinMode(RECORD_LED , OUTPUT);
  pinMode(PLAY_LED , OUTPUT);
  pinMode(HWW_LED , OUTPUT);
  
  pinMode(MIKE_IN , INPUT);
  touchAttachInterrupt(T0, gotTouch, TOUCH_THRESHOLD);
  touchAttachInterrupt(T4, gotTouch4, TOUCH_THRESHOLD);
  touchAttachInterrupt(T3, gotTouch3, TOUCH_THRESHOLD);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D  (for the 128x64)
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
}//End Of setup

//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼---------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼---------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼----------------------------------------
uint16_t loopcount = 0;
void loop() {

  if (TouchedT0 && touchRead(T0) > TOUCH_THRESHOLD ) {
    TouchedT0 = false;
    record();

  }
    if (TouchedT4 && touchRead(T4) > TOUCH_THRESHOLD ) {
    TouchedT4 = false;
    playHww();

  }
  if (TouchedT3 && touchRead(T3) > TOUCH_THRESHOLD ) {
    TouchedT3 = false;
    play();

  }

  pcnt_get_counter_value(PCNT_UNIT_0, &pulsecount);
  if (pulsecount != 0) {
    pcnt_counter_clear(PCNT_UNIT_0);
    volumeDiv += pulsecount;
    if (volumeDiv > 100) volumeDiv = 100;
    if (volumeDiv < 0) volumeDiv = 0;
  }

  loopcount++;
  if (loopcount == 1000) loopcount = 0;
  dispstatus();
  delay(100);
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

void dispstatus(){
  display.clearDisplay();
  display.setTextColor(BLACK,WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(" r_p_test ");
  display.setTextColor(WHITE);
  display.setCursor(0, 8);
  display.print("volumeDiv:");
  display.print(volumeDiv);
    display.setCursor(100, 56);
  display.print(loopcount);
  display.display();
}

void play() {
  digitalWrite(PLAY_LED , HIGH);
  uint64_t count = 0;
  while (count < 88200) {
    dacWrite(PLAY_PIN_L, recordArray[count]  / volumeDiv);
    dacWrite(PLAY_PIN_R, recordArray[count]  / volumeDiv);
    ets_delay_us(SAMPLE_DELAY_US);//44.1kHz
    ++count;
  }
  digitalWrite(PLAY_LED , LOW);
}
void playHww() {
  digitalWrite(HWW_LED , HIGH);
  uint64_t count = 0;
  while (count < HWWCOUNT) {
    dacWrite(PLAY_PIN_L, raw_audio[count] / volumeDiv);
    dacWrite(PLAY_PIN_R, raw_audio[count] / volumeDiv);
    ets_delay_us(SAMPLE_DELAY_US);//44.1kHz
    ++count;
  }
  digitalWrite(HWW_LED , LOW);
}
void record() {
  digitalWrite(RECORD_LED , HIGH);
  for (int i = 0; i < 88200; i++) {
    recordArray[i] = (uint8_t)(analogRead(A7) / 16);//analogReadは1023
    ets_delay_us(SAMPLE_DELAY_US);//44.1kHz
  }
  Serial.println("record done");
  digitalWrite(RECORD_LED , LOW);
}
