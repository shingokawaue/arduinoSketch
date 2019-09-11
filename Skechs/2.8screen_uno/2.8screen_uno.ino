#include <RTClib.h>//for DateTime and TimeSpan
#include "MyDisp.h"

////////////////TFTLCD////////////////////
// IMPORTANT: ELEGOO_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Elegoo_TFTLCD.h FOR SETUP.
//Technical support:goodtft@163.com

#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library

//#include <Wire.h> // for i2c communication

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Elegoo_TFTLCD tft;


String mcid_arr[7] = {"???", "MEGA", "GateUNO", "esp32A", "esp32d", "GateCon", "NANOd"};
String sdpp_arr[17] = {"???", "value report", "abnormality detection", "eriodic confirm", "on command", "off command",
                       "confirm on" , "confirm off" , "on report" , "off report" , "sw on command" , "sw off command" ,
                       "sw on report" , "sw off report" , "time request" , "time report" , "time command"
                      };
String pid_arr[12] = {"???", "distanceSensor1", "distanceSensor2", "distanceSensor3", "distanceSensor4", "distanceSensor5",
                      "GateUNO cpu" , "GateUNO vcc" , "Gate LED" , "Gate Photo" , "master lightsOffTime", "lightsOffTime"
                     };
////////////////TFTLCD//////END//////////////

//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼--------------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼--------------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼----------------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼-------------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼---------------------------

//MyNetSerial mynet(MCID_SCREEN_UNO);
uint16_t loopcount = 0;
uint32_t megaMasterStamp = 0;
uint32_t megaSocStamp = 0;
uint32_t esp32cStamp = 0;
uint32_t gate1Stamp = 0;

uint32_t countsec = 0;//時計用
DateTime ntime,dtime;
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//------▼▼▼-------------------------------------------------------------------------------------------------
//--------▼------▼--▼▼---------▼------------------------------------------------------------------------------
//--------▼------▼▼----▼---▼▼▼▼▼▼------▼▼▼-----▼--▼▼-----▼--▼▼-----▼-----▼---▼▼▼▼-------▼--------
//--------▼------▼------▼-------▼--------▼-----▼----▼▼----▼---▼▼----▼---▼-----▼---▼-----▼--▼▼▼▼▼--------
//--------▼------▼------▼-------▼--------▼▼▼▼▼----▼----------▼-----------▼-----▼---▼▼▼▼-------▼-----------
//--------▼------▼------▼-------▼--------▼-----------▼----------▼-----------▼-----▼---▼------------▼-------
//------▼▼▼----.▼-----▼-------▼----------▼▼▼------▼----------▼-------------▼▼▼▼---▼------------▼-------


ISR(TIMER1_COMPA_vect) {//TC1 1秒ごとのイベント
  countsec++;
  if (dtime.unixtime() != 0){
  ntime = DateTime(dtime.unixtime() + countsec);
  }
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          setup()

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void setup(void)
{
  // Wire.begin(8);                // join i2c bus with address #8
  // Wire.onReceive(receiveEvent); // register event
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
  Serial.begin(9600);


//割り込み設定
  //Timer/Counter1 16bit(時計用
  TCCR1A = 0b00000000;//コンペアマッチA,B共にオフ　WGM(WaveGenerationMode) 1100 ICR1がTOPのCTCモード
  TCCR1B = 0b00011101;//1024分周
  OCR1A = 15624; //16mHz / 1024 = 15625 (1second)
  ICR1 = 15624; //16mHz / 1024 = 15625 (1second)
  TIMSK1 |= _BV(OCIE1A);//コンペアマッチAの割り込みを有効に。 TIMSK1 - TC1割り込み許可ﾚｼﾞｽﾀ (TC1 Interrupt Mask Register)
  //OCIE1A : ﾀｲﾏ/ｶｳﾝﾀ1比較A割り込み許可 (Output Compare A Match Interrupt Enable)
  sei();//割り込み許可
  //割り込みの設定終わり


////////////////TFTLCD////////////////////
#ifdef USE_Elegoo_SHIELD_PINOUT
//  Serial.println(F("Using Elegoo 2.4\" TFT Arduino Shield Pinout"));
#else
//  Serial.println(F("Using Elegoo 2.4\" TFT Breakout Board Pinout"));
#endif

//  Serial.print("TFT size is ");
//  Serial.print(tft.width());
//  Serial.print("x");
//  Serial.println(tft.height());

  tft.reset();

  uint16_t identifier = tft.readID();
  if (identifier == 0x9325)
  {
//    Serial.println(F("Found ILI9325 LCD driver"));
  }
  else if (identifier == 0x9328)
  {
//    Serial.println(F("Found ILI9328 LCD driver"));
  }
  else if (identifier == 0x4535)
  {
//    Serial.println(F("Found LGDP4535 LCD driver"));
  }
  else if (identifier == 0x7575)
  {
//    Serial.println(F("Found HX8347G LCD driver"));
  }
  else if (identifier == 0x9341)
  {
//    Serial.println(F("Found ILI9341 LCD driver"));
  }
  else if (identifier == 0x8357)
  {
//    Serial.println(F("Found HX8357D LCD driver"));
  }
  else if (identifier == 0x0101)
  {
    identifier = 0x9341;
//    Serial.println(F("Found 0x9341 LCD driver"));
  }
  else if (identifier == 0x1111)
  {
    identifier = 0x9328;
//    Serial.println(F("Found 0x9328 LCD driver"));
  }
  else
  {
//    Serial.print(F("Unknown LCD driver chip: "));
//    Serial.println(identifier, HEX);
//    Serial.println(F("If using the Elegoo 2.8\" TFT Arduino shield, the line:"));
//    Serial.println(F("  #define USE_Elegoo_SHIELD_PINOUT"));
//    Serial.println(F("should appear in the library header (Elegoo_TFT.h)."));
//    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
//    Serial.println(F("Also if using the breakout, double-check that all wiring"));
//    Serial.println(F("matches the tutorial."));
    identifier = 0x9328;
  }
  tft.begin(identifier);
////////////////TFTLCD//////END//////////////


  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.print("test");
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          loop()

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
static const int MARGIN = 82;
static const int ROW_MARGIN = 18;
void loop(void)
{



  unsigned long start = micros();

  /*
    tft.println();
    tft.setTextColor(GREEN);
    tft.setTextSize(5);
    tft.println("Groop");
    tft.setTextSize(2);
    tft.println("I implore thee,");
    tft.setTextSize(1);
    tft.println("my foonting turlingdromes.");
    tft.println("And hooptiously drangle me");
    tft.println("with crinkly bindlewurdles,");
    tft.println("Or I will rend thee");
    tft.println("in the gobberwarts");
    tft.println("with my blurglecruncheon,");
    tft.println("see if I don't!");*/



  if (Serial.available())
  {

  }
  
  //loopcount process-------------------------------

  if ( (loopcount % 10 - 3) == 0) loop10();

  //loopcount process---end-------------------------
  loopcount++;
  if (loopcount == 1000) loopcount = 0;

  dispStatus();
  delay(50);
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          MyMethods

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
//void receiveEvent(int howMany) {//howMany is the num of read bytes from the master
//  while (1 < Wire.available()) { // loop through all but the last
//    char c = Wire.read(); // receive byte as a character
//    tft.print(c);         // print the character
//  }
//  uint8_t x = Wire.read();    // receive byte as an integer
//  tft.println(x);         // print the integer
//}

void loop10(){
if(dtime.unixtime() == 0)
Serial.write('t');
}

void dispStatus() {

  tft.fillScreen(BLACK);
  tft.setCursor(30, 50);
  tft.setTextColor(WHITE);
////  tft.print(MyDisp::zeroInt(ntime.hour(),2));
  tft.print(":");
//  tft.print(MyDisp::zeroInt(ntime.minute(),2));
}
