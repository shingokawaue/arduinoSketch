#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



// If using software SPI (the default case):
#define OLED_MOSI   23
#define OLED_CLK   18
#define OLED_DC    16
#define OLED_CS    15
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

/* Uncomment this block to use hardware SPI
#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);
*/
const int TRIG_1ST = 2; // HY-SRF05(超音波距離センサ)出力ピン
const int ECHO_1ST = 3; // HY-SRF05(超音波距離センサ)入力ピン
const int TRIG_2ND = 4; // HY-SRF05(超音波距離センサ)出力ピン
const int ECHO_2ND = 5; // HY-SRF05(超音波距離センサ)入力ピン
const int KYORISENSORSAMPLINGNUM = 1;//距離センサー、サンプリング数
const int KYORISENSORSAMPLINGMARGIN = 50;//距離センサーのサンプリングの間隔

const int TXDEN = 8;//.;rs485

float distance1st[KYORISENSORSAMPLINGNUM];
float distance2nd[KYORISENSORSAMPLINGNUM];
int sampleCounter = 0;
float longestdistance1st;
float longestdistance2nd;

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
void setup() {
  Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done
    // Clear the buffer.
  display.clearDisplay();

  // draw a single pixel
  display.drawPixel(10, 10, WHITE);
  // Show the display buffer on the hardware.
  // NOTE: You _must_ call display after making any drawing commands
  // to make them visible on the display hardware!
  display.display();
  delay(2000);
  display.clearDisplay();

  // draw many lines
  testdrawline();
  display.display();
  delay(2000);
  display.clearDisplay();

  // draw rectangles
  testdrawrect();
  display.display();
  delay(2000);
  display.clearDisplay();

  // draw multiple rectangles
  testfillrect();
  display.display();
  delay(2000);
  display.clearDisplay();

  // draw mulitple circles
  testdrawcircle();
  display.display();
  delay(2000);
  display.clearDisplay();

  // draw a white circle, 10 pixel radius
  display.fillCircle(display.width()/2, display.height()/2, 10, WHITE);
  display.display();
  delay(2000);
  display.clearDisplay();

  testdrawroundrect();
  delay(2000);
  display.clearDisplay();

  testfillroundrect();
  delay(2000);
  display.clearDisplay();

  testdrawtriangle();
  delay(2000);
  display.clearDisplay();
   
  testfilltriangle();
  delay(2000);
  display.clearDisplay();

  // draw the first ~12 characters in the font
  testdrawchar();
  display.display();
  delay(2000);
  display.clearDisplay();

  // draw scrolling text
  testscrolltext();
  delay(2000);
  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Hello, world!");
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println(3.141592);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print("0x"); display.println(0xDEADBEEF, HEX);
  display.display();
  delay(2000);
  display.clearDisplay();

//  // miniature bitmap display
//  display.drawBitmap(30, 16,  logo16_glcd_bmp, 16, 16, 1);
  display.display();

  // invert the display
  display.invertDisplay(true);
  delay(1000); 
  display.invertDisplay(false);
  delay(1000); 


  pinMode(TRIG_1ST,OUTPUT);
  pinMode(ECHO_1ST,INPUT);
  pinMode(TRIG_2ND,OUTPUT);
  pinMode(ECHO_2ND,INPUT);
  pinMode(TXDEN, OUTPUT);
 digitalWrite(TXDEN, HIGH); // master
  }

void loop() {
sampleCounter = 0;
longestdistance1st = 0;
longestdistance2nd = 0;


  while(sampleCounter < KYORISENSORSAMPLINGNUM){   //サンプリング
  delay(KYORISENSORSAMPLINGMARGIN);
  // 超音波の出力終了
  digitalWrite(TRIG_1ST,LOW);
  digitalWrite(TRIG_2ND,LOW);
  delayMicroseconds(1);
  // 超音波を出力
  digitalWrite(TRIG_1ST,HIGH);
  delayMicroseconds(50);
  // 超音波を出力終了して、出力した超音波が返って来る時間を計測
  digitalWrite(TRIG_1ST,LOW);
  int pulseLenMicroSecond1 = pulseIn(ECHO_1ST,HIGH,35000);
  digitalWrite(TRIG_2ND,HIGH);
  delayMicroseconds(50);
  digitalWrite(TRIG_2ND,LOW);
  int pulseLenMicroSecond2 = pulseIn(ECHO_2ND,HIGH,35000);
  // 計測した時間と音速から反射物までの距離を計算
  if (pulseLenMicroSecond1 == 0) distance1st[sampleCounter] = 455.55;
  else distance1st[sampleCounter] = pulseLenMicroSecond1*0.017;//マイクロ秒ごとに０.０３４cm進む、反射して帰ってくるので２で割る
    if (pulseLenMicroSecond2 == 0) distance2nd[sampleCounter] = 455.55;
  else distance2nd[sampleCounter] = pulseLenMicroSecond2*0.017;//マイクロ秒ごとに０.０３４cm進む、反射して帰ってくるので２で割る
  if(longestdistance1st < distance1st[sampleCounter]) longestdistance1st = distance1st[sampleCounter];
  if(longestdistance2nd < distance2nd[sampleCounter]) longestdistance2nd = distance2nd[sampleCounter];
  sampleCounter++;
  }

  
  // 計算結果をシリアル通信で出力
 float data[1];
 data[0] = longestdistance1st;
//  gateSerial.write((byte*)data, sizeof(float));
  data[0] = longestdistance2nd;
//   gateSerial.write((byte*)data, sizeof(float));
   Serial.print(longestdistance1st);
 Serial.print(" : ");
  Serial.println(longestdistance2nd);
 
}//loop



void testdrawchar(void) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  for (uint8_t i=0; i < 168; i++) {
    if (i == '\n') continue;
    display.write(i);
    if ((i > 0) && (i % 21 == 0))
      display.println();
  }    
  display.display();
}

void testdrawcircle(void) {
  for (int16_t i=0; i<display.height(); i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, WHITE);
    display.display();
  }
}

void testfillrect(void) {
  uint8_t color = 1;
  for (int16_t i=0; i<display.height()/2; i+=3) {
    // alternate colors
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, color%2);
    display.display();
    color++;
  }
}

void testdrawtriangle(void) {
  for (int16_t i=0; i<min(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(display.width()/2, display.height()/2-i,
                     display.width()/2-i, display.height()/2+i,
                     display.width()/2+i, display.height()/2+i, WHITE);
    display.display();
  }
}

void testfilltriangle(void) {
  uint8_t color = WHITE;
  for (int16_t i=min(display.width(),display.height())/2; i>0; i-=5) {
    display.fillTriangle(display.width()/2, display.height()/2-i,
                     display.width()/2-i, display.height()/2+i,
                     display.width()/2+i, display.height()/2+i, WHITE);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    display.display();
  }
}

void testdrawroundrect(void) {
  for (int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i, display.height()/4, WHITE);
    display.display();
  }
}

void testfillroundrect(void) {
  uint8_t color = WHITE;
  for (int16_t i=0; i<display.height()/2-2; i+=2) {
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i, display.height()/4, color);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    display.display();
  }
}
   
void testdrawrect(void) {
  for (int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, WHITE);
    display.display();
  }
}

void testdrawline() {  
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, WHITE);
    display.display();
  }
  for (int16_t i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, WHITE);
    display.display();
  }
  delay(250);
  
  display.clearDisplay();
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, WHITE);
    display.display();
  }
  for (int16_t i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, WHITE);
    display.display();
  }
  delay(250);
  
  display.clearDisplay();
  for (int16_t i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, WHITE);
    display.display();
  }
  for (int16_t i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, WHITE);
    display.display();
  }
  delay(250);

  display.clearDisplay();
  for (int16_t i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, WHITE);
    display.display();
  }
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, WHITE); 
    display.display();
  }
  delay(250);
}

void testscrolltext(void) {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,0);
  display.clearDisplay();
  display.println("scroll");
  display.display();
 
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);    
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
}
