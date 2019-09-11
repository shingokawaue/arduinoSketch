#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64 )
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
#include <WiFi.h>
#include <HTTPClient.h>
#include <driver/dac.h>
#include "esp32dactone.h"
#include <RTClib.h>
#include "DateTimeToString.h"
//---▼--------▼--------------------▼---▼------------▼---------------------▼▼---▼-------------------------------
//---▼-▼---▼-▼----▼----▼-----▼▼▼▼▼▼-----------▼------▼▼▼----------▼----------▼--▼▼--------▼▼-----------
//---▼---▼----▼-----▼-▼----------▼---▼--------▼▼▼----▼------▼-----▼▼▼▼---▼---▼▼----▼----▼----▼---------
//---▼---------▼------▼------------▼--▼-------▼---▼----▼▼▼▼▼---------▼-----▼---▼------▼---▼▼▼▼▼----------
//---▼---------▼-----▼--------▼▼▼▼▼▼▼-----▼---▼-----▼------▼--------▼-----▼---▼-----▼----▼----------------
//---▼---------▼----▼--------------▼-▼---------▼▼▼-------▼▼▼----------▼-----▼---▼------▼-----▼▼▼---------
/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/
const int TOUCH_THRESHOLD = 20;
//wifi
const char* ssid     = "TP-LINK_3E8A";
const char* password = "93574297";
const int httpPort = 80;
//wifi end
/*****************************************************************/
/* PINS                                                  */
/*****************************************************************/
//T0 4
const uint8_t BUILT_IN_LED = 2;
const uint8_t RX2 = 16;
const uint8_t TX2 = 17;
const uint8_t PLAY_PIN_L = 25;
const uint8_t PLAY_PIN_R = 26;
const uint8_t AMP_ONOFF_PIN = 27;
//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼--------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼-------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼--------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼---------------------
RTC_DS3231 rtc;
HTTPClient http;
#include "HardwareSerial.h"
HardwareSerial MySerial(2);//UART2 ( デフォルト RX=GPIO16, TX=GPIO17 )
bool toggle = false;
bool playFlag = false;
String httpGetString;
uint16_t loopcount = 0;
uint32_t countsec = 0;//時計用
volatile SemaphoreHandle_t timerSemaphore;//タイマー計測用
IPAddress ip(192, 168, 0, 149);           // for fixed IP Address
IPAddress gateway(192,168, 0, 1);        //
IPAddress subnet(255, 255, 255, 0);      //
IPAddress DNS(192, 168, 0, 1);          //

WiFiServer server(80);

DateTime preBuzzerTime,prepreBuzzerTime;

boolean TouchedT0 = false;
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
    touchAttachInterrupt(T0, gotTouch, TOUCH_THRESHOLD);
    dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  pinMode(AMP_ONOFF_PIN, OUTPUT);
  Serial.begin(115200);
  pinMode(5, OUTPUT);      // set the LED pin mode
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
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
 //DS3231 RTC リアルタイムクロックモジュール----------------------------------------------------------------
  if (! rtc.begin()) {
    display.println("Couldn't find RTC");
    while (1);
  }
  display.println("find RTC!");
  display.display();
  //rtc.adjust( DateTime(2019, 6,2, 17, 10, 0)  );
  display.println("TC!");
  display.display();

  //-------------------------------------------------------------------------------------------------------
}//End Of setup
//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼---------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼------------------------------
void loop() {

  if (TouchedT0 && touchRead(T0) > TOUCH_THRESHOLD ) {
    TouchedT0 = false;
    play();

  }


 WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

             // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/BUZZER\">here</a> to buzzer on.<br>");
            
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /BUZZER":
        if (currentLine.endsWith("GET /BUZZER")) {
          playFlag = true;
          prepreBuzzerTime = preBuzzerTime;
          preBuzzerTime = rtc.now().unixtime();
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
  if (loopcount % 100 == 0){
    
  }
  
if(playFlag){
  play();
  playFlag = false;
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
void play() {
  digitalWrite(AMP_ONOFF_PIN , HIGH);
  delay(1000);
  esp32dactone::buzz2526(536,500);
  esp32dactone::buzz2526(715,500);
    esp32dactone::buzz2526(536,500);
  esp32dactone::buzz2526(715,500);
    esp32dactone::buzz2526(536,500);
  esp32dactone::buzz2526(715,500);
  delay(100);
  digitalWrite(AMP_ONOFF_PIN , LOW);
}
void dispStatus() {
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(" ESP32_OMOYA ");
  display.setTextColor(WHITE);
  display.setCursor(0, 8);
  display.print(WiFi.localIP());display.print(" connect:");
  display.setCursor(0, 16);
  display.print("WiFi.status:");
  display.print(WiFi.status());
   display.setCursor(0, 24);
  display.print(DateTimeToString::HourMinute(preBuzzerTime));
    display.setCursor(40, 24);
  display.print(DateTimeToString::HourMinute(prepreBuzzerTime));
    //now time
  display.setCursor(0, 48);
  display.print(DateTimeToString::YearToSecond(rtc.now()));
  display.setCursor(106, 56);
  display.print(loopcount);
  display.display();
}
