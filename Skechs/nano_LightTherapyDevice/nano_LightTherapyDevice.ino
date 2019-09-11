#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//#include <Wire.h>

const uint8_t TEMPERATURE_PRECISION = 16;//精度


const int startStopPin = 15;
const int dataPin = 13;  //
const int latchPin = 12;  // 74HC595のST_CPへ
const int clockPin = 14; // 74HC595のSH_CPへ
const uint8_t DS18B20_SINGLE_BUS = 2;

float temp1, temp2, temp3, temp4, temp5;
uint16_t a0val;

bool preStartStop = false;
bool startStopFlag = false;

uint16_t startStamp = 0;

OneWire oneWire(DS18B20_SINGLE_BUS);// Setup a oneWire instance(not just Maxim/Dallas temperature ICs)
DallasTemperature dallastemp(&oneWire);// Pass our oneWire reference to Dallas Temperature.
DeviceAddress thermometer1, thermometer2, thermometer3, thermometer4, thermometer5;
//DeviceAddress thermometer1, thermometer2 ,thermometer3 ,thermometer4 ,thermometer5;
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
bool testbool = false;
uint16_t loopcount = 0;
void setup() {
  Serial.begin(115200);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(startStopPin, INPUT);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  Serial.println("display.begin");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3 (for the 128x32)
  // init done

  delay(100);
  Serial.println("display.began");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 0);
  display.clearDisplay();
  display.println("Starting work!");
  display.display();
  //Wire.begin();
  dallastemp.begin();
  // locate devices on the bus
  display.print("Locating devices...");
  display.print("Found ");
  display.print(dallastemp.getDeviceCount(), DEC);
  display.println(" devices.");

  // report parasite power requirements
  display.print("Parasite power is: ");
  if (dallastemp.isParasitePowerMode()) display.println("ON");
  else display.println("OFF");

  if (!dallastemp.getAddress(thermometer1, 0)) display.println("Unable to find address for Device 0");
  // set the resolution to 9 bit per device
  dallastemp.setResolution(thermometer1, TEMPERATURE_PRECISION);
  if (!dallastemp.getAddress(thermometer2, 1)) display.println("Unable to find address for Device 1");
  // set the resolution to 9 bit per device
  dallastemp.setResolution(thermometer2, TEMPERATURE_PRECISION);
  if (!dallastemp.getAddress(thermometer3, 2)) display.println("Unable to find address for Device 2");
  // set the resolution to 9 bit per device
  dallastemp.setResolution(thermometer3, TEMPERATURE_PRECISION);
  if (!dallastemp.getAddress(thermometer4, 3)) display.println("Unable to find address for Device 3");
  // set the resolution to 9 bit per device
  dallastemp.setResolution(thermometer4, TEMPERATURE_PRECISION);
  if (!dallastemp.getAddress(thermometer5, 4)) display.println("Unable to find address for Device 4");
  // set the resolution to 9 bit per device
  dallastemp.setResolution(thermometer5, TEMPERATURE_PRECISION);
  display.display();
  delay(2000);
}

void loop() {
  a0val = analogRead(A0);

  if (digitalRead(startStopPin)) {
    if (preStartStop == false) startStopFlag = true;
    preStartStop = true;
  } else {
    preStartStop = false;
  }


  //  if (loopcount % 10 == 0){
  //   int j = loopcount / 10 % 8;
  //    // 送信中のlatchPinはグランド(LOW)レベル
  //    digitalWrite(latchPin, LOW);
  //    // シフト演算を使って点灯するLEDを選択しています
  //    shiftOut(dataPin, clockPin, LSBFIRST, ~(1 << j) ); //LSBFIRST:LOWBYTEから読み込む
  //    // 送信終了後latchPinをHIGHにする
  //    digitalWrite(latchPin, HIGH);
  //
  //    testbool = !testbool;
  //    digitalWrite(2, testbool);
  //  }


  if (startStopFlag) {
    if (startStamp == 0){
      startStamp = millis();
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, 0x00); //LSBFIRST:LOWBYTEから読み込む
    digitalWrite(latchPin, HIGH);
    }else{
      startStamp = 0;
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, 0xff); //LSBFIRST:LOWBYTEから読み込む
    digitalWrite(latchPin, HIGH);
    }
    
    startStopFlag = false;
  }

  if (loopcount % 20 == 0) {
    getTemp();
  }

  dispStatus();
  loopcount++;
  if (loopcount == 1000) loopcount = 0;

  delay(100);
}//loop()

void dispStatus() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0, 0);
  display.print(" LightTherapy ");
  display.setTextColor(WHITE);
  display.setCursor(100, 0);
  display.print(a0val);
  display.setCursor(0, 8);
  display.print("t1:");
  display.print(temp1);
  display.print("t2:");
  display.println(temp2);
  display.print("t3:");
  display.print(temp3);
  display.print("t4:");
  display.println(temp4);
  display.print("t5:");
  display.println(temp5);
    display.setCursor(0, 56);
  display.print(startStamp);
  display.setCursor(100, 56);
  display.print(loopcount);
  display.display();
}

void getTemp() {
  dallastemp.requestTemperatures(); // Send the command to get temperatures
  temp1 = dallastemp.getTempCByIndex(3);
  temp2 = dallastemp.getTempCByIndex(2);
  temp3 = dallastemp.getTempCByIndex(1);
  temp4 = dallastemp.getTempCByIndex(0);
  temp5 = dallastemp.getTempCByIndex(4);
}
