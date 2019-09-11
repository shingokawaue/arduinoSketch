#include <ArduinoSTL.h>
#include <MyNetSerial.h>

#include <RTClib.h>//DateTime用　とりあえず

#include <Wire.h>
//-------------------------------------------------------------------------------------------------------
// include the SevenSegmentTM1637 library
#include "SevenSegmentTM1637.h"
#include "SevenSegmentExtended.h"
// Module connection pins (Digital Pins)
#define TM1637_LIGHTS_OFF_CLK 2
#define TM1637_LIGHTS_OFF_DIO 3
SevenSegmentExtended lightsOffClock(TM1637_LIGHTS_OFF_CLK, TM1637_LIGHTS_OFF_DIO);

//-------------------------------------------------------------------------------------------------------

MyNetSerial mynet(MCID_NANO_DOORSIDE);
int8_t lightsOffHour = -1;
int8_t lightsOffMinute = -1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(MyNetSerial::BPS);//esp32_doorside
  mynet.beginPort(Serial,MCID_ESP32_DOORSIDE);
  mynet.debugSerial();
  lightsOffClock.begin();            // initializes the display
  lightsOffClock.setBacklight(100);  // set the brightness to 100 %
  //lightsOffClock.printTime(lightsOffHour, lightsOffMinute, true);
    lightsOffClock.printTime(3, 0, true);
}
uint16_t loopcount = 0;
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available())
  {
    Serial.println("receive");
    mynet.receive();
    processMyData();
  }
  loopcount++;
  if (loopcount == 100) loopcount = 0;
  Serial.print(loopcount);
  delay(100);
}

void processMyData()
{
    SerialData* sd = mynet.containedPick();
  while ( sd != NULL){
      if (sd->sender() == MCID_ESP32_DOORSIDE) {
        esp32DoorSideToMe(sd);
        continue;
      }
    sd = mynet.containedPick();
  }//while
  
}

void esp32DoorSideToMe(SerialData* sd) {
  switch (sd->sdpp()) {
    case SDPP_TIME_REPORT:
      int buf = sd->int16();
      Serial.println(buf);
      lightsOffHour = buf / 100;
      lightsOffMinute = buf % 100;
      lightsOffClock.printTime(lightsOffHour, lightsOffMinute, true);
      break;
  }
}
