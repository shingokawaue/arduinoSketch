#include <Arduino.h>
#include <ArduinoSTL.h>
#include <MyNetSerial.h>
#include <Wire.h>//lcd
#include <LiquidCrystal_I2C.h>//lcd


union {//float,と１バイトデータ変換用 //MyNetSerial通信
  float f;
  uint8_t u[4];
} Float;


/*****************************************************************/
/*                        PINS                                    */
/*****************************************************************/
const uint8_t PHOTOREGISTER_VOLTAGE_PIN = 0;//フォトレジスタ電圧測定ピン　アナログ０
const uint8_t GATELIGHT_RELAY_PIN = 8; //ゲートの街灯用リレー　OUTPUT
const uint8_t CONTRAST_PIN  = 9;//lcd 16x2
const uint8_t BACKLIGHT_PIN = 7;//lcd 16x2




/*****************************************************************/
/* CONSTANTS                                                  */
/*****************************************************************/
const uint16_t GATELED_ON_THRESHOLD = 6000;
const uint16_t GATELED_OFF_THRESHOLD = 4000;
const uint8_t  CONTRAST = 125;//lcd 16x2


//---▼---------▼-------------------------------------------------------------------------------------------
//----▼-------▼----------------▼-------▼---------------▼---------▼------▼▼--------▼▼------------------------------
//-----▼-----▼-----▼▼▼▼-----▼▼▼-----------▼▼▼----▼▼▼------▼----▼----▼-----▼---▼-----------------------------
//------▼---▼-----▼-----▼-----▼---▼--▼----▼-----▼--▼----▼----▼---▼▼▼▼▼----▼▼▼-------------------------------
//-------▼-▼------▼-----▼-----▼-------▼----▼-----▼--▼----▼----▼----▼---------------▼----------------------------------
//--------▼---------▼▼▼--▼---▼-------▼------▼▼▼-▼--▼▼▼-----▼-----▼▼▼------▼▼▼----------------------------------

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address 16x2
MyNetSerial mynet(MCID_GATE_328P);
float temp , Vcc;
bool photoswitch = false;//フォトレジスタによるスイッチ
bool ledswitch = false;//暗い時にGateLedをつけるかどうかのスイッチ
bool ledison = false;//GateLedが付いているかどうか
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//                ▼▼▼       ▼▼        ▼             ▼    ▼    ▼▼▼▼
//               ▼          ▼   ▼    ▼▼▼▼          ▼    ▼    ▼     ▼
//                 ▼▼      ▼▼▼▼      ▼             ▼    ▼    ▼▼▼▼
//                    ▼     ▼           ▼             ▼   ▼     ▼
//-----------------▼▼--------▼▼--------▼--------------▼▼-------▼-----------------------------------------------
//-------------------------------------------------------------------------------------------------------
void setup() {


  Serial.begin(MyNetSerial::BPS);
  mynet.beginPort(Serial, MCID_MASTER_MEGA);
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(GATELIGHT_RELAY_PIN, OUTPUT);
  //lcdモニターーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー

  lcd.begin(16, 2);
  lcd.backlight();
  for (int i = 0; i < 3; i++) {
    lcd.backlight();
    delay(100);
    lcd.noBacklight();
    delay(100);
  }
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
  //lcdモニタ終ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー

}//setup
//-----------------▼-----------------▼▼▼▼▼-------------▼▼▼▼▼---------▼▼▼▼▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼-----------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------▼------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼▼▼▼▼-------------------------------
//-----------------▼----------------▼--------▼----------▼--------▼-------▼-------------------------------------
//-----------------▼▼▼▼▼▼---------▼▼▼▼▼-------------▼▼▼▼▼--------▼-------------------------------------
int loopcount = 0;

void loop() {

  if (loopcount % 20 == 10) photo(); //フォトトランジスタチェック

  //cpu温度、電圧
  if (loopcount % 20 == 0) {
    temp = cpuTemp();
    Vcc = cpuVcc();

    lcd.setCursor(0, 0);
    lcd.print("cpu");
    lcd.print(temp, 1);
    lcd.setCursor(7, 0);
    lcd.print(": vcc");
    lcd.print(Vcc, 2);
  }

  if (Serial.available())//シリアルデータが届いていれば、mydata[4]に読み込んで、内容を表示、もしくは命令実行
  {
    mynet.receive(Serial);
    processMyData();
  }

  delay(100);
  loopcount++;
  if (loopcount == 1000) loopcount = 0;
}//loop
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//                          MyMethods

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

void photo() {
  int i = analogRead( PHOTOREGISTER_VOLTAGE_PIN );
  float v = i * 5.0 / 1023.0;
  float o = (v * 10000) / ( 5 - v);
  mynet.sendFloat(MCID_MASTER_MEGA, SDPP_SENSOR_VALUE_REPORT, PID_GATE_PHOTO, DTCT_OHM, o);
  if ((long)o > GATELED_ON_THRESHOLD) {
    photoswitch = true;
    if (ledswitch) gateLedOn();
    lcd.setCursor(0, 1);
    lcd.print("t");
    lcd.print((o / 1000) , 1);
    lcd.print("k  ");
  }
  if ((long)o < GATELED_OFF_THRESHOLD) {
    photoswitch = false;
    gateLedOff();
    lcd.setCursor(0, 1);
    lcd.print("f");
    lcd.print((o / 1000) , 1);
    lcd.print("k  ");
  }
}

float cpuTemp() {
  long sum = 0;
  adcSetup(0xC8);
  for (int n = 0; n < 100; n++) {
    sum = sum + adc();
  }
  return (sum * 1.1 / 102.4) - 342.5; //AT328pの内部基準電圧は1.1V
}

float cpuVcc() {
  long sum = 0;
  adcSetup(0x4E);//01001110
  //ADMUXのビットパターン
  //REFS1:REFS0:ADLAR:-:MUX3:MUX2:MUX1:MUX0
  //REFS1とREFS０は０は参照電圧の設定
  //ADLARは、読み取った値を格納するレジスタADCLとADCHが左詰(1)か右詰(0)かを設定。aruduinoでは0固定
  //MUX3~0はアナログピン選択のためのビット
  for (int n = 0; n < 10; n++) {
    sum = sum + adc();
  }
  return (1.1 * 10240.0) / sum;
}

void adcSetup(byte data) {
  ADMUX = data;
  ADCSRA |= (1 << ADSC);
  ADCSRA |= 0x07;
  delay(10);
}

unsigned int adc() {
  unsigned int dL, dH;
  //ADCSRAのビットパターン ※２５６０と同じ！（ADC Control and Status Register A
  //ADEN:ADSC:ADATE:ADIF:ADIE:ADPS2:ADPS1:ADPS0
  ADCSRA |= ( 1 << ADSC);
  while (ADCSRA & (1 << ADSC) ) {
  }
  dL = ADCL;
  dH = ADCH;
  return dL | (dH << 8);
}

void gateLedOn() {
  digitalWrite(GATELIGHT_RELAY_PIN , HIGH);
  mynet.send(MCID_MASTER_MEGA, SDPP_ON_REPORT, PID_GATE_LED);
  lcd.setCursor(13, 1);
  lcd.write("on ");
}
void gateLedOff() {
  digitalWrite(GATELIGHT_RELAY_PIN , LOW);
  mynet.send(MCID_MASTER_MEGA, SDPP_OFF_REPORT, PID_GATE_LED);
  lcd.setCursor(13, 1);
  lcd.write("off");
}


//-------------Methods related to MyNet------------------------------------------------------------------------------

void processMyData()
{
  for (int i = 0; i < 4; i++)
  {
    SerialData* sd = mynet.containedPick();
    while ( sd != NULL) {

      switch (sd->receiver())
      { //受信者MCIDで分岐
        case MCID_GATE_328P:
          dataToMe(sd);
          break;
      }
      sd = mynet.containedPick();
    }
  }
}//End Of processMyData()


//argument :ID of the data array(mydata) to be displayed
void dataToMe(SerialData * sd)
{
  lcd.clear();
  lcd.setCursor(8, 1);
  lcd.print(sd->sdpp());
  lcd.print(sd->value(0));
  lcd.print(sd->value(1));
  lcd.print(sd->value(2));
  lcd.print(sd->value(3));

  switch (sd->pid()) {

    case PID_GATE_LED_SW: {

        switch (sd->sdpp()) {
          case SDPP_ON_COMMAND :
            photo();//フォトトランジスタチェック
          case SDPP_CONFIRM_ON :
            ledswitch = true;
            if (photoswitch) {
              gateLedOn();
            } else {
              gateLedOff();
            }
            break;

          case SDPP_OFF_COMMAND :
          case SDPP_CONFIRM_OFF :
            ledswitch = false;
            gateLedOff();
            break;
        }//switch (sd->sdpp)
        break;
      }//case PID_GATE_LED_SW:
      break;

  }//switch (sd->pid)


}//End Of dataToMe()
