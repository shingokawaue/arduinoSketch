#include <Wire.h>//i2c
#include <BME280I2C.h>
#define SERIAL_BAUD 115200
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

#define SER 2 ////8bit shift register (SerialData)
#define RCLK 3 //8bit shift register (LatchClock)
#define SRCLK 4 //8bit shift register (ShiftClock)

#define LED_DELAY 2
#define NUMOF_TEMPCATHODE 3
const uint8_t tempCathode[NUMOF_TEMPCATHODE] ={
  5,
  6,
  7
};
#define NUMOF_HUMCATHODE 3
const uint8_t humCathode[NUMOF_HUMCATHODE] ={
  8,
  9,
  10
};
#define NUMOF_PRESCATHODE 3
const uint8_t presCathode[NUMOF_PRESCATHODE] = {
  11,
  12,
  14
};
#define NUMOF_CATHODE 9
const uint8_t cathode[NUMOF_CATHODE] = {
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  14
};

//  0b10000000   A
//  0b01000000   F
//  0b00100000   B
//  0b00010000   E
//  0b00001000   D
//  0b00000100   dp
//  0b00000010   C
//  0b00000001   G
  const uint8_t  num[] = {
   0b11111010,  //0
   0b00100010,  //1
   0b10111001,  //2
   0b10101011,  //3
   0b01100011,  //4
   0b11001011,  //5
   0b11011011,  //6
   0b10100010,  //7
   0b11111011,  //8
   0b11101011,  //9
   0b00000000, // all off
   0b11111111 //all on
};
////////////////////////////////////////////////////////////
//----------------------------------------------------------
//                  setup
//----------------------------------------------------------
////////////////////////////////////////////////////////////
void setup() {
 pinMode(SRCLK, OUTPUT);//8bit shift register
  pinMode(RCLK,  OUTPUT);//8bit shift register
  pinMode(SER,   OUTPUT);//8bit shift register
  
  for (int i = 0; i < NUMOF_CATHODE; i++){
  pinMode(cathode[i], OUTPUT);//Cathode common (7segmentLED)
  }
   for (int i = 0;i < NUMOF_CATHODE ; i++){
  digitalWrite(cathode[i],HIGH);
  }
  
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  
 while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
}



  int count = 0;
   float temp(NAN), hum(NAN), pres(NAN);
   char tempArray[10];
   char humArray[10];
   char presArray[10];
   ////////////////////////////////////////////////////////////
  //----------------------------------------------------------
  //                    loop
  //----------------------------------------------------------
  ////////////////////////////////////////////////////////////
void loop() {


  //温度の表示
  int j = 0;
  for (int i = 0; i < NUMOF_TEMPCATHODE; i++){
    uint8_t buf = num[ctoi(tempArray[i+j])];
    if(ctoi(tempArray[i+j+1]) == -1){
      buf |= 0b00000100;
      j++;
    }
digitalWrite(RCLK,LOW);//シフトレジスタにはラッチクロックをLOWにしてから書き込む
  shiftOut(SER,SRCLK,LSBFIRST,buf);//シフトレジスタにデータを書き込む便利関数shiftOut
delay(LED_DELAY);
digitalWrite(RCLK,HIGH);//書き込み終わったらラッチクロックをHIGHに（シフトレジスタからパラレル出力される
 cathodeSelect(tempCathode[i]);
  }
  
 //湿度の表示
 j = 0;
  for (int i = 0; i < NUMOF_HUMCATHODE; i++){
    uint8_t buf = num[ctoi(humArray[i+j])];
    if(ctoi(humArray[i+j+1]) == -1){
      buf |= 0b00000100;
      j++;
    }
digitalWrite(RCLK,LOW);//シフトレジスタにはラッチクロックをLOWにしてから書き込む
  shiftOut(SER,SRCLK,LSBFIRST,buf);//シフトレジスタにデータを書き込む便利関数shiftOut
delay(LED_DELAY);
digitalWrite(RCLK,HIGH);//書き込み終わったらラッチクロックをHIGHに（シフトレジスタからパラレル出力される
 cathodeSelect(humCathode[i]);
  }
//気圧の表示
j = 0;
  for (int i = 0; i < NUMOF_PRESCATHODE; i++){
   if(((int)pres >= 100000) && i == 0) continue;
    uint8_t buf = num[ctoi(presArray[i+j])];
    if(ctoi(presArray[i+j+1]) == -1){
      buf |= 0b00000100;
      j++;
    }
digitalWrite(RCLK,LOW);//シフトレジスタにはラッチクロックをLOWにしてから書き込む
  shiftOut(SER,SRCLK,LSBFIRST,buf);//シフトレジスタにデータを書き込む便利関数shiftOut
delay(LED_DELAY);
digitalWrite(RCLK,HIGH);//書き込み終わったらラッチクロックをHIGHに（シフトレジスタからパラレル出力される
 cathodeSelect(presCathode[i]);
  }

  

  if(count == 0) {
    bme280read();
//    ■文字列を文字配列に変換
//　char buf[100]; 
//String str="abcdef";
//int len=str.length(); // 文字列長さ
//str.toCharArray(buf,len); 
String str = String(temp);
int len = str.length();
str.toCharArray(tempArray,len); 

str = String(hum);
len = str.length();
str.toCharArray(humArray,len); 

str = String(pres);
len = str.length();
str.toCharArray(presArray,len); 

Serial.println(temp);
Serial.println(hum);
Serial.println(pres);
Serial.println(tempArray);
Serial.println(humArray);
Serial.println(presArray);
  }


    
      if (count == 99)
    count = 0;
  else
    count++;
}




//////////////////////////////////////////////////////////////////
void cathodeSelect(uint8_t pinId){
  for(int i = 0; i < NUMOF_CATHODE ; i++){
    if(cathode[i] == pinId){
      digitalWrite(cathode[i],LOW);
    }else{
      digitalWrite(cathode[i],HIGH);
    }
  }
}

void bme280read()
{
   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);
}


int ctoi(char c) {
  switch (c) {
    case '0': return 0;
    case '1': return 1;
    case '2': return 2;
    case '3': return 3;
    case '4': return 4;
    case '5': return 5;
    case '6': return 6;
    case '7': return 7;
    case '8': return 8;
    case '9': return 9;
    case '.': return -1;
    default: return 0;
  }
}
