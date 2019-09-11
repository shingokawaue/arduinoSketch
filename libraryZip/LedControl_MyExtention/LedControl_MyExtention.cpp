#include <LedControl_MyExtention.h>



void LedControl_MyExtention::setCharMat(int addr,char value,int direction = DIR_180){

    switch(direction){
case DIR_0:
for (int i = 0; i < 8; i++){
setRow(addr,i,matlixChars_raw[(uint8_t)value][i]);
}
break;

case DIR_180:
int i;
byte buf[8] = {B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000};
for (i = 0; i < 8; i++){//ビットの左右反転
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B10000000) >> 7;
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B01000000) >> 5;
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B00100000) >> 3;
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B00010000) >> 1;
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B00001000) << 1;
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B00000100) << 3;
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B00000010) << 5;
buf[i] |= (matlixChars_raw[(uint8_t)value][i] & B00000001) << 7;
}
for (i = 0; i < 8; i++){
setRow(addr,7 - i,buf[i]);
Serial.println(buf[i] , BIN);
}
break;
    }
}//setCharMat

void LedControl_MyExtention::setStringMat(int addr,String value,int direction = DIR_180){
    for(int i = 0; i < value.length() ; i++){
setCharMat(addr + i,  value.charAt(i)  ,direction);
    }
}

void LedControl_MyExtention::setIntMat(int addr, int value,int direction = DIR_180){
    String str = String(value);
    for(int i = 0; i < str.length() ; i++){
        setCharMat(addr + i, value.charAt(i)  ,direction);
    }
}

void LedControl_MyExtention::setInt(int addr, int digit, int value){
    String str = String(value);
    for(int i = 0; i < str.length() ; i++){
setChar(addr , digit + i,  value.charAt(i),false);
    }
}
