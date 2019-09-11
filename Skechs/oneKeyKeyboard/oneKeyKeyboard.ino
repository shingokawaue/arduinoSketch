#include "Keyboard.h"

const int rowNum = 2;
const int colNum = 2;

const int rowPin[rowNum] = { 2, 3 };
const int colPin[colNum] = { 4, 5 };
const byte keyMap[rowNum][colNum]  = {
  { 0x61, 0x62 },
  { 0x63, 0x64 }
};

bool currentState[rowNum][colNum];
bool beforeState[rowNum][colNum];

int i,j;

void setup() {
  
  for( i = 0; i < rowNum; i++){
    pinMode(rowPin[i],OUTPUT);
  }

  for( i = 0; i < colNum; i++){
    pinMode(colPin[i],INPUT);
  }

  for( i = 0; i < rowNum; i++){
    for( j = 0; j < colNum; j++){
      currentState[i][j] = LOW;
      beforeState[i][j] = LOW;
    }
    digitalWrite(rowPin[i],LOW);
  }

  Serial.begin(9600);//9600bpsでポートを開く
  Keyboard.begin();
}

void loop() {
   for( i = 0; i < rowNum; i++){
    digitalWrite( rowPin[i], HIGH );

    for( j = 0; j < colNum; j++){
      currentState[i][j] = digitalRead(colPin[j]);

      if ( currentState[i][j] != beforeState[i][j] ){

        Serial.print("key(");
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(")");

        if ( currentState[i][j] == HIGH){
          Serial.println(" Push!");
   Keyboard.press( keyMap[i][j] );
        } else {
          Serial.println(" Release!");
   Keyboard.release( keyMap[i][j] );
        }
      beforeState[i][j] = currentState[i][j];
      }
    }
    digitalWrite( rowPin[i], LOW );
   }
}
