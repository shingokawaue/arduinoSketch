#include "Keyboard.h"

const int outputNum = 4;//キーマトリックス用　outputの頭数
const int inputNum = 5;//キーマトリックス用　inputの頭数

const int outputPin[outputNum] = { 2, 3,4,5 };//ｷｰﾏﾄﾘｸｽ用　outputポート番号配列
const int inputPin[inputNum] = { 6, 7, 8, 9 ,10};//ｷｰﾏﾄﾘｸｽ用　inputポート番号配列
const byte keyMap[outputNum][inputNum]  = {
  { 0x61, 0x62 ,0x63, 0x64, 0x65 },
  { 0x66, 0x67, 0x68, 0x69, 0x6a },
  { 0x6b, 0x6c, 0x6d, 0x6e, 0x6f },
  { 0x70, 0x71, 0x72, 0x73, 0x74 }
};//ASCII文字コード

bool currentState[outputNum][inputNum];//現在のloopで押されているかどうか
bool beforeState[outputNum][inputNum];//1loop前に押されていたかどうか

int i,j;
void setup() {//初期化。1回のみ実行
  
  for( i = 0; i < outputNum; i++){
    pinMode(outputPin[i],OUTPUT);
  }

  for( i = 0; i < inputNum; i++){
    pinMode(inputPin[i],INPUT_PULLUP);//アクティブロー回路
    //arduinoに内臓されたプルアップ抵抗を使うので、自分で抵抗を組み込む必要なし
  }//ボタン押したときにLOWになる回路

  for( i = 0; i < outputNum; i++){
    for( j = 0; j < inputNum; j++){
      currentState[i][j] = HIGH;
      beforeState[i][j] = HIGH;
    }
    digitalWrite(outputPin[i],HIGH);
  }

  Serial.begin(9600);//9600bpsでシリアルポートを開く(USBの線で通信開始)
  Keyboard.begin();
}

void loop() {
   for( i = 0; i < outputNum; i++){
    digitalWrite( outputPin[i], LOW );

    for( j = 0; j < inputNum; j++){
      currentState[i][j] = digitalRead(inputPin[j]);//inputポート1巡読み取り

      if ( currentState[i][j] != beforeState[i][j] ){//もし状態が変わってたら!

        Serial.print("key(");
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(")");

        if ( currentState[i][j] == LOW){
          Serial.println(" 押したっっ!!");
   Keyboard.press( keyMap[i][j] );
        } else {
          Serial.println(" 離したっっ!!");
   Keyboard.release( keyMap[i][j] );
        }
      beforeState[i][j] = currentState[i][j];
      }
    }
    digitalWrite( outputPin[i], HIGH );
   }
}
