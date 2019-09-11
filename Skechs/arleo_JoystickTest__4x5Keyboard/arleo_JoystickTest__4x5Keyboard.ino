#include "Joystick.h"
#include "Keyboard.h"

const int outputNum = 4;//キーマトリックス用　outputの頭数
const int inputNum = 5;//キーマトリックス用　inputの頭数

//Joystick
//可変抵抗をつなげるpin
const int x_pin = A0;
const int y_pin = A1;

//L3ボタン(11ボタン)に該当するジョイスティック押し込み
const int L3 = 11;

//可変抵抗の値
int x_value = 0;
int y_value = 0;

//AJLクラスオブジェクト
Joystick_ Joystick;
//Joystick


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


//Joystick
  //AJLの初期化
  Joystick.begin(true);

  Joystick.setXAxisRange(266,770);//518が中間値になるように設定
  Joystick.setYAxisRange(245,749);//497が中間値になるように設定

  //L3に該当するボタン
  Joystick.setButton(L3,0); 
//Joystick


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

//Joystick
  //ジョイスティックの可変抵抗値読み取り
  x_value = analogRead(x_pin);//norm 519 MAX 771(right) min 251(left) push 1023
  y_value = analogRead(y_pin);//norm 514 MAX 778(under) min 244(top)

  if(x_value != 1023){
    //L3ボタン解除
    Joystick.releaseButton(L3);

    //ジョイスティック値設定
    Joystick.setXAxis(x_value);
    Joystick.setYAxis(y_value);
  }else{
       //L3ボタン押下
       Joystick.pressButton(L3);
  }
//Joystick

   delay(50);
}
