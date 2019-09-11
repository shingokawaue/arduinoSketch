#include "Joystick.h"

//可変抵抗をつなげるpin
const int x_pin = A0;
const int y_pin = A1;

//L3ボタン(11ボタン)に該当するジョイスティック押し込み
const int L3 = 10;

//可変抵抗の値
int x_value = 0;
int y_value = 0;

//AJLクラスオブジェクト
Joystick_ Joystick;

void setup() {
  // put your setup code here, to run once:
  //AJLの初期化
  Joystick.begin(true);

  Joystick.setXAxisRange(267,771);//519が中間値になるように設定
  Joystick.setYAxisRange(262,766);//514が中間値になるように設定

  //L3に該当するボタン
  Joystick.setButton(L3,0); 
}

void loop() {
  // put your main code here, to run repeatedly:

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


  delay(50);
}
