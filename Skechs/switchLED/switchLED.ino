void setup() {
     pinMode(2,INPUT) ;    //スイッチに接続ピンをデジタル入力に設定
     pinMode(13,OUTPUT) ;  //ＬＥＤに接続ピンをデジタル出力に設定
}
void loop() {
     if (digitalRead(2) == HIGH) {     //スイッチの状態を調べる
          digitalWrite(13,HIGH) ;      //スイッチが押されているならLEDを点灯
     } else {
          digitalWrite(13,LOW) ;       //スイッチが押されていないならLEDを消灯
     }
}
