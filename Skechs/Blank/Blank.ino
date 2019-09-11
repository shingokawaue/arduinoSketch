int val=0; //入力される値を格納する為の変数
int ledval=0;
void setup() {
Serial.begin(9800); //モニターに出力するための設定
}
void loop() {
//ANALOG INの０番ピンからデータを受け付ける
val=analogRead(0);
Serial.println(val/4); //入力された値をモニターに出力

if ( val < 100 ) {ledval = 255;}
else if ( val/4 < 177 ){ledval = 100;}
else if ( val/4 < 180 ){ledval = 50;}
else if ( val/4 < 200 ){ledval = 20;}
else if ( val/4 < 220 ){ledval = 8;}
else  {ledval = 3;}
analogWrite(3,ledval);
Serial.println(ledval); //入力された値をモニターに出力  

delay(100);
}
