int val=0;
int ledVal = 0;
void setup() {
Serial.begin(9800);
}

void loop() {
  // put your main code here, to run repeatedly:
val=analogRead(0);
ledVal = val/4;
Serial.println(val/4);//モニターへ

if(ledVal < 50){
//入力値が50以下の時は消灯
analogWrite(3,0); //出力は0
}
else if(ledVal < 100){
//入力値が50〜100の範囲の時は消灯
analogWrite(3,100); //出力は100
}
else if(ledVal < 150){
//入力値が100〜150の範囲の時は消灯
analogWrite(3,150); //出力は150
}
else if(ledVal < 200){
//入力値が150〜200の範囲の時は消灯
analogWrite(3,200); //出力は200
}
else{
//入力値が200以上の時は最大出力
analogWrite(3,255); //出力は255
}
delay(100);
}
