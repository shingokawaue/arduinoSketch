//********************************
//加速度センサの値を取得するプログラム
//********************************
void setup()
{
// シリアルモニターの初期化をする
Serial.begin(9600) ;

pinMode(4,OUTPUT);

digitalWrite(4,LOW);
}

void loop()
{
long x , y ;
x = y = 0 ;
x = analogRead(0) ; // Ｘ軸
y = analogRead(1) ; // Ｙ軸

Serial.print("X:") ;
Serial.print(x) ;
Serial.print(" Y:") ;
Serial.println(y) ;

if(x > 400 || y > 400){
digitalWrite(4,HIGH);
delay(500);
digitalWrite(4,LOW);
}

delay(50) ;

}

