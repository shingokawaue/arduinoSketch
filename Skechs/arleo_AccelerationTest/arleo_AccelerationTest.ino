//********************************
//加速度センサの値を取得するプログラム
//********************************
void setup()
{
// シリアルモニターの初期化をする
Serial.begin(9600) ;
}


long beforeX = 0;
long beforeY = 0;
long beforeZ = 0;

void loop()
{
long x , y , z ;
x = y = z = 0 ;
x = analogRead(0) ; // Ｘ軸
y = analogRead(1) ; // Ｙ軸
z = analogRead(2) ; // Ｚ軸


            x = (beforeX * 0.9 + x * 0.1);
            y = (beforeY * 0.9 + y * 0.1);
            z = (beforeZ * 0.9 + z * 0.1);

            
Serial.print("X:") ;
Serial.print(x) ;
Serial.print(" Y:") ;
Serial.print(y) ;
Serial.print(" Z:") ;
Serial.println(z) ;

beforeX = x;
beforeY = y;
beforeZ = z;

delay(50) ;
}

